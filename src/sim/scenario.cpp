#include "bcm/sim/scenario.hpp"

#include <algorithm>
#include <fstream>
#include <memory>
#include <random>
#include <sstream>

#include "bcm/app/bcm.hpp"
#include "bcm/app/naive_bcm.hpp"
#include "bcm/can/bcm_db.hpp"

namespace bcm::sim {

using hal::Din;
using hal::Dout;

double Profile::at(uint32_t t) const {
  if (pts.empty()) return 0.0;
  if (t <= pts.front().first) return pts.front().second;
  for (size_t i = 1; i < pts.size(); ++i) {
    if (t <= pts[i].first) {
      const auto& a = pts[i - 1];
      const auto& b = pts[i];
      const double x = static_cast<double>(t - a.first) / static_cast<double>(b.first - a.first);
      return a.second + x * (b.second - a.second);
    }
  }
  return pts.back().second;
}

const std::vector<std::string>& fault_classes() {
  static const std::vector<std::string> c = {
      "nominal",      "esc_timeout",     "acu_timeout",   "rls_timeout",  "e2e_burst",
      "e2e_sporadic", "frozen_counter",  "bus_off",       "stuck_switch", "input_glitch",
      "wiper_stall",  "engine_no_start", "rls_sensor_fault", "combined"};
  return c;
}

namespace {

using Rng = std::mt19937_64;

uint32_t uni(Rng& r, uint32_t lo, uint32_t hi) { return std::uniform_int_distribution<uint32_t>(lo, hi)(r); }
double unid(Rng& r, double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(r); }
bool chance(Rng& r, double p) { return unid(r, 0, 1) < p; }
uint32_t q10(uint32_t t) { return t / 10 * 10; }

void press(Scenario& sc, Din pin, uint32_t t, uint32_t hold) {
  sc.at(q10(t), [pin](VehicleSim& s) { s.set_switch(pin, true); });
  sc.at(q10(t + hold), [pin](VehicleSim& s) { s.set_switch(pin, false); });
}

void set_wiper(Scenario& sc, uint32_t t, int pos) {  // 0 off 1 auto 2 low 3 high
  sc.at(q10(t), [pos](VehicleSim& s) {
    s.set_switch(Din::WiperAuto, pos == 1);
    s.set_switch(Din::WiperLow, pos == 2);
    s.set_switch(Din::WiperHigh, pos == 3);
  });
}

uint32_t pick_msg(Rng& r) {
  static const uint32_t ids[] = {can::id::kEscVehicleSpeed, can::id::kEcmEngineStatus, can::id::kAcuCrashStatus,
                                 can::id::kRlsRainLight};
  return ids[uni(r, 0, 3)];
}

const char* msg_name(uint32_t id) { return can::message(id).name.c_str(); }

void apply_fault(Scenario& sc, const std::string& fc, Rng& r, std::ostringstream& d) {
  if (fc == "esc_timeout") {
    if (chance(r, 0.5)) {
      const uint32_t a = uni(r, 20000, 75000), len = uni(r, 5000, 20000);
      sc.at(q10(a), [](VehicleSim& s) { s.fault(can::id::kEscVehicleSpeed).drop = true; });
      sc.at(q10(a + len), [](VehicleSim& s) { s.fault(can::id::kEscVehicleSpeed).drop = false; });
      d << "ESC silent " << a << "+" << len << "ms; ";
    } else {
      sc.at(85000, [](VehicleSim& s) { s.fault(can::id::kEscVehicleSpeed).drop = true; });
      d << "ESC silent from 85000ms while decelerating; ";
    }
  } else if (fc == "acu_timeout") {
    const uint32_t a = uni(r, 15000, 80000), len = uni(r, 5000, 20000);
    sc.at(q10(a), [](VehicleSim& s) { s.fault(can::id::kAcuCrashStatus).drop = true; });
    sc.at(q10(a + len), [](VehicleSim& s) { s.fault(can::id::kAcuCrashStatus).drop = false; });
    d << "ACU silent " << a << "+" << len << "ms; ";
  } else if (fc == "rls_timeout") {
    const uint32_t a = uni(r, 15000, 70000), len = uni(r, 10000, 30000);
    sc.at(q10(a), [](VehicleSim& s) { s.fault(can::id::kRlsRainLight).drop = true; });
    sc.at(q10(a + len), [](VehicleSim& s) { s.fault(can::id::kRlsRainLight).drop = false; });
    d << "RLS silent " << a << "+" << len << "ms; ";
  } else if (fc == "e2e_burst") {
    const uint32_t id = pick_msg(r), a = uni(r, 15000, 80000), len = uni(r, 2000, 10000);
    sc.at(q10(a), [id](VehicleSim& s) { s.fault(id).corrupt_prob = 1.0; });
    sc.at(q10(a + len), [id](VehicleSim& s) { s.fault(id).corrupt_prob = 0.0; });
    d << msg_name(id) << " corrupted " << a << "+" << len << "ms; ";
  } else if (fc == "e2e_sporadic") {
    const double p = unid(r, 0.02, 0.08);
    sc.at(0, [p](VehicleSim& s) {
      for (uint32_t id : {can::id::kEscVehicleSpeed, can::id::kEcmEngineStatus, can::id::kAcuCrashStatus,
                          can::id::kRlsRainLight, can::id::kRkeCommand})
        s.fault(id).corrupt_prob = p;
    });
    d << "all messages corrupted with p=" << p << "; ";
  } else if (fc == "frozen_counter") {
    const uint32_t id = pick_msg(r), a = uni(r, 15000, 80000), len = uni(r, 5000, 15000);
    sc.at(q10(a), [id](VehicleSim& s) { s.fault(id).freeze_counter = true; });
    sc.at(q10(a + len), [id](VehicleSim& s) { s.fault(id).freeze_counter = false; });
    d << msg_name(id) << " counter frozen " << a << "+" << len << "ms; ";
  } else if (fc == "bus_off") {
    const int n = static_cast<int>(uni(r, 1, 3));
    for (int k = 0; k < n; ++k) {
      const uint32_t a = uni(r, 10000 + k * 30000, 30000 + k * 30000), len = uni(r, 200, 4000);
      sc.at(q10(a), [](VehicleSim& s) { s.set_bus_fault(true); });
      sc.at(q10(a + len), [](VehicleSim& s) { s.set_bus_fault(false); });
      d << "bus fault " << a << "+" << len << "ms; ";
    }
  } else if (fc == "stuck_switch") {
    static const Din pins[] = {Din::HazardSw, Din::LockSw, Din::UnlockSw, Din::WasherSw, Din::TurnLeft};
    static const char* names[] = {"HazardSw", "LockSw", "UnlockSw", "WasherSw", "TurnLeft"};
    const uint32_t k = uni(r, 0, 4), a = uni(r, 20000, 60000), len = uni(r, 15000, 40000);
    const Din pin = pins[k];
    sc.at(q10(a), [pin](VehicleSim& s) { s.set_stuck(pin, true); });
    sc.at(q10(a + len), [pin](VehicleSim& s) { s.set_stuck(pin, false); });
    d << names[k] << " stuck closed " << a << "+" << len << "ms; ";
  } else if (fc == "input_glitch") {
    const double p = unid(r, 0.0005, 0.003);
    sc.at(0, [p](VehicleSim& s) {
      s.glitch_prob = p;
      s.bounce_ms = 20;
    });
    d << "switch glitches p=" << p << "/tick + 20ms bounce; ";
  } else if (fc == "wiper_stall") {
    const uint32_t a = uni(r, 25000, 50000), len = uni(r, 6000, 15000);
    set_wiper(sc, a - 2000, 2);
    sc.at(q10(a), [](VehicleSim& s) { s.wiper_stalled = true; });
    sc.at(q10(a + len), [](VehicleSim& s) { s.wiper_stalled = false; });
    set_wiper(sc, a + len + 3000, 0);
    d << "wiper stalled " << a << "+" << len << "ms; ";
  } else if (fc == "engine_no_start") {
    sc.at(0, [](VehicleSim& s) { s.engine_can_start = false; });
    sc.at(6000, [](VehicleSim& s) { s.set_key(KeyPos::Start); });  // keep holding the key
    sc.at(16500, [](VehicleSim& s) { s.set_key(KeyPos::Run); });
    sc.at(17000, [](VehicleSim& s) { s.engine_can_start = true; });
    sc.at(18000, [](VehicleSim& s) { s.set_key(KeyPos::Start); });
    sc.at(19500, [](VehicleSim& s) { s.set_key(KeyPos::Run); });
    d << "engine does not start, key held in START 12 s; ";
  } else if (fc == "rls_sensor_fault") {
    const uint32_t a = uni(r, 15000, 70000), len = uni(r, 10000, 30000);
    sc.at(q10(a), [](VehicleSim& s) { s.rls_sensor_fault = true; });
    sc.at(q10(a + len), [](VehicleSim& s) { s.rls_sensor_fault = false; });
    d << "RLS reports SensorFault " << a << "+" << len << "ms; ";
  }
}

}  // namespace

Scenario make_scenario(const std::string& fault_class, uint64_t seed, int index) {
  Rng r(seed);
  Scenario sc;
  sc.seed = seed;
  sc.fault_class = fault_class;
  sc.name = "S" + std::to_string(index) + "_" + fault_class;
  std::ostringstream d;

  // --- crash ----------------------------------------------------------------
  sc.has_crash = chance(r, 0.25);
  const uint32_t t_crash = q10(uni(r, 30000, 85000));

  // --- speed profile ----------------------------------------------------------
  const double v1 = unid(r, 30, 80), v2 = unid(r, 20, 90), v3 = unid(r, 40, 80);
  sc.speed.pts = {{0, 0}, {10000, 0}, {25000, v1}, {45000, v2}, {60000, v3}, {85000, v3}, {92000, 0}, {200000, 0}};
  if (sc.has_crash) {
    std::vector<std::pair<uint32_t, double>> p;
    for (auto& pt : sc.speed.pts) {
      if (pt.first < t_crash) p.push_back(pt);
    }
    p.push_back({t_crash, sc.speed.at(t_crash)});
    p.push_back({t_crash + 1500, 0});
    p.push_back({200000, 0});
    sc.speed.pts = p;
    sc.at(t_crash, [](VehicleSim& s) { s.crash = true; });
    d << "crash at " << t_crash << "ms; ";
  }

  // --- ambient light ------------------------------------------------------------
  const uint32_t lt = uni(r, 0, 3);
  if (lt == 0) {
    sc.lux.pts = {{0, unid(r, 15000, 40000)}};
    d << "day; ";
  } else if (lt == 1) {
    sc.lux.pts = {{0, unid(r, 2500, 4000)}, {120000, unid(r, 200, 600)}};
    d << "dusk; ";
  } else if (lt == 2) {
    sc.lux.pts = {{0, unid(r, 2, 20)}};
    d << "night; ";
  } else {
    const double day = unid(r, 15000, 40000);
    const uint32_t a = uni(r, 20000, 60000), len = uni(r, 8000, 20000);
    sc.lux.pts = {{0, day}, {a, day}, {a + 300, 40}, {a + len, 40}, {a + len + 300, day}};
    d << "tunnel " << a << "+" << len << "ms; ";
  }

  // --- rain + wiper switch ----------------------------------------------------------
  const uint32_t rt = uni(r, 0, 4);
  if (rt == 0) {
    sc.rain.pts = {{0, 0}};
  } else if (rt == 4) {
    for (uint32_t t = 0; t <= 120000; t += 15000) sc.rain.pts.push_back({t, unid(r, 0, 85)});
  } else {
    const double lvl = rt == 1 ? unid(r, 8, 20) : (rt == 2 ? unid(r, 25, 50) : unid(r, 60, 90));
    sc.rain.pts = {{0, lvl}, {60000, lvl * unid(r, 0.6, 1.3)}, {120000, lvl}};
  }
  d << "rain profile " << rt << "; ";
  if (rt != 0) {
    const int start_pos = chance(r, 0.7) ? 1 : static_cast<int>(uni(r, 2, 3));
    set_wiper(sc, 3700, start_pos);
    const int toggles = static_cast<int>(uni(r, 1, 3));
    uint32_t t = 15000;
    for (int k = 0; k < toggles; ++k) {
      t += uni(r, 5000, 20000);
      set_wiper(sc, t, 0);
      t += uni(r, 2000, 8000);
      set_wiper(sc, t, static_cast<int>(uni(r, 1, 3)));
    }
    set_wiper(sc, 93000, 0);
  } else if (chance(r, 0.4)) {
    const uint32_t t = uni(r, 15000, 70000);
    set_wiper(sc, t, 2);
    set_wiper(sc, t + uni(r, 3000, 10000), 0);
  }
  if (chance(r, 0.3)) press(sc, Din::WasherSw, uni(r, 15000, 80000), uni(r, 500, 2000));

  // --- headlamp switch ---------------------------------------------------------
  const double hl = unid(r, 0, 1);
  const bool auto_forced = fault_class == "rls_timeout" || fault_class == "rls_sensor_fault";
  if (hl < 0.8 || auto_forced) {
    sc.at(3600, [](VehicleSim& s) { s.set_switch(Din::HeadlampAuto, true); });
  } else if (hl < 0.9) {
    sc.at(3600, [](VehicleSim& s) { s.set_switch(Din::HeadlampOn, true); });
  }
  if (chance(r, 0.3)) {
    const uint32_t t = uni(r, 20000, 80000);
    press(sc, Din::HighBeamSw, t, uni(r, 2000, 8000));
  }

  // --- turn signals / hazards ------------------------------------------------------
  const int nturn = static_cast<int>(uni(r, 4, 8));
  for (int k = 0; k < nturn; ++k) {
    const Din side = chance(r, 0.5) ? Din::TurnLeft : Din::TurnRight;
    const uint32_t t = uni(r, 12000, 88000);
    const uint32_t hold = chance(r, 0.4) ? uni(r, 150, 400) : uni(r, 1500, 5000);
    press(sc, side, t, hold);
  }
  if (chance(r, 0.3)) {
    const uint32_t t = uni(r, 15000, 70000);
    press(sc, Din::HazardSw, t, 150);
    press(sc, Din::HazardSw, t + uni(r, 5000, 15000), 150);
  }

  // --- base drive cycle --------------------------------------------------------------
  sc.at(300, [](VehicleSim& s) { s.rke_unlock(); });
  sc.at(500, [](VehicleSim& s) { s.set_switch(Din::DriverDoorAjar, true); });
  sc.at(2000, [](VehicleSim& s) { s.set_switch(Din::DriverDoorAjar, false); });
  sc.at(2500, [](VehicleSim& s) { s.set_key(KeyPos::Acc); });
  sc.at(3500, [](VehicleSim& s) { s.set_key(KeyPos::Run); });
  sc.at(4500, [](VehicleSim& s) { s.set_key(KeyPos::Start); });
  sc.at(6000, [](VehicleSim& s) { s.set_key(KeyPos::Run); });
  if (chance(r, 0.2)) {
    press(sc, Din::UnlockSw, uni(r, 30000, 80000), 200);  // unlock while driving
    d << "unlock while driving; ";
  }
  if (chance(r, 0.2)) press(sc, Din::LockSw, uni(r, 30000, 80000), 200);
  press(sc, Din::UnlockSw, 95000, 200);
  sc.at(96500, [](VehicleSim& s) { s.set_switch(Din::DriverDoorAjar, true); });
  sc.at(98500, [](VehicleSim& s) { s.set_switch(Din::DriverDoorAjar, false); });
  sc.at(99500, [](VehicleSim& s) { s.set_key(KeyPos::Acc); });
  sc.at(100000, [](VehicleSim& s) { s.set_key(KeyPos::Off); });
  sc.at(103000, [](VehicleSim& s) { s.rke_lock(); });

  // --- faults --------------------------------------------------------------------
  if (fault_class == "combined") {
    std::vector<std::string> pool(fault_classes().begin() + 1, fault_classes().end() - 1);
    std::shuffle(pool.begin(), pool.end(), r);
    sc.fault_class = "combined";
    for (int k = 0; k < 3; ++k) apply_fault(sc, pool[k], r, d);
  } else {
    apply_fault(sc, fault_class, r, d);
  }

  std::stable_sort(sc.actions.begin(), sc.actions.end(),
                   [](const auto& a, const auto& b) { return a.first < b.first; });
  sc.description = d.str();
  return sc;
}

std::vector<Scenario> make_scenario_set(int n, uint64_t seed) {
  std::vector<Scenario> out;
  const auto& fc = fault_classes();
  for (int i = 0; i < n; ++i) {
    out.push_back(make_scenario(fc[i % fc.size()], seed * 1000003ULL + static_cast<uint64_t>(i), i));
  }
  return out;
}

RunResult run_scenario(const Scenario& sc, ControllerKind kind, const std::string& trace_csv) {
  hal::SimHal hal;
  VehicleSim sim(hal, sc.seed ^ 0x9E3779B97F4A7C15ULL);
  std::unique_ptr<app::Controller> ctrl;
  app::Bcm* bcm = nullptr;
  if (kind == ControllerKind::StateMachine) {
    auto b = std::make_unique<app::Bcm>(hal);
    bcm = b.get();
    ctrl = std::move(b);
  } else {
    ctrl = std::make_unique<app::NaiveBcm>(hal);
  }
  RequirementMonitor mon;
  std::ofstream trace;
  if (!trace_csv.empty()) {
    trace.open(trace_csv);
    trace << "t_ms,key,speed,lux,rain,locked,lamp_l,lamp_r,low_beam,wiper_low,wiper_high,park,starter,engine,"
             "can_state,fault_flags\n";
  }
  RunResult res;
  size_t next = 0;
  for (uint32_t t = 0; t <= sc.duration_ms; t += app::kTickMs) {
    while (next < sc.actions.size() && sc.actions[next].first <= t) {
      sc.actions[next].second(sim);
      ++next;
    }
    sim.speed_kph = sc.speed.at(t);
    sim.ambient_lux = sc.lux.at(t);
    sim.rain_pct = sc.rain.at(t);
    sim.pre_step(t);
    ctrl->step();
    sim.post_step(t);
    mon.observe(t, sim, hal);
    if (bcm) res.fault_flags_seen |= bcm->fault_flags();
    if (trace.is_open() && t % 50 == 0) {
      trace << t << ',' << static_cast<int>(sim.key()) << ',' << sim.speed_kph << ',' << sim.ambient_lux << ','
            << sim.rain_pct << ',' << sim.doors_locked() << ',' << hal.output(Dout::TurnLeftLamp) << ','
            << hal.output(Dout::TurnRightLamp) << ',' << hal.output(Dout::LowBeam) << ','
            << hal.output(Dout::WiperLow) << ',' << hal.output(Dout::WiperHigh) << ',' << sim.wiper_parked() << ','
            << hal.output(Dout::StarterRelay) << ',' << sim.engine_running() << ','
            << static_cast<int>(hal.can_state()) << ',' << ctrl->fault_flags() << '\n';
    }
  }
  res.violations = mon.violations();
  res.total = mon.total();
  res.crash_unlock_latency_ms = mon.crash_unlock_latency_ms();
  res.busoff_recovery_ms = mon.busoff_recovery_ms();
  return res;
}

}  // namespace bcm::sim

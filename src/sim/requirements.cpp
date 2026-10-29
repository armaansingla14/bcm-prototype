#include "bcm/sim/requirements.hpp"

#include "bcm/can/bcm_db.hpp"

namespace bcm::sim {

using hal::Din;
using hal::Dout;

const char* to_string(Req r) {
  static const char* n[] = {"R1_CrashResponse", "R2_UncommandedLock", "R3_StarterProtect", "R4_FlasherTiming",
                            "R5_AutoLampsOn",   "R6_LampFlicker",     "R7_WiperStall",     "R8_WiperPark",
                            "R9_LockActuator",  "R10_BusOffRecovery", "R11_StatusPeriod"};
  return n[static_cast<int>(r)];
}

const char* describe(Req r) {
  static const char* d[] = {
      "Doors unlocked <=300 ms after a confirmed crash frame and kept unlocked 10 s; both indicators lit <=500 ms",
      "Door lock state changes only after a lock/unlock request (switch, RKE), speed >15 km/h, or a crash",
      "Starter energised <=10 s continuously and never >300 ms while the engine runs",
      "Indicator on-pulses 250-450 ms and off-gaps >=250 ms",
      "Headlamp AUTO in RUN: low beam on within 3 s of darkness (<1000 lux) or light-sensor loss",
      "Headlamp AUTO in RUN: no two low-beam transitions within 2 s",
      "Wiper motor not energised for >3.5 s without park-switch movement (stall)",
      "Wiper parked with motor off 2.5 s after the switch is turned off",
      "Lock actuator energised <=500 ms per activation and <=10 activations per 30 s",
      "Bus-off handling: <=6 recovery attempts in any 1 s window",
      "BCM_Status transmitted with gaps <=150 ms while the bus is healthy",
  };
  return d[static_cast<int>(r)];
}

int RequirementMonitor::total() const {
  int s = 0;
  for (int x : v_) s += x;
  return s;
}

void RequirementMonitor::observe(uint32_t t, const VehicleSim& sim, const hal::SimHal& hal) {
  const int64_t now = t;
  const bool locked = sim.doors_locked();
  const bool lamp_l = hal.output(Dout::TurnLeftLamp), lamp_r = hal.output(Dout::TurnRightLamp);

  // ---- R1 crash response -----------------------------------------------------
  if (sim.crash_confirmed_ms >= 0 && !r1_done_) {
    const int64_t tc = sim.crash_confirmed_ms;
    const int64_t dt = now - tc;
    if (dt >= 0) {
      if (dt == 0) r1_locked_at_crash_ = locked;
      if (r1_locked_at_crash_ && !locked && crash_latency_ < 0) crash_latency_ = static_cast<int32_t>(dt);
      if (lamp_l && lamp_r && dt <= 500) r1_hazard_seen_ = true;
      bool bad = false;
      if (dt >= 300 && locked) bad = true;
      if (dt == 500 && !r1_hazard_seen_) bad = true;
      if (bad && !r1_flagged_) {
        flag(Req::CrashResponse);
        r1_flagged_ = true;
      }
      if (dt >= 10000) r1_done_ = true;
    }
  }

  // ---- R2 uncommanded lock changes ----------------------------------------------
  if (sim.speed_kph > 15.0) last_fast_ms_ = static_cast<int32_t>(t);
  if (!first_ && locked != prev_locked_) {
    bool legit;
    if (locked) {
      legit = (now - sim.last_lock_request_ms <= 1000) || (now - last_fast_ms_ <= 1000);
    } else {
      legit = (now - sim.last_unlock_request_ms <= 1000) || sim.crash_frames_delivered > 0;
    }
    if (!legit) flag(Req::UncommandedLock);
  }
  prev_locked_ = locked;
  first_ = false;

  // ---- R3 starter protection -------------------------------------------------
  const bool starter = hal.output(Dout::StarterRelay);
  if (starter && !starter_prev_) {
    starter_since_ = t;
    r3_long_flag_ = false;
  }
  if (starter && t - starter_since_ > 10100 && !r3_long_flag_) {
    flag(Req::StarterProtect);
    r3_long_flag_ = true;
  }
  if (starter && sim.engine_running()) {
    starter_running_ms_ += 10;
    if (starter_running_ms_ > 300 && !r3_run_flag_) {
      flag(Req::StarterProtect);
      r3_run_flag_ = true;
    }
  } else {
    starter_running_ms_ = 0;
    r3_run_flag_ = false;
  }
  starter_prev_ = starter;

  // ---- R4 flasher timing --------------------------------------------------------
  const bool lamp_now[2] = {lamp_l, lamp_r};
  for (int i = 0; i < 2; ++i) {
    Lamp& L = lamps_[i];
    if (lamp_now[i] && !L.prev) {
      if (L.fall >= 0 && now - L.fall < 250) flag(Req::FlasherTiming);
      L.rise = now;
    } else if (!lamp_now[i] && L.prev) {
      const int64_t on = now - L.rise;
      if (on < 250) flag(Req::FlasherTiming);  // long pulses caught below
      L.fall = now;
    } else if (lamp_now[i] && L.rise >= 0 && now - L.rise == 460) {
      flag(Req::FlasherTiming);  // lamp stuck on
    }
    L.prev = lamp_now[i];
  }

  // ---- R5 / R6 automatic headlamps ---------------------------------------------
  const bool run_key = sim.key() == KeyPos::Run || sim.key() == KeyPos::Start;
  const bool autosw = sim.switch_level(Din::HeadlampAuto) && !sim.switch_level(Din::HeadlampOn);
  const bool low = hal.output(Dout::LowBeam);
  const bool need = run_key && autosw && (sim.ambient_lux < 1000.0 || !sim.rls_available());
  if (need) {
    if (r5_since_ < 0) {
      r5_since_ = now;
      r5_flag_ = false;
    }
    if (now - r5_since_ > 3000 && !low && !r5_flag_) {
      flag(Req::AutoLampsOn);
      r5_flag_ = true;
    }
  } else {
    r5_since_ = -1;
  }
  if (run_key && autosw) {
    if (r6_active_ && low != r6_prev_beam_) {
      if (r6_last_change_ >= 0 && now - r6_last_change_ < 2000) flag(Req::LampFlicker);
      r6_last_change_ = now;
    }
    r6_active_ = true;
  } else {
    r6_active_ = false;
    r6_last_change_ = -1;
  }
  r6_prev_beam_ = low;

  // ---- R7 wiper stall ---------------------------------------------------------------
  const bool motor = hal.output(Dout::WiperLow) || hal.output(Dout::WiperHigh);
  const bool park = sim.wiper_parked();
  if (park != r7_prev_park_) r7_last_edge_ = now;
  if (motor && !r7_motor_prev_) {
    r7_last_edge_ = now;
    r7_flag_ = false;
  }
  if (motor && now - r7_last_edge_ > 3500 && !r7_flag_) {
    flag(Req::WiperStall);
    r7_flag_ = true;
  }
  r7_prev_park_ = park;
  r7_motor_prev_ = motor;

  // ---- R8 wiper park after switch-off -------------------------------------------------
  if (sim.last_wiper_off_ms >= 0 && sim.last_wiper_off_ms != r8_checked_for_ &&
      now - sim.last_wiper_off_ms >= 2500) {
    r8_checked_for_ = sim.last_wiper_off_ms;
    const bool sw_off = !sim.switch_level(Din::WiperAuto) && !sim.switch_level(Din::WiperLow) &&
                        !sim.switch_level(Din::WiperHigh) && !sim.switch_level(Din::WasherSw);
    const bool powered = sim.key() == KeyPos::Acc || sim.key() == KeyPos::Run;
    if (sw_off && powered && !sim.wiper_stalled && !sim.stuck(Din::WasherSw) && (motor || !park)) {
      flag(Req::WiperPark);
    }
  }

  // ---- R9 lock actuator -----------------------------------------------------------------
  const bool lm = hal.output(Dout::LockMotor), um = hal.output(Dout::UnlockMotor);
  if (lm || um) {
    lockmot_on_ms_ += 10;
    if (lockmot_on_ms_ > 500 && !r9_cont_flag_) {
      flag(Req::LockActuator);
      r9_cont_flag_ = true;
    }
  } else {
    lockmot_on_ms_ = 0;
    r9_cont_flag_ = false;
  }
  if ((lm && !r9_prev_lm_) || (um && !r9_prev_um_)) act_times_.push_back(t);
  while (!act_times_.empty() && t - act_times_.front() >= 30000) act_times_.pop_front();
  if (act_times_.size() > 10) {
    if (!r9_rate_flag_) flag(Req::LockActuator);
    r9_rate_flag_ = true;
  } else {
    r9_rate_flag_ = false;
  }
  r9_prev_lm_ = lm;
  r9_prev_um_ = um;

  // ---- R10 bus-off recovery rate ---------------------------------------------------------
  const uint32_t rec = hal.recovery_requests();
  for (uint32_t k = prev_recov_; k < rec; ++k) recov_times_.push_back(t);
  prev_recov_ = rec;
  while (!recov_times_.empty() && t - recov_times_.front() >= 1000) recov_times_.pop_front();
  if (recov_times_.size() > 6 && !r10_flag_) {
    flag(Req::BusOffRecovery);
    r10_flag_ = true;
  }
  if (!hal.bus_fault() && hal.can_state() == hal::CanState::Active) r10_flag_ = false;

  // ---- R11 status period + bus-off recovery time -----------------------------------------
  const auto& log = hal.tx_log();
  bool status_now = false;
  for (; tx_seen_ < log.size(); ++tx_seen_) {
    if (log[tx_seen_].frame.id == can::id::kBcmStatus) {
      last_status_ = log[tx_seen_].t_ms;
      status_now = true;
    }
  }
  const bool fault = hal.bus_fault();
  if (bus_fault_prev_ && !fault) fault_cleared_ = now;
  if (fault_cleared_ >= 0 && status_now) {
    busoff_rec_.push_back(static_cast<uint32_t>(now - fault_cleared_));
    fault_cleared_ = -1;
  }
  bus_fault_prev_ = fault;
  const bool healthy = !fault && hal.can_state() == hal::CanState::Active;
  if (healthy && !was_healthy_) {
    healthy_since_ = now;
    r11_flag_ = false;
  }
  was_healthy_ = healthy;
  if (healthy && now - healthy_since_ >= 1500 && last_status_ >= 0 && now - last_status_ > 150 && !r11_flag_) {
    flag(Req::StatusPeriod);
    r11_flag_ = true;
  }
  if (status_now) r11_flag_ = false;
}

}  // namespace bcm::sim

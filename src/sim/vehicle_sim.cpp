#include "bcm/sim/vehicle_sim.hpp"

#include "bcm/can/bcm_db.hpp"

namespace bcm::sim {

using hal::Din;
using hal::Dout;

VehicleSim::VehicleSim(hal::SimHal& hal, uint64_t seed) : hal_(hal), rng_(seed) {
  intended_.fill(false);
  stuck_.fill(false);
  changed_ms_.fill(0);
  for (uint32_t id : {can::id::kEscVehicleSpeed, can::id::kEcmEngineStatus, can::id::kAcuCrashStatus,
                      can::id::kRlsRainLight, can::id::kRkeCommand}) {
    faults_.push_back({id, MsgFault{}});
    counters_.push_back({id, 0});
  }
  intended_[static_cast<int>(Din::WiperPark)] = true;
}

MsgFault& VehicleSim::fault(uint32_t can_id) {
  for (auto& f : faults_) {
    if (f.first == can_id) return f.second;
  }
  faults_.push_back({can_id, MsgFault{}});
  return faults_.back().second;
}

void VehicleSim::set_key(KeyPos k) {
  key_ = k;
  set_switch(Din::IgnAcc, k == KeyPos::Acc || k == KeyPos::Run);
  set_switch(Din::IgnRun, k == KeyPos::Run || k == KeyPos::Start);
  set_switch(Din::IgnStart, k == KeyPos::Start);
}

void VehicleSim::set_switch(Din pin, bool v) {
  const int i = static_cast<int>(pin);
  if (intended_[i] == v) return;
  intended_[i] = v;
  changed_ms_[i] = last_now_;
  if (v && pin == Din::LockSw) last_lock_request_ms = static_cast<int32_t>(last_now_);
  if (v && pin == Din::UnlockSw) last_unlock_request_ms = static_cast<int32_t>(last_now_);
  const bool wiper_pin = pin == Din::WiperAuto || pin == Din::WiperLow || pin == Din::WiperHigh;
  if (wiper_pin && !intended_[static_cast<int>(Din::WiperAuto)] && !intended_[static_cast<int>(Din::WiperLow)] &&
      !intended_[static_cast<int>(Din::WiperHigh)]) {
    last_wiper_off_ms = static_cast<int32_t>(last_now_);
  }
}

void VehicleSim::rke_lock() { pending_rke_lock_ = true; }
void VehicleSim::rke_unlock() { pending_rke_unlock_ = true; }

void VehicleSim::set_stuck(Din pin, bool s, bool level) {
  stuck_[static_cast<int>(pin)] = s;
  if (s) {
    hal_.set_stuck(pin, level);
    if (level && pin == Din::LockSw) last_lock_request_ms = static_cast<int32_t>(last_now_);
    if (level && pin == Din::UnlockSw) last_unlock_request_ms = static_cast<int32_t>(last_now_);
  } else {
    hal_.set_stuck(pin, std::nullopt);
  }
}

bool VehicleSim::rls_available() const {
  for (const auto& f : faults_) {
    if (f.first == can::id::kRlsRainLight) {
      if (f.second.drop || f.second.corrupt_prob >= 0.5 || f.second.freeze_counter) return false;
    }
  }
  return !rls_sensor_fault && !hal_.bus_fault() && hal_.can_state() != hal::CanState::BusOff;
}

void VehicleSim::transmit(can::Frame f, uint32_t now_ms) {
  const MsgFault& flt = fault(f.id);
  if (flt.drop) return;
  for (auto& c : counters_) {
    if (c.first != f.id) continue;
    can::seal(f, c.second);
    if (!flt.freeze_counter) c.second = static_cast<uint8_t>((c.second + 1) & 0x0F);
  }
  ++frames_sent;
  bool corrupted = false;
  if (flt.corrupt_prob > 0.0 && std::uniform_real_distribution<double>(0, 1)(rng_) < flt.corrupt_prob) {
    std::uniform_int_distribution<int> bit(0, 55);
    const int nbits = 1 + static_cast<int>(rng_() % 3);
    for (int k = 0; k < nbits; ++k) {
      const int b = bit(rng_);
      f.data[b / 8] ^= static_cast<uint8_t>(1U << (b % 8));
    }
    corrupted = true;
    ++frames_corrupted;
  }
  const bool delivered = hal_.deliver(f);
  if (delivered && f.id == can::id::kAcuCrashStatus && !corrupted && !flt.freeze_counter && crash) {
    ++crash_frames_delivered;
    if (crash_frames_delivered == 2) crash_confirmed_ms = static_cast<int32_t>(now_ms);
  }
}

void VehicleSim::send_periodic(uint32_t now_ms) {
  if (now_ms % 20 == 0) {
    can::Frame f = can::make_frame(can::id::kEscVehicleSpeed);
    can::set(f, "VehicleSpeed", speed_kph);
    can::set(f, "SpeedValid", 1);
    transmit(f, now_ms);
  }
  if (now_ms % 100 == 0) {
    can::Frame f = can::make_frame(can::id::kEcmEngineStatus);
    can::set(f, "EngineRunning", engine_running_ ? 1 : 0);
    can::set(f, "EngineSpeed", engine_running_ ? 800.0 + 30.0 * speed_kph : 0.0);
    can::set(f, "CoolantTemp", 85);
    transmit(f, now_ms);
  }
  if (now_ms % 50 == 10) {
    can::Frame f = can::make_frame(can::id::kAcuCrashStatus);
    can::set(f, "CrashDetected", crash ? 1 : 0);
    can::set(f, "CrashSeverity", crash ? 5 : 0);
    transmit(f, now_ms);
  }
  if (now_ms % 100 == 30) {
    can::Frame f = can::make_frame(can::id::kRlsRainLight);
    can::set(f, "RainIntensity", rain_pct);
    can::set(f, "AmbientLight", ambient_lux);
    can::set(f, "SensorFault", rls_sensor_fault ? 1 : 0);
    transmit(f, now_ms);
  }
  if (pending_rke_lock_ || pending_rke_unlock_) {
    can::Frame f = can::make_frame(can::id::kRkeCommand);
    can::set(f, "RkeLockReq", pending_rke_lock_ ? 1 : 0);
    can::set(f, "RkeUnlockReq", pending_rke_unlock_ ? 1 : 0);
    const MsgFault& flt = fault(can::id::kRkeCommand);
    if (!flt.drop && hal_.can_state() != hal::CanState::BusOff) {
      if (pending_rke_lock_) last_lock_request_ms = static_cast<int32_t>(now_ms);
      if (pending_rke_unlock_) last_unlock_request_ms = static_cast<int32_t>(now_ms);
    }
    transmit(f, now_ms);
    pending_rke_lock_ = pending_rke_unlock_ = false;
  }
}

void VehicleSim::apply_inputs(uint32_t now_ms) {
  std::uniform_real_distribution<double> u(0, 1);
  for (int i = 0; i < hal::kNumDin; ++i) {
    const Din pin = static_cast<Din>(i);
    bool level = intended_[i];
    if (pin == Din::WiperPark) {
      level = wiper_parked();
    } else if (pin != Din::IgnAcc && pin != Din::IgnRun && pin != Din::IgnStart && pin != Din::DriverDoorAjar) {
      if (bounce_ms > 0 && now_ms - changed_ms_[i] < bounce_ms) level = (rng_() & 1U) != 0;
      if (glitch_prob > 0.0 && u(rng_) < glitch_prob) level = !level;
    }
    hal_.set_input(pin, level);
  }
}

void VehicleSim::pre_step(uint32_t now_ms) {
  last_now_ = now_ms;
  hal_.set_time(now_ms);
  send_periodic(now_ms);
  apply_inputs(now_ms);
}

void VehicleSim::post_step(uint32_t now_ms) {
  (void)now_ms;
  constexpr uint32_t kStep = 10;  // controller and plant both run every 10 ms
  // door lock actuator: needs >= 80 ms of drive to complete its travel
  const bool lm = hal_.output(Dout::LockMotor), um = hal_.output(Dout::UnlockMotor);
  lock_on_ms_ = (lm && !um) ? lock_on_ms_ + kStep : 0;
  unlock_on_ms_ = (um && !lm) ? unlock_on_ms_ + kStep : 0;
  if (lock_on_ms_ >= 80 && !locked_) {
    locked_ = true;
    ++lock_changes_;
  }
  if (unlock_on_ms_ >= 80 && locked_) {
    locked_ = false;
    ++lock_changes_;
  }

  // wiper motor: 1.0 s (low) / 0.65 s (high) per park-to-park cycle
  const bool wl = hal_.output(Dout::WiperLow), wh = hal_.output(Dout::WiperHigh);
  if ((wl || wh) && !wiper_stalled) {
    const double period = wh ? 650.0 : 1000.0;
    wiper_phase_ += kStep / period;
    if (wiper_phase_ >= 1.0) wiper_phase_ -= 1.0;
  }

  // engine: starts after 800 ms of cranking when startable, needs RUN relay
  const bool starter = hal_.output(Dout::StarterRelay), run = hal_.output(Dout::RunRelay);
  if (starter && run && !engine_running_) {
    crank_ms_ += kStep;
    if (engine_can_start && crank_ms_ >= 800) engine_running_ = true;
  } else if (!starter) {
    crank_ms_ = 0;
  }
  if (!run) engine_running_ = false;
}

}  // namespace bcm::sim

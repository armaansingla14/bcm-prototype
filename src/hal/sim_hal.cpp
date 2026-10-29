#include "bcm/hal/sim_hal.hpp"

namespace bcm::hal {

const char* to_string(Din d) {
  static const char* names[] = {"IgnAcc",   "IgnRun",     "IgnStart",   "TurnLeft",     "TurnRight",
                                "HazardSw", "HeadlampAuto", "HeadlampOn", "HighBeamSw", "WiperAuto",
                                "WiperLow", "WiperHigh",  "WasherSw",   "LockSw",       "UnlockSw",
                                "DriverDoorAjar", "WiperPark"};
  const int i = static_cast<int>(d);
  return i < kNumDin ? names[i] : "?";
}

const char* to_string(Dout d) {
  static const char* names[] = {"TurnLeftLamp", "TurnRightLamp", "LowBeam",     "HighBeam",
                                "WiperLow",     "WiperHigh",     "WasherPump",  "LockMotor",
                                "UnlockMotor",  "AccRelay",      "RunRelay",    "StarterRelay"};
  const int i = static_cast<int>(d);
  return i < kNumDout ? names[i] : "?";
}

bool SimHal::read(Din pin) const {
  const int i = static_cast<int>(pin);
  if (stuck_[i]) return *stuck_[i];
  return inputs_[i];
}

bool SimHal::can_transmit(const can::Frame& f) {
  if (state_ == CanState::BusOff) return false;
  tx_log_.push_back({now_, f});
  return true;
}

bool SimHal::can_receive(can::Frame& f) {
  if (rx_.empty()) return false;
  f = rx_.front();
  rx_.pop_front();
  return true;
}

bool SimHal::deliver(const can::Frame& f) {
  if (state_ == CanState::BusOff) {
    ++rx_dropped_bus_off_;
    return false;
  }
  if (rx_.size() >= rx_fifo_depth) {
    ++rx_overruns_;
    return false;
  }
  rx_.push_back(f);
  return true;
}

void SimHal::set_bus_fault(bool active) {
  bus_fault_ = active;
  if (active) {
    // TEC climbs past 255 within a few frames on a shorted bus
    state_ = CanState::BusOff;
    rx_.clear();
  }
}

void SimHal::can_request_recovery() {
  ++recovery_requests_;
  if (!bus_fault_) state_ = CanState::Active;
}

}  // namespace bcm::hal

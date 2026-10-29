#pragma once
#include <cstdint>
#include <deque>

#include "bcm/app/power_mode.hpp"

namespace bcm::app {

/// Central locking state machine.
///
/// * actuators are driven with fixed-length pulses (no position feedback)
/// * auto-lock once per drive cycle when a *fresh, valid* vehicle speed
///   exceeds autolock_kph in RUN; re-armed when the vehicle is stationary
///   with a door open, or at ignition off
/// * crash unlock: on a validated crash event all doors are unlocked twice
///   (second pulse after crash_repulse_ms) and locking is inhibited for
///   crash_inhibit_ms; crash unlock bypasses thermal protection
/// * remote lock is refused while the driver door is ajar
/// * thermal protection: at most max_actuations pulses per window_ms
class DoorLockSm {
 public:
  enum class State : uint8_t { Unlocked, Locked, Locking, Unlocking };
  struct Config {
    uint32_t pulse_ms = 200;
    double autolock_kph = 15.0;
    double rearm_kph = 3.0;
    uint32_t crash_inhibit_ms = 10000;
    uint32_t crash_repulse_ms = 1000;
    uint8_t max_actuations = 10;
    uint32_t window_ms = 30000;
  };
  struct Inputs {
    bool lock_sw_edge = false;
    bool unlock_sw_edge = false;
    bool rke_lock = false;
    bool rke_unlock = false;
    double speed_kph = 0.0;
    bool speed_valid = false;
    bool crash_event = false;
    bool door_ajar = false;
    PowerMode mode = PowerMode::Off;
  };

  DoorLockSm() = default;
  explicit DoorLockSm(const Config& c) : cfg_(c) {}

  void step(const Inputs& in, uint32_t now_ms);
  State state() const { return state_; }
  bool locked() const { return locked_; }
  bool lock_motor() const { return state_ == State::Locking; }
  bool unlock_motor() const { return state_ == State::Unlocking; }
  bool thermal_limited() const { return thermal_; }
  bool autolock_armed() const { return armed_; }
  bool crash_inhibit_active(uint32_t now_ms) const { return crash_active_ && now_ms < crash_until_; }
  uint32_t autolocks() const { return autolocks_; }

 private:
  bool budget_ok(uint32_t now_ms);
  void start_pulse(bool lock, uint32_t now_ms);

  Config cfg_{};
  State state_ = State::Unlocked;
  bool locked_ = false;
  uint32_t pulse_start_ = 0;
  bool armed_ = true;
  bool crash_active_ = false;
  uint32_t crash_until_ = 0;
  bool repulse_pending_ = false;
  uint32_t repulse_at_ = 0;
  bool thermal_ = false;
  std::deque<uint32_t> history_;
  uint32_t autolocks_ = 0;
};

}  // namespace bcm::app

#pragma once
#include <cstdint>

#include "bcm/app/power_mode.hpp"

namespace bcm::app {

enum class WiperSwitch : uint8_t { Off, Auto, Low, High };

/// Front wiper state machine.
///
/// * OFF / AUTO / LOW / HIGH column switch, washer push button
/// * AUTO with a valid rain sensor maps rain intensity to
///   none / intermittent (variable delay) / low / high with hysteresis;
///   without a valid rain sensor AUTO degrades to fixed intermittent
/// * switching off always completes the sweep to the park position
///   (park cam switch), intermittent wipes are single park-to-park sweeps
/// * washer: pump + low speed while held, then after_wash_wipes dry wipes
/// * stall supervision: motor energised without a park-switch edge for
///   stall_ms -> motor off, STALLED until the switch is turned off
/// * wipers are suspended during CRANK (load shedding)
class WiperSm {
 public:
  enum class State : uint8_t { Parked, IntWait, IntSweep, Low, High, Parking, Wash, AfterWash, Stalled, Suspended };
  struct Config {
    uint32_t fallback_int_ms = 4000;
    uint32_t int_min_ms = 1500;
    uint32_t int_max_ms = 6000;
    uint32_t stall_ms = 3000;
    uint8_t after_wash_wipes = 3;
  };
  struct Inputs {
    WiperSwitch sw = WiperSwitch::Off;
    bool washer = false;
    double rain_pct = 0.0;
    bool rain_valid = false;
    bool park = true;  ///< park switch closed
    PowerMode mode = PowerMode::Off;
  };

  WiperSm() = default;
  explicit WiperSm(const Config& c) : cfg_(c) {}

  void step(const Inputs& in, uint32_t now_ms);
  State state() const { return state_; }
  bool motor_low() const { return motor_low_; }
  bool motor_high() const { return motor_high_; }
  bool pump() const { return pump_; }
  bool continuous() const { return state_ == State::Low || state_ == State::High; }
  bool stalled() const { return state_ == State::Stalled; }
  bool degraded() const { return degraded_; }
  uint8_t status_code() const;
  uint32_t interval_ms() const { return interval_ms_; }

 private:
  enum class Target : uint8_t { None, Int, Low, High };
  Target compute_target(const Inputs& in);
  void enter_motion(Target t, uint32_t now_ms);

  Config cfg_{};
  State state_ = State::Parked;
  bool motor_low_ = false;
  bool motor_high_ = false;
  bool pump_ = false;
  bool degraded_ = false;
  uint8_t auto_level_ = 0;  // 0 none, 1 int, 2 low, 3 high
  uint32_t interval_ms_ = 4000;
  uint32_t wait_start_ = 0;
  bool left_park_ = false;
  bool prev_park_ = true;
  uint32_t last_edge_ms_ = 0;
  bool motor_was_on_ = false;
  uint8_t wipes_left_ = 0;
};

}  // namespace bcm::app

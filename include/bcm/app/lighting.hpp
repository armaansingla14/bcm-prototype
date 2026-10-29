#pragma once
#include <cstdint>

#include "bcm/app/power_mode.hpp"

namespace bcm::app {

/// Exterior lighting state machine (low/high beam, automatic headlamps).
///
/// * switch ON: low beam in ACC/RUN/CRANK
/// * switch AUTO (RUN/CRANK): ambient-light control with hysteresis and
///   dwell times; lamps forced on when the light sensor is unavailable
///   (timeout, E2E error or sensor fault flag) - fail-safe towards "on"
/// * rain-light rule: continuous wiping for >= rain_on_ms turns lamps on
/// * exit delay: lamps that were on in AUTO stay on for exit_delay_ms after
///   ignition off
/// * high beam only together with low beam
class LightingSm {
 public:
  enum class State : uint8_t { Off, ManualOn, AutoOff, AutoOn, ExitDelay };
  struct Config {
    double on_lux = 1000.0;
    double off_lux = 2500.0;
    uint32_t on_dwell_ms = 2000;
    uint32_t off_dwell_ms = 5000;
    uint32_t rain_on_ms = 5000;
    uint32_t rain_off_ms = 10000;
    uint32_t exit_delay_ms = 30000;
  };
  struct Inputs {
    bool sw_auto = false;
    bool sw_on = false;
    bool high_beam_sw = false;
    double ambient_lux = 0.0;
    bool ambient_valid = false;
    bool wipers_continuous = false;
    PowerMode mode = PowerMode::Off;
  };

  LightingSm() = default;
  explicit LightingSm(const Config& c) : cfg_(c) {}

  void step(const Inputs& in, uint32_t now_ms);
  bool low_beam() const { return state_ == State::ManualOn || state_ == State::AutoOn || state_ == State::ExitDelay; }
  bool high_beam() const { return high_; }
  State state() const { return state_; }
  bool fail_safe_active() const { return fail_safe_; }

 private:
  void update_auto(const Inputs& in, uint32_t now_ms);

  Config cfg_{};
  State state_ = State::Off;
  bool high_ = false;
  bool dark_ = false;
  bool fail_safe_ = false;
  bool rain_on_ = false;
  uint32_t below_since_ = 0;
  uint32_t above_since_ = 0;
  bool below_ = false;
  bool above_ = false;
  uint32_t wipe_since_ = 0;
  uint32_t nowipe_since_ = 0;
  bool wiping_ = false;
  uint32_t exit_start_ = 0;
  PowerMode prev_mode_ = PowerMode::Off;
};

}  // namespace bcm::app

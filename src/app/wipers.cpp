#include "bcm/app/wipers.hpp"

namespace bcm::app {

uint8_t WiperSm::status_code() const {
  switch (state_) {
    case State::Parked:
    case State::Suspended: return 0;
    case State::IntWait:
    case State::IntSweep: return 1;
    case State::Low: return 2;
    case State::High: return 3;
    case State::Parking: return 4;
    case State::Stalled: return 5;
    case State::Wash:
    case State::AfterWash: return 6;
  }
  return 0;
}

WiperSm::Target WiperSm::compute_target(const Inputs& in) {
  degraded_ = false;
  switch (in.sw) {
    case WiperSwitch::Off: return Target::None;
    case WiperSwitch::Low: return Target::Low;
    case WiperSwitch::High: return Target::High;
    case WiperSwitch::Auto: break;
  }
  if (!in.rain_valid) {
    degraded_ = true;
    interval_ms_ = cfg_.fallback_int_ms;
    return Target::Int;
  }
  // thresholds with hysteresis (up / down)
  static const double up[3] = {5.0, 30.0, 70.0};
  static const double down[3] = {3.0, 25.0, 62.0};
  while (auto_level_ < 3 && in.rain_pct >= up[auto_level_]) ++auto_level_;
  while (auto_level_ > 0 && in.rain_pct < down[auto_level_ - 1]) --auto_level_;
  if (auto_level_ == 1) {
    // linear map 5..30 % -> int_max..int_min
    double x = (in.rain_pct - 5.0) / 25.0;
    x = x < 0 ? 0 : (x > 1 ? 1 : x);
    interval_ms_ = static_cast<uint32_t>(cfg_.int_max_ms - x * (cfg_.int_max_ms - cfg_.int_min_ms));
  }
  switch (auto_level_) {
    case 0: return Target::None;
    case 1: return Target::Int;
    case 2: return Target::Low;
    default: return Target::High;
  }
}

void WiperSm::enter_motion(Target t, uint32_t now_ms) {
  switch (t) {
    case Target::None: state_ = State::Parking; break;
    case Target::Int:
      state_ = State::IntSweep;
      left_park_ = false;
      (void)now_ms;
      break;
    case Target::Low: state_ = State::Low; break;
    case Target::High: state_ = State::High; break;
  }
}

void WiperSm::step(const Inputs& in, uint32_t now_ms) {
  const bool park_edge = in.park != prev_park_;
  const bool park_closed_edge = in.park && !prev_park_;
  prev_park_ = in.park;
  if (park_edge) last_edge_ms_ = now_ms;
  if (!in.park) left_park_ = true;

  const bool powered = (in.mode == PowerMode::Acc || in.mode == PowerMode::Run);
  const Target target = powered ? compute_target(in) : Target::None;

  if (in.mode == PowerMode::Crank && state_ != State::Stalled) {
    state_ = State::Suspended;
  } else if (state_ == State::Suspended) {
    state_ = in.park ? State::Parked : State::Parking;
  }

  if (state_ != State::Stalled && state_ != State::Suspended && powered && in.washer) {
    if (state_ != State::Wash) state_ = State::Wash;
  }

  switch (state_) {
    case State::Parked:
      if (target != Target::None) enter_motion(target, now_ms);
      break;
    case State::IntWait:
      if (target == Target::Int) {
        if (now_ms - wait_start_ >= interval_ms_) enter_motion(Target::Int, now_ms);
      } else if (target == Target::None) {
        state_ = State::Parked;
      } else {
        enter_motion(target, now_ms);
      }
      break;
    case State::IntSweep:
      if (target == Target::Low || target == Target::High) {
        enter_motion(target, now_ms);
      } else if (left_park_ && in.park) {
        state_ = (target == Target::Int) ? State::IntWait : State::Parked;
        wait_start_ = now_ms;
      }
      break;
    case State::Low:
    case State::High:
      if (target == Target::Low || target == Target::High) {
        state_ = (target == Target::Low) ? State::Low : State::High;
      } else if (target == Target::Int) {
        state_ = State::IntSweep;  // finish this sweep, then wait
        left_park_ = !in.park;
      } else {
        state_ = State::Parking;
      }
      break;
    case State::Parking:
      if (target == Target::Low || target == Target::High) {
        enter_motion(target, now_ms);
      } else if (in.park) {
        state_ = (target == Target::Int) ? State::IntWait : State::Parked;
        wait_start_ = now_ms;
      }
      break;
    case State::Wash:
      if (!in.washer || !powered) {
        state_ = State::AfterWash;
        wipes_left_ = cfg_.after_wash_wipes;
      }
      break;
    case State::AfterWash:
      if (park_closed_edge && wipes_left_ > 0) --wipes_left_;
      if (wipes_left_ == 0 || !powered) state_ = State::Parking;
      break;
    case State::Stalled:
      // switching off acknowledges the fault and tries one park run; if the
      // motor is still blocked the stall supervision trips again
      if (in.sw == WiperSwitch::Off && !in.washer) state_ = in.park ? State::Parked : State::Parking;
      break;
    case State::Suspended:
      break;
  }

  motor_low_ = (state_ == State::Low || state_ == State::IntSweep || state_ == State::Parking ||
                state_ == State::Wash || state_ == State::AfterWash);
  motor_high_ = (state_ == State::High);
  pump_ = (state_ == State::Wash);

  // stall supervision
  const bool motor_on = motor_low_ || motor_high_;
  if (motor_on && !motor_was_on_) last_edge_ms_ = now_ms;
  if (motor_on && now_ms - last_edge_ms_ >= cfg_.stall_ms) {
    state_ = State::Stalled;
    motor_low_ = motor_high_ = pump_ = false;
  }
  motor_was_on_ = motor_low_ || motor_high_;
}

}  // namespace bcm::app

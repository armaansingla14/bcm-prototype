#include "bcm/app/lighting.hpp"

namespace bcm::app {

void LightingSm::update_auto(const Inputs& in, uint32_t now_ms) {
  if (!auto_prev_) {
    // AUTO newly selected: take an immediate decision from the current
    // reading instead of carrying over stale dwell state
    dark_ = !in.ambient_valid || in.ambient_lux < cfg_.on_lux;
    below_ = above_ = false;
  }
  fail_safe_ = !in.ambient_valid;
  if (fail_safe_) {
    dark_ = true;
    below_ = above_ = false;
  } else {
    const bool below = in.ambient_lux < cfg_.on_lux;
    const bool above = in.ambient_lux > cfg_.off_lux;
    if (below && !below_) below_since_ = now_ms;
    if (above && !above_) above_since_ = now_ms;
    below_ = below;
    above_ = above;
    if (below_ && now_ms - below_since_ >= cfg_.on_dwell_ms) dark_ = true;
    if (above_ && now_ms - above_since_ >= cfg_.off_dwell_ms) dark_ = false;
  }

  if (in.wipers_continuous && !wiping_) wipe_since_ = now_ms;
  if (!in.wipers_continuous && wiping_) nowipe_since_ = now_ms;
  wiping_ = in.wipers_continuous;
  if (wiping_ && now_ms - wipe_since_ >= cfg_.rain_on_ms) rain_on_ = true;
  // do not release the rain-light request while it is getting dark anyway,
  // otherwise the lamps would go off and back on within the dwell time
  if (!wiping_ && rain_on_ && now_ms - nowipe_since_ >= cfg_.rain_off_ms && !below_) rain_on_ = false;
}

void LightingSm::step(const Inputs& in, uint32_t now_ms) {
  const bool running = (in.mode == PowerMode::Run || in.mode == PowerMode::Crank);
  const bool was_auto_on = (state_ == State::AutoOn);

  // automatic control is evaluated in RUN/CRANK only; entering it (switch
  // to AUTO or ignition on) takes an immediate decision from the sensor
  const bool auto_active = in.sw_auto && running;
  if (auto_active) update_auto(in, now_ms);

  const bool was_running = (prev_mode_ == PowerMode::Run || prev_mode_ == PowerMode::Crank);
  if (!running && was_running && was_auto_on && in.sw_auto && !in.sw_on) {
    // ignition switched off (via ACC or directly) with AUTO lamps on
    state_ = State::ExitDelay;
    exit_start_ = now_ms;
  } else if (state_ == State::ExitDelay && in.sw_auto && !in.sw_on && !running &&
             now_ms - exit_start_ < cfg_.exit_delay_ms) {
    // keep exit delay
  } else if (in.sw_on && in.mode != PowerMode::Off) {
    state_ = State::ManualOn;
  } else if (auto_active) {
    state_ = (dark_ || rain_on_) ? State::AutoOn : State::AutoOff;
  } else {
    state_ = State::Off;  // parking lamps are not modelled
  }
  auto_prev_ = auto_active;
  if (!auto_active) {
    rain_on_ = false;
    wiping_ = false;
    fail_safe_ = false;
  }
  high_ = in.high_beam_sw && low_beam() && state_ != State::ExitDelay;
  prev_mode_ = in.mode;
}

}  // namespace bcm::app

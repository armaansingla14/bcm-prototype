#include "bcm/app/lighting.hpp"

namespace bcm::app {

void LightingSm::update_auto(const Inputs& in, uint32_t now_ms) {
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
  if (!wiping_ && rain_on_ && now_ms - nowipe_since_ >= cfg_.rain_off_ms) rain_on_ = false;
}

void LightingSm::step(const Inputs& in, uint32_t now_ms) {
  const bool running = (in.mode == PowerMode::Run || in.mode == PowerMode::Crank);
  const bool was_auto_on = (state_ == State::AutoOn);

  // keep auto bookkeeping alive whenever AUTO is selected so dwell timers
  // are meaningful at the moment the ignition is switched on
  if (in.sw_auto) update_auto(in, now_ms);

  if (in.mode == PowerMode::Off) {
    if (prev_mode_ != PowerMode::Off && was_auto_on && in.sw_auto) {
      state_ = State::ExitDelay;
      exit_start_ = now_ms;
    } else if (state_ == State::ExitDelay) {
      if (!in.sw_auto || now_ms - exit_start_ >= cfg_.exit_delay_ms) state_ = State::Off;
    } else {
      state_ = State::Off;  // parking lamps are not modelled
    }
  } else if (in.sw_on) {
    state_ = State::ManualOn;
  } else if (in.sw_auto && running) {
    state_ = (dark_ || rain_on_) ? State::AutoOn : State::AutoOff;
  } else {
    state_ = State::Off;
  }
  if (!in.sw_auto) {
    rain_on_ = false;
    wiping_ = false;
    fail_safe_ = false;
  }
  high_ = in.high_beam_sw && low_beam() && state_ != State::ExitDelay;
  prev_mode_ = in.mode;
}

}  // namespace bcm::app

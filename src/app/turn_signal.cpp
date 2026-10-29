#include "bcm/app/turn_signal.hpp"

namespace bcm::app {

void TurnSignalSm::step(const Inputs& in, uint32_t now_ms) {
  if (in.hazard_pressed_edge) hazard_ = !hazard_;
  if (in.crash_event) hazard_ = true;

  const bool run = (in.mode == PowerMode::Run);
  const bool left = run && in.stalk_left && !in.stalk_right;
  const bool right = run && in.stalk_right && !in.stalk_left;

  // stalk edge handling for comfort flashing
  if ((left && !prev_left_) || (right && !prev_right_)) {
    stalk_press_ms_ = now_ms;
    comfort_side_ = Request::None;
    comfort_remaining_ = 0;
  }
  if (prev_left_ && !left && now_ms - stalk_press_ms_ < cfg_.comfort_tap_ms && run) {
    comfort_side_ = Request::Left;
    comfort_remaining_ = cfg_.comfort_flashes;
  }
  if (prev_right_ && !right && now_ms - stalk_press_ms_ < cfg_.comfort_tap_ms && run) {
    comfort_side_ = Request::Right;
    comfort_remaining_ = cfg_.comfort_flashes;
  }
  if (!run) comfort_remaining_ = 0;
  prev_left_ = left;
  prev_right_ = right;

  if (hazard_) {
    request_ = Request::Hazard;
  } else if (left) {
    request_ = Request::Left;
  } else if (right) {
    request_ = Request::Right;
  } else if (comfort_remaining_ > 0) {
    request_ = comfort_side_;
  } else {
    request_ = Request::None;
  }

  // flasher; a crash pre-empts the normal cadence so the hazard pattern
  // starts immediately on both sides
  if (in.crash_event) {
    lamp_on_ = true;
    side_ = Request::Hazard;
    phase_start_ms_ = now_ms;
    phase_started_ = true;
    ++flashes_;
  } else if (lamp_on_) {
    if (now_ms - phase_start_ms_ >= cfg_.on_ms) {
      lamp_on_ = false;
      phase_start_ms_ = now_ms;
      // flashes completed after the tap was released count towards the
      // comfort sequence (including the flash started by the tap itself)
      if (comfort_side_ != Request::None && side_ == comfort_side_) {
        if (comfort_remaining_ > 0 && !(left || right)) --comfort_remaining_;
      }
    }
  } else if (request_ != Request::None) {
    if (!phase_started_ || now_ms - phase_start_ms_ >= cfg_.off_ms) {
      lamp_on_ = true;
      side_ = request_;
      phase_start_ms_ = now_ms;
      phase_started_ = true;
      ++flashes_;
    }
  }
}

}  // namespace bcm::app

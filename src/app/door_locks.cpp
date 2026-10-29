#include "bcm/app/door_locks.hpp"

namespace bcm::app {

bool DoorLockSm::budget_ok(uint32_t now_ms) {
  while (!history_.empty() && now_ms - history_.front() >= cfg_.window_ms) history_.pop_front();
  thermal_ = history_.size() >= cfg_.max_actuations;
  return !thermal_;
}

void DoorLockSm::start_pulse(bool lock, uint32_t now_ms) {
  state_ = lock ? State::Locking : State::Unlocking;
  pulse_start_ = now_ms;
  history_.push_back(now_ms);
}

void DoorLockSm::step(const Inputs& in, uint32_t now_ms) {
  // finish an active pulse
  if ((state_ == State::Locking || state_ == State::Unlocking) && now_ms - pulse_start_ >= cfg_.pulse_ms) {
    locked_ = (state_ == State::Locking);
    state_ = locked_ ? State::Locked : State::Unlocked;
  }
  const bool idle = (state_ == State::Locked || state_ == State::Unlocked);

  // crash has absolute priority and may pre-empt a lock pulse
  if (in.crash_event) {
    crash_active_ = true;
    crash_until_ = now_ms + cfg_.crash_inhibit_ms;
    repulse_pending_ = true;
    repulse_at_ = now_ms + cfg_.crash_repulse_ms;
    armed_ = false;
    if (state_ != State::Unlocking) start_pulse(false, now_ms);
    return;
  }
  if (crash_active_ && now_ms >= crash_until_) crash_active_ = false;
  if (repulse_pending_ && idle && now_ms >= repulse_at_) {
    repulse_pending_ = false;
    start_pulse(false, now_ms);
    return;
  }

  // auto-lock re-arm conditions
  if (in.mode == PowerMode::Off) armed_ = true;
  if (in.speed_valid && in.speed_kph < cfg_.rearm_kph && in.door_ajar) armed_ = true;

  if (!idle) return;

  const bool want_unlock = in.unlock_sw_edge || in.rke_unlock;
  bool want_lock = in.lock_sw_edge || (in.rke_lock && !in.door_ajar);
  bool autolock = false;
  if (!want_lock && armed_ && in.mode == PowerMode::Run && in.speed_valid && in.speed_kph >= cfg_.autolock_kph &&
      !locked_) {
    want_lock = true;
    autolock = true;
  }
  if (crash_active_) want_lock = false;

  if (want_unlock) {
    if (budget_ok(now_ms)) start_pulse(false, now_ms);
    // a manual unlock while driving must not be undone by auto-lock
    if (in.speed_valid && in.speed_kph >= cfg_.autolock_kph) armed_ = false;
  } else if (want_lock) {
    if (budget_ok(now_ms)) {
      start_pulse(true, now_ms);
      if (autolock) ++autolocks_;
    }
    if (autolock) armed_ = false;
  } else {
    budget_ok(now_ms);  // refresh thermal flag
  }
}

}  // namespace bcm::app

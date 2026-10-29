#include "bcm/app/debounce.hpp"

namespace bcm::app {

bool Debouncer::update(bool raw, uint32_t now_ms) {
  rose_ = fell_ = false;
  if (init_) {
    // first sample after power-up is taken as-is (no spurious edge)
    stable_ = candidate_ = raw;
    since_ms_ = now_ms;
    init_ = false;
    return stable_;
  }
  if (raw != candidate_) {
    candidate_ = raw;
    since_ms_ = now_ms;
  }
  if (candidate_ != stable_ && now_ms - since_ms_ >= debounce_ms_) {
    stable_ = candidate_;
    rose_ = stable_;
    fell_ = !stable_;
  }
  return stable_;
}

void MomentarySwitch::update(bool raw, uint32_t now_ms) {
  deb_.update(raw, now_ms);
  edge_ = false;
  if (deb_.rose()) press_start_ms_ = now_ms;
  if (!deb_.value()) {
    stuck_ = false;
    pressed_ = false;
    return;
  }
  if (!stuck_ && now_ms - press_start_ms_ >= stuck_ms_) stuck_ = true;
  const bool was = pressed_;
  pressed_ = !stuck_;
  edge_ = pressed_ && !was && deb_.rose();
}

}  // namespace bcm::app

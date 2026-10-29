#include "bcm/can/rx_monitor.hpp"

namespace bcm::can {

const char* to_string(RxStatus s) {
  switch (s) {
    case RxStatus::NeverReceived: return "NeverReceived";
    case RxStatus::Valid: return "Valid";
    case RxStatus::Timeout: return "Timeout";
    case RxStatus::Invalid: return "Invalid";
  }
  return "?";
}

RxMonitor::RxMonitor(const MessageDef& def) {
  cfg_.timeout_ms = def.timeout_ms;
  // event-driven messages carry a counter that increments per event, not per
  // period, so jumps are expected; only repeats are rejected there.
  cfg_.max_counter_jump = def.cycle_ms == 0 ? 15 : 2;
}

void RxMonitor::reset() {
  status_ = RxStatus::NeverReceived;
  have_counter_ = false;
  bad_run_ = good_run_ = 0;
}

bool RxMonitor::on_frame(const Frame& f, uint32_t now_ms) {
  bool ok = checksum_ok(f);
  const uint8_t ctr = alive_counter(f);
  if (ok && cfg_.check_counter && have_counter_) {
    const uint8_t delta = static_cast<uint8_t>((ctr - last_counter_) & 0x0FU);
    // delta 0 = repeated/frozen counter, > max = lost frames or out of order
    if (delta == 0 || delta > cfg_.max_counter_jump) ok = false;
  }
  if (checksum_ok(f)) {
    // track the counter even when the jump check fails so we resynchronise
    last_counter_ = ctr;
    have_counter_ = true;
  }
  if (!ok) {
    ++error_frames_;
    good_run_ = 0;
    if (bad_run_ < 255) ++bad_run_;
    if (bad_run_ >= cfg_.invalid_threshold) status_ = RxStatus::Invalid;
    return false;
  }
  ++good_frames_;
  bad_run_ = 0;
  if (good_run_ < 255) ++good_run_;
  last_valid_ms_ = now_ms;
  if (status_ == RxStatus::NeverReceived) {
    status_ = RxStatus::Valid;
  } else if (status_ != RxStatus::Valid && good_run_ >= cfg_.recover_threshold) {
    status_ = RxStatus::Valid;
  }
  // payload is only used once the status is Valid; while recovering from a
  // timeout/invalid phase the first frames are checked but not consumed.
  return status_ == RxStatus::Valid;
}

void RxMonitor::update(uint32_t now_ms) {
  if (cfg_.timeout_ms == 0) return;
  if (status_ == RxStatus::Valid && now_ms - last_valid_ms_ > cfg_.timeout_ms) {
    status_ = RxStatus::Timeout;
    have_counter_ = false;
    good_run_ = 0;
    ++timeouts_;
  }
}

}  // namespace bcm::can

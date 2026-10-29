#include "bcm/can/can_if.hpp"

namespace bcm::can {

void CanIf::update(uint32_t now_ms) {
  const bool hw_bus_off = hal_.can_state() == hal::CanState::BusOff;
  switch (state_) {
    case State::Online:
      if (hw_bus_off) {
        state_ = State::BusOffWait;
        entered_ms_ = now_ms;
        ++bus_off_events_;
      } else if (attempts_in_episode_ != 0 && now_ms - online_since_ms_ >= cfg_.stable_ms) {
        attempts_in_episode_ = 0;
      }
      break;
    case State::BusOffWait: {
      if (!hw_bus_off) {
        state_ = State::Online;
        online_since_ms_ = now_ms;
        break;
      }
      const uint32_t delay =
          attempts_in_episode_ < cfg_.fast_attempts ? cfg_.fast_delay_ms : cfg_.slow_delay_ms;
      if (now_ms - entered_ms_ >= delay) {
        hal_.can_request_recovery();
        ++recovery_attempts_;
        if (attempts_in_episode_ < 255) ++attempts_in_episode_;
        entered_ms_ = now_ms;
      }
      break;
    }
  }
}

bool CanIf::send(const Frame& f) {
  if (state_ != State::Online || !hal_.can_transmit(f)) {
    ++tx_dropped_;
    return false;
  }
  ++tx_ok_;
  return true;
}

}  // namespace bcm::can

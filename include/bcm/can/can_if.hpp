#pragma once
#include <cstdint>

#include "bcm/hal/hal.hpp"

namespace bcm::can {

/// CAN interface layer: transmit wrapper plus software bus-off handling.
///
/// Recovery strategy (common OEM pattern): after bus-off, wait
/// fast_delay_ms and re-enable the controller, up to fast_attempts times;
/// further attempts use slow_delay_ms. The attempt counter is reset once the
/// bus has been error-free (not bus-off) for stable_ms.
class CanIf {
 public:
  struct Config {
    uint32_t fast_delay_ms = 50;
    uint32_t slow_delay_ms = 1000;
    uint8_t fast_attempts = 5;
    uint32_t stable_ms = 1000;
  };
  enum class State : uint8_t { Online, BusOffWait };

  explicit CanIf(hal::IHal& hal) : hal_(hal) {}
  CanIf(hal::IHal& hal, const Config& cfg) : hal_(hal), cfg_(cfg) {}

  /// Run once per tick before the application.
  void update(uint32_t now_ms);
  /// Transmit; frames are dropped while bus-off (periodic senders re-send).
  bool send(const Frame& f);
  bool receive(Frame& f) { return state_ == State::Online && hal_.can_receive(f); }

  State state() const { return state_; }
  bool bus_off_active() const { return state_ == State::BusOffWait; }
  uint32_t bus_off_events() const { return bus_off_events_; }
  uint32_t recovery_attempts() const { return recovery_attempts_; }
  uint32_t tx_dropped() const { return tx_dropped_; }
  uint32_t tx_ok() const { return tx_ok_; }

 private:
  hal::IHal& hal_;
  Config cfg_{};
  State state_ = State::Online;
  uint32_t entered_ms_ = 0;
  uint32_t online_since_ms_ = 0;
  uint8_t attempts_in_episode_ = 0;
  uint32_t bus_off_events_ = 0;
  uint32_t recovery_attempts_ = 0;
  uint32_t tx_dropped_ = 0;
  uint32_t tx_ok_ = 0;
};

}  // namespace bcm::can

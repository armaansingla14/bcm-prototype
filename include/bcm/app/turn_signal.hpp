#pragma once
#include <cstdint>

#include "bcm/app/power_mode.hpp"

namespace bcm::app {

/// Turn indicator / hazard state machine with an integrated flasher.
///
/// * hazard is a latched toggle (momentary switch) and works in every power
///   mode; a validated crash event forces hazard on
/// * turn indicators work in RUN only; a stalk tap shorter than
///   comfort_tap_ms gives comfort_flashes flashes (lane change)
/// * the flasher never produces an on-pulse shorter than on_ms or an
///   off-gap shorter than off_ms: a started flash is always completed and a
///   new activation waits for the off time to elapse (relay-flasher
///   behaviour, avoids lamp flicker from switch chatter)
class TurnSignalSm {
 public:
  enum class Request : uint8_t { None, Left, Right, Hazard };
  struct Config {
    uint32_t on_ms = 333;
    uint32_t off_ms = 333;
    uint32_t comfort_tap_ms = 500;
    uint8_t comfort_flashes = 3;
  };
  struct Inputs {
    bool stalk_left = false;
    bool stalk_right = false;
    bool hazard_pressed_edge = false;
    bool crash_event = false;
    PowerMode mode = PowerMode::Off;
  };

  TurnSignalSm() = default;
  explicit TurnSignalSm(const Config& c) : cfg_(c) {}

  void step(const Inputs& in, uint32_t now_ms);
  bool left_lamp() const { return lamp_on_ && (side_ == Request::Left || side_ == Request::Hazard); }
  bool right_lamp() const { return lamp_on_ && (side_ == Request::Right || side_ == Request::Hazard); }
  Request request() const { return request_; }
  bool hazard_active() const { return hazard_; }
  uint32_t flash_count() const { return flashes_; }

 private:
  Config cfg_{};
  bool hazard_ = false;
  Request request_ = Request::None;
  Request side_ = Request::None;
  bool lamp_on_ = false;
  uint32_t phase_start_ms_ = 0;
  bool phase_started_ = false;
  // comfort (lane change) handling
  bool prev_left_ = false;
  bool prev_right_ = false;
  uint32_t stalk_press_ms_ = 0;
  Request comfort_side_ = Request::None;
  int comfort_remaining_ = 0;
  uint32_t flashes_ = 0;
};

}  // namespace bcm::app

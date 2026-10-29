#pragma once
#include <cstdint>

namespace bcm::app {

/// Time-based debouncer: the output follows the raw input only after the
/// raw input has been stable for debounce_ms.
class Debouncer {
 public:
  explicit Debouncer(uint32_t debounce_ms = 30) : debounce_ms_(debounce_ms) {}
  bool update(bool raw, uint32_t now_ms);
  bool value() const { return stable_; }
  bool rose() const { return rose_; }
  bool fell() const { return fell_; }
  void reset(bool v) {
    stable_ = candidate_ = v;
    rose_ = fell_ = false;
    init_ = false;
  }

 private:
  uint32_t debounce_ms_;
  bool stable_ = false;
  bool candidate_ = false;
  bool init_ = true;
  bool rose_ = false;
  bool fell_ = false;
  uint32_t since_ms_ = 0;
};

/// Momentary switch supervision: a press held longer than stuck_ms is
/// considered a stuck contact; the switch is then ignored (reads released)
/// until the contact actually opens again.
class MomentarySwitch {
 public:
  MomentarySwitch(uint32_t debounce_ms, uint32_t stuck_ms) : deb_(debounce_ms), stuck_ms_(stuck_ms) {}
  void update(bool raw, uint32_t now_ms);
  bool pressed() const { return pressed_; }   ///< debounced and not stuck
  bool pressed_edge() const { return edge_; }  ///< rising edge this tick
  bool stuck() const { return stuck_; }

 private:
  Debouncer deb_;
  uint32_t stuck_ms_;
  uint32_t press_start_ms_ = 0;
  bool pressed_ = false;
  bool edge_ = false;
  bool stuck_ = false;
};

}  // namespace bcm::app

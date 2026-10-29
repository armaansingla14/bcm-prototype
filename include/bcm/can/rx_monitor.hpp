#pragma once
#include <cstdint>

#include "bcm/can/frame.hpp"
#include "bcm/can/signal.hpp"

namespace bcm::can {

enum class RxStatus : uint8_t {
  NeverReceived,  ///< nothing valid since startup / reset
  Valid,          ///< fresh, E2E-checked data available
  Timeout,        ///< no valid frame within timeout_ms
  Invalid,        ///< frames arrive but repeatedly fail checksum/counter
};

const char* to_string(RxStatus s);

/// Per-message reception supervision: timeout, checksum and alive-counter
/// checks with an error-counter filter (similar to an AUTOSAR E2E state
/// machine, simplified).
class RxMonitor {
 public:
  struct Config {
    uint32_t timeout_ms = 100;
    uint8_t max_counter_jump = 2;   ///< allowed counter delta (1 = strict)
    uint8_t invalid_threshold = 3;  ///< consecutive bad frames -> Invalid
    uint8_t recover_threshold = 2;  ///< consecutive good frames -> Valid
    bool check_counter = true;
  };

  RxMonitor() = default;
  explicit RxMonitor(const Config& cfg) : cfg_(cfg) {}
  explicit RxMonitor(const MessageDef& def);

  /// Feed a received frame. Returns true if the payload may be used.
  bool on_frame(const Frame& f, uint32_t now_ms);
  /// Advance time (timeout detection). Call once per scheduler tick.
  void update(uint32_t now_ms);
  void reset();

  RxStatus status() const { return status_; }
  bool usable() const { return status_ == RxStatus::Valid; }
  uint32_t last_valid_ms() const { return last_valid_ms_; }
  uint32_t error_frames() const { return error_frames_; }
  uint32_t good_frames() const { return good_frames_; }
  uint32_t timeouts() const { return timeouts_; }

 private:
  Config cfg_{};
  RxStatus status_ = RxStatus::NeverReceived;
  uint32_t last_valid_ms_ = 0;
  bool have_counter_ = false;
  uint8_t last_counter_ = 0;
  uint8_t bad_run_ = 0;
  uint8_t good_run_ = 0;
  uint32_t error_frames_ = 0;
  uint32_t good_frames_ = 0;
  uint32_t timeouts_ = 0;
};

}  // namespace bcm::can

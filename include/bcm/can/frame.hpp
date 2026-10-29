#pragma once
#include <array>
#include <cstdint>

namespace bcm::can {

/// Classic CAN 2.0A/B frame as seen by the application (bxCAN mailbox view).
struct Frame {
  uint32_t id = 0;
  bool extended = false;
  uint8_t dlc = 8;
  std::array<uint8_t, 8> data{};
};

}  // namespace bcm::can

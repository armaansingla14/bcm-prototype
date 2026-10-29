#pragma once
#include <cstdint>

#include "bcm/hal/hal.hpp"

namespace bcm::app {

/// Common interface for the production controller and the naive baseline so
/// the scenario runner can drive either through the same HAL.
class Controller {
 public:
  virtual ~Controller() = default;
  /// One scheduler tick (called every kTickMs by the main loop).
  virtual void step() = 0;
  virtual const char* name() const = 0;
  virtual uint16_t fault_flags() const { return 0; }
};

constexpr uint32_t kTickMs = 10;

}  // namespace bcm::app

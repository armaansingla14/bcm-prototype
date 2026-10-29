#pragma once
#include <cstdint>

#include "bcm/app/controller.hpp"
#include "bcm/hal/hal.hpp"

namespace bcm::app {

/// Baseline controller used for comparison in the scenario runner.
///
/// It implements the same features with straightforward if/else logic as a
/// first prototype typically would: raw (undebounced) inputs, last received
/// CAN values used without E2E or timeout checks, a free-running flasher
/// restarted on every request change, time-based intermittent wiping without
/// park-switch handling, no stall/crank supervision, auto-lock whenever the
/// speed is above the threshold, and immediate bus-off recovery.
class NaiveBcm : public Controller {
 public:
  explicit NaiveBcm(hal::IHal& hal) : hal_(hal) {}
  void step() override;
  const char* name() const override { return "naive"; }

 private:
  hal::IHal& hal_;
  // CAN data, last value wins
  double speed_ = 0.0;
  bool crash_ = false;
  double rain_ = 0.0;
  double lux_ = 50000.0;
  // switches
  bool prev_hazard_ = false;
  bool prev_lock_ = false;
  bool prev_unlock_ = false;
  bool hazard_ = false;
  // flasher
  int req_ = 0;
  uint32_t flash_start_ = 0;
  // locks
  bool locked_ = false;
  int pulse_ = 0;  // +1 lock, -1 unlock
  uint32_t pulse_start_ = 0;
  // wipers
  uint32_t int_start_ = 0;
  // status
  uint32_t last_status_ = 0;
  uint8_t counter_ = 0;
};

}  // namespace bcm::app

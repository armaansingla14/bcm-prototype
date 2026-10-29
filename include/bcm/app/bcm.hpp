#pragma once
#include <cstdint>

#include "bcm/app/controller.hpp"
#include "bcm/app/debounce.hpp"
#include "bcm/app/door_locks.hpp"
#include "bcm/app/dtc.hpp"
#include "bcm/app/lighting.hpp"
#include "bcm/app/power_mode.hpp"
#include "bcm/app/turn_signal.hpp"
#include "bcm/app/wipers.hpp"
#include "bcm/can/can_if.hpp"
#include "bcm/can/rx_monitor.hpp"
#include "bcm/hal/hal.hpp"

namespace bcm::app {

/// Decoded, supervised view of the CAN inputs.
struct CanInputs {
  double speed_kph = 0.0;
  bool speed_valid = false;
  bool engine_running = false;
  bool engine_valid = false;
  bool crash_event = false;  ///< confirmed rising edge this tick
  double rain_pct = 0.0;
  double ambient_lux = 0.0;
  bool rls_valid = false;
  bool rke_lock = false;
  bool rke_unlock = false;
};

/// Body control module application: input conditioning, CAN reception with
/// E2E/timeout supervision, the five feature state machines, output
/// driving, DTC handling and the BCM_Status broadcast.
class Bcm : public Controller {
 public:
  struct Config {
    uint32_t debounce_ms = 30;
    uint32_t ign_debounce_ms = 50;
    uint32_t momentary_stuck_ms = 10000;
    uint32_t status_period_ms = 100;
    uint8_t crash_confirm_frames = 2;
    double max_speed_step_kph = 5.0;  ///< per ESC frame (20 ms)
    uint8_t speed_resync_frames = 3;
  };

  explicit Bcm(hal::IHal& hal);
  Bcm(hal::IHal& hal, const Config& cfg);

  void step() override;
  const char* name() const override { return "state-machine"; }
  uint16_t fault_flags() const override { return dtc_.active_mask(); }

  const PowerModeSm& power() const { return power_; }
  const TurnSignalSm& turn() const { return turn_; }
  const LightingSm& lighting() const { return lighting_; }
  const DoorLockSm& locks() const { return locks_; }
  const WiperSm& wipers() const { return wipers_; }
  const DtcStore& dtc() const { return dtc_; }
  DtcStore& dtc() { return dtc_; }
  const can::CanIf& canif() const { return canif_; }
  const CanInputs& can_inputs() const { return can_; }
  const can::RxMonitor& esc_monitor() const { return esc_; }
  uint32_t speed_rejects() const { return speed_rejects_; }
  const can::RxMonitor& ecm_monitor() const { return ecm_; }
  const can::RxMonitor& acu_monitor() const { return acu_; }
  const can::RxMonitor& rls_monitor() const { return rls_; }

 private:
  void read_inputs(uint32_t now);
  void process_can(uint32_t now);
  void update_dtcs();
  void send_status(uint32_t now);

  hal::IHal& hal_;
  Config cfg_;
  can::CanIf canif_;
  can::RxMonitor esc_, ecm_, acu_, rls_, rke_;

  Debouncer ign_acc_, ign_run_, ign_start_;
  Debouncer stalk_l_, stalk_r_, hl_auto_, hl_on_, hb_sw_;
  Debouncer wip_auto_, wip_low_, wip_high_, door_ajar_;
  MomentarySwitch hazard_sw_, lock_sw_, unlock_sw_, washer_sw_;

  PowerModeSm power_;
  TurnSignalSm turn_;
  LightingSm lighting_;
  DoorLockSm locks_;
  WiperSm wipers_;
  DtcStore dtc_;

  CanInputs can_;
  uint8_t crash_frames_ = 0;
  bool speed_init_ = false;
  uint8_t speed_jump_run_ = 0;
  double speed_jump_value_ = 0.0;
  uint32_t speed_rejects_ = 0;
  bool crash_latched_ = false;
  uint32_t last_status_ms_ = 0;
  bool status_sent_once_ = false;
  uint8_t tx_counter_ = 0;
};

}  // namespace bcm::app

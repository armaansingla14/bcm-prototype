#pragma once
#include <array>
#include <cstdint>
#include <random>
#include <vector>

#include "bcm/can/frame.hpp"
#include "bcm/hal/sim_hal.hpp"

namespace bcm::sim {

enum class KeyPos : uint8_t { Off, Acc, Run, Start };

/// Per-message fault injection settings on the simulated bus.
struct MsgFault {
  bool drop = false;           ///< sender silent (timeout)
  double corrupt_prob = 0.0;   ///< probability a frame gets bit errors (checksum not updated)
  bool freeze_counter = false; ///< sender's alive counter stuck
};

/// Plant and environment model around the BCM: remote ECUs sending their
/// periodic CAN frames, the driver's switches, the lock actuators, the wiper
/// motor with its park cam switch and the engine/starter. All physical
/// truth used by the requirement monitor lives here.
class VehicleSim {
 public:
  VehicleSim(hal::SimHal& hal, uint64_t seed);

  // --- driver controls -----------------------------------------------------
  void set_key(KeyPos k);
  KeyPos key() const { return key_; }
  void set_switch(hal::Din pin, bool v);  ///< intended (clean) switch level
  bool switch_level(hal::Din pin) const { return intended_[static_cast<int>(pin)]; }
  void rke_lock();
  void rke_unlock();

  // --- environment ---------------------------------------------------------
  double speed_kph = 0.0;
  double ambient_lux = 20000.0;
  double rain_pct = 0.0;
  bool crash = false;           ///< ACU reports crash (latched by the ACU)
  bool engine_can_start = true;
  bool rls_sensor_fault = false;

  // --- faults --------------------------------------------------------------
  MsgFault& fault(uint32_t can_id);
  void set_bus_fault(bool on) { hal_.set_bus_fault(on); }
  void set_stuck(hal::Din pin, bool stuck, bool level = true);
  bool stuck(hal::Din pin) const { return stuck_[static_cast<int>(pin)]; }
  bool wiper_stalled = false;
  double glitch_prob = 0.0;     ///< per tick per switch input
  uint32_t bounce_ms = 0;       ///< contact bounce after each intended change

  // --- simulation steps (call around controller.step()) ---------------------
  void pre_step(uint32_t now_ms);   ///< bus traffic and input pins
  void post_step(uint32_t now_ms);  ///< plant reacts to outputs

  // --- plant truth -----------------------------------------------------------
  bool doors_locked() const { return locked_; }
  bool engine_running() const { return engine_running_; }
  bool wiper_parked() const { return wiper_phase_ < kParkZone; }
  double wiper_phase() const { return wiper_phase_; }
  bool rls_available() const;  ///< whether usable RLS data can reach the BCM
  uint32_t lock_changes() const { return lock_changes_; }

  // --- event bookkeeping for the requirement monitor ----------------------------
  int32_t last_lock_request_ms = -1000000;    ///< lock switch press / RKE lock
  int32_t last_unlock_request_ms = -1000000;  ///< unlock switch press / RKE unlock
  int32_t crash_frames_delivered = 0;         ///< valid crash=1 frames delivered
  int32_t crash_confirmed_ms = -1;            ///< time of 2nd valid crash frame
  int32_t last_wiper_off_ms = -1;             ///< intended wiper switch -> off
  uint32_t frames_sent = 0;
  uint32_t frames_corrupted = 0;

  static constexpr double kParkZone = 0.08;

 private:
  void send_periodic(uint32_t now_ms);
  void transmit(can::Frame f, uint32_t now_ms);
  void apply_inputs(uint32_t now_ms);

  hal::SimHal& hal_;
  std::mt19937_64 rng_;
  KeyPos key_ = KeyPos::Off;
  std::array<bool, hal::kNumDin> intended_{};
  std::array<bool, hal::kNumDin> stuck_{};
  std::array<uint32_t, hal::kNumDin> changed_ms_{};
  std::vector<std::pair<uint32_t, MsgFault>> faults_;
  std::vector<std::pair<uint32_t, uint8_t>> counters_;
  uint32_t last_now_ = 0;
  bool pending_rke_lock_ = false;
  bool pending_rke_unlock_ = false;
  // plant state
  bool locked_ = false;
  uint32_t lock_on_ms_ = 0;
  uint32_t unlock_on_ms_ = 0;
  uint32_t lock_changes_ = 0;
  double wiper_phase_ = 0.02;
  bool engine_running_ = false;
  uint32_t crank_ms_ = 0;
};

}  // namespace bcm::sim

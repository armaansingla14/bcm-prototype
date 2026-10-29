#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <vector>

#include "bcm/hal/sim_hal.hpp"
#include "bcm/sim/vehicle_sim.hpp"

namespace bcm::sim {

/// Black-box requirements checked against plant truth and the BCM's pin and
/// bus outputs. The monitor knows nothing about controller internals.
enum class Req : uint8_t {
  CrashResponse,     ///< R1 doors unlocked <=300 ms after crash and kept unlocked 10 s; hazards on <=500 ms
  UncommandedLock,   ///< R2 lock state changes only with a request (switch/RKE), speed >15 km/h or crash
  StarterProtect,    ///< R3 starter <=10 s continuous and never >300 ms with the engine running
  FlasherTiming,     ///< R4 indicator on-pulses 250..450 ms, off-gaps >=250 ms
  AutoLampsOn,       ///< R5 AUTO: low beam on within 3 s when dark or light sensor unavailable
  LampFlicker,       ///< R6 AUTO: no two low-beam transitions within 2 s
  WiperStall,        ///< R7 wiper motor not energised >3.5 s without park-switch movement
  WiperPark,         ///< R8 wiper parked and motor off 2.5 s after switching off
  LockActuator,      ///< R9 lock motor <=500 ms continuous, <=10 activations per 30 s
  BusOffRecovery,    ///< R10 <=6 recovery attempts in any 1 s window
  StatusPeriod,      ///< R11 BCM_Status gap <=150 ms while the bus is healthy
  Count
};
constexpr int kNumReq = static_cast<int>(Req::Count);
const char* to_string(Req r);
const char* describe(Req r);

class RequirementMonitor {
 public:
  void observe(uint32_t t, const VehicleSim& sim, const hal::SimHal& hal);

  const std::array<int, kNumReq>& violations() const { return v_; }
  int total() const;
  /// crash: time from confirmed crash frame to doors physically unlocked
  /// (-1 if the doors were not locked at the crash)
  int32_t crash_unlock_latency_ms() const { return crash_latency_; }
  /// bus-off: time from bus fault removal to the next BCM_Status on the bus
  const std::vector<uint32_t>& busoff_recovery_ms() const { return busoff_rec_; }

 private:
  void flag(Req r) { ++v_[static_cast<int>(r)]; }

  std::array<int, kNumReq> v_{};
  // R1
  bool r1_done_ = false;
  bool r1_hazard_seen_ = false;
  bool r1_flagged_ = false;
  bool r1_locked_at_crash_ = false;
  int32_t crash_latency_ = -1;
  // R2
  bool prev_locked_ = false;
  bool first_ = true;
  int32_t last_fast_ms_ = -1000000;
  // R3
  uint32_t starter_since_ = 0;
  bool starter_prev_ = false;
  bool r3_long_flag_ = false;
  uint32_t starter_running_ms_ = 0;
  bool r3_run_flag_ = false;
  // R4
  struct Lamp {
    bool prev = false;
    int64_t rise = -1;
    int64_t fall = -1;
  };
  Lamp lamps_[2];
  // R5 / R6
  int64_t r5_since_ = -1;
  bool r5_flag_ = false;
  bool r6_active_ = false;
  bool r6_prev_beam_ = false;
  int64_t r6_last_change_ = -1;
  // R7
  bool r7_prev_park_ = true;
  int64_t r7_last_edge_ = 0;
  bool r7_motor_prev_ = false;
  bool r7_flag_ = false;
  // R8
  int32_t r8_checked_for_ = -1;
  // R9
  uint32_t lockmot_on_ms_ = 0;
  bool r9_cont_flag_ = false;
  bool r9_prev_lm_ = false, r9_prev_um_ = false;
  std::deque<uint32_t> act_times_;
  bool r9_rate_flag_ = false;
  // R10
  uint32_t prev_recov_ = 0;
  std::deque<uint32_t> recov_times_;
  bool r10_flag_ = false;
  // R11
  size_t tx_seen_ = 0;
  int64_t last_status_ = -1;
  int64_t healthy_since_ = 0;
  bool was_healthy_ = true;
  bool r11_flag_ = false;
  bool bus_fault_prev_ = false;
  int64_t fault_cleared_ = -1;
  std::vector<uint32_t> busoff_rec_;
};

}  // namespace bcm::sim

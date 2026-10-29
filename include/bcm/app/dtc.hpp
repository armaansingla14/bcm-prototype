#pragma once
#include <cstdint>

namespace bcm::app {

/// Diagnostic trouble codes, reported as a bit field in BCM_Status.FaultFlags.
enum class Dtc : uint8_t {
  EscTimeout = 0,
  EcmTimeout,
  AcuTimeout,
  RlsTimeout,
  E2eError,  ///< a supervised message is in Invalid state
  CanBusOff,
  HazardSwStuck,
  LockSwStuck,
  UnlockSwStuck,
  WasherSwStuck,
  WiperStall,
  CrankTimeout,
  LockThermal,
  RainSensorFault,
  IgnitionImplausible,
  Count
};

class DtcStore {
 public:
  void set(Dtc d, bool active) {
    const uint32_t bit = 1U << static_cast<unsigned>(d);
    if (active) {
      if (!(active_ & bit)) occurrences_[static_cast<unsigned>(d)]++;
      active_ |= bit;
      latched_ |= bit;
    } else {
      active_ &= ~bit;
    }
  }
  bool active(Dtc d) const { return active_ & (1U << static_cast<unsigned>(d)); }
  bool latched(Dtc d) const { return latched_ & (1U << static_cast<unsigned>(d)); }
  uint16_t active_mask() const { return static_cast<uint16_t>(active_ & 0xFFFFU); }
  uint32_t latched_mask() const { return latched_; }
  uint32_t occurrences(Dtc d) const { return occurrences_[static_cast<unsigned>(d)]; }
  void clear() {
    active_ = latched_ = 0;
    for (auto& o : occurrences_) o = 0;
  }

 private:
  uint32_t active_ = 0;
  uint32_t latched_ = 0;
  uint32_t occurrences_[static_cast<unsigned>(Dtc::Count)] = {};
};

}  // namespace bcm::app

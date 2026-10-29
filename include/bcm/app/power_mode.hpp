#pragma once
#include <cstdint>

namespace bcm::app {

enum class PowerMode : uint8_t { Off = 0, Acc = 1, Run = 2, Crank = 3 };
const char* to_string(PowerMode m);

/// Ignition / power-mode state machine.
///
///   OFF <-> ACC <-> RUN -> CRANK -> RUN
///
/// * key position decoded from three ignition contacts (ACC, RUN, START)
/// * CRANK only entered from RUN, and only if the engine is not reported as
///   running; START without RUN is implausible and ignored
/// * starter is cut when the engine reports running, when the key leaves
///   START, or after max_crank_ms (then inhibited until START is released)
/// * ACC loads are shed during CRANK
class PowerModeSm {
 public:
  struct Config {
    uint32_t max_crank_ms = 10000;
  };
  struct Inputs {
    bool ign_acc = false;
    bool ign_run = false;
    bool ign_start = false;
    bool engine_running = false;        ///< from ECM_EngineStatus
    bool engine_status_valid = false;   ///< ECM message fresh and E2E ok
  };
  struct Outputs {
    bool acc_relay = false;
    bool run_relay = false;
    bool starter = false;
  };

  PowerModeSm() = default;
  explicit PowerModeSm(const Config& c) : cfg_(c) {}

  void step(const Inputs& in, uint32_t now_ms);
  PowerMode mode() const { return mode_; }
  Outputs outputs() const;
  bool crank_timeout() const { return crank_timeout_; }
  bool implausible() const { return implausible_; }
  bool entered_off() const { return entered_off_; }  ///< transition this tick

 private:
  Config cfg_{};
  PowerMode mode_ = PowerMode::Off;
  uint32_t crank_start_ms_ = 0;
  bool start_inhibit_ = false;
  bool crank_timeout_ = false;
  bool implausible_ = false;
  bool entered_off_ = false;
};

}  // namespace bcm::app

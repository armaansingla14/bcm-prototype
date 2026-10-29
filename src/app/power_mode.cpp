#include "bcm/app/power_mode.hpp"

namespace bcm::app {

const char* to_string(PowerMode m) {
  switch (m) {
    case PowerMode::Off: return "OFF";
    case PowerMode::Acc: return "ACC";
    case PowerMode::Run: return "RUN";
    case PowerMode::Crank: return "CRANK";
  }
  return "?";
}

void PowerModeSm::step(const Inputs& in, uint32_t now_ms) {
  implausible_ = in.ign_start && !in.ign_run;
  const bool key_start = in.ign_start && in.ign_run;
  PowerMode key = PowerMode::Off;
  if (key_start) {
    key = PowerMode::Crank;
  } else if (in.ign_run) {
    key = PowerMode::Run;
  } else if (in.ign_acc) {
    key = PowerMode::Acc;
  }
  if (!key_start) start_inhibit_ = false;  // releasing START re-arms cranking

  const PowerMode prev = mode_;
  switch (mode_) {
    case PowerMode::Off:
    case PowerMode::Acc:
      // a key that jumps straight to START (fast turn) passes through RUN
      mode_ = (key == PowerMode::Crank) ? PowerMode::Run : key;
      break;
    case PowerMode::Run:
      if (key == PowerMode::Crank) {
        const bool engine_on = in.engine_status_valid && in.engine_running;
        if (!start_inhibit_ && !engine_on) {
          mode_ = PowerMode::Crank;
          crank_start_ms_ = now_ms;
        } else {
          start_inhibit_ = true;
        }
      } else {
        mode_ = key;
      }
      break;
    case PowerMode::Crank:
      if (key != PowerMode::Crank) {
        mode_ = key;
      } else if (in.engine_status_valid && in.engine_running) {
        mode_ = PowerMode::Run;
        start_inhibit_ = true;
      } else if (now_ms - crank_start_ms_ >= cfg_.max_crank_ms) {
        mode_ = PowerMode::Run;
        start_inhibit_ = true;
        crank_timeout_ = true;
      }
      break;
  }
  if (mode_ == PowerMode::Crank && prev != PowerMode::Crank) crank_timeout_ = false;
  entered_off_ = (mode_ == PowerMode::Off && prev != PowerMode::Off);
}

PowerModeSm::Outputs PowerModeSm::outputs() const {
  Outputs o;
  o.acc_relay = (mode_ == PowerMode::Acc || mode_ == PowerMode::Run);
  o.run_relay = (mode_ == PowerMode::Run || mode_ == PowerMode::Crank);
  o.starter = (mode_ == PowerMode::Crank);
  return o;
}

}  // namespace bcm::app

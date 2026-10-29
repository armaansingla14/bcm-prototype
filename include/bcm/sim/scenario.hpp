#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "bcm/sim/requirements.hpp"
#include "bcm/sim/vehicle_sim.hpp"

namespace bcm::sim {

/// Piecewise-linear profile over time.
struct Profile {
  std::vector<std::pair<uint32_t, double>> pts;
  double at(uint32_t t) const;
};

struct Scenario {
  std::string name;
  std::string fault_class;  ///< "nominal" or the injected fault family
  std::string description;
  uint64_t seed = 0;
  uint32_t duration_ms = 120000;
  bool has_crash = false;
  Profile speed, lux, rain;
  std::vector<std::pair<uint32_t, std::function<void(VehicleSim&)>>> actions;

  void at(uint32_t t, std::function<void(VehicleSim&)> fn) { actions.emplace_back(t, std::move(fn)); }
};

const std::vector<std::string>& fault_classes();

/// Randomised drive cycle (key on, crank, drive, stop, exit, lock) with
/// driver inputs and environment, plus faults of the given class.
Scenario make_scenario(const std::string& fault_class, uint64_t seed, int index);
/// Stratified set: n scenarios cycling through all fault classes.
std::vector<Scenario> make_scenario_set(int n, uint64_t seed);

enum class ControllerKind { StateMachine, Naive };

struct RunResult {
  std::array<int, kNumReq> violations{};
  int total = 0;
  int32_t crash_unlock_latency_ms = -1;
  std::vector<uint32_t> busoff_recovery_ms;
  uint16_t fault_flags_seen = 0;  ///< OR of DTC bits reported (state machine only)
};

/// Run one scenario with a fresh controller. If trace_csv is non-empty, a
/// per-tick trace of key signals is written there.
RunResult run_scenario(const Scenario& sc, ControllerKind kind, const std::string& trace_csv = "");

}  // namespace bcm::sim

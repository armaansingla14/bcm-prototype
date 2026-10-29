#include "bcm/app/power_mode.hpp"
#include "testfw.hpp"

using namespace bcm::app;

namespace {
PowerModeSm::Inputs key(int pos, bool engine = false, bool valid = true) {
  PowerModeSm::Inputs in;
  in.ign_acc = pos == 1 || pos == 2;
  in.ign_run = pos >= 2;
  in.ign_start = pos == 3;
  in.engine_running = engine;
  in.engine_status_valid = valid;
  return in;
}
}  // namespace

TEST(PowerMode, StartsOff) {
  PowerModeSm sm;
  CHECK(sm.mode() == PowerMode::Off);
  const auto o = sm.outputs();
  CHECK(!o.acc_relay && !o.run_relay && !o.starter);
}

TEST(PowerMode, KeySequence) {
  PowerModeSm sm;
  sm.step(key(1), 0);
  CHECK(sm.mode() == PowerMode::Acc);
  sm.step(key(2), 10);
  CHECK(sm.mode() == PowerMode::Run);
  sm.step(key(3), 20);
  CHECK(sm.mode() == PowerMode::Crank);
  sm.step(key(2), 30);
  CHECK(sm.mode() == PowerMode::Run);
  sm.step(key(0), 40);
  CHECK(sm.mode() == PowerMode::Off);
  CHECK(sm.entered_off());
}

TEST(PowerMode, RelayOutputsPerMode) {
  PowerModeSm sm;
  sm.step(key(1), 0);
  CHECK(sm.outputs().acc_relay && !sm.outputs().run_relay);
  sm.step(key(2), 10);
  CHECK(sm.outputs().acc_relay && sm.outputs().run_relay && !sm.outputs().starter);
  sm.step(key(3), 20);
  // ACC loads shed while cranking
  CHECK(!sm.outputs().acc_relay && sm.outputs().run_relay && sm.outputs().starter);
}

TEST(PowerMode, DirectToStartPassesThroughRun) {
  PowerModeSm sm;
  sm.step(key(3), 0);
  CHECK(sm.mode() == PowerMode::Run);
  sm.step(key(3), 10);
  CHECK(sm.mode() == PowerMode::Crank);
}

TEST(PowerMode, StarterCutWhenEngineRuns) {
  PowerModeSm sm;
  sm.step(key(2), 0);
  sm.step(key(3), 10);
  REQUIRE(sm.mode() == PowerMode::Crank);
  sm.step(key(3, true), 800);
  CHECK(sm.mode() == PowerMode::Run);
  CHECK(!sm.outputs().starter);
  sm.step(key(3, true), 900);  // key still held
  CHECK(sm.mode() == PowerMode::Run);
}

TEST(PowerMode, NoCrankWhenEngineAlreadyRunning) {
  PowerModeSm sm;
  sm.step(key(2, true), 0);
  sm.step(key(3, true), 10);
  CHECK(sm.mode() == PowerMode::Run);
}

TEST(PowerMode, CrankAllowedWhenEngineStatusUnknown) {
  PowerModeSm sm;
  sm.step(key(2, true, false), 0);
  sm.step(key(3, true, false), 10);
  CHECK(sm.mode() == PowerMode::Crank);
}

TEST(PowerMode, CrankTimeout) {
  PowerModeSm sm;
  sm.step(key(2), 0);
  sm.step(key(3), 10);
  sm.step(key(3), 10009);
  CHECK(sm.mode() == PowerMode::Crank);
  sm.step(key(3), 10010);
  CHECK(sm.mode() == PowerMode::Run);
  CHECK(sm.crank_timeout());
  sm.step(key(3), 15000);  // still held: inhibited
  CHECK(sm.mode() == PowerMode::Run);
}

TEST(PowerMode, ReleaseRearmsCranking) {
  PowerModeSm sm;
  sm.step(key(2), 0);
  sm.step(key(3), 10);
  sm.step(key(3), 10010);
  sm.step(key(2), 10100);
  sm.step(key(3), 10200);
  CHECK(sm.mode() == PowerMode::Crank);
  CHECK(!sm.crank_timeout());
}

TEST(PowerMode, ImplausibleStartWithoutRun) {
  PowerModeSm sm;
  PowerModeSm::Inputs in;
  in.ign_acc = true;
  in.ign_start = true;
  sm.step(in, 0);
  CHECK(sm.implausible());
  CHECK(sm.mode() == PowerMode::Acc);
  CHECK(!sm.outputs().starter);
}

TEST(PowerMode, KeyOffDuringCrank) {
  PowerModeSm sm;
  sm.step(key(2), 0);
  sm.step(key(3), 10);
  sm.step(key(0), 20);
  CHECK(sm.mode() == PowerMode::Off);
}

TEST(PowerMode, ToStringNames) {
  CHECK_EQ(std::string(to_string(PowerMode::Crank)), std::string("CRANK"));
  CHECK_EQ(std::string(to_string(PowerMode::Acc)), std::string("ACC"));
}

TEST_MAIN()

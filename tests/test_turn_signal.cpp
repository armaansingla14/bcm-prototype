#include <vector>

#include "bcm/app/turn_signal.hpp"
#include "testfw.hpp"

using namespace bcm::app;
using Req = TurnSignalSm::Request;

namespace {
struct Pulses {
  std::vector<uint32_t> on, off;
};

// run the SM and collect left-lamp on/off durations
Pulses run(TurnSignalSm& sm, TurnSignalSm::Inputs in, uint32_t& t, uint32_t ms, bool right = false) {
  Pulses p;
  bool prev = right ? sm.right_lamp() : sm.left_lamp();
  uint32_t since = t;
  for (uint32_t end = t + ms; t < end; t += 10) {
    sm.step(in, t);
    in.hazard_pressed_edge = false;
    in.crash_event = false;
    const bool now = right ? sm.right_lamp() : sm.left_lamp();
    if (now != prev) {
      (prev ? p.on : p.off).push_back(t - since);
      since = t;
      prev = now;
    }
  }
  return p;
}

TurnSignalSm::Inputs run_mode() {
  TurnSignalSm::Inputs in;
  in.mode = PowerMode::Run;
  return in;
}
}  // namespace

TEST(TurnSignal, IdleNoLamps) {
  TurnSignalSm sm;
  uint32_t t = 0;
  run(sm, run_mode(), t, 1000);
  CHECK(!sm.left_lamp() && !sm.right_lamp());
  CHECK(sm.request() == Req::None);
}

TEST(TurnSignal, LeftFlashesAt90PerMinute) {
  TurnSignalSm sm;
  auto in = run_mode();
  in.stalk_left = true;
  uint32_t t = 0;
  const Pulses p = run(sm, in, t, 5000);
  REQUIRE(p.on.size() >= 6);
  for (size_t i = 1; i < p.on.size(); ++i) CHECK_NEAR(p.on[i], 340, 10);
  for (size_t i = 1; i < p.off.size(); ++i) CHECK_NEAR(p.off[i], 340, 10);
  CHECK(!sm.right_lamp());
}

TEST(TurnSignal, LampOnImmediately) {
  TurnSignalSm sm;
  auto in = run_mode();
  in.stalk_right = true;
  sm.step(in, 0);
  CHECK(sm.right_lamp());
}

TEST(TurnSignal, TurnIgnoredOutsideRun) {
  TurnSignalSm sm;
  TurnSignalSm::Inputs in;
  in.mode = PowerMode::Acc;
  in.stalk_left = true;
  uint32_t t = 0;
  run(sm, in, t, 1000);
  CHECK(sm.flash_count() == 0u);
}

TEST(TurnSignal, HazardWorksWithIgnitionOff) {
  TurnSignalSm sm;
  TurnSignalSm::Inputs in;
  in.mode = PowerMode::Off;
  in.hazard_pressed_edge = true;
  sm.step(in, 0);
  CHECK(sm.hazard_active());
  CHECK(sm.left_lamp() && sm.right_lamp());
}

TEST(TurnSignal, HazardToggles) {
  TurnSignalSm sm;
  TurnSignalSm::Inputs in;
  in.hazard_pressed_edge = true;
  sm.step(in, 0);
  sm.step(in, 10);
  CHECK(!sm.hazard_active());
}

TEST(TurnSignal, HazardOverridesTurn) {
  TurnSignalSm sm;
  auto in = run_mode();
  in.stalk_left = true;
  sm.step(in, 0);
  in.hazard_pressed_edge = true;
  sm.step(in, 10);
  CHECK(sm.request() == Req::Hazard);
}

TEST(TurnSignal, ComfortTapGivesThreeFlashes) {
  TurnSignalSm sm;
  auto in = run_mode();
  in.stalk_left = true;
  uint32_t t = 0;
  run(sm, in, t, 200);
  in.stalk_left = false;
  const Pulses p = run(sm, in, t, 4000);
  CHECK_EQ(sm.flash_count(), 3u);
  CHECK(!sm.left_lamp());
  (void)p;
}

TEST(TurnSignal, LongHoldNoComfortAfterRelease) {
  TurnSignalSm sm;
  auto in = run_mode();
  in.stalk_left = true;
  uint32_t t = 0;
  run(sm, in, t, 2000);
  const uint32_t n = sm.flash_count();
  in.stalk_left = false;
  run(sm, in, t, 3000);
  CHECK(sm.flash_count() <= n + 1);  // at most the flash already in progress
}

TEST(TurnSignal, ReleaseCompletesCurrentFlash) {
  TurnSignalSm sm;
  auto in = run_mode();
  in.stalk_right = true;
  uint32_t t = 0;
  run(sm, in, t, 2050);  // inside an on-phase
  in.stalk_right = false;
  const Pulses p = run(sm, in, t, 2000, true);
  for (auto d : p.on) CHECK(d >= 250);
}

TEST(TurnSignal, ChatteringStalkNoShortPulses) {
  TurnSignalSm sm;
  auto in = run_mode();
  uint32_t t = 0;
  std::vector<uint32_t> on;
  bool prev = false;
  uint32_t since = 0;
  for (int i = 0; i < 400; ++i, t += 10) {
    in.stalk_left = (i / 7) % 2 == 0;  // 70 ms on/off chatter
    sm.step(in, t);
    if (sm.left_lamp() != prev) {
      if (prev) on.push_back(t - since);
      since = t;
      prev = sm.left_lamp();
    }
  }
  for (auto d : on) CHECK(d >= 250);
}

TEST(TurnSignal, CrashForcesHazardImmediately) {
  TurnSignalSm sm;
  auto in = run_mode();
  in.stalk_left = true;
  sm.step(in, 0);
  sm.step(in, 340);  // left lamp now in off phase
  in.crash_event = true;
  sm.step(in, 350);
  CHECK(sm.hazard_active());
  CHECK(sm.left_lamp() && sm.right_lamp());
}

TEST(TurnSignal, BothStalksIgnored) {
  TurnSignalSm sm;
  auto in = run_mode();
  in.stalk_left = in.stalk_right = true;
  uint32_t t = 0;
  run(sm, in, t, 500);
  CHECK_EQ(sm.flash_count(), 0u);
}

TEST_MAIN()

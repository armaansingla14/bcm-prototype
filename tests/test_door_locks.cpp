#include "bcm/app/door_locks.hpp"
#include "testfw.hpp"

using namespace bcm::app;
using State = DoorLockSm::State;

namespace {
void run(DoorLockSm& sm, DoorLockSm::Inputs in, uint32_t& t, uint32_t ms) {
  for (uint32_t end = t + ms; t < end; t += 10) {
    sm.step(in, t);
    in.lock_sw_edge = in.unlock_sw_edge = in.rke_lock = in.rke_unlock = in.crash_event = false;
  }
}
DoorLockSm::Inputs driving(double kph) {
  DoorLockSm::Inputs in;
  in.mode = PowerMode::Run;
  in.speed_kph = kph;
  in.speed_valid = true;
  return in;
}
}  // namespace

TEST(DoorLocks, SwitchLockPulse) {
  DoorLockSm sm;
  DoorLockSm::Inputs in;
  in.lock_sw_edge = true;
  uint32_t t = 0;
  sm.step(in, t);
  CHECK(sm.lock_motor());
  run(sm, DoorLockSm::Inputs{}, t, 300);
  CHECK(!sm.lock_motor());
  CHECK(sm.locked());
}

TEST(DoorLocks, PulseLength200ms) {
  DoorLockSm sm;
  DoorLockSm::Inputs in;
  in.unlock_sw_edge = true;
  uint32_t t = 0, on = 0;
  for (; t < 1000; t += 10) {
    sm.step(in, t);
    in.unlock_sw_edge = false;
    if (sm.unlock_motor()) on += 10;
  }
  CHECK_EQ(on, 200u);
}

TEST(DoorLocks, AutoLockAboveThreshold) {
  DoorLockSm sm;
  uint32_t t = 0;
  run(sm, driving(10), t, 1000);
  CHECK(!sm.locked());
  run(sm, driving(20), t, 500);
  CHECK(sm.locked());
  CHECK_EQ(sm.autolocks(), 1u);
}

TEST(DoorLocks, AutoLockOncePerCycle) {
  DoorLockSm sm;
  uint32_t t = 0;
  run(sm, driving(30), t, 500);
  auto in = driving(30);
  in.unlock_sw_edge = true;  // driver unlocks while moving
  run(sm, in, t, 500);
  run(sm, driving(30), t, 5000);
  CHECK(!sm.locked());
  CHECK_EQ(sm.autolocks(), 1u);
}

TEST(DoorLocks, RearmWhenStoppedWithDoorOpen) {
  DoorLockSm sm;
  uint32_t t = 0;
  run(sm, driving(30), t, 500);
  auto stop = driving(0);
  stop.unlock_sw_edge = true;
  run(sm, stop, t, 500);
  stop.door_ajar = true;
  run(sm, stop, t, 500);
  CHECK(sm.autolock_armed());
  run(sm, driving(30), t, 500);
  CHECK(sm.locked());
  CHECK_EQ(sm.autolocks(), 2u);
}

TEST(DoorLocks, NoAutoLockWithInvalidSpeed) {
  DoorLockSm sm;
  uint32_t t = 0;
  auto in = driving(80);
  in.speed_valid = false;
  run(sm, in, t, 2000);
  CHECK(!sm.locked());
}

TEST(DoorLocks, NoAutoLockOutsideRun) {
  DoorLockSm sm;
  uint32_t t = 0;
  auto in = driving(80);
  in.mode = PowerMode::Acc;
  run(sm, in, t, 2000);
  CHECK(!sm.locked());
}

TEST(DoorLocks, CrashUnlocksAndInhibits) {
  DoorLockSm sm;
  uint32_t t = 0;
  run(sm, driving(50), t, 500);
  REQUIRE(sm.locked());
  auto in = driving(50);
  in.crash_event = true;
  run(sm, in, t, 500);
  CHECK(!sm.locked());
  auto lock = driving(0);
  lock.lock_sw_edge = true;
  run(sm, lock, t, 1000);
  CHECK(!sm.locked());  // inhibited for 10 s
  CHECK(sm.crash_inhibit_active(t));
}

TEST(DoorLocks, CrashSecondUnlockPulse) {
  DoorLockSm sm;
  uint32_t t = 0;
  auto in = driving(50);
  in.crash_event = true;
  int pulses = 0;
  bool prev = false;
  for (; t < 3000; t += 10) {
    sm.step(in, t);
    in.crash_event = false;
    if (sm.unlock_motor() && !prev) ++pulses;
    prev = sm.unlock_motor();
  }
  CHECK_EQ(pulses, 2);
}

TEST(DoorLocks, CrashPreemptsLockPulse) {
  DoorLockSm sm;
  DoorLockSm::Inputs in;
  in.lock_sw_edge = true;
  sm.step(in, 0);
  REQUIRE(sm.lock_motor());
  DoorLockSm::Inputs crash;
  crash.crash_event = true;
  sm.step(crash, 50);
  CHECK(sm.unlock_motor());
  CHECK(!sm.lock_motor());
}

TEST(DoorLocks, LockAllowedAfterCrashInhibit) {
  DoorLockSm sm;
  uint32_t t = 0;
  DoorLockSm::Inputs crash;
  crash.crash_event = true;
  run(sm, crash, t, 10500);
  DoorLockSm::Inputs lock;
  lock.lock_sw_edge = true;
  run(sm, lock, t, 500);
  CHECK(sm.locked());
}

TEST(DoorLocks, RemoteLockRefusedWithDoorAjar) {
  DoorLockSm sm;
  uint32_t t = 0;
  DoorLockSm::Inputs in;
  in.rke_lock = true;
  in.door_ajar = true;
  run(sm, in, t, 500);
  CHECK(!sm.locked());
  in.door_ajar = false;
  in.rke_lock = true;
  run(sm, in, t, 500);
  CHECK(sm.locked());
}

TEST(DoorLocks, ThermalProtectionLimitsActuations) {
  DoorLockSm sm;
  uint32_t t = 0;
  int pulses = 0;
  bool prev = false;
  for (int i = 0; i < 30; ++i) {
    DoorLockSm::Inputs in;
    (i % 2 ? in.unlock_sw_edge : in.lock_sw_edge) = true;
    for (uint32_t end = t + 300; t < end; t += 10) {
      sm.step(in, t);
      in.lock_sw_edge = in.unlock_sw_edge = false;
      const bool m = sm.lock_motor() || sm.unlock_motor();
      if (m && !prev) ++pulses;
      prev = m;
    }
  }
  CHECK_EQ(pulses, 10);
  CHECK(sm.thermal_limited());
}

TEST(DoorLocks, ThermalWindowExpires) {
  DoorLockSm sm;
  uint32_t t = 0;
  for (int i = 0; i < 12; ++i) {
    DoorLockSm::Inputs in;
    in.lock_sw_edge = true;
    run(sm, in, t, 300);
  }
  run(sm, DoorLockSm::Inputs{}, t, 31000);
  DoorLockSm::Inputs in;
  in.unlock_sw_edge = true;
  run(sm, in, t, 300);
  CHECK(!sm.locked());
  CHECK(!sm.thermal_limited());
}

TEST_MAIN()

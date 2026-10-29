#include "bcm/app/wipers.hpp"
#include "testfw.hpp"

using namespace bcm::app;
using State = WiperSm::State;

namespace {
// wiper motor + park cam model: 1.0 s (low) / 0.65 s (high) per cycle
struct Motor {
  double phase = 0.02;
  bool stalled = false;
  int wipes = 0;
  bool park() const { return phase < 0.08; }
  void step(const WiperSm& sm) {
    if ((sm.motor_low() || sm.motor_high()) && !stalled) {
      phase += 10.0 / (sm.motor_high() ? 650.0 : 1000.0);
      if (phase >= 1.0) {
        phase -= 1.0;
        ++wipes;
      }
    }
  }
};

struct Bench {
  WiperSm sm;
  Motor m;
  uint32_t t = 0;
  WiperSm::Inputs in;
  Bench() { in.mode = PowerMode::Run; }
  void run(uint32_t ms) {
    for (uint32_t end = t + ms; t < end; t += 10) {
      in.park = m.park();
      sm.step(in, t);
      m.step(sm);
    }
  }
};
}  // namespace

TEST(Wipers, OffStaysParked) {
  Bench b;
  b.run(5000);
  CHECK(b.sm.state() == State::Parked);
  CHECK_EQ(b.m.wipes, 0);
}

TEST(Wipers, LowSpeedContinuous) {
  Bench b;
  b.in.sw = WiperSwitch::Low;
  b.run(10000);
  CHECK(b.sm.continuous());
  CHECK(b.m.wipes >= 9 && b.m.wipes <= 10);
}

TEST(Wipers, HighSpeedFaster) {
  Bench b;
  b.in.sw = WiperSwitch::High;
  b.run(10000);
  CHECK(b.sm.motor_high());
  CHECK(b.m.wipes >= 14);
}

TEST(Wipers, SwitchOffParks) {
  Bench b;
  b.in.sw = WiperSwitch::Low;
  b.run(1450);  // mid-sweep
  b.in.sw = WiperSwitch::Off;
  b.run(1500);
  CHECK(b.sm.state() == State::Parked);
  CHECK(b.m.park());
  CHECK(!b.sm.motor_low());
}

TEST(Wipers, AutoLightRainIntermittent) {
  Bench b;
  b.in.sw = WiperSwitch::Auto;
  b.in.rain_valid = true;
  b.in.rain_pct = 10.0;
  b.run(30000);
  // interval at 10 %: 6000 - 0.2 * 4500 = 5100 ms (+ ~1 s sweep)
  CHECK_EQ(b.sm.interval_ms(), 5100u);
  CHECK(b.m.wipes >= 4 && b.m.wipes <= 6);
  CHECK(!b.sm.continuous());
}

TEST(Wipers, AutoHeavyRainHigh) {
  Bench b;
  b.in.sw = WiperSwitch::Auto;
  b.in.rain_valid = true;
  b.in.rain_pct = 85.0;
  b.run(1000);
  CHECK(b.sm.state() == State::High);
}

TEST(Wipers, AutoHysteresis) {
  Bench b;
  b.in.sw = WiperSwitch::Auto;
  b.in.rain_valid = true;
  b.in.rain_pct = 35.0;
  b.run(1000);
  REQUIRE(b.sm.state() == State::Low);
  b.in.rain_pct = 27.0;  // below 30 up-threshold, above 25 down-threshold
  b.run(3000);
  CHECK(b.sm.state() == State::Low);
  b.in.rain_pct = 20.0;
  b.run(3000);
  CHECK(b.sm.state() != State::Low);
}

TEST(Wipers, AutoDryNoWipes) {
  Bench b;
  b.in.sw = WiperSwitch::Auto;
  b.in.rain_valid = true;
  b.in.rain_pct = 0.0;
  b.run(20000);
  CHECK_EQ(b.m.wipes, 0);
}

TEST(Wipers, AutoDegradedWithoutSensor) {
  Bench b;
  b.in.sw = WiperSwitch::Auto;
  b.in.rain_valid = false;
  b.run(20000);
  CHECK(b.sm.degraded());
  CHECK_EQ(b.sm.interval_ms(), 4000u);
  CHECK(b.m.wipes >= 3);
}

TEST(Wipers, WasherPlusAfterWipes) {
  Bench b;
  b.in.washer = true;
  b.run(1000);
  CHECK(b.sm.pump());
  b.in.washer = false;
  const int before = b.m.wipes;
  b.run(6000);
  CHECK(!b.sm.pump());
  CHECK(b.sm.state() == State::Parked);
  CHECK(b.m.wipes - before >= 3);
}

TEST(Wipers, StallDetectedWithin3s) {
  Bench b;
  b.in.sw = WiperSwitch::Low;
  b.run(1500);
  b.m.stalled = true;
  b.run(2000);
  CHECK(!b.sm.stalled());
  b.run(1500);  // <= 3 s since the last park-switch edge + margin
  CHECK(b.sm.stalled());
  CHECK(!b.sm.motor_low() && !b.sm.motor_high());
}

TEST(Wipers, StallClearedBySwitchOffAndParks) {
  Bench b;
  b.in.sw = WiperSwitch::Low;
  b.run(1500);
  b.m.stalled = true;
  b.run(5000);
  REQUIRE(b.sm.stalled());
  b.m.stalled = false;
  b.in.sw = WiperSwitch::Off;
  b.run(2000);
  CHECK(b.sm.state() == State::Parked);
  CHECK(b.m.park());
}

TEST(Wipers, StallPersistsWhileSwitchOn) {
  Bench b;
  b.in.sw = WiperSwitch::High;
  b.run(500);
  b.m.stalled = true;
  b.run(4000);
  b.m.stalled = false;
  b.run(5000);
  CHECK(b.sm.stalled());
}

TEST(Wipers, SuspendedDuringCrank) {
  Bench b;
  b.in.sw = WiperSwitch::Low;
  b.run(500);
  b.in.mode = PowerMode::Crank;
  b.run(100);
  CHECK(b.sm.state() == State::Suspended);
  CHECK(!b.sm.motor_low());
  b.in.mode = PowerMode::Run;
  b.run(200);
  CHECK(b.sm.motor_low());
}

TEST(Wipers, IgnitionOffCompletesPark) {
  Bench b;
  b.in.sw = WiperSwitch::High;
  b.run(1300);
  b.in.mode = PowerMode::Off;
  b.run(2000);
  CHECK(b.m.park());
  CHECK(!b.sm.motor_low() && !b.sm.motor_high());
}

TEST(Wipers, StatusCodes) {
  Bench b;
  CHECK_EQ(b.sm.status_code(), 0);
  b.in.sw = WiperSwitch::Low;
  b.run(100);
  CHECK_EQ(b.sm.status_code(), 2);
  b.in.sw = WiperSwitch::High;
  b.run(100);
  CHECK_EQ(b.sm.status_code(), 3);
}

TEST_MAIN()

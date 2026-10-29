#include "bcm/app/lighting.hpp"
#include "testfw.hpp"

using namespace bcm::app;

namespace {
LightingSm::Inputs auto_run(double lux, bool valid = true) {
  LightingSm::Inputs in;
  in.sw_auto = true;
  in.mode = PowerMode::Run;
  in.ambient_lux = lux;
  in.ambient_valid = valid;
  return in;
}
void run(LightingSm& sm, const LightingSm::Inputs& in, uint32_t& t, uint32_t ms) {
  for (uint32_t end = t + ms; t < end; t += 10) sm.step(in, t);
}
}  // namespace

TEST(Lighting, ManualOnInAcc) {
  LightingSm sm;
  LightingSm::Inputs in;
  in.sw_on = true;
  in.mode = PowerMode::Acc;
  sm.step(in, 0);
  CHECK(sm.low_beam());
  in.mode = PowerMode::Off;
  sm.step(in, 10);
  CHECK(!sm.low_beam());
}

TEST(Lighting, AutoBrightStaysOff) {
  LightingSm sm;
  uint32_t t = 0;
  run(sm, auto_run(20000), t, 10000);
  CHECK(!sm.low_beam());
}

TEST(Lighting, AutoDarkOnSelection) {
  LightingSm sm;
  sm.step(auto_run(50), 0);
  CHECK(sm.low_beam());  // immediate decision when AUTO is selected
}

TEST(Lighting, AutoDarkeningUsesDwell) {
  LightingSm sm;
  uint32_t t = 0;
  run(sm, auto_run(20000), t, 1000);
  run(sm, auto_run(500), t, 1900);
  CHECK(!sm.low_beam());
  run(sm, auto_run(500), t, 200);
  CHECK(sm.low_beam());
}

TEST(Lighting, HysteresisBandHoldsState) {
  LightingSm sm;
  uint32_t t = 0;
  run(sm, auto_run(500), t, 3000);
  REQUIRE(sm.low_beam());
  run(sm, auto_run(1800), t, 20000);  // between on (1000) and off (2500)
  CHECK(sm.low_beam());
}

TEST(Lighting, BrighteningNeedsFiveSeconds) {
  LightingSm sm;
  uint32_t t = 0;
  run(sm, auto_run(500), t, 3000);
  run(sm, auto_run(30000), t, 4900);
  CHECK(sm.low_beam());
  run(sm, auto_run(30000), t, 200);
  CHECK(!sm.low_beam());
}

TEST(Lighting, ShortShadowIgnored) {
  LightingSm sm;
  uint32_t t = 0;
  run(sm, auto_run(30000), t, 1000);
  run(sm, auto_run(100), t, 1500);  // bridge shadow
  run(sm, auto_run(30000), t, 1000);
  CHECK(!sm.low_beam());
}

TEST(Lighting, SensorLossFailSafeOn) {
  LightingSm sm;
  uint32_t t = 0;
  run(sm, auto_run(30000), t, 1000);
  sm.step(auto_run(30000, false), t);
  CHECK(sm.low_beam());
  CHECK(sm.fail_safe_active());
}

TEST(Lighting, RainLightRule) {
  LightingSm sm;
  uint32_t t = 0;
  auto in = auto_run(30000);
  in.wipers_continuous = true;
  run(sm, in, t, 4900);
  CHECK(!sm.low_beam());
  run(sm, in, t, 200);
  CHECK(sm.low_beam());
  in.wipers_continuous = false;
  run(sm, in, t, 9900);
  CHECK(sm.low_beam());
  run(sm, in, t, 200);
  CHECK(!sm.low_beam());
}

TEST(Lighting, HighBeamRequiresLowBeam) {
  LightingSm sm;
  auto in = auto_run(30000);
  in.high_beam_sw = true;
  sm.step(in, 0);
  CHECK(!sm.high_beam());
  in.sw_auto = false;
  in.sw_on = true;
  sm.step(in, 10);
  CHECK(sm.high_beam());
}

TEST(Lighting, ExitDelayAfterIgnitionOff) {
  LightingSm sm;
  uint32_t t = 0;
  run(sm, auto_run(10), t, 1000);
  REQUIRE(sm.low_beam());
  auto off = auto_run(10);
  off.mode = PowerMode::Off;
  run(sm, off, t, 29000);
  CHECK(sm.low_beam());
  CHECK(sm.state() == LightingSm::State::ExitDelay);
  run(sm, off, t, 2000);
  CHECK(!sm.low_beam());
}

TEST(Lighting, ExitDelayCancelledBySwitch) {
  LightingSm sm;
  uint32_t t = 0;
  run(sm, auto_run(10), t, 1000);
  auto off = auto_run(10);
  off.mode = PowerMode::Off;
  run(sm, off, t, 1000);
  off.sw_auto = false;
  sm.step(off, t);
  CHECK(!sm.low_beam());
}

TEST(Lighting, AutoInactiveInAcc) {
  LightingSm sm;
  auto in = auto_run(10);
  in.mode = PowerMode::Acc;
  sm.step(in, 0);
  CHECK(!sm.low_beam());
}

TEST_MAIN()

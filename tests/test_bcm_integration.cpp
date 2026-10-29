// Closed-loop tests: BCM + SimHal + vehicle plant model.
#include "bcm/can/bcm_db.hpp"
#include "rig.hpp"
#include "testfw.hpp"

using namespace bcm;
using hal::Din;
using hal::Dout;
using sim::KeyPos;

TEST(Integration, PowerUpQuiet) {
  Rig r;
  r.run(1000);
  for (int i = 0; i < hal::kNumDout; ++i) CHECK(!r.out(static_cast<Dout>(i)));
  CHECK(r.bcm.power().mode() == app::PowerMode::Off);
}

TEST(Integration, EngineStartSequence) {
  Rig r;
  r.sim.set_key(KeyPos::Acc);
  r.run(200);
  r.sim.set_key(KeyPos::Run);
  r.run(200);
  r.sim.set_key(KeyPos::Start);
  r.run(300);
  CHECK(r.out(Dout::StarterRelay));
  CHECK(!r.out(Dout::AccRelay));
  r.run(900);  // engine fires after 800 ms of cranking
  CHECK(r.sim.engine_running());
  CHECK(!r.out(Dout::StarterRelay));  // cut by ECM status although key held
  r.sim.set_key(KeyPos::Run);
  r.run(200);
  CHECK(r.out(Dout::AccRelay) && r.out(Dout::RunRelay));
}

TEST(Integration, StatusMessagePeriodAndContent) {
  Rig r;
  r.start_engine();
  r.hal.clear_tx_log();
  r.run(1000);
  int n = 0;
  for (const auto& rec : r.hal.tx_log()) {
    if (rec.frame.id != can::id::kBcmStatus) continue;
    ++n;
    CHECK(can::checksum_ok(rec.frame));
    CHECK_NEAR(can::get(rec.frame, "PowerMode"), 2, 0);
  }
  CHECK_EQ(n, 10);
}

TEST(Integration, StatusCounterIncrements) {
  Rig r;
  r.run(500);
  const auto& log = r.hal.tx_log();
  REQUIRE(log.size() >= 3);
  for (size_t i = 1; i < log.size(); ++i) {
    CHECK_EQ(can::alive_counter(log[i].frame), static_cast<uint8_t>((can::alive_counter(log[i - 1].frame) + 1) & 15));
  }
}

TEST(Integration, AutoLockWhileDriving) {
  Rig r;
  r.start_engine();
  r.sim.speed_kph = 30;
  r.run(1000);
  CHECK(r.sim.doors_locked());
}

TEST(Integration, UnlockAtStandstillAndDoorOpenRearms) {
  Rig r;
  r.start_engine();
  r.sim.speed_kph = 30;
  r.run(1000);
  r.sim.speed_kph = 0;
  r.run(200);
  r.press(Din::UnlockSw);
  r.run(500);
  CHECK(!r.sim.doors_locked());
  r.sim.set_switch(Din::DriverDoorAjar, true);
  r.run(500);
  r.sim.set_switch(Din::DriverDoorAjar, false);
  r.sim.speed_kph = 25;
  r.run(1000);
  CHECK(r.sim.doors_locked());
}

TEST(Integration, CrashUnlocksDoorsAndHazards) {
  Rig r;
  r.start_engine();
  r.sim.speed_kph = 50;
  r.run(1000);
  REQUIRE(r.sim.doors_locked());
  r.sim.crash = true;
  r.run(300);
  CHECK(!r.sim.doors_locked());
  CHECK(r.bcm.turn().hazard_active());
  r.run(5000);
  CHECK(!r.sim.doors_locked());
}

TEST(Integration, RemoteKeylessLockUnlock) {
  Rig r;
  r.run(200);
  r.sim.rke_lock();
  r.run(500);
  CHECK(r.sim.doors_locked());
  r.sim.rke_unlock();
  r.run(500);
  CHECK(!r.sim.doors_locked());
}

TEST(Integration, AutoHeadlampsAtNight) {
  Rig r;
  r.sim.ambient_lux = 5;
  r.sim.set_switch(Din::HeadlampAuto, true);
  r.start_engine();
  r.run(500);
  CHECK(r.out(Dout::LowBeam));
}

TEST(Integration, RainSensorDrivesWipers) {
  Rig r;
  r.sim.rain_pct = 80;
  r.sim.set_switch(Din::WiperAuto, true);
  r.start_engine();
  r.run(1000);
  CHECK(r.out(Dout::WiperHigh));
  r.sim.set_switch(Din::WiperAuto, false);
  r.run(1500);
  CHECK(r.sim.wiper_parked());
  CHECK(!r.out(Dout::WiperLow) && !r.out(Dout::WiperHigh));
}

TEST(Integration, TurnIndicatorFlashes) {
  Rig r;
  r.start_engine();
  r.sim.set_switch(Din::TurnLeft, true);
  int edges = 0;
  bool prev = false;
  for (int i = 0; i < 300; ++i) {
    r.tick();
    if (r.out(Dout::TurnLeftLamp) != prev) ++edges;
    prev = r.out(Dout::TurnLeftLamp);
  }
  CHECK(edges >= 8 && edges <= 10);  // 3 s at 1.5 Hz
  CHECK(!r.out(Dout::TurnRightLamp));
}

TEST(Integration, ExitDelayAfterIgnitionOff) {
  Rig r;
  r.sim.ambient_lux = 5;
  r.sim.set_switch(Din::HeadlampAuto, true);
  r.start_engine();
  r.run(500);
  r.sim.set_key(KeyPos::Off);
  r.run(10000);
  CHECK(r.out(Dout::LowBeam));
  r.run(21000);
  CHECK(!r.out(Dout::LowBeam));
}

TEST(Integration, StatusReportsLockAndLamps) {
  Rig r;
  r.sim.set_switch(Din::HeadlampOn, true);
  r.start_engine();
  r.sim.rke_lock();
  r.run(500);
  const auto& log = r.hal.tx_log();
  REQUIRE(!log.empty());
  const auto& f = log.back().frame;
  CHECK_NEAR(can::get(f, "LockState"), 1, 0);
  CHECK_NEAR(can::get(f, "LowBeamOn"), 1, 0);
}

TEST(Integration, NaiveBaselineBasicFunctions) {
  hal::SimHal hal;
  sim::VehicleSim s(hal, 1);
  app::NaiveBcm n(hal);
  uint32_t t = 0;
  auto run = [&](uint32_t ms) {
    for (uint32_t end = t + ms; t < end; t += 10) {
      s.pre_step(t);
      n.step();
      s.post_step(t);
    }
  };
  s.set_key(KeyPos::Run);
  s.speed_kph = 40;
  run(500);
  CHECK(s.doors_locked());
  s.set_switch(Din::HazardSw, true);
  run(50);
  CHECK(hal.output(Dout::TurnLeftLamp) && hal.output(Dout::TurnRightLamp));
}

TEST_MAIN()

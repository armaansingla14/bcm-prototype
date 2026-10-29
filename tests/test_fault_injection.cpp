// Fault injection: CAN timeouts, E2E errors, stuck/chattering inputs,
// bus-off, actuator faults, plus scenario-level comparisons.
#include "bcm/can/bcm_db.hpp"
#include "bcm/sim/scenario.hpp"
#include "rig.hpp"
#include "testfw.hpp"

using namespace bcm;
using app::Dtc;
using hal::Din;
using hal::Dout;
using sim::KeyPos;

TEST(Fault, EscTimeoutSetsDtcAndBlocksAutolock) {
  Rig r;
  r.start_engine();
  r.sim.fault(can::id::kEscVehicleSpeed).drop = true;
  r.sim.speed_kph = 60;
  r.run(2000);
  CHECK(r.bcm.dtc().active(Dtc::EscTimeout));
  CHECK(!r.sim.doors_locked());
}

TEST(Fault, StaleSpeedDoesNotRelockAtStop) {
  Rig r;
  r.start_engine();
  r.sim.speed_kph = 60;
  r.run(1000);
  REQUIRE(r.sim.doors_locked());
  r.sim.fault(can::id::kEscVehicleSpeed).drop = true;  // last value 60 km/h
  r.sim.speed_kph = 0;
  r.run(2000);
  r.press(Din::UnlockSw);
  r.run(3000);
  CHECK(!r.sim.doors_locked());
}

TEST(Fault, EscRecoveryClearsDtc) {
  Rig r;
  r.start_engine();
  r.sim.fault(can::id::kEscVehicleSpeed).drop = true;
  r.run(500);
  REQUIRE(r.bcm.dtc().active(Dtc::EscTimeout));
  r.sim.fault(can::id::kEscVehicleSpeed).drop = false;
  r.run(200);
  CHECK(!r.bcm.dtc().active(Dtc::EscTimeout));
  CHECK(r.bcm.dtc().latched(Dtc::EscTimeout));
}

TEST(Fault, RlsTimeoutLightsOnWipersDegraded) {
  Rig r;
  r.sim.ambient_lux = 30000;
  r.sim.rain_pct = 40;
  r.sim.set_switch(Din::HeadlampAuto, true);
  r.sim.set_switch(Din::WiperAuto, true);
  r.start_engine();
  r.run(1000);
  CHECK(!r.out(Dout::LowBeam));
  r.sim.fault(can::id::kRlsRainLight).drop = true;
  r.run(1000);
  CHECK(r.out(Dout::LowBeam));
  CHECK(r.bcm.wipers().degraded());
  CHECK(r.bcm.dtc().active(Dtc::RlsTimeout));
}

TEST(Fault, RlsSensorFaultFlag) {
  Rig r;
  r.sim.ambient_lux = 30000;
  r.sim.set_switch(Din::HeadlampAuto, true);
  r.start_engine();
  r.sim.rls_sensor_fault = true;
  r.run(500);
  CHECK(r.out(Dout::LowBeam));
  CHECK(r.bcm.dtc().active(Dtc::RainSensorFault));
}

TEST(Fault, AcuTimeoutNoSpuriousCrashAction) {
  Rig r;
  r.start_engine();
  r.sim.speed_kph = 50;
  r.run(1000);
  r.sim.fault(can::id::kAcuCrashStatus).drop = true;
  r.run(2000);
  CHECK(r.bcm.dtc().active(Dtc::AcuTimeout));
  CHECK(r.sim.doors_locked());
  CHECK(!r.bcm.turn().hazard_active());
}

TEST(Fault, CrashAfterAcuRecoveryStillHandled) {
  Rig r;
  r.start_engine();
  r.sim.speed_kph = 50;
  r.run(1000);
  r.sim.fault(can::id::kAcuCrashStatus).drop = true;
  r.run(1000);
  r.sim.crash = true;
  r.sim.fault(can::id::kAcuCrashStatus).drop = false;
  r.run(400);
  CHECK(!r.sim.doors_locked());
}

TEST(Fault, CorruptedCrashFramesIgnored) {
  Rig r;
  r.start_engine();
  r.sim.speed_kph = 50;
  r.run(1000);
  r.sim.fault(can::id::kAcuCrashStatus).corrupt_prob = 1.0;
  r.run(3000);
  CHECK(r.sim.doors_locked());
  CHECK(!r.bcm.turn().hazard_active());
  CHECK(r.bcm.dtc().active(Dtc::E2eError));
}

TEST(Fault, FrozenCounterTreatedAsInvalid) {
  Rig r;
  r.start_engine();
  r.sim.fault(can::id::kEscVehicleSpeed).freeze_counter = true;
  r.sim.speed_kph = 60;
  r.run(1000);
  CHECK(r.bcm.esc_monitor().status() == can::RxStatus::Invalid);
  CHECK(!r.sim.doors_locked());
}

TEST(Fault, SpeedPlausibilityRejectsJump) {
  Rig r;
  r.start_engine();
  r.run(500);
  // one frame with an impossible jump, correct checksum
  can::Frame f = can::make_frame(can::id::kEscVehicleSpeed);
  can::set(f, "VehicleSpeed", 163.0);
  can::set(f, "SpeedValid", 1);
  const auto& prev = r.bcm.esc_monitor();
  (void)prev;
  // inject directly after the regular frame of this tick
  r.sim.fault(can::id::kEscVehicleSpeed).drop = true;
  can::seal(f, 0);
  // find a counter the monitor accepts by trying successive values
  bool injected = false;
  for (uint8_t c = 0; c < 16 && !injected; ++c) {
    can::seal(f, c);
    r.hal.deliver(f);
    r.tick();
    injected = r.bcm.speed_rejects() > 0;
  }
  CHECK(injected);
  CHECK_NEAR(r.bcm.can_inputs().speed_kph, 0.0, 1e-9);
  CHECK(!r.sim.doors_locked());
}

TEST(Fault, BusOffRecoveryAndDtc) {
  Rig r;
  r.start_engine();
  r.sim.set_bus_fault(true);
  r.run(2000);
  CHECK(r.bcm.dtc().active(Dtc::CanBusOff));
  CHECK(r.hal.recovery_requests() <= 7u);  // 5 fast + slow ones
  r.sim.set_bus_fault(false);
  r.hal.clear_tx_log();
  r.run(1500);
  CHECK(!r.bcm.dtc().active(Dtc::CanBusOff));
  CHECK(!r.hal.tx_log().empty());
}

TEST(Fault, BusOffDoesNotRaiseTimeoutDtcs) {
  Rig r;
  r.start_engine();
  r.sim.set_bus_fault(true);
  r.run(1000);
  CHECK(!r.bcm.dtc().active(Dtc::EscTimeout));
  CHECK(!r.bcm.dtc().active(Dtc::RlsTimeout));
}

TEST(Fault, BusOffLightsFailSafe) {
  Rig r;
  r.sim.ambient_lux = 30000;
  r.sim.set_switch(Din::HeadlampAuto, true);
  r.start_engine();
  r.run(500);
  r.sim.set_bus_fault(true);
  r.run(1000);
  CHECK(r.out(Dout::LowBeam));
}

TEST(Fault, ShortBusOffRecoversFast) {
  Rig r;
  r.run(500);
  r.sim.set_bus_fault(true);
  r.run(30);
  r.sim.set_bus_fault(false);
  r.hal.clear_tx_log();
  r.run(200);
  CHECK(!r.hal.tx_log().empty());  // first fast attempt at 50 ms
}

TEST(Fault, StuckHazardSwitch) {
  Rig r;
  r.start_engine();
  r.sim.set_stuck(Din::HazardSw, true);
  r.run(500);
  CHECK(r.bcm.turn().hazard_active());  // first press is real
  r.run(11000);
  CHECK(r.bcm.dtc().active(Dtc::HazardSwStuck));
  r.sim.set_stuck(Din::HazardSw, false);
  r.run(500);
  CHECK(!r.bcm.dtc().active(Dtc::HazardSwStuck));
  CHECK(r.bcm.turn().hazard_active());  // release is not a press
}

TEST(Fault, StuckLockSwitchNoHammering) {
  Rig r;
  r.run(500);
  r.sim.set_stuck(Din::LockSw, true);
  int pulses = 0;
  bool prev = false;
  for (int i = 0; i < 3000; ++i) {
    r.tick();
    if (r.out(Dout::LockMotor) && !prev) ++pulses;
    prev = r.out(Dout::LockMotor);
  }
  CHECK_EQ(pulses, 1);
  CHECK(r.bcm.dtc().active(Dtc::LockSwStuck));
}

TEST(Fault, StuckWasherStopsPump) {
  Rig r;
  r.start_engine();
  r.sim.set_stuck(Din::WasherSw, true);
  r.run(2000);
  CHECK(r.out(Dout::WasherPump));
  r.run(31000);
  CHECK(!r.out(Dout::WasherPump));
  CHECK(r.bcm.dtc().active(Dtc::WasherSwStuck));
}

TEST(Fault, SwitchStuckAtPowerUpNeverActuates) {
  Rig r;
  r.sim.set_stuck(Din::LockSw, true);
  r.run(15000);
  CHECK(!r.sim.doors_locked());
  CHECK(r.bcm.dtc().active(Dtc::LockSwStuck));
}

TEST(Fault, GlitchesDoNotToggleHazard) {
  Rig r;
  r.start_engine();
  r.sim.glitch_prob = 0.01;
  r.run(30000);
  CHECK(!r.bcm.turn().hazard_active());
  CHECK(!r.sim.doors_locked());
}

TEST(Fault, ContactBounceSinglePress) {
  Rig r;
  r.sim.bounce_ms = 20;
  r.start_engine();
  r.press(Din::HazardSw, 200);
  r.run(500);
  CHECK(r.bcm.turn().hazard_active());
}

TEST(Fault, WiperStallStopsMotor) {
  Rig r;
  r.sim.set_switch(Din::WiperLow, true);
  r.start_engine();
  r.run(1000);
  r.sim.wiper_stalled = true;
  r.run(3600);
  CHECK(!r.out(Dout::WiperLow));
  CHECK(r.bcm.dtc().active(Dtc::WiperStall));
}

TEST(Fault, EngineNoStartCrankTimeout) {
  Rig r;
  r.sim.engine_can_start = false;
  r.sim.set_key(KeyPos::Run);
  r.run(200);
  r.sim.set_key(KeyPos::Start);
  r.run(9500);
  CHECK(r.out(Dout::StarterRelay));
  r.run(1000);
  CHECK(!r.out(Dout::StarterRelay));
  CHECK(r.bcm.dtc().active(Dtc::CrankTimeout));
}

TEST(Fault, ScenarioNominalStateMachineClean) {
  const auto sc = sim::make_scenario("nominal", 7, 0);
  const auto res = sim::run_scenario(sc, sim::ControllerKind::StateMachine);
  CHECK_EQ(res.total, 0);
}

TEST(Fault, ScenarioNaiveStarterViolation) {
  // the naive controller follows the key: starter stays on after the engine fires
  const auto sc = sim::make_scenario("nominal", 7, 0);
  const auto res = sim::run_scenario(sc, sim::ControllerKind::Naive);
  CHECK(res.violations[static_cast<int>(sim::Req::StarterProtect)] > 0);
}

TEST(Fault, ScenarioBusOffNaiveRecoveryStorm) {
  const auto sc = sim::make_scenario("bus_off", 11, 0);
  const auto sm = sim::run_scenario(sc, sim::ControllerKind::StateMachine);
  const auto nv = sim::run_scenario(sc, sim::ControllerKind::Naive);
  CHECK_EQ(sm.violations[static_cast<int>(sim::Req::BusOffRecovery)], 0);
  CHECK(nv.violations[static_cast<int>(sim::Req::BusOffRecovery)] > 0);
}

TEST(Fault, ScenarioDeterministic) {
  const auto a = sim::make_scenario("combined", 99, 3);
  const auto b = sim::make_scenario("combined", 99, 3);
  CHECK_EQ(a.description, b.description);
  const auto ra = sim::run_scenario(a, sim::ControllerKind::Naive);
  const auto rb = sim::run_scenario(b, sim::ControllerKind::Naive);
  CHECK_EQ(ra.total, rb.total);
}

TEST_MAIN()

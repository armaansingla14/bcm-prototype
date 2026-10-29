#include "bcm/can/bcm_db.hpp"
#include "bcm/can/can_if.hpp"
#include "bcm/hal/sim_hal.hpp"
#include "testfw.hpp"

using namespace bcm;

namespace {
void run(can::CanIf& cif, hal::SimHal& hal, uint32_t from, uint32_t to) {
  for (uint32_t t = from; t < to; t += 10) {
    hal.set_time(t);
    cif.update(t);
  }
}
}  // namespace

TEST(CanIf, SendsWhenOnline) {
  hal::SimHal hal;
  can::CanIf cif(hal);
  cif.update(0);
  CHECK(cif.send(can::make_frame(can::id::kBcmStatus)));
  CHECK_EQ(hal.tx_log().size(), 1u);
  CHECK_EQ(cif.tx_ok(), 1u);
}

TEST(CanIf, DetectsBusOffAndDropsTx) {
  hal::SimHal hal;
  can::CanIf cif(hal);
  hal.set_bus_fault(true);
  cif.update(0);
  CHECK(cif.bus_off_active());
  CHECK(!cif.send(can::make_frame(can::id::kBcmStatus)));
  CHECK_EQ(cif.tx_dropped(), 1u);
  CHECK_EQ(cif.bus_off_events(), 1u);
}

TEST(CanIf, FastRecoveryAfter50ms) {
  hal::SimHal hal;
  can::CanIf cif(hal);
  hal.set_bus_fault(true);
  cif.update(0);
  hal.set_bus_fault(false);  // fault gone, controller still bus-off
  run(cif, hal, 10, 50);
  CHECK_EQ(hal.recovery_requests(), 0u);
  run(cif, hal, 50, 70);
  CHECK_EQ(hal.recovery_requests(), 1u);
  CHECK(!cif.bus_off_active());
}

TEST(CanIf, FiveFastThenSlowAttempts) {
  hal::SimHal hal;
  can::CanIf cif(hal);
  hal.set_bus_fault(true);
  run(cif, hal, 0, 300);  // 5 fast attempts at 50 ms
  CHECK_EQ(hal.recovery_requests(), 5u);
  run(cif, hal, 300, 1200);
  CHECK_EQ(hal.recovery_requests(), 5u);  // waiting 1 s for the 6th
  run(cif, hal, 1200, 1300);
  CHECK_EQ(hal.recovery_requests(), 6u);
}

TEST(CanIf, AttemptRateBoundedOverLongFault) {
  hal::SimHal hal;
  can::CanIf cif(hal);
  hal.set_bus_fault(true);
  run(cif, hal, 0, 10000);
  // 5 fast + one per second afterwards
  CHECK(hal.recovery_requests() <= 5u + 10u);
  CHECK(hal.recovery_requests() >= 5u + 8u);
}

TEST(CanIf, CounterResetsAfterStablePeriod) {
  hal::SimHal hal;
  can::CanIf cif(hal);
  hal.set_bus_fault(true);
  run(cif, hal, 0, 300);
  hal.set_bus_fault(false);
  run(cif, hal, 300, 2500);  // recover, then stable >1 s
  REQUIRE(!cif.bus_off_active());
  const uint32_t before = hal.recovery_requests();
  hal.set_bus_fault(true);
  run(cif, hal, 2500, 2600);
  CHECK_EQ(hal.recovery_requests(), before + 1);  // fast again (50 ms)
}

TEST(CanIf, ReceiveBlockedWhileBusOff) {
  hal::SimHal hal;
  can::CanIf cif(hal);
  hal.deliver(can::make_frame(can::id::kEscVehicleSpeed));
  hal.set_bus_fault(true);  // clears FIFO, controller offline
  cif.update(0);
  can::Frame f;
  CHECK(!cif.receive(f));
  CHECK(!hal.deliver(can::make_frame(can::id::kEscVehicleSpeed)));
  CHECK_EQ(hal.rx_dropped_bus_off(), 1u);
}

TEST(CanIf, RxFifoOverrunCounted) {
  hal::SimHal hal;
  hal.rx_fifo_depth = 3;
  for (int i = 0; i < 5; ++i) hal.deliver(can::make_frame(can::id::kEscVehicleSpeed));
  CHECK_EQ(hal.rx_overruns(), 2u);
}

TEST_MAIN()

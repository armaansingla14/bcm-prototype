#include "bcm/app/bcm.hpp"
#include "bcm/can/bcm_db.hpp"
#include "bcm/hal/stm32_hal.hpp"
#include "testfw.hpp"

using namespace bcm;
using hal::Din;
using hal::Dout;

namespace {
CAN_TypeDef can_regs;
CAN_HandleTypeDef hcan{&can_regs};
IWDG_HandleTypeDef hiwdg{0};

void reset() {
  stub_reset();
  can_regs.ESR = 0;
  hiwdg.reloads = 0;
  // all switch inputs released: pulled up -> high
  for (int i = 0; i < 5; ++i) stub_gpio[i].IDR = 0xFFFF;
}
}  // namespace

TEST(Stm32Port, ActiveLowInputs) {
  reset();
  hal::Stm32Hal board(&hcan, &hiwdg);
  CHECK(!board.read(Din::HazardSw));
  const auto& m = hal::din_map(Din::HazardSw);
  m.port->IDR &= ~static_cast<uint32_t>(m.pin);  // contact closes to ground
  CHECK(board.read(Din::HazardSw));
}

TEST(Stm32Port, OutputsDriveOdr) {
  reset();
  hal::Stm32Hal board(&hcan, &hiwdg);
  board.write(Dout::LowBeam, true);
  const auto& m = hal::dout_map(Dout::LowBeam);
  CHECK(m.port->ODR & m.pin);
  board.write(Dout::LowBeam, false);
  CHECK(!(m.port->ODR & m.pin));
}

TEST(Stm32Port, PinMapUnique) {
  for (int a = 0; a < hal::kNumDout; ++a) {
    for (int b = a + 1; b < hal::kNumDout; ++b) {
      const auto& x = hal::dout_map(static_cast<Dout>(a));
      const auto& y = hal::dout_map(static_cast<Dout>(b));
      CHECK(!(x.port == y.port && x.pin == y.pin));
    }
  }
  for (int a = 0; a < hal::kNumDin; ++a) {
    for (int b = a + 1; b < hal::kNumDin; ++b) {
      const auto& x = hal::din_map(static_cast<Din>(a));
      const auto& y = hal::din_map(static_cast<Din>(b));
      CHECK(!(x.port == y.port && x.pin == y.pin));
    }
  }
}

TEST(Stm32Port, CanRxTxThroughHal) {
  reset();
  hal::Stm32Hal board(&hcan, &hiwdg);
  can::Frame f = can::make_frame(can::id::kEscVehicleSpeed);
  can::set(f, "VehicleSpeed", 33.0);
  can::seal(f, 1);
  stub_push_rx(f.id, f.data.data());
  can::Frame g;
  REQUIRE(board.can_receive(g));
  CHECK_EQ(g.id, f.id);
  CHECK(can::checksum_ok(g));
  CHECK(!board.can_receive(g));
  CHECK(board.can_transmit(f));
  CHECK_EQ(stub_tx_count(), 1u);
}

TEST(Stm32Port, BusOffFromEsrAndRecovery) {
  reset();
  hal::Stm32Hal board(&hcan, &hiwdg);
  CHECK(board.can_state() == hal::CanState::Active);
  can_regs.ESR = CAN_ESR_EPVF;
  CHECK(board.can_state() == hal::CanState::ErrorPassive);
  can_regs.ESR = CAN_ESR_BOFF;
  CHECK(board.can_state() == hal::CanState::BusOff);
  CHECK(!board.can_transmit(can::make_frame(can::id::kBcmStatus)));
  board.can_request_recovery();
  CHECK_EQ(stub_can_restarts(), 1u);
  CHECK(board.can_state() == hal::CanState::Active);
}

TEST(Stm32Port, BcmRunsOnStm32Hal) {
  reset();
  hal::Stm32Hal board(&hcan, &hiwdg);
  app::Bcm bcm(board);
  // press hazard switch (active low) for 100 ms
  const auto& hz = hal::din_map(Din::HazardSw);
  for (uint32_t t = 0; t < 300; t += 10) {
    stub_set_tick(t);
    if (t == 50) hz.port->IDR &= ~static_cast<uint32_t>(hz.pin);
    if (t == 150) hz.port->IDR |= hz.pin;
    bcm.step();
  }
  CHECK(bcm.turn().hazard_active());
  const auto& l = hal::dout_map(Dout::TurnLeftLamp);
  const auto& r = hal::dout_map(Dout::TurnRightLamp);
  CHECK(((l.port->ODR & l.pin) != 0) == ((r.port->ODR & r.pin) != 0));
  CHECK(stub_tx_count() >= 2u);  // BCM_Status every 100 ms
  CHECK_EQ(hiwdg.reloads, 30u);  // watchdog refreshed every step
}

TEST_MAIN()

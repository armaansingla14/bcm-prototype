#include "bcm/hal/stm32_hal.hpp"

namespace bcm::hal {

namespace {

// Board pin-out (BCM prototype board, STM32F446 in LQFP100). Switch inputs
// use internal pull-ups and close to ground; the park cam switch and door
// ajar contact also close to ground.
const PinMap kDin[kNumDin] = {
    {GPIOA, GPIO_PIN_0, true},   // IgnAcc
    {GPIOA, GPIO_PIN_1, true},   // IgnRun
    {GPIOA, GPIO_PIN_2, true},   // IgnStart
    {GPIOB, GPIO_PIN_0, true},   // TurnLeft
    {GPIOB, GPIO_PIN_1, true},   // TurnRight
    {GPIOB, GPIO_PIN_2, true},   // HazardSw
    {GPIOB, GPIO_PIN_3, true},   // HeadlampAuto
    {GPIOB, GPIO_PIN_4, true},   // HeadlampOn
    {GPIOB, GPIO_PIN_5, true},   // HighBeamSw
    {GPIOC, GPIO_PIN_0, true},   // WiperAuto
    {GPIOC, GPIO_PIN_1, true},   // WiperLow
    {GPIOC, GPIO_PIN_2, true},   // WiperHigh
    {GPIOC, GPIO_PIN_3, true},   // WasherSw
    {GPIOC, GPIO_PIN_4, true},   // LockSw
    {GPIOC, GPIO_PIN_5, true},   // UnlockSw
    {GPIOC, GPIO_PIN_6, true},   // DriverDoorAjar
    {GPIOC, GPIO_PIN_7, true},   // WiperPark
};

// High-side driver enables (VNx series smart switches), active high.
const PinMap kDout[kNumDout] = {
    {GPIOD, GPIO_PIN_0, false},   // TurnLeftLamp
    {GPIOD, GPIO_PIN_1, false},   // TurnRightLamp
    {GPIOD, GPIO_PIN_2, false},   // LowBeam
    {GPIOD, GPIO_PIN_3, false},   // HighBeam
    {GPIOD, GPIO_PIN_4, false},   // WiperLow
    {GPIOD, GPIO_PIN_5, false},   // WiperHigh
    {GPIOD, GPIO_PIN_6, false},   // WasherPump
    {GPIOE, GPIO_PIN_0, false},   // LockMotor (H-bridge IN1)
    {GPIOE, GPIO_PIN_1, false},   // UnlockMotor (H-bridge IN2)
    {GPIOE, GPIO_PIN_2, false},   // AccRelay
    {GPIOE, GPIO_PIN_3, false},   // RunRelay
    {GPIOE, GPIO_PIN_4, false},   // StarterRelay
};

}  // namespace

const PinMap& din_map(Din d) { return kDin[static_cast<int>(d)]; }
const PinMap& dout_map(Dout d) { return kDout[static_cast<int>(d)]; }

bool Stm32Hal::read(Din pin) const {
  const PinMap& m = din_map(pin);
  const bool level = HAL_GPIO_ReadPin(m.port, m.pin) == GPIO_PIN_SET;
  return m.active_low ? !level : level;
}

void Stm32Hal::write(Dout pin, bool on) {
  const PinMap& m = dout_map(pin);
  const bool level = m.active_low ? !on : on;
  HAL_GPIO_WritePin(m.port, m.pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool Stm32Hal::can_transmit(const can::Frame& f) {
  CAN_TxHeaderTypeDef hdr{};
  hdr.IDE = f.extended ? CAN_ID_EXT : CAN_ID_STD;
  if (f.extended) {
    hdr.ExtId = f.id;
  } else {
    hdr.StdId = f.id;
  }
  hdr.RTR = CAN_RTR_DATA;
  hdr.DLC = f.dlc;
  uint32_t mailbox = 0;
  uint8_t buf[8];
  for (int i = 0; i < 8; ++i) buf[i] = f.data[i];
  return HAL_CAN_AddTxMessage(hcan_, &hdr, buf, &mailbox) == HAL_OK;
}

bool Stm32Hal::can_receive(can::Frame& f) {
  if (HAL_CAN_GetRxFifoFillLevel(hcan_, CAN_RX_FIFO0) == 0) return false;
  CAN_RxHeaderTypeDef hdr{};
  uint8_t buf[8] = {0};
  if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &hdr, buf) != HAL_OK) return false;
  f.extended = hdr.IDE == CAN_ID_EXT;
  f.id = f.extended ? hdr.ExtId : hdr.StdId;
  f.dlc = static_cast<uint8_t>(hdr.DLC);
  for (int i = 0; i < 8; ++i) f.data[i] = buf[i];
  return true;
}

CanState Stm32Hal::can_state() const {
  const uint32_t esr = hcan_->Instance->ESR;
  if (esr & CAN_ESR_BOFF) return CanState::BusOff;
  if (esr & CAN_ESR_EPVF) return CanState::ErrorPassive;
  return CanState::Active;
}

void Stm32Hal::can_request_recovery() {
  // With ABOM disabled, leaving bus-off requires software to re-enter
  // initialisation mode and back; HAL_CAN_Stop/Start does exactly that.
  HAL_CAN_Stop(hcan_);
  HAL_CAN_Start(hcan_);
}

void Stm32Hal::watchdog_kick() {
  if (hiwdg_ != nullptr) HAL_IWDG_Refresh(hiwdg_);
}

}  // namespace bcm::hal

#pragma once
#include "bcm/hal/hal.hpp"

extern "C" {
#include "stm32_hal_stub.h"
}

namespace bcm::hal {

/// Pin assignment for one logical signal.
struct PinMap {
  GPIO_TypeDef* port;
  uint16_t pin;
  bool active_low;  ///< switches to ground with pull-up read low when active
};

const PinMap& din_map(Din d);
const PinMap& dout_map(Dout d);

/// IHal implementation on top of the STM32Cube HAL (GPIO, SysTick, bxCAN,
/// IWDG). On the host it links against port/stm32/stm32_hal_stub.c.
class Stm32Hal : public IHal {
 public:
  Stm32Hal(CAN_HandleTypeDef* hcan, IWDG_HandleTypeDef* hiwdg) : hcan_(hcan), hiwdg_(hiwdg) {}

  uint32_t tick_ms() const override { return HAL_GetTick(); }
  bool read(Din pin) const override;
  void write(Dout pin, bool on) override;
  bool can_transmit(const can::Frame& f) override;
  bool can_receive(can::Frame& f) override;
  CanState can_state() const override;
  void can_request_recovery() override;
  void watchdog_kick() override;

 private:
  CAN_HandleTypeDef* hcan_;
  IWDG_HandleTypeDef* hiwdg_;
};

}  // namespace bcm::hal

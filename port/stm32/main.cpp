// Target entry point (not built on the host). CubeMX-generated init code
// (clock tree, GPIO, bxCAN filters, IWDG) is assumed to provide the MX_*
// functions and handles below.
#include "stm32f4xx_hal.h"

#include "bcm/app/bcm.hpp"
#include "bcm/hal/stm32_hal.hpp"

extern CAN_HandleTypeDef hcan1;
extern IWDG_HandleTypeDef hiwdg;
extern "C" void SystemClock_Config(void);
extern "C" void MX_GPIO_Init(void);
extern "C" void MX_CAN1_Init(void);
extern "C" void MX_IWDG_Init(void);

int main() {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_IWDG_Init();
  HAL_CAN_Start(&hcan1);

  static bcm::hal::Stm32Hal board(&hcan1, &hiwdg);
  static bcm::app::Bcm bcm(board);

  uint32_t next = HAL_GetTick();
  for (;;) {
    // 10 ms cooperative super-loop; the IWDG (window ~50 ms) is refreshed
    // at the end of every completed step.
    if (static_cast<int32_t>(HAL_GetTick() - next) >= 0) {
      next += bcm::app::kTickMs;
      bcm.step();
    }
    __WFI();
  }
}

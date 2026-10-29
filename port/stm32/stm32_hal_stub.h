/*
 * Minimal stand-in for the STM32Cube F4 HAL headers (GPIO, SysTick, bxCAN,
 * IWDG). Type and function names follow the real HAL so that
 * src/hal/stm32_hal.cpp compiles unchanged against either. The register
 * structs only contain the fields used here. On target, replace this header
 * with "stm32f4xx_hal.h".
 */
#ifndef STM32_HAL_STUB_H
#define STM32_HAL_STUB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;

typedef struct {
  volatile uint32_t IDR;
  volatile uint32_t ODR;
} GPIO_TypeDef;

extern GPIO_TypeDef stub_gpio[5];
#define GPIOA (&stub_gpio[0])
#define GPIOB (&stub_gpio[1])
#define GPIOC (&stub_gpio[2])
#define GPIOD (&stub_gpio[3])
#define GPIOE (&stub_gpio[4])

#define GPIO_PIN_0 ((uint16_t)0x0001)
#define GPIO_PIN_1 ((uint16_t)0x0002)
#define GPIO_PIN_2 ((uint16_t)0x0004)
#define GPIO_PIN_3 ((uint16_t)0x0008)
#define GPIO_PIN_4 ((uint16_t)0x0010)
#define GPIO_PIN_5 ((uint16_t)0x0020)
#define GPIO_PIN_6 ((uint16_t)0x0040)
#define GPIO_PIN_7 ((uint16_t)0x0080)
#define GPIO_PIN_8 ((uint16_t)0x0100)
#define GPIO_PIN_9 ((uint16_t)0x0200)
#define GPIO_PIN_10 ((uint16_t)0x0400)
#define GPIO_PIN_11 ((uint16_t)0x0800)
#define GPIO_PIN_12 ((uint16_t)0x1000)
#define GPIO_PIN_13 ((uint16_t)0x2000)
#define GPIO_PIN_14 ((uint16_t)0x4000)
#define GPIO_PIN_15 ((uint16_t)0x8000)

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* port, uint16_t pin);
void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state);
uint32_t HAL_GetTick(void);

/* bxCAN */
typedef struct {
  volatile uint32_t ESR;
} CAN_TypeDef;
#define CAN_ESR_BOFF (0x1UL << 2)
#define CAN_ESR_EPVF (0x1UL << 1)

typedef struct {
  CAN_TypeDef* Instance;
} CAN_HandleTypeDef;

typedef struct {
  uint32_t StdId;
  uint32_t ExtId;
  uint32_t IDE;
  uint32_t RTR;
  uint32_t DLC;
  uint32_t TransmitGlobalTime;
} CAN_TxHeaderTypeDef;

typedef struct {
  uint32_t StdId;
  uint32_t ExtId;
  uint32_t IDE;
  uint32_t RTR;
  uint32_t DLC;
  uint32_t Timestamp;
  uint32_t FilterMatchIndex;
} CAN_RxHeaderTypeDef;

#define CAN_ID_STD 0x00000000U
#define CAN_ID_EXT 0x00000004U
#define CAN_RTR_DATA 0x00000000U
#define CAN_RX_FIFO0 0x00000000U

HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* hdr, uint8_t data[],
                                       uint32_t* mailbox);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef* hcan);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef* hcan, uint32_t fifo);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef* hcan, uint32_t fifo, CAN_RxHeaderTypeDef* hdr,
                                       uint8_t data[]);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef* hcan);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* hcan);

/* IWDG */
typedef struct {
  uint32_t reloads;
} IWDG_HandleTypeDef;
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef* hiwdg);

/* --- stub-only helpers for host tests --- */
void stub_reset(void);
void stub_set_tick(uint32_t t);
uint32_t stub_tx_count(void);
uint32_t stub_can_restarts(void);
void stub_push_rx(uint32_t id, const uint8_t data[8]);

#ifdef __cplusplus
}
#endif

#endif

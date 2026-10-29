/* Host-side stub implementation of the HAL subset in stm32_hal_stub.h.
 * GPIO ports are plain register structs; bxCAN has a 3-deep RX FIFO and
 * three TX mailboxes that complete instantly. */
#include "stm32_hal_stub.h"

#include <string.h>

GPIO_TypeDef stub_gpio[5];

static uint32_t g_tick;
static uint32_t g_tx_count;
static uint32_t g_restarts;

typedef struct {
  uint32_t id;
  uint8_t data[8];
} stub_frame_t;
static stub_frame_t g_rx[3];
static uint32_t g_rx_n;

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* port, uint16_t pin) {
  return (port->IDR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state) {
  if (state == GPIO_PIN_SET) {
    port->ODR |= pin;
  } else {
    port->ODR &= ~(uint32_t)pin;
  }
}

uint32_t HAL_GetTick(void) { return g_tick; }

HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* hdr, uint8_t data[],
                                       uint32_t* mailbox) {
  (void)hdr;
  (void)data;
  if (hcan->Instance->ESR & CAN_ESR_BOFF) return HAL_ERROR;
  *mailbox = 1;
  ++g_tx_count;
  return HAL_OK;
}

uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef* hcan) {
  return (hcan->Instance->ESR & CAN_ESR_BOFF) ? 0U : 3U;
}

uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef* hcan, uint32_t fifo) {
  (void)hcan;
  (void)fifo;
  return g_rx_n;
}

HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef* hcan, uint32_t fifo, CAN_RxHeaderTypeDef* hdr,
                                       uint8_t data[]) {
  (void)hcan;
  (void)fifo;
  if (g_rx_n == 0) return HAL_ERROR;
  memset(hdr, 0, sizeof(*hdr));
  hdr->StdId = g_rx[0].id;
  hdr->IDE = CAN_ID_STD;
  hdr->DLC = 8;
  memcpy(data, g_rx[0].data, 8);
  memmove(&g_rx[0], &g_rx[1], sizeof(stub_frame_t) * (g_rx_n - 1));
  --g_rx_n;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef* hcan) {
  (void)hcan;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef* hcan) {
  hcan->Instance->ESR &= ~CAN_ESR_BOFF;
  ++g_restarts;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef* hiwdg) {
  ++hiwdg->reloads;
  return HAL_OK;
}

void stub_reset(void) {
  memset(stub_gpio, 0, sizeof(stub_gpio));
  g_tick = 0;
  g_tx_count = 0;
  g_restarts = 0;
  g_rx_n = 0;
}

void stub_set_tick(uint32_t t) { g_tick = t; }
uint32_t stub_tx_count(void) { return g_tx_count; }
uint32_t stub_can_restarts(void) { return g_restarts; }

void stub_push_rx(uint32_t id, const uint8_t data[8]) {
  if (g_rx_n >= 3) return; /* FIFO overrun: frame lost, as on bxCAN */
  g_rx[g_rx_n].id = id;
  memcpy(g_rx[g_rx_n].data, data, 8);
  ++g_rx_n;
}

/**
 * @file    stm32f1xx_it.c
 * @brief   Interrupt handlers — SysTick drives control loop flag
 */
#include "main.h"
#include "stm32f1xx_hal.h"

extern PCD_HandleTypeDef hpcd_USB_FS;

/* ========================== Exported variables ========================== */
volatile uint8_t control_flag = 0; /* Set every 5ms for control loop */
volatile uint32_t systick_ms = 0;  /* Free-running ms counter */

static volatile uint8_t systick_divider = 0;

/* ========================== Cortex-M3 Handlers ========================== */

void NMI_Handler(void) {}

void HardFault_Handler(void) {
  while (1) {
  }
}

void MemManage_Handler(void) {
  while (1) {
  }
}

void BusFault_Handler(void) {
  while (1) {
  }
}

void UsageFault_Handler(void) {
  while (1) {
  }
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

/**
 * @brief  SysTick Handler — 1 kHz (1ms)
 *         Sets control_flag every 5 ticks (200 Hz = 5 ms)
 */
void SysTick_Handler(void) {
  HAL_IncTick();
  systick_ms++;

  systick_divider++;
  if (systick_divider >= (1000 / CONTROL_FREQ_HZ)) /* 5 ticks */
  {
    systick_divider = 0;
    control_flag = 1;
  }
}

/**
 * @brief  USB Low Priority / CAN RX0 interrupt
 */
void USB_LP_CAN1_RX0_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_FS); }

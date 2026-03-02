/**
 * @file    system_stm32f1xx.c
 * @brief   System clock configuration — 72 MHz from 8 MHz HSE + PLL
 */
#include "main.h"
#include "stm32f1xx_hal.h"

/* Global variables required by HAL */
uint32_t SystemCoreClock = 72000000UL; /* Updated by SystemClock_Config */

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0,
                                   1, 2, 3, 4, 6, 7, 8, 9};

const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

/**
 * @brief  System Clock Configuration
 *         HSE 8 MHz → PLL x9 → SYSCLK 72 MHz
 *         AHB = 72 MHz, APB1 = 36 MHz, APB2 = 72 MHz
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* HSE oscillator + PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; /* 8 MHz x 9 = 72 MHz */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* SYSCLK = 72 MHz, AHB = 72 MHz, APB1 = 36 MHz, APB2 = 72 MHz */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; /* Max 36 MHz */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief  Minimal SystemInit (called from startup before main)
 */
void SystemInit(void) {
  /* Nothing needed — HAL_Init() + SystemClock_Config() handle everything */
}

/**
 * @brief  Error handler — infinite loop with LED blink
 */
void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

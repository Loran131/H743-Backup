#include "quadspi.h"

QSPI_HandleTypeDef hqspi;

void MX_QUADSPI_Init(void)
{
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK) Error_Handler();
}

void HAL_QSPI_MspInit(QSPI_HandleTypeDef *handle)
{
  GPIO_InitTypeDef gpio = {0};
  RCC_PeriphCLKInitTypeDef clock = {0};
  if (handle->Instance != QUADSPI) return;

  clock.PeriphClockSelection = RCC_PERIPHCLK_QSPI;
  clock.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) Error_Handler();
  __HAL_RCC_QSPI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOF, &gpio);
  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  gpio.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOF, &gpio);
  gpio.Pin = GPIO_PIN_2;
  gpio.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Pin = GPIO_PIN_6;
  gpio.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOB, &gpio);
}

void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef *handle)
{
  if (handle->Instance != QUADSPI) return;
  __HAL_RCC_QSPI_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOF, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2 | GPIO_PIN_6);
}

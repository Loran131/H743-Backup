/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

#include "c552.h"
#include "z_axis_link.h"
#include <string.h>

static uint8_t g_usart3_rx_dma[USART3_RX_DMA_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
static volatile uint16_t g_usart3_rx_dma_position;
static volatile uint8_t g_usart3_rx_restart_pending;
static uint8_t g_usart3_rx_failure_recorded;
static uint32_t g_usart3_rx_last_restart_tick;

#define UART4_RX_DMA_SIZE 64U
static uint8_t g_uart4_rx_dma[UART4_RX_DMA_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
static volatile uint16_t g_uart4_rx_dma_position;
static volatile uint8_t g_uart4_rx_restart_pending;
static uint32_t g_uart4_rx_last_restart_tick;

#define CLI_BUF_SIZE 256U
static uint8_t g_cli_rx_buf[CLI_BUF_SIZE];
static uint8_t g_cli_line_buf[CLI_BUF_SIZE];
static volatile uint16_t g_cli_rx_state;
static uint8_t g_uart1_rx_byte;

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

/* UART4 init function */
void MX_UART4_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK ||
      HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK ||
      HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_uart4_rx.Instance = DMA1_Stream1;
    hdma_uart4_rx.Init.Request = DMA_REQUEST_UART4_RX;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_uart4_rx);

    hdma_uart4_tx.Instance = DMA1_Stream2;
    hdma_uart4_tx.Init.Request = DMA_REQUEST_UART4_TX;
    hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_tx.Init.Mode = DMA_NORMAL;
    hdma_uart4_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_uart4_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart4_tx) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(uartHandle, hdmatx, hdma_uart4_tx);

    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_usart3_rx.Instance = DMA1_Stream0;
    hdma_usart3_rx.Init.Request = DMA_REQUEST_USART3_RX;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart3_rx);

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
    __HAL_RCC_UART4_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Stream2_IRQn);
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

  /* USER CODE BEGIN USART3_MspDeInit 1 */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

int __io_putchar(int ch)
{
  while ((USART1->ISR & USART_ISR_TXE_TXFNF) == 0U) {}
  USART1->TDR = (uint8_t)ch;
  return ch;
}

int __io_getchar(void)
{
  while ((USART1->ISR & USART_ISR_RXNE_RXFNE) == 0U) {}
  return (int)(USART1->RDR & 0xFFU);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint8_t ch = g_uart1_rx_byte;
    uint16_t count = (uint16_t)(g_cli_rx_state & 0x3FFFU);

    if ((g_cli_rx_state & 0x8000U) == 0U)
    {
      if (ch == '\r')
      {
        g_cli_rx_state |= 0x4000U;
      }
      else if (ch == '\n')
      {
        g_cli_rx_state |= 0x8000U;
      }
      else if (count < (CLI_BUF_SIZE - 1U))
      {
        g_cli_rx_buf[count++] = ch;
        g_cli_rx_state = (uint16_t)((g_cli_rx_state & 0xC000U) | count);
      }
      else
      {
        g_cli_rx_state = 0U;
      }
    }
    (void)HAL_UART_Receive_IT(huart, &g_uart1_rx_byte, 1U);
  }
}

static void usart3_process_dma_position(uint16_t position)
{
  uint16_t previous = g_usart3_rx_dma_position;
  uint32_t now = HAL_GetTick();

  if (position > USART3_RX_DMA_SIZE)
  {
    C552_RecordDmaRestartFailure();
    return;
  }
  if (position > previous)
  {
    C552_ProcessBytes(&g_usart3_rx_dma[previous],
                      (uint16_t)(position - previous), now);
  }
  else if (position < previous)
  {
    C552_ProcessBytes(&g_usart3_rx_dma[previous],
                      (uint16_t)(USART3_RX_DMA_SIZE - previous), now);
    if (position > 0U)
    {
      C552_ProcessBytes(g_usart3_rx_dma, position, now);
    }
  }
  g_usart3_rx_dma_position =
      (position == USART3_RX_DMA_SIZE) ? 0U : position;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart->Instance == USART3)
  {
    usart3_process_dma_position(size);
  }
  else if (huart->Instance == UART4)
  {
    uint16_t previous = g_uart4_rx_dma_position;
    uint32_t now = HAL_GetTick();
    if (size > UART4_RX_DMA_SIZE)
    {
      ZAxisLink_OnUartError(HAL_UART_ERROR_DMA, now);
      return;
    }
    if (size > previous)
    {
      ZAxisLink_ProcessBytes(&g_uart4_rx_dma[previous],
                             (uint16_t)(size - previous), now);
    }
    else if (size < previous)
    {
      ZAxisLink_ProcessBytes(&g_uart4_rx_dma[previous],
                             (uint16_t)(UART4_RX_DMA_SIZE - previous), now);
      if (size > 0U)
      {
        ZAxisLink_ProcessBytes(g_uart4_rx_dma, size, now);
      }
    }
    g_uart4_rx_dma_position =
        (size == UART4_RX_DMA_SIZE) ? 0U : size;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    ZAxisLink_OnUartError(huart->ErrorCode, HAL_GetTick());
    g_uart4_rx_dma_position = 0U;
    g_uart4_rx_restart_pending = 1U;
    g_uart4_rx_last_restart_tick = HAL_GetTick();
    return;
  }
  if (huart->Instance != USART3) return;
  C552_RecordUartError(huart->ErrorCode);
  C552_ResetStream();
  g_usart3_rx_dma_position = 0U;
  g_usart3_rx_restart_pending = 1U;
  g_usart3_rx_failure_recorded = 0U;
  g_usart3_rx_last_restart_tick = HAL_GetTick();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    C552_OnTxComplete(HAL_GetTick());
  }
}

HAL_StatusTypeDef USART3_TransmitAsync(const uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U)) return HAL_ERROR;
  return HAL_UART_Transmit_IT(&huart3, data, length);
}

HAL_StatusTypeDef UART4_TransmitDMA(const uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U)) return HAL_ERROR;
  return HAL_UART_Transmit_DMA(&huart4, data, length);
}

HAL_StatusTypeDef UART4_StartRx(void)
{
  HAL_StatusTypeDef status;
  g_uart4_rx_dma_position = 0U;
  g_uart4_rx_restart_pending = 0U;
  ZAxisLink_ResetStream();
  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart4, g_uart4_rx_dma,
                                        UART4_RX_DMA_SIZE);
  if (status == HAL_OK) ZAxisLink_SetRxReady(1U);
  if (status != HAL_OK)
  {
    g_uart4_rx_restart_pending = 1U;
    g_uart4_rx_last_restart_tick = HAL_GetTick();
    ZAxisLink_OnUartError(HAL_UART_ERROR_DMA, HAL_GetTick());
  }
  return status;
}

void UART4_RxPoll(void)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t now;

  if (g_uart4_rx_restart_pending == 0U) return;
  now = HAL_GetTick();
  if ((uint32_t)(now - g_uart4_rx_last_restart_tick) < 10U) return;
  g_uart4_rx_last_restart_tick = now;

  HAL_NVIC_DisableIRQ(UART4_IRQn);
  HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
  (void)HAL_UART_AbortReceive(&huart4);
  if (hdma_uart4_rx.State == HAL_DMA_STATE_BUSY)
  {
    (void)HAL_DMA_Abort(&hdma_uart4_rx);
  }
  if (hdma_uart4_rx.State != HAL_DMA_STATE_READY)
  {
    (void)HAL_DMA_DeInit(&hdma_uart4_rx);
    status = HAL_DMA_Init(&hdma_uart4_rx);
    if (status == HAL_OK) __HAL_LINKDMA(&huart4, hdmarx, hdma_uart4_rx);
  }
  if (status == HAL_OK)
  {
    __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_OREF | UART_CLEAR_NEF |
                          UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_IDLEF);
    __HAL_UART_SEND_REQ(&huart4, UART_RXDATA_FLUSH_REQUEST);
    g_uart4_rx_dma_position = 0U;
    ZAxisLink_ResetStream();
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart4, g_uart4_rx_dma,
                                          UART4_RX_DMA_SIZE);
  }
  HAL_NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn);
  HAL_NVIC_ClearPendingIRQ(UART4_IRQn);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
  if (status == HAL_OK)
  {
    g_uart4_rx_restart_pending = 0U;
    ZAxisLink_SetRxReady(1U);
  }
}

HAL_StatusTypeDef USART3_StartRx(void)
{
  HAL_StatusTypeDef status;
  g_usart3_rx_dma_position = 0U;
  g_usart3_rx_restart_pending = 0U;
  g_usart3_rx_failure_recorded = 0U;
  C552_ResetStream();
  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_usart3_rx_dma,
                                        USART3_RX_DMA_SIZE);
  if (status != HAL_OK)
  {
    C552_RecordDmaRestartFailure();
    g_usart3_rx_restart_pending = 1U;
    g_usart3_rx_failure_recorded = 1U;
    g_usart3_rx_last_restart_tick = HAL_GetTick();
  }
  return status;
}

void USART3_RxPoll(void)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t now;

  if (g_usart3_rx_restart_pending == 0U)
  {
    return;
  }

  now = HAL_GetTick();
  if ((uint32_t)(now - g_usart3_rx_last_restart_tick) < 10U) return;
  g_usart3_rx_last_restart_tick = now;

  HAL_NVIC_DisableIRQ(USART3_IRQn);
  HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
  (void)HAL_UART_AbortReceive(&huart3);
  if (hdma_usart3_rx.State == HAL_DMA_STATE_BUSY)
  {
    (void)HAL_DMA_Abort(&hdma_usart3_rx);
  }
  if (hdma_usart3_rx.State != HAL_DMA_STATE_READY)
  {
    (void)HAL_DMA_DeInit(&hdma_usart3_rx);
    status = HAL_DMA_Init(&hdma_usart3_rx);
    if (status == HAL_OK) __HAL_LINKDMA(&huart3, hdmarx, hdma_usart3_rx);
  }
  if (status == HAL_OK)
  {
    __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_OREF | UART_CLEAR_NEF |
                          UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_IDLEF);
    __HAL_UART_SEND_REQ(&huart3, UART_RXDATA_FLUSH_REQUEST);
    g_usart3_rx_dma_position = 0U;
    C552_ResetStream();
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_usart3_rx_dma,
                                          USART3_RX_DMA_SIZE);
  }
  HAL_NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

  if (status == HAL_OK)
  {
    g_usart3_rx_restart_pending = 0U;
    g_usart3_rx_failure_recorded = 0U;
  }
  else if (g_usart3_rx_failure_recorded == 0U)
  {
    C552_RecordDmaRestartFailure();
    g_usart3_rx_failure_recorded = 1U;
  }
}

void CLI_StartRx(void)
{
  g_cli_rx_state = 0U;
  memset(g_cli_rx_buf, 0, sizeof(g_cli_rx_buf));
  memset(g_cli_line_buf, 0, sizeof(g_cli_line_buf));
  (void)HAL_UART_Receive_IT(&huart1, &g_uart1_rx_byte, 1U);
}

char *CLI_GetLine(void)
{
  uint16_t length;
  uint32_t primask;
  if ((g_cli_rx_state & 0x8000U) == 0U) return NULL;

  primask = __get_PRIMASK();
  __disable_irq();
  length = (uint16_t)(g_cli_rx_state & 0x3FFFU);
  memcpy(g_cli_line_buf, g_cli_rx_buf, length);
  g_cli_line_buf[length] = '\0';
  g_cli_rx_state = 0U;
  if (primask == 0U) __enable_irq();
  return (char *)g_cli_line_buf;
}

uint8_t CLI_TakeAbortLine(void)
{
  uint16_t length;
  uint16_t begin = 0U;
  uint16_t end;
  uint32_t primask;
  uint8_t is_abort = 0U;
  if ((g_cli_rx_state & 0x8000U) == 0U) return 0U;

  primask = __get_PRIMASK();
  __disable_irq();
  length = (uint16_t)(g_cli_rx_state & 0x3FFFU);
  end = length;
  while ((begin < end) && ((g_cli_rx_buf[begin] == ' ') ||
         (g_cli_rx_buf[begin] == '\t'))) ++begin;
  while ((end > begin) && ((g_cli_rx_buf[end - 1U] == ' ') ||
         (g_cli_rx_buf[end - 1U] == '\t'))) --end;
  if ((end - begin) == 5U) {
    static const char abort_word[] = "abort";
    uint16_t index;
    is_abort = 1U;
    for (index = 0U; index < 5U; ++index) {
      uint8_t ch = g_cli_rx_buf[begin + index];
      if ((ch >= 'A') && (ch <= 'Z')) ch = (uint8_t)(ch + ('a' - 'A'));
      if (ch != (uint8_t)abort_word[index]) {
        is_abort = 0U;
        break;
      }
    }
  }
  if (is_abort != 0U) g_cli_rx_state = 0U;
  if (primask == 0U) __enable_irq();
  return is_abort;
}

/* USER CODE END 1 */

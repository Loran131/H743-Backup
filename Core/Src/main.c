/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "eth.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BENCHMARK_LOOPS  1000U
#define STEPPER_MOVE_PULSES   6400U
#define KEY_DEBOUNCE_MS       50U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint8_t flag_music = 0U;
static volatile uint32_t stepper_pulse_count = 0U;
static volatile uint8_t stepper_busy = 0U;
static volatile int8_t stepper_move_request = 0;

static uint32_t key2_last_tick = 0U;
static uint32_t key3_last_tick = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
  uint8_t data = (uint8_t)ch;
  HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY);
  return ch;
}

static void GetDeviceID(uint32_t id[3])
{
  id[0] = HAL_GetUIDw0();
  id[1] = HAL_GetUIDw1();
  id[2] = HAL_GetUIDw2();
}

static void DWT_CycleCounterEnable(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void PrintTiming(const char *name, uint32_t cycles)
{
  uint32_t total_us;
  uint32_t average_cycles;
  uint32_t average_ns;

  total_us = (uint32_t)(((uint64_t)cycles * 1000000ULL) / SystemCoreClock);
  average_cycles = cycles / BENCHMARK_LOOPS;
  average_ns = (uint32_t)(((uint64_t)cycles * 1000000000ULL) /
                          ((uint64_t)BENCHMARK_LOOPS * SystemCoreClock));

  printf("%s: total=%lu cycles (%lu us), avg=%lu cycles/op (%lu ns/op)\r\n",
         name, (unsigned long)cycles, (unsigned long)total_us,
         (unsigned long)average_cycles, (unsigned long)average_ns);
}

static void RunMathBenchmark(void)
{
  volatile uint32_t integer_result = 1U;
  volatile float float_result = 1.0f;
  volatile float trig_result = 0.0f;
  uint32_t start_cycles;
  uint32_t cycles;
  uint32_t i;

  start_cycles = DWT->CYCCNT;
  for (i = 0U; i < BENCHMARK_LOOPS; i++)
  {
    integer_result = integer_result * 3U + i;
    integer_result ^= integer_result >> 7U;
  }
  cycles = DWT->CYCCNT - start_cycles;
  PrintTiming("Integer arithmetic", cycles);

  start_cycles = DWT->CYCCNT;
  for (i = 0U; i < BENCHMARK_LOOPS; i++)
  {
    float_result = (float_result * 1.0001f + 0.1234f) / 1.00001f;
  }
  cycles = DWT->CYCCNT - start_cycles;
  PrintTiming("Float arithmetic", cycles);

  start_cycles = DWT->CYCCNT;
  for (i = 0U; i < BENCHMARK_LOOPS; i++)
  {
    float angle = (float)i * 0.001f;
    trig_result = sinf(angle) + cosf(angle) + tanf(angle * 0.1f);
  }
  cycles = DWT->CYCCNT - start_cycles;
  PrintTiming("Trigonometric functions", cycles);

  /* Prevents the compiler from removing calculations. */
  (void)integer_result;
  (void)float_result;
  (void)trig_result;
}

typedef struct
{
  uint16_t frequency;   // 频率，0 表示休止
  uint16_t duration_ms; // 时长
} Note_t;

#define NOTE_C5  523U
#define NOTE_D5  587U
#define NOTE_E5  659U
#define NOTE_F5  698U
#define NOTE_G5  784U
#define NOTE_A5  880U
#define NOTE_C6  1047U

static void Buzzer_PlayTone(uint16_t frequency, uint16_t duration_ms)
{
  uint32_t period;
  uint16_t sound_ms;

  if (frequency == 0U)
  {
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
    HAL_Delay(duration_ms);
    return;
  }

  /* TIM4 已被 CubeMX 配为：240 MHz / (239 + 1) = 1 MHz */
  period = 1000000U / frequency - 1U;

  __HAL_TIM_SET_AUTORELOAD(&htim4, period);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (period + 1U) / 2U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  HAL_TIM_GenerateEvent(&htim4, TIM_EVENTSOURCE_UPDATE);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

  /* 留出约 10% 的停顿，让每个音更清晰。 */
  sound_ms = (uint16_t)(duration_ms * 9U / 10U);
  HAL_Delay(sound_ms);

  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
  HAL_Delay(duration_ms - sound_ms);
}

static void Buzzer_PlayTwoTigers(void)
{
  static const Note_t melody[] =
  {
    {NOTE_C5, 300}, {NOTE_D5, 300}, {NOTE_E5, 300}, {NOTE_C5, 300},
    {NOTE_C5, 300}, {NOTE_D5, 300}, {NOTE_E5, 300}, {NOTE_C5, 300},

    {NOTE_E5, 300}, {NOTE_F5, 300}, {NOTE_G5, 600},
    {NOTE_E5, 300}, {NOTE_F5, 300}, {NOTE_G5, 600},

    {NOTE_G5, 220}, {NOTE_A5, 220}, {NOTE_G5, 220},
    {NOTE_F5, 220}, {NOTE_E5, 300}, {NOTE_C5, 300},

    {NOTE_G5, 220}, {NOTE_A5, 220}, {NOTE_G5, 220},
    {NOTE_F5, 220}, {NOTE_E5, 300}, {NOTE_C5, 300},

    {NOTE_C5, 300}, {NOTE_G5, 300}, {NOTE_C6, 600},
    {NOTE_C5, 300}, {NOTE_G5, 300}, {NOTE_C6, 600},
  };

  for (uint32_t i = 0U; i < sizeof(melody) / sizeof(melody[0]); i++)
  {
    Buzzer_PlayTone(melody[i].frequency, melody[i].duration_ms);
  }
}

static void Stepper_StartMove(GPIO_PinState direction)
{
  stepper_busy = 1U;
  stepper_pulse_count = 0U;

  /* 设置方向：按键 2 为正向，按键 3 为反向。若实际方向相反，交换 SET/RESET。 */
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, direction);

  /* DIR 改变后再发 STEP，留出充足建立时间。 */
  HAL_Delay(1U);

  /* 从一个完整的新 PWM 周期开始计数。 */
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1);

  /*
   * 使用带中断的 PWM。
   * 每个 PWM 周期产生一个 STEP 上升沿，随后在 CC1 处产生一次回调。
   */
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_ETH_Init();
  /* USER CODE BEGIN 2 */
  uint32_t device_id[3];

  SystemCoreClockUpdate();
  DWT_CycleCounterEnable();
  GetDeviceID(device_id);

  printf("\r\nDevice ID: %08lX-%08lX-%08lX\r\n",
         (unsigned long)device_id[0], (unsigned long)device_id[1],
         (unsigned long)device_id[2]);
  printf("System clock: %lu Hz\r\n", (unsigned long)SystemCoreClock);
  RunMathBenchmark();
  // Buzzer_PlayTwoTigers();
  flag_music = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  HAL_Delay(500U);
  HAL_StatusTypeDef status_id1;
  HAL_StatusTypeDef status_id2;
  HAL_StatusTypeDef status_bmsr;
  uint32_t phy_id1 = 0xDEADBEEFU;
  uint32_t phy_id2 = 0xDEADBEEFU;
  uint32_t bmsr = 0xDEADBEEFU;

  status_id1 = HAL_ETH_ReadPHYRegister(&heth, 0U, 2U, &phy_id1);
  status_id2 = HAL_ETH_ReadPHYRegister(&heth, 0U, 3U, &phy_id2);
  status_bmsr = HAL_ETH_ReadPHYRegister(&heth, 0U, 1U, &bmsr);

  printf("MDIO status: ID1=%d ID2=%d BMSR=%d\r\n",
         status_id1, status_id2, status_bmsr);
  printf("PHY ID: %08lX %08lX, BMSR=%08lX\r\n",
         phy_id1, phy_id2, bmsr);
  //
  //   if ((stepper_busy == 0U) && (stepper_move_request != 0))
  //   {
  //     int8_t request;
  //
  //     __disable_irq();
  //     request = stepper_move_request;
  //     stepper_move_request = 0;
  //     __enable_irq();
  //
  //     if (request > 0)
  //     {
  //       Stepper_StartMove(GPIO_PIN_SET);    /* 按键 2：正向 */
  //     }
  //     else
  //     {
  //       Stepper_StartMove(GPIO_PIN_RESET);  /* 按键 3：反向 */
  //     }
  //   }
  //   if (flag_music == 1) {
  //     Buzzer_PlayTwoTigers();
  //     HAL_Delay(1000);
  //     flag_music = 0;
  //     HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  //   }
  //
    // MX_LWIP_Process();

    HAL_Delay(5000U);
    //
    // HAL_StatusTypeDef status_id1;
    // HAL_StatusTypeDef status_id2;
    // HAL_StatusTypeDef status_bmsr;
    // uint32_t phy_id1 = 0xDEADBEEFU;
    // uint32_t phy_id2 = 0xDEADBEEFU;
    // uint32_t bmsr = 0xDEADBEEFU;
    //
    // status_id1 = HAL_ETH_ReadPHYRegister(&heth, 0U, 2U, &phy_id1);
    // status_id2 = HAL_ETH_ReadPHYRegister(&heth, 0U, 3U, &phy_id2);
    // status_bmsr = HAL_ETH_ReadPHYRegister(&heth, 0U, 1U, &bmsr);
    //
    // printf("MDIO status: ID1=%d ID2=%d BMSR=%d\r\n",
    //        status_id1, status_id2, status_bmsr);
    // printf("PHY ID: %08lX %08lX, BMSR=%08lX\r\n",
    //        phy_id1, phy_id2, bmsr);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t now = HAL_GetTick();

  if (GPIO_Pin == KEY1_Pin && flag_music == 0U)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    flag_music = 1U;
  }
  else if (GPIO_Pin == KEY2_Pin)
  {
    if ((stepper_busy == 0U) &&
        (stepper_move_request == 0) &&
        ((now - key2_last_tick) >= KEY_DEBOUNCE_MS))
    {
      key2_last_tick = now;
      stepper_move_request = 1;       /* 请求正向 */
      HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    }
  }
  else if (GPIO_Pin == KEY3_Pin)
  {
    if ((stepper_busy == 0U) &&
        (stepper_move_request == 0) &&
        ((now - key3_last_tick) >= KEY_DEBOUNCE_MS))
    {
      key3_last_tick = now;
      stepper_move_request = -1;      /* 请求反向 */
      HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    }
  }
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    stepper_pulse_count++;

    if (stepper_pulse_count >= STEPPER_MOVE_PULSES)
    {
      /*
       * CC1 发生在当前 PWM 高电平结束后，
       * 因而停止时最后一个 STEP 脉冲已完整输出。
       */
      HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
      stepper_busy = 0U;
    }
  }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

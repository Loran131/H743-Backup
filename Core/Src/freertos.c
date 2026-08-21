/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "c552.h"
#include "fdcan.h"
#include "iwdg.h"
#include "led.h"
#include "motion_coordinator.h"
#include "mission_subflow.h"
#include "mission_task.h"
#include "remote_control.h"
#include "shell.h"
#include "smd.h"
#include "tcp_control_server.h"
#include "usart.h"
#include "vision_calibration.h"
#include "xy_motor.h"
#include "xy_vision_align.h"
#include "z_axis_link.h"
#include "z_axis.h"
#include "xz_vision_calibration.h"
#include "xz_vision_align.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LWIP_READY_FLAG          (1U)
#define HEALTH_TASK_COUNT        (3U)
#define HEALTH_DEFAULT_TASK      (0U)
#define HEALTH_LEGACY_IO_TASK    (1U)
#define HEALTH_SHELL_TASK        (2U)
#define HEALTH_MAX_AGE_MS        (1000U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static osEventFlagsId_t lwipReadyEventHandle;
static osMutexId_t canAccessMutexHandle;
static volatile uint32_t criticalTaskHeartbeat[HEALTH_TASK_COUNT];
static volatile uint8_t criticalTaskSeen[HEALTH_TASK_COUNT];

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SocketTask */
osThreadId_t SocketTaskHandle;
const osThreadAttr_t SocketTask_attributes = {
  .name = "SocketTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t legacyIoTaskHandle;
const osThreadAttr_t legacyIoTask_attributes = {
  .name = "LegacyIoTask",
  .stack_size = 2048,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
osThreadId_t shellTaskHandle;
const osThreadAttr_t shellTask_attributes = {
  .name = "ShellTask",
  .stack_size = 3072,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t monitorTaskHandle;
const osThreadAttr_t monitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 768,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void StartLegacyIoTask(void *argument);
static void StartShellTask(void *argument);
static void StartMonitorTask(void *argument);
static void CriticalTaskBeat(uint8_t task_index);
static uint8_t CriticalTasksHealthy(uint32_t now);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
   (void)xTask;
   (void)pcTaskName;
   Error_Handler();
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
   Error_Handler();
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  lwipReadyEventHandle = osEventFlagsNew(NULL);
  canAccessMutexHandle = osMutexNew(NULL);
  if ((lwipReadyEventHandle == NULL) || (canAccessMutexHandle == NULL))
  {
    Error_Handler();
  }
  XY_Motor_Init(HAL_GetTick());
  ZAxis_Init(HAL_GetTick());
  XZCalibration_Init(HAL_GetTick());
  VisionCalibration_Init(HAL_GetTick());
  XY_VisionAlign_Init(HAL_GetTick());
  XZVisionAlign_Init(HAL_GetTick());
  MotionCoordinator_Init(HAL_GetTick());
  MissionSubflow_Init(HAL_GetTick());
  MissionTask_Init(HAL_GetTick());
  RemoteControl_Init();

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SocketTask */
  SocketTaskHandle = osThreadNew(StartTask02, NULL, &SocketTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  legacyIoTaskHandle = osThreadNew(StartLegacyIoTask, NULL,
                                   &legacyIoTask_attributes);
  shellTaskHandle = osThreadNew(StartShellTask, NULL, &shellTask_attributes);
  monitorTaskHandle = osThreadNew(StartMonitorTask, NULL,
                                  &monitorTask_attributes);
  if ((legacyIoTaskHandle == NULL) || (shellTaskHandle == NULL) ||
      (monitorTaskHandle == NULL))
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */

  (void)osEventFlagsSet(lwipReadyEventHandle, LWIP_READY_FLAG);

  /* Infinite loop */
  for(;;)
  {
    CriticalTaskBeat(HEALTH_DEFAULT_TASK);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the SocketTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */

  uint32_t flags;

  flags = osEventFlagsWait(lwipReadyEventHandle, LWIP_READY_FLAG,
                           osFlagsWaitAny, osWaitForever);
  if ((flags & osFlagsError) != 0U)
  {
    Error_Handler();
  }

  TcpControlServer_Run();

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static void StartLegacyIoTask(void *argument)
{
  (void)argument;
  for (;;)
  {
    USART3_RxPoll();
    C552_Poll(HAL_GetTick());
    USART6_RxPoll();
    ZAxisLink_Poll(HAL_GetTick());
    Shell_PollEmergency();
    MotionCoordinator_PollEmergency(HAL_GetTick());

    if (osMutexAcquire(canAccessMutexHandle, 0U) == osOK)
    {
      RemoteControl_Poll(HAL_GetTick());
      ZAxis_Poll(HAL_GetTick());
      XZCalibration_Poll(HAL_GetTick());
      can_rx_timeout_check();
      can_recovery_poll();
      if (g_can_rx_frame.frame_done != 0U)
      {
        uint8_t response[CAN_RX_BUF_SIZE];
        uint16_t response_length;
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        response_length = g_can_rx_frame.len;
        if (response_length > CAN_RX_BUF_SIZE) response_length = CAN_RX_BUF_SIZE;
        memcpy(response, g_can_rx_frame.buf, response_length);
        g_can_rx_frame.len = 0U;
        g_can_rx_frame.frame_done = 0U;
        if (primask == 0U) __enable_irq();
        smd_process_response(response, response_length);
      }
      XY_Motor_Poll(HAL_GetTick());
      MotionCoordinator_Poll(HAL_GetTick());
      MissionSubflow_Poll(HAL_GetTick());
      MissionTask_Poll(HAL_GetTick());
      VisionCalibration_Poll(HAL_GetTick());
      XY_VisionAlign_Poll(HAL_GetTick());
      XZVisionAlign_Poll(HAL_GetTick());
      (void)osMutexRelease(canAccessMutexHandle);
    }
    CriticalTaskBeat(HEALTH_LEGACY_IO_TASK);
    osDelay(1U);
  }
}

static void StartShellTask(void *argument)
{
  (void)argument;
  Shell_Init();
  for (;;)
  {
    Shell_PollEmergency();
    if (osMutexAcquire(canAccessMutexHandle, 0U) == osOK)
    {
      Shell_Poll();
      (void)osMutexRelease(canAccessMutexHandle);
    }
    CriticalTaskBeat(HEALTH_SHELL_TASK);
    osDelay(2U);
  }
}

static void StartMonitorTask(void *argument)
{
  uint8_t watchdog_started = 0U;
  (void)argument;
  for (;;)
  {
    LED_Toggle(LED_ID_0);
    if (CriticalTasksHealthy(HAL_GetTick()) != 0U)
    {
      if (watchdog_started == 0U)
      {
        if (IWDG_Init() != HAL_OK) Error_Handler();
        watchdog_started = 1U;
        printf("IWDG: health-gated watchdog started\r\n");
      }
      else
      {
        (void)IWDG_Refresh();
      }
    }
    osDelay(250U);
  }
}

static void CriticalTaskBeat(uint8_t task_index)
{
  if (task_index >= HEALTH_TASK_COUNT) return;
  criticalTaskHeartbeat[task_index] = HAL_GetTick();
  criticalTaskSeen[task_index] = 1U;
}

static uint8_t CriticalTasksHealthy(uint32_t now)
{
  uint8_t task_index;
  for (task_index = 0U; task_index < HEALTH_TASK_COUNT; ++task_index)
  {
    if ((criticalTaskSeen[task_index] == 0U) ||
        ((uint32_t)(now - criticalTaskHeartbeat[task_index]) >
         HEALTH_MAX_AGE_MS))
    {
      return 0U;
    }
  }
  return 1U;
}

/* USER CODE END Application */


#include "iwdg.h"

static IWDG_HandleTypeDef hiwdg1;

HAL_StatusTypeDef IWDG_Init(void)
{
  hiwdg1.Instance = IWDG1;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg1.Init.Reload = 249U;
  hiwdg1.Init.Window = IWDG_WINDOW_DISABLE;
  return HAL_IWDG_Init(&hiwdg1);
}

HAL_StatusTypeDef IWDG_Refresh(void)
{
  return HAL_IWDG_Refresh(&hiwdg1);
}

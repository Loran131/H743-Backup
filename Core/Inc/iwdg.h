#ifndef __IWDG_H__
#define __IWDG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

HAL_StatusTypeDef IWDG_Init(void);
HAL_StatusTypeDef IWDG_Refresh(void);

#ifdef __cplusplus
}
#endif
#endif

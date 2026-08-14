/**
 ******************************************************************************
 * @file    shell.h
 * @brief   Serial command-line shell for PD42S1 motor control
 ******************************************************************************
 */

#ifndef __SHELL_H__
#define __SHELL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported functions --------------------------------------------------------*/
void Shell_Init(void);
void Shell_PollEmergency(void); /* Parse lock-free ABORT publication only. */
void Shell_Poll(void);   /* Call in main loop */

#ifdef __cplusplus
}
#endif

#endif /* __SHELL_H__ */

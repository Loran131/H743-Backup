/**
 ******************************************************************************
 * @file    fdcan.h
 * @brief   FDCAN driver header with CAN transport for PD42S1 motor protocol
 ******************************************************************************
 */

#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdint.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan2;

/* CAN timing constants ------------------------------------------------------*/
#define CAN_INTERNAL_LOOPBACK_TEST  0U

/* 125 kbps: 25MHz HSE / 10 / (1+16+3) = 125k */
#define CAN_NOMINAL_PRESCALER   10
#define CAN_NOMINAL_TIME_SEG1   16
#define CAN_NOMINAL_TIME_SEG2   3
#define CAN_NOMINAL_SJW         1

/* CAN ID */
#define CAN_EXTID_DEFAULT       0x1000   /* Default motor driver CAN ID (29-bit) */

/* RX reassembly timeout (ms) — no CAN message for this long → frame complete */
#define CAN_RX_TIMEOUT_MS       5

/* Fault recovery timing. Warning/passive states remain operational; after the
 * grace period only a stuck TX request is cancelled, without restarting CAN. */
#define CAN_BUS_OFF_RECOVERY_DELAY_MS  100U
#define CAN_ERROR_STATE_GRACE_MS       500U
#define CAN_RECOVERY_RETRY_MS          1000U

/* RX frame buffer size */
#define CAN_RX_BUF_SIZE         512

/* ====================== RX FRAME REASSEMBLY BUFFER ======================== */

typedef struct {
    uint8_t  buf[CAN_RX_BUF_SIZE];
    uint16_t len;
    uint8_t  frame_done;       /* 1 = complete frame ready for processing */
    uint32_t last_rx_tick;     /* SysTick of last received CAN message */
} CanRxFrame_t;

extern CanRxFrame_t g_can_rx_frame;

typedef struct {
    volatile uint32_t bus_off_count;
    volatile uint32_t error_passive_count;
    volatile uint32_t error_warning_count;
    volatile uint32_t protocol_error_count;
    volatile uint32_t rx_lost_count;
    volatile uint32_t rx_frame_count;
    volatile uint32_t last_rx_id;
    volatile uint32_t last_rx_dlc;
    volatile uint32_t last_rx_id_type;
    volatile uint32_t tx_enqueue_fail_count;
    volatile uint32_t recovery_count;
    volatile uint32_t recovery_fail_count;
    volatile uint32_t last_error_events;
} CanDiagnostics_t;

extern CanDiagnostics_t g_can_diagnostics;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* Function prototypes -------------------------------------------------------*/
void MX_FDCAN2_Init(void);

/* CAN transport API */
uint8_t can_send_msg(uint32_t ext_id, const uint8_t *data, uint8_t len);
uint8_t can_send_long_msg(uint32_t ext_id, const uint8_t *data, uint16_t len);
void    can_rx_timeout_check(void);   /* Call in main loop */
void    can_recovery_poll(void);      /* Call in main loop */
void    can_print_status(void);
void    can_set_filter_id(uint32_t ext_id);
void    can_set_filter_mask(uint32_t ext_id, uint32_t mask);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

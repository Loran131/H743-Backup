#ifndef Z_AXIS_LINK_H
#define Z_AXIS_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define Z_AXIS_FRAME_SIZE       17U
#define Z_AXIS_MIN_SPEED_HZ     1U
#define Z_AXIS_MAX_SPEED_HZ     100000U

typedef enum {
    Z_AXIS_REQUEST_OK = 0,
    Z_AXIS_REQUEST_BUSY,
    Z_AXIS_REQUEST_INVALID_ARGUMENT,
    Z_AXIS_REQUEST_IO_ERROR
} ZAxisRequestResult;

typedef enum {
    Z_AXIS_STATE_IDLE = 0,
    Z_AXIS_STATE_WAIT_ACCEPT,
    Z_AXIS_STATE_MOVING,
    Z_AXIS_STATE_STOPPING,
    Z_AXIS_STATE_WAIT_CLEAR_FAULT,
    Z_AXIS_STATE_RECOVERY_VERIFY,
    Z_AXIS_STATE_WAIT_QUERY_STATUS,
    Z_AXIS_STATE_FAULT
} ZAxisState;

typedef struct {
    ZAxisState state;
    uint8_t rx_ready;
    uint8_t move_active;
    uint8_t move_accepted;
    uint8_t last_response_command;
    uint8_t last_status;
    uint32_t actual_speed_hz;
    uint32_t completed_steps;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t frame_errors;
    uint32_t unexpected_frames;
    uint32_t uart_errors;
    uint32_t uart_ore_errors;
    uint32_t uart_fe_errors;
    uint32_t uart_ne_errors;
    uint32_t uart_pe_errors;
    uint32_t uart_dma_errors;
    uint32_t last_uart_error_code;
    uint32_t timeouts;
    uint32_t last_response_tick;
    uint32_t motion_result_seq;
    int32_t motion_result_signed_steps;
    uint8_t motion_result_status;
    uint32_t recovery_result_seq;
    uint8_t recovery_result_status;
    uint32_t controller_fault_status;
} ZAxisStatus;

void ZAxisLink_Init(uint32_t now);
ZAxisRequestResult ZAxisLink_MoveRelative(int32_t pulses,
                                          uint32_t speed_hz,
                                          uint32_t now);
ZAxisRequestResult ZAxisLink_Stop(uint32_t now);
void ZAxisLink_ProcessBytes(const uint8_t *data, uint16_t length,
                            uint32_t now);
void ZAxisLink_Poll(uint32_t now);
void ZAxisLink_GetStatus(ZAxisStatus *status);
void ZAxisLink_OnUartError(uint32_t error_code, uint32_t now);
void ZAxisLink_SetRxReady(uint8_t ready);
ZAxisRequestResult ZAxisLink_ClearFault(uint32_t now);
void ZAxisLink_ResetStream(void);
const char *ZAxisLink_StateString(ZAxisState state);

#ifdef __cplusplus
}
#endif

#endif

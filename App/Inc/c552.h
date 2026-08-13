#ifndef __C552_H__
#define __C552_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define C552_FRAME_SIZE              62U
#define C552_FRAME_VERSION           0x03U
#define C552_PAYLOAD_LENGTH          0x38U
#define C552_COMMAND_MAX_PAYLOAD     8U
#define C552_LINK_TIMEOUT_MS         50U
#define C552_RECOVERY_FRAMES         3U
#define C552_TOF_STALE_MS            500U
#define C552_K230_STALE_MS           150U
#define C552_COMMAND_ACCEPT_TIMEOUT_MS 100U
#define C552_COMMAND_APPLY_TIMEOUT_MS  500U

#define C552_ID_TOF1                 0x01U
#define C552_ID_TOF2                 0x02U
#define C552_ID_TOF3                 0x03U
#define C552_ID_K230_1               0x11U
#define C552_ID_K230_2               0x12U
#define C552_ID_GRIPPER              0x20U

#define C552_DEVICE_TOF1             0x01U
#define C552_DEVICE_TOF2             0x02U
#define C552_DEVICE_K230_1           0x04U
#define C552_DEVICE_K230_2           0x08U
#define C552_DEVICE_TOF3             0x10U
#define C552_DEVICE_ALL              0x1FU
#define C552_DEVICE_REQUIRED_DEFAULT C552_DEVICE_K230_1

#define C552_COMMAND_SET_K230_MODE   0x01U
#define C552_COMMAND_SET_GRIPPER     0x02U
#define C552_ACK_MARKER              0x80U

typedef enum {
    C552_K230_MODE_APRILTAG = 0x01,
    C552_K230_MODE_RED_BLOCK = 0x02
} C552_K230Mode;

typedef enum {
    C552_GRIPPER_BOTH = 0,
    C552_GRIPPER_PWM1 = 1,
    C552_GRIPPER_PWM2 = 2
} C552_GripperChannel;

typedef enum {
    C552_GRIPPER_OPEN = 0,
    C552_GRIPPER_CLOSED = 1
} C552_GripperState;

typedef enum {
    C552_ACK_APPLIED = 0x00,
    C552_ACK_ACCEPTED = 0x01,
    C552_ACK_INVALID_COMMAND = 0x02,
    C552_ACK_FORWARD_FAILED = 0x03,
    C552_ACK_K230_TIMEOUT = 0x04,
    C552_ACK_K230_REJECTED = 0x05,
    C552_ACK_SEQUENCE_MISMATCH = 0x06,
    C552_ACK_BUSY = 0x07,
    C552_ACK_LOCAL_TIMEOUT = 0xFE,
    C552_ACK_LOCAL_TX_ERROR = 0xFF
} C552_AckResult;

typedef enum {
    C552_COMMAND_IDLE = 0,
    C552_COMMAND_TX_PENDING,
    C552_COMMAND_WAIT_ACCEPTED,
    C552_COMMAND_WAIT_APPLIED,
    C552_COMMAND_APPLIED,
    C552_COMMAND_FAILED,
    C552_COMMAND_TIMEOUT
} C552_CommandState;

typedef enum {
    C552_REQUEST_OK = 0,
    C552_REQUEST_BUSY,
    C552_REQUEST_INVALID_ARGUMENT
} C552_RequestResult;

typedef struct {
    uint16_t filtered_mm;
    uint16_t min3_raw_mm;
    uint16_t sample_seq;
    uint16_t sample_age_ms;
} C552_TofData;

typedef struct {
    int16_t center_x;
    int16_t center_y;
    uint16_t x_rotation_cdeg;
    uint16_t y_rotation_cdeg;
    uint16_t sample_seq;
    uint16_t sample_age_ms;
} C552_K230Data;

typedef struct {
    uint8_t seq;
    uint8_t status;
    C552_TofData tof1;
    C552_TofData tof2;
    C552_K230Data k230_1;
    C552_K230Data k230_2;
    C552_TofData tof3;
    uint8_t tof3_flags;
    uint32_t rx_tick;
} C552_Data;

typedef struct {
    uint8_t has_snapshot;
    uint8_t link_online;
    uint8_t link_timeout_warning;
    uint8_t uart_error_warning;
    uint8_t dma_error_warning;
    uint8_t valid_mask;
    uint8_t stale_mask;
    uint8_t sensor_invalid_mask;
    uint8_t sensor_stale_mask;
    uint8_t data_implausible_mask;
    uint8_t ready_mask;
    uint8_t required_mask;
    uint8_t motion_allowed;
    uint32_t last_valid_frame_tick;
} C552_Health;

typedef struct {
    C552_CommandState state;
    uint8_t id;
    uint8_t command;
    uint8_t sequence;
    uint8_t requested_value;
    uint8_t result;
    uint8_t response_value;
    uint32_t queued_tick;
    uint32_t tx_complete_tick;
    uint32_t response_tick;
} C552_CommandStatus;

typedef struct {
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t valid_ack_frames;
    uint32_t version_errors;
    uint32_t length_errors;
    uint32_t id_errors;
    uint32_t crc_errors;
    uint32_t ack_format_errors;
    uint32_t unexpected_acks;
    uint32_t sequence_gaps;
    uint32_t duplicate_frames;
    uint32_t tx_commands;
    uint32_t tx_completed;
    uint32_t tx_start_errors;
    uint32_t tx_errors;
    uint32_t command_timeouts;
    uint32_t uart_ore_errors;
    uint32_t uart_fe_errors;
    uint32_t uart_ne_errors;
    uint32_t uart_pe_errors;
    uint32_t dma_errors;
    uint32_t dma_restart_failures;
} C552_Diagnostics;

void C552_Init(uint32_t now);
void C552_ResetStream(void);
void C552_ProcessBytes(const uint8_t *data, uint16_t length, uint32_t now);
void C552_Poll(uint32_t now);
void C552_RecordUartError(uint32_t error_code);
void C552_RecordDmaRestartFailure(void);
void C552_OnTxComplete(uint32_t now);
void C552_OnTxError(uint32_t now);

uint16_t C552_Crc16CcittFalse(const uint8_t *data, uint16_t length);
uint8_t C552_GetSnapshot(C552_Data *data, C552_Health *health);
void C552_GetDiagnostics(C552_Diagnostics *diagnostics);
uint8_t C552_IsMotionAllowed(void);
uint8_t C552_IsDeviceReady(uint8_t device_mask);
uint8_t C552_SetRequiredMask(uint8_t required_mask);

C552_RequestResult C552_SetK230Mode(uint8_t k230_id,
                                    C552_K230Mode mode, uint32_t now);
C552_RequestResult C552_SetGripper(C552_GripperChannel channel,
                                   C552_GripperState state, uint32_t now);
void C552_GetCommandStatus(C552_CommandStatus *status);
uint8_t C552_CommandIsActive(void);
const char *C552_CommandStateString(C552_CommandState state);
const char *C552_AckResultString(uint8_t result);

#ifdef __cplusplus
}
#endif

#endif /* __C552_H__ */

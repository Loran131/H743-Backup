#ifndef __C552_H__
#define __C552_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define C552_FRAME_SIZE              52U
#define C552_FRAME_VERSION           0x02U
#define C552_PAYLOAD_LENGTH          0x2EU
#define C552_LINK_TIMEOUT_MS         50U
#define C552_RECOVERY_FRAMES         3U
#define C552_TOF_STALE_MS            500U
#define C552_K230_STALE_MS           150U

#define C552_DEVICE_TOF1             0x01U
#define C552_DEVICE_TOF2             0x02U
#define C552_DEVICE_K230_1           0x04U
#define C552_DEVICE_K230_2           0x08U
#define C552_DEVICE_ALL              0x0FU
#define C552_DEVICE_REQUIRED_DEFAULT (C552_DEVICE_TOF1 | C552_DEVICE_TOF2)

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
    uint32_t rx_tick;
} C552_Data;

typedef struct {
    uint8_t has_snapshot;
    uint8_t link_online;
    uint8_t link_timeout_warning;
    uint8_t uart_error_warning;
    uint8_t dma_error_warning;
    uint8_t sensor_invalid_mask;
    uint8_t sensor_stale_mask;
    uint8_t data_implausible_mask;
    uint8_t ready_mask;
    uint8_t required_mask;
    uint8_t motion_allowed;
    uint32_t last_valid_frame_tick;
} C552_Health;

typedef struct {
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t version_errors;
    uint32_t length_errors;
    uint32_t id_errors;
    uint32_t crc_errors;
    uint32_t sequence_gaps;
    uint32_t duplicate_frames;
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

uint8_t C552_GetSnapshot(C552_Data *data, C552_Health *health);
void C552_GetDiagnostics(C552_Diagnostics *diagnostics);
uint8_t C552_IsMotionAllowed(void);

#ifdef __cplusplus
}
#endif

#endif /* __C552_H__ */

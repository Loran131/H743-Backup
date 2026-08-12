#include "c552.h"
#include <string.h>

#define C552_HEADER_1                0xAAU
#define C552_HEADER_2                0x55U
#define C552_TOF1_ID                 0x01U
#define C552_TOF2_ID                 0x02U
#define C552_K230_1_ID               0x11U
#define C552_K230_2_ID               0x12U
#define C552_TOF_MAX_MM              1600U
#define C552_ROTATION_PERIOD_CDEG     36000U

static uint8_t g_candidate[C552_FRAME_SIZE];
static uint8_t g_candidate_length;
static C552_Data g_data;
static C552_Health g_health;
static C552_Diagnostics g_diagnostics;
static uint32_t g_init_tick;
static uint8_t g_sequence_initialized;
static uint8_t g_last_sequence;
static uint8_t g_link_recovery_streak;
static uint8_t g_device_recovery_streak[4];

static uint32_t c552_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void c552_exit_critical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static uint16_t c552_read_be_u16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int16_t c552_read_be_i16(const uint8_t *data)
{
    uint16_t raw = c552_read_be_u16(data);
    if ((raw & 0x8000U) != 0U) {
        return (int16_t)((int32_t)raw - 65536L);
    }
    return (int16_t)raw;
}

static void c552_reset_recovery(void)
{
    g_link_recovery_streak = 0U;
    memset(g_device_recovery_streak, 0, sizeof(g_device_recovery_streak));
    g_health.ready_mask = 0U;
}

static void c552_resync_candidate(void)
{
    static const uint8_t signature[] = {
        C552_HEADER_1, C552_HEADER_2, C552_FRAME_VERSION, C552_PAYLOAD_LENGTH
    };
    uint8_t i;

    for (i = 1U; i < g_candidate_length; ++i) {
        uint8_t remaining = (uint8_t)(g_candidate_length - i);
        uint8_t compare_length = remaining;

        if (compare_length > (uint8_t)sizeof(signature)) {
            compare_length = (uint8_t)sizeof(signature);
        }
        if (memcmp(&g_candidate[i], signature, compare_length) == 0) {
            g_candidate_length = (uint8_t)(g_candidate_length - i);
            memmove(g_candidate, &g_candidate[i], g_candidate_length);
            return;
        }
    }
    g_candidate_length = 0U;
}

static uint16_t c552_crc16_ccitt_false(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    uint8_t i;

    for (i = 0U; i < length; ++i) {
        uint8_t bit;
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static uint8_t c552_validate_frame(const uint8_t *frame)
{
    uint16_t calculated_crc;
    uint16_t received_crc;

    if ((frame[6] != C552_TOF1_ID) || (frame[15] != C552_TOF2_ID) ||
        (frame[24] != C552_K230_1_ID) || (frame[37] != C552_K230_2_ID)) {
        ++g_diagnostics.id_errors;
        return 0U;
    }

    calculated_crc = c552_crc16_ccitt_false(&frame[2], 48U);
    received_crc = c552_read_be_u16(&frame[50]);
    if (calculated_crc != received_crc) {
        ++g_diagnostics.crc_errors;
        return 0U;
    }
    return 1U;
}

static void c552_update_sequence(uint8_t sequence)
{
    uint8_t delta;

    if (g_sequence_initialized == 0U) {
        g_last_sequence = sequence;
        g_sequence_initialized = 1U;
        return;
    }

    delta = (uint8_t)(sequence - g_last_sequence);
    if (delta == 0U) {
        ++g_diagnostics.duplicate_frames;
        return;
    }

    g_diagnostics.sequence_gaps += (uint32_t)((uint8_t)(delta - 1U));
    g_last_sequence = sequence;
}

static void c552_update_device_health(const C552_Data *data)
{
    uint8_t valid_mask = (uint8_t)(data->status & C552_DEVICE_ALL);
    uint8_t stale_mask = (uint8_t)((data->status >> 4) & C552_DEVICE_ALL);
    uint8_t plausible_mask = 0U;
    uint8_t usable_mask;
    uint8_t i;

    if ((valid_mask & C552_DEVICE_TOF1) != 0U) {
        if ((data->tof1.filtered_mm > C552_TOF_MAX_MM) ||
            (data->tof1.min3_raw_mm > C552_TOF_MAX_MM) ||
            (data->tof1.sample_age_ms > C552_TOF_STALE_MS)) {
            plausible_mask |= C552_DEVICE_TOF1;
        }
    }
    if ((valid_mask & C552_DEVICE_TOF2) != 0U) {
        if ((data->tof2.filtered_mm > C552_TOF_MAX_MM) ||
            (data->tof2.min3_raw_mm > C552_TOF_MAX_MM) ||
            (data->tof2.sample_age_ms > C552_TOF_STALE_MS)) {
            plausible_mask |= C552_DEVICE_TOF2;
        }
    }
    if ((valid_mask & C552_DEVICE_K230_1) != 0U) {
        if ((data->k230_1.x_rotation_cdeg >= C552_ROTATION_PERIOD_CDEG) ||
            (data->k230_1.y_rotation_cdeg >= C552_ROTATION_PERIOD_CDEG) ||
            (data->k230_1.sample_age_ms > C552_K230_STALE_MS)) {
            plausible_mask |= C552_DEVICE_K230_1;
        }
    }
    if ((valid_mask & C552_DEVICE_K230_2) != 0U) {
        if ((data->k230_2.x_rotation_cdeg >= C552_ROTATION_PERIOD_CDEG) ||
            (data->k230_2.y_rotation_cdeg >= C552_ROTATION_PERIOD_CDEG) ||
            (data->k230_2.sample_age_ms > C552_K230_STALE_MS)) {
            plausible_mask |= C552_DEVICE_K230_2;
        }
    }

    /* C552 never emits VALID and STALE together for the same device. */
    plausible_mask |= (uint8_t)(valid_mask & stale_mask);

    g_health.sensor_invalid_mask = (uint8_t)((~valid_mask) & C552_DEVICE_ALL);
    g_health.sensor_stale_mask = stale_mask;
    g_health.data_implausible_mask = plausible_mask;
    usable_mask = (uint8_t)(valid_mask & (uint8_t)(~stale_mask) &
                            (uint8_t)(~plausible_mask) & C552_DEVICE_ALL);

    for (i = 0U; i < 4U; ++i) {
        uint8_t bit = (uint8_t)(1U << i);
        if ((usable_mask & bit) != 0U) {
            if (g_device_recovery_streak[i] < C552_RECOVERY_FRAMES) {
                ++g_device_recovery_streak[i];
            }
            if (g_device_recovery_streak[i] >= C552_RECOVERY_FRAMES) {
                g_health.ready_mask |= bit;
            }
        } else {
            g_device_recovery_streak[i] = 0U;
            g_health.ready_mask &= (uint8_t)(~bit);
        }
    }
}

static void c552_publish_frame(const uint8_t *frame, uint32_t now)
{
    C552_Data next;

    next.seq = frame[4];
    next.status = frame[5];
    next.tof1.filtered_mm = c552_read_be_u16(&frame[7]);
    next.tof1.min3_raw_mm = c552_read_be_u16(&frame[9]);
    next.tof1.sample_seq = c552_read_be_u16(&frame[11]);
    next.tof1.sample_age_ms = c552_read_be_u16(&frame[13]);
    next.tof2.filtered_mm = c552_read_be_u16(&frame[16]);
    next.tof2.min3_raw_mm = c552_read_be_u16(&frame[18]);
    next.tof2.sample_seq = c552_read_be_u16(&frame[20]);
    next.tof2.sample_age_ms = c552_read_be_u16(&frame[22]);
    next.k230_1.center_x = c552_read_be_i16(&frame[25]);
    next.k230_1.center_y = c552_read_be_i16(&frame[27]);
    next.k230_1.x_rotation_cdeg = c552_read_be_u16(&frame[29]);
    next.k230_1.y_rotation_cdeg = c552_read_be_u16(&frame[31]);
    next.k230_1.sample_seq = c552_read_be_u16(&frame[33]);
    next.k230_1.sample_age_ms = c552_read_be_u16(&frame[35]);
    next.k230_2.center_x = c552_read_be_i16(&frame[38]);
    next.k230_2.center_y = c552_read_be_i16(&frame[40]);
    next.k230_2.x_rotation_cdeg = c552_read_be_u16(&frame[42]);
    next.k230_2.y_rotation_cdeg = c552_read_be_u16(&frame[44]);
    next.k230_2.sample_seq = c552_read_be_u16(&frame[46]);
    next.k230_2.sample_age_ms = c552_read_be_u16(&frame[48]);
    next.rx_tick = now;

    c552_update_sequence(next.seq);
    g_data = next;
    g_health.has_snapshot = 1U;
    g_health.last_valid_frame_tick = now;
    ++g_diagnostics.valid_frames;

    if (g_link_recovery_streak < C552_RECOVERY_FRAMES) {
        ++g_link_recovery_streak;
    }
    c552_update_device_health(&next);

    if (g_link_recovery_streak >= C552_RECOVERY_FRAMES) {
        g_health.link_online = 1U;
        g_health.link_timeout_warning = 0U;
        g_health.uart_error_warning = 0U;
        g_health.dma_error_warning = 0U;
    }
}

static void c552_process_byte(uint8_t byte, uint32_t now)
{
    ++g_diagnostics.rx_bytes;

    if (g_candidate_length == 0U) {
        if (byte == C552_HEADER_1) {
            g_candidate[0] = byte;
            g_candidate_length = 1U;
        }
        return;
    }

    if (g_candidate_length == 1U) {
        if (byte == C552_HEADER_2) {
            g_candidate[1] = byte;
            g_candidate_length = 2U;
        } else if (byte != C552_HEADER_1) {
            g_candidate_length = 0U;
        }
        return;
    }

    g_candidate[g_candidate_length++] = byte;

    if ((g_candidate_length == 3U) && (byte != C552_FRAME_VERSION)) {
        ++g_diagnostics.version_errors;
        if (!g_health.link_online) {
            c552_reset_recovery();
        }
        c552_resync_candidate();
        return;
    }

    if ((g_candidate_length == 4U) && (byte != C552_PAYLOAD_LENGTH)) {
        ++g_diagnostics.length_errors;
        if (!g_health.link_online) {
            c552_reset_recovery();
        }
        c552_resync_candidate();
        return;
    }

    if (g_candidate_length < C552_FRAME_SIZE) {
        return;
    }

    if (c552_validate_frame(g_candidate)) {
        c552_publish_frame(g_candidate, now);
        g_candidate_length = 0U;
    } else {
        if (!g_health.link_online) {
            c552_reset_recovery();
        }
        c552_resync_candidate();
    }
}

void C552_Init(uint32_t now)
{
    uint32_t primask = c552_enter_critical();

    memset(g_candidate, 0, sizeof(g_candidate));
    memset(&g_data, 0, sizeof(g_data));
    memset(&g_health, 0, sizeof(g_health));
    memset(&g_diagnostics, 0, sizeof(g_diagnostics));
    memset(g_device_recovery_streak, 0, sizeof(g_device_recovery_streak));
    g_candidate_length = 0U;
    g_init_tick = now;
    g_sequence_initialized = 0U;
    g_last_sequence = 0U;
    g_link_recovery_streak = 0U;
    g_health.sensor_invalid_mask = C552_DEVICE_ALL;
    g_health.sensor_stale_mask = C552_DEVICE_ALL;
    g_health.required_mask = C552_DEVICE_REQUIRED_DEFAULT;

    c552_exit_critical(primask);
}

void C552_ResetStream(void)
{
    g_candidate_length = 0U;
    if (!g_health.link_online) {
        c552_reset_recovery();
    }
}

void C552_ProcessBytes(const uint8_t *data, uint16_t length, uint32_t now)
{
    uint16_t i;
    if (data == NULL) {
        return;
    }
    for (i = 0U; i < length; ++i) {
        c552_process_byte(data[i], now);
    }
}

void C552_Poll(uint32_t now)
{
    uint32_t primask = c552_enter_critical();
    uint32_t reference_tick = g_health.has_snapshot ?
                              g_health.last_valid_frame_tick : g_init_tick;
    uint32_t elapsed = now - reference_tick;

    /* A receive IRQ may publish a frame after the caller sampled now. */
    if ((int32_t)elapsed < 0) {
        elapsed = 0U;
    }

    if (elapsed > C552_LINK_TIMEOUT_MS) {
        g_health.link_online = 0U;
        g_health.link_timeout_warning = 1U;
        g_health.motion_allowed = 0U;
        g_sequence_initialized = 0U;
        c552_reset_recovery();
    } else {
        g_health.motion_allowed =
            (uint8_t)(g_health.link_online &&
                      ((g_health.ready_mask & g_health.required_mask) ==
                       g_health.required_mask));
    }

    c552_exit_critical(primask);
}

void C552_RecordUartError(uint32_t error_code)
{
    g_health.uart_error_warning = 1U;
    if ((error_code & HAL_UART_ERROR_ORE) != 0U) {
        ++g_diagnostics.uart_ore_errors;
    }
    if ((error_code & HAL_UART_ERROR_FE) != 0U) {
        ++g_diagnostics.uart_fe_errors;
    }
    if ((error_code & HAL_UART_ERROR_NE) != 0U) {
        ++g_diagnostics.uart_ne_errors;
    }
    if ((error_code & HAL_UART_ERROR_PE) != 0U) {
        ++g_diagnostics.uart_pe_errors;
    }
    if ((error_code & HAL_UART_ERROR_DMA) != 0U) {
        ++g_diagnostics.dma_errors;
        g_health.dma_error_warning = 1U;
    }
}

void C552_RecordDmaRestartFailure(void)
{
    ++g_diagnostics.dma_restart_failures;
    g_health.dma_error_warning = 1U;
}

uint8_t C552_GetSnapshot(C552_Data *data, C552_Health *health)
{
    uint8_t has_snapshot;
    uint32_t primask = c552_enter_critical();

    if (data != NULL) {
        *data = g_data;
    }
    if (health != NULL) {
        *health = g_health;
    }
    has_snapshot = g_health.has_snapshot;

    c552_exit_critical(primask);
    return has_snapshot;
}

void C552_GetDiagnostics(C552_Diagnostics *diagnostics)
{
    uint32_t primask;
    if (diagnostics == NULL) {
        return;
    }
    primask = c552_enter_critical();
    *diagnostics = g_diagnostics;
    c552_exit_critical(primask);
}

uint8_t C552_IsMotionAllowed(void)
{
    uint8_t allowed;
    uint32_t primask = c552_enter_critical();
    allowed = g_health.motion_allowed;
    c552_exit_critical(primask);
    return allowed;
}

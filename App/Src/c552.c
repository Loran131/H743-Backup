#include "c552.h"
#include "usart.h"
#include <string.h>

#define C552_HEADER_1                 0xAAU
#define C552_HEADER_2                 0x55U
#define C552_TOF12_MAX_MM             1600U
#define C552_TOF3_MAX_MM              2000U
#define C552_ROTATION_PERIOD_CDEG      36000U
#define C552_ACK_PAYLOAD_LENGTH        5U
#define C552_ACK_FRAME_SIZE            11U
#define C552_COMMAND_FRAME_MAX_SIZE    (C552_COMMAND_MAX_PAYLOAD + 6U)

static uint8_t g_candidate[C552_FRAME_SIZE];
static uint8_t g_candidate_length;
static uint8_t g_candidate_expected_length;
static C552_Data g_data;
static C552_Health g_health;
static C552_Diagnostics g_diagnostics;
static C552_CommandStatus g_command_status;
static uint8_t g_command_frame[C552_COMMAND_FRAME_MAX_SIZE];
static uint8_t g_command_frame_length;
static uint8_t g_command_tx_started;
static uint8_t g_next_command_sequence;
static uint32_t g_init_tick;
static uint8_t g_sequence_initialized;
static uint8_t g_last_sequence;
static uint8_t g_link_recovery_streak;
static uint8_t g_device_recovery_streak[5];

static uint32_t c552_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void c552_exit_critical(uint32_t primask)
{
    if (primask == 0U) __enable_irq();
}

static uint16_t c552_read_be_u16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int16_t c552_read_be_i16(const uint8_t *data)
{
    return (int16_t)c552_read_be_u16(data);
}

uint16_t C552_Crc16CcittFalse(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;

    if (data == NULL) return crc;
    for (i = 0U; i < length; ++i) {
        uint8_t bit;
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; ++bit) {
            crc = ((crc & 0x8000U) != 0U) ?
                  (uint16_t)((crc << 1) ^ 0x1021U) :
                  (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void c552_reset_recovery(void)
{
    g_link_recovery_streak = 0U;
    memset(g_device_recovery_streak, 0, sizeof(g_device_recovery_streak));
    g_health.ready_mask = 0U;
}

static void c552_resync_candidate(void)
{
    uint8_t i;

    for (i = 1U; i < g_candidate_length; ++i) {
        if (g_candidate[i] != C552_HEADER_1) continue;
        if (((uint8_t)(i + 1U) == g_candidate_length) ||
            (g_candidate[i + 1U] == C552_HEADER_2)) {
            g_candidate_length = (uint8_t)(g_candidate_length - i);
            memmove(g_candidate, &g_candidate[i], g_candidate_length);
            g_candidate_expected_length = 0U;
            return;
        }
    }
    g_candidate_length = 0U;
    g_candidate_expected_length = 0U;
}

static uint8_t c552_validate_snapshot(const uint8_t *frame)
{
    uint16_t calculated_crc;
    uint16_t received_crc;

    if ((frame[6] != C552_ID_TOF1) || (frame[15] != C552_ID_TOF2) ||
        (frame[24] != C552_ID_K230_1) ||
        (frame[37] != C552_ID_K230_2) ||
        (frame[50] != C552_ID_TOF3)) {
        ++g_diagnostics.id_errors;
        return 0U;
    }
    calculated_crc = C552_Crc16CcittFalse(&frame[2], 58U);
    received_crc = c552_read_be_u16(&frame[60]);
    if (calculated_crc != received_crc) {
        ++g_diagnostics.crc_errors;
        return 0U;
    }
    return 1U;
}

static uint8_t c552_validate_ack(const uint8_t *frame)
{
    uint16_t calculated_crc;
    uint16_t received_crc;

    calculated_crc = C552_Crc16CcittFalse(&frame[2],
                                          (uint16_t)(frame[3] + 2U));
    received_crc = c552_read_be_u16(&frame[4U + frame[3]]);
    if (calculated_crc != received_crc) {
        ++g_diagnostics.crc_errors;
        return 0U;
    }
    if ((frame[3] != C552_ACK_PAYLOAD_LENGTH) ||
        (frame[4] != C552_ACK_MARKER)) {
        ++g_diagnostics.ack_format_errors;
        return 0U;
    }
    if (((frame[2] != C552_ID_K230_1) &&
         (frame[2] != C552_ID_K230_2) &&
         (frame[2] != C552_ID_GRIPPER)) ||
        ((frame[5] != C552_COMMAND_SET_K230_MODE) &&
         (frame[5] != C552_COMMAND_SET_GRIPPER)) ||
        (frame[7] > C552_ACK_BUSY)) {
        ++g_diagnostics.ack_format_errors;
        return 0U;
    }
    if (((frame[2] == C552_ID_GRIPPER) &&
         (frame[5] != C552_COMMAND_SET_GRIPPER)) ||
        ((frame[2] != C552_ID_GRIPPER) &&
         (frame[5] != C552_COMMAND_SET_K230_MODE))) {
        ++g_diagnostics.ack_format_errors;
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

static uint8_t c552_device_bit_from_index(uint8_t index)
{
    return (uint8_t)(1U << index);
}

static uint16_t c552_effective_age(uint16_t sample_age_ms,
                                   uint32_t snapshot_age_ms)
{
    uint32_t age = (uint32_t)sample_age_ms + snapshot_age_ms;
    return (age > 0xFFFFU) ? 0xFFFFU : (uint16_t)age;
}

static void c552_expire_local_device_ages(uint32_t now)
{
    uint32_t snapshot_age;
    uint8_t expired_mask = 0U;

    if (g_health.has_snapshot == 0U) return;
    snapshot_age = now - g_data.rx_tick;
    if (c552_effective_age(g_data.tof1.sample_age_ms, snapshot_age) >
        C552_TOF_STALE_MS) expired_mask |= C552_DEVICE_TOF1;
    if (c552_effective_age(g_data.tof2.sample_age_ms, snapshot_age) >
        C552_TOF_STALE_MS) expired_mask |= C552_DEVICE_TOF2;
    if (c552_effective_age(g_data.tof3.sample_age_ms, snapshot_age) >
        C552_TOF_STALE_MS) expired_mask |= C552_DEVICE_TOF3;
    if (c552_effective_age(g_data.k230_1.sample_age_ms, snapshot_age) >
        C552_K230_STALE_MS) expired_mask |= C552_DEVICE_K230_1;
    if (c552_effective_age(g_data.k230_2.sample_age_ms, snapshot_age) >
        C552_K230_STALE_MS) expired_mask |= C552_DEVICE_K230_2;

    g_health.stale_mask |= expired_mask;
    g_health.sensor_stale_mask = g_health.stale_mask;
    g_health.ready_mask &= (uint8_t)(~expired_mask);
    for (uint8_t i = 0U; i < 5U; ++i) {
        if ((expired_mask & c552_device_bit_from_index(i)) != 0U) {
            g_device_recovery_streak[i] = 0U;
        }
    }
}

static void c552_update_device_health(const C552_Data *data)
{
    uint8_t valid_mask = (uint8_t)(data->status & 0x0FU);
    uint8_t stale_mask = (uint8_t)((data->status >> 4) & 0x0FU);
    uint8_t plausible_mask = 0U;
    uint8_t usable_mask;
    uint8_t i;

    if ((data->tof3_flags & 0x01U) != 0U) valid_mask |= C552_DEVICE_TOF3;
    if ((data->tof3_flags & 0x02U) != 0U) stale_mask |= C552_DEVICE_TOF3;
    if ((data->tof3_flags & 0xFCU) != 0U) plausible_mask |= C552_DEVICE_TOF3;

    if ((valid_mask & C552_DEVICE_TOF1) != 0U) {
        if ((data->tof1.filtered_mm > C552_TOF12_MAX_MM) ||
            (data->tof1.min3_raw_mm > C552_TOF12_MAX_MM) ||
            (data->tof1.sample_age_ms > C552_TOF_STALE_MS)) {
            plausible_mask |= C552_DEVICE_TOF1;
        }
    }
    if ((valid_mask & C552_DEVICE_TOF2) != 0U) {
        if ((data->tof2.filtered_mm > C552_TOF12_MAX_MM) ||
            (data->tof2.min3_raw_mm > C552_TOF12_MAX_MM) ||
            (data->tof2.sample_age_ms > C552_TOF_STALE_MS)) {
            plausible_mask |= C552_DEVICE_TOF2;
        }
    }
    if ((valid_mask & C552_DEVICE_TOF3) != 0U) {
        if ((data->tof3.filtered_mm > C552_TOF3_MAX_MM) ||
            (data->tof3.min3_raw_mm > C552_TOF3_MAX_MM) ||
            (data->tof3.sample_age_ms > C552_TOF_STALE_MS)) {
            plausible_mask |= C552_DEVICE_TOF3;
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

    plausible_mask |= (uint8_t)(valid_mask & stale_mask);
    g_health.valid_mask = valid_mask;
    g_health.stale_mask = stale_mask;
    g_health.sensor_invalid_mask = (uint8_t)((~valid_mask) & C552_DEVICE_ALL);
    g_health.sensor_stale_mask = stale_mask;
    g_health.data_implausible_mask = plausible_mask;
    usable_mask = (uint8_t)(valid_mask & (uint8_t)(~stale_mask) &
                            (uint8_t)(~plausible_mask) & C552_DEVICE_ALL);

    for (i = 0U; i < 5U; ++i) {
        uint8_t bit = c552_device_bit_from_index(i);
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

static void c552_publish_snapshot(const uint8_t *frame, uint32_t now)
{
    C552_Data next;
    memset(&next, 0, sizeof(next));

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
    next.tof3.filtered_mm = c552_read_be_u16(&frame[51]);
    next.tof3.min3_raw_mm = c552_read_be_u16(&frame[53]);
    next.tof3.sample_seq = c552_read_be_u16(&frame[55]);
    next.tof3.sample_age_ms = c552_read_be_u16(&frame[57]);
    next.tof3_flags = frame[59];
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

static uint8_t c552_command_is_active_state(C552_CommandState state)
{
    return ((state == C552_COMMAND_TX_PENDING) ||
            (state == C552_COMMAND_WAIT_ACCEPTED) ||
            (state == C552_COMMAND_WAIT_APPLIED)) ? 1U : 0U;
}

static void c552_process_ack(const uint8_t *frame, uint32_t now)
{
    uint8_t id = frame[2];
    uint8_t command = frame[5];
    uint8_t sequence = frame[6];
    uint8_t result = frame[7];
    uint8_t value = frame[8];

    ++g_diagnostics.valid_ack_frames;
    if ((c552_command_is_active_state(g_command_status.state) == 0U) ||
        (id != g_command_status.id) ||
        (command != g_command_status.command) ||
        (sequence != g_command_status.sequence)) {
        ++g_diagnostics.unexpected_acks;
        return;
    }

    g_command_status.result = result;
    g_command_status.response_value = value;
    g_command_status.response_tick = now;
    if (g_command_status.tx_complete_tick == 0U) {
        g_command_status.tx_complete_tick = now;
    }
    if (result == C552_ACK_ACCEPTED) {
        g_command_status.state = C552_COMMAND_WAIT_APPLIED;
    } else if ((result == C552_ACK_APPLIED) &&
               (value == g_command_status.requested_value)) {
        g_command_status.state = C552_COMMAND_APPLIED;
    } else {
        g_command_status.state = C552_COMMAND_FAILED;
    }
}

static void c552_dispatch_candidate(uint32_t now)
{
    uint8_t valid;
    if ((g_candidate[2] == C552_FRAME_VERSION) &&
        (g_candidate[3] == C552_PAYLOAD_LENGTH)) {
        valid = c552_validate_snapshot(g_candidate);
        if (valid != 0U) c552_publish_snapshot(g_candidate, now);
    } else {
        valid = c552_validate_ack(g_candidate);
        if (valid != 0U) c552_process_ack(g_candidate, now);
    }
    if (valid != 0U) {
        g_candidate_length = 0U;
        g_candidate_expected_length = 0U;
    } else {
        c552_resync_candidate();
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

    if (g_candidate_length >= C552_FRAME_SIZE) {
        c552_resync_candidate();
        return;
    }
    g_candidate[g_candidate_length++] = byte;

    if (g_candidate_length == 4U) {
        if (g_candidate[2] == C552_FRAME_VERSION) {
            if (g_candidate[3] != C552_PAYLOAD_LENGTH) {
                ++g_diagnostics.length_errors;
                c552_resync_candidate();
                return;
            }
            g_candidate_expected_length = C552_FRAME_SIZE;
        } else {
            if (g_candidate[3] == C552_PAYLOAD_LENGTH) {
                ++g_diagnostics.version_errors;
                c552_resync_candidate();
                return;
            }
            if (g_candidate[3] > C552_COMMAND_MAX_PAYLOAD) {
                ++g_diagnostics.length_errors;
                c552_resync_candidate();
                return;
            }
            g_candidate_expected_length = (uint8_t)(g_candidate[3] + 6U);
        }
    }
    if ((g_candidate_expected_length != 0U) &&
        (g_candidate_length >= g_candidate_expected_length)) {
        c552_dispatch_candidate(now);
    }
}

static C552_RequestResult c552_queue_command(uint8_t id, uint8_t command,
                                             const uint8_t *parameters,
                                             uint8_t parameter_length,
                                             uint8_t requested_value,
                                             uint32_t now)
{
    uint8_t payload_length = (uint8_t)(parameter_length + 2U);
    uint8_t pos = 0U;
    uint16_t crc;
    uint32_t primask = c552_enter_critical();

    if (c552_command_is_active_state(g_command_status.state) != 0U) {
        c552_exit_critical(primask);
        return C552_REQUEST_BUSY;
    }
    if ((payload_length > C552_COMMAND_MAX_PAYLOAD) ||
        ((parameter_length > 0U) && (parameters == NULL))) {
        c552_exit_critical(primask);
        return C552_REQUEST_INVALID_ARGUMENT;
    }

    g_command_frame[pos++] = C552_HEADER_1;
    g_command_frame[pos++] = C552_HEADER_2;
    g_command_frame[pos++] = id;
    g_command_frame[pos++] = payload_length;
    g_command_frame[pos++] = command;
    g_command_frame[pos++] = g_next_command_sequence++;
    if (parameter_length > 0U) {
        memcpy(&g_command_frame[pos], parameters, parameter_length);
        pos = (uint8_t)(pos + parameter_length);
    }
    crc = C552_Crc16CcittFalse(&g_command_frame[2],
                               (uint16_t)(payload_length + 2U));
    g_command_frame[pos++] = (uint8_t)(crc >> 8);
    g_command_frame[pos++] = (uint8_t)crc;
    g_command_frame_length = pos;
    g_command_tx_started = 0U;
    memset(&g_command_status, 0, sizeof(g_command_status));
    g_command_status.state = C552_COMMAND_TX_PENDING;
    g_command_status.id = id;
    g_command_status.command = command;
    g_command_status.sequence = g_command_frame[5];
    g_command_status.requested_value = requested_value;
    g_command_status.result = C552_ACK_LOCAL_TX_ERROR;
    g_command_status.queued_tick = now;
    ++g_diagnostics.tx_commands;
    c552_exit_critical(primask);
    return C552_REQUEST_OK;
}

void C552_Init(uint32_t now)
{
    uint32_t primask = c552_enter_critical();
    memset(g_candidate, 0, sizeof(g_candidate));
    memset(&g_data, 0, sizeof(g_data));
    memset(&g_health, 0, sizeof(g_health));
    memset(&g_diagnostics, 0, sizeof(g_diagnostics));
    memset(&g_command_status, 0, sizeof(g_command_status));
    memset(g_device_recovery_streak, 0, sizeof(g_device_recovery_streak));
    g_candidate_length = 0U;
    g_candidate_expected_length = 0U;
    g_command_frame_length = 0U;
    g_command_tx_started = 0U;
    g_next_command_sequence = 0U;
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
    g_candidate_expected_length = 0U;
    if (g_health.link_online == 0U) c552_reset_recovery();
}

void C552_ProcessBytes(const uint8_t *data, uint16_t length, uint32_t now)
{
    uint16_t i;
    if (data == NULL) return;
    for (i = 0U; i < length; ++i) c552_process_byte(data[i], now);
}

static void c552_poll_command(uint32_t now)
{
    HAL_StatusTypeDef tx_status;

    if ((g_command_status.state == C552_COMMAND_TX_PENDING) &&
        (g_command_tx_started == 0U)) {
        tx_status = USART3_TransmitAsync(g_command_frame,
                                         g_command_frame_length);
        if (tx_status == HAL_OK) {
            g_command_tx_started = 1U;
        } else if (tx_status != HAL_BUSY) {
            ++g_diagnostics.tx_start_errors;
        }
    }

    if (((g_command_status.state == C552_COMMAND_TX_PENDING) &&
         ((uint32_t)(now - g_command_status.queued_tick) >
          C552_COMMAND_ACCEPT_TIMEOUT_MS)) ||
        ((g_command_status.state == C552_COMMAND_WAIT_ACCEPTED) &&
         ((uint32_t)(now - g_command_status.tx_complete_tick) >
          C552_COMMAND_ACCEPT_TIMEOUT_MS)) ||
        ((g_command_status.state == C552_COMMAND_WAIT_APPLIED) &&
         ((uint32_t)(now - g_command_status.tx_complete_tick) >
          C552_COMMAND_APPLY_TIMEOUT_MS))) {
        g_command_status.state = C552_COMMAND_TIMEOUT;
        g_command_status.result = C552_ACK_LOCAL_TIMEOUT;
        g_command_status.response_tick = now;
        ++g_diagnostics.command_timeouts;
    }
}

void C552_Poll(uint32_t now)
{
    uint32_t primask = c552_enter_critical();
    uint32_t reference_tick = g_health.has_snapshot ?
                              g_health.last_valid_frame_tick : g_init_tick;
    uint32_t elapsed = now - reference_tick;

    if ((int32_t)elapsed < 0) elapsed = 0U;
    if (elapsed > C552_LINK_TIMEOUT_MS) {
        g_health.link_online = 0U;
        g_health.link_timeout_warning = 1U;
        g_health.motion_allowed = 0U;
        g_sequence_initialized = 0U;
        c552_reset_recovery();
    } else {
        c552_expire_local_device_ages(now);
        g_health.motion_allowed =
            (uint8_t)(g_health.link_online &&
                      ((g_health.ready_mask & g_health.required_mask) ==
                       g_health.required_mask));
    }
    c552_poll_command(now);
    c552_exit_critical(primask);
}

void C552_RecordUartError(uint32_t error_code)
{
    g_health.uart_error_warning = 1U;
    if ((error_code & HAL_UART_ERROR_ORE) != 0U) ++g_diagnostics.uart_ore_errors;
    if ((error_code & HAL_UART_ERROR_FE) != 0U) ++g_diagnostics.uart_fe_errors;
    if ((error_code & HAL_UART_ERROR_NE) != 0U) ++g_diagnostics.uart_ne_errors;
    if ((error_code & HAL_UART_ERROR_PE) != 0U) ++g_diagnostics.uart_pe_errors;
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

void C552_OnTxComplete(uint32_t now)
{
    uint32_t primask = c552_enter_critical();
    ++g_diagnostics.tx_completed;
    g_command_tx_started = 0U;
    if ((c552_command_is_active_state(g_command_status.state) != 0U) &&
        (g_command_status.tx_complete_tick == 0U)) {
        g_command_status.tx_complete_tick = now;
    }
    if (g_command_status.state == C552_COMMAND_TX_PENDING) {
        g_command_status.state = C552_COMMAND_WAIT_ACCEPTED;
    }
    c552_exit_critical(primask);
}

void C552_OnTxError(uint32_t now)
{
    uint32_t primask = c552_enter_critical();
    ++g_diagnostics.tx_errors;
    g_command_tx_started = 0U;
    if (c552_command_is_active_state(g_command_status.state) != 0U) {
        g_command_status.state = C552_COMMAND_FAILED;
        g_command_status.result = C552_ACK_LOCAL_TX_ERROR;
        g_command_status.response_tick = now;
    }
    c552_exit_critical(primask);
}

uint8_t C552_GetSnapshot(C552_Data *data, C552_Health *health)
{
    uint8_t has_snapshot;
    uint32_t primask = c552_enter_critical();
    if (data != NULL) *data = g_data;
    if (health != NULL) *health = g_health;
    has_snapshot = g_health.has_snapshot;
    c552_exit_critical(primask);
    return has_snapshot;
}

void C552_GetDiagnostics(C552_Diagnostics *diagnostics)
{
    uint32_t primask;
    if (diagnostics == NULL) return;
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

uint8_t C552_IsDeviceReady(uint8_t device_mask)
{
    uint8_t ready;
    uint32_t primask;
    if ((device_mask == 0U) || ((device_mask & ~C552_DEVICE_ALL) != 0U)) {
        return 0U;
    }
    primask = c552_enter_critical();
    ready = (uint8_t)(g_health.link_online &&
                      ((g_health.ready_mask & device_mask) == device_mask));
    c552_exit_critical(primask);
    return ready;
}

uint8_t C552_SetRequiredMask(uint8_t required_mask)
{
    uint32_t primask;
    if ((required_mask & ~C552_DEVICE_ALL) != 0U) return 0U;
    primask = c552_enter_critical();
    g_health.required_mask = required_mask;
    g_health.motion_allowed =
        (uint8_t)(g_health.link_online &&
                  ((g_health.ready_mask & required_mask) == required_mask));
    c552_exit_critical(primask);
    return 1U;
}

C552_RequestResult C552_SetK230Mode(uint8_t k230_id,
                                    C552_K230Mode mode, uint32_t now)
{
    uint8_t parameters[1];
    if (((k230_id != C552_ID_K230_1) && (k230_id != C552_ID_K230_2)) ||
        ((mode != C552_K230_MODE_APRILTAG) &&
         (mode != C552_K230_MODE_RED_BLOCK))) {
        return C552_REQUEST_INVALID_ARGUMENT;
    }
    parameters[0] = (uint8_t)mode;
    return c552_queue_command(k230_id, C552_COMMAND_SET_K230_MODE,
                              parameters, sizeof(parameters),
                              (uint8_t)mode, now);
}

C552_RequestResult C552_SetGripper(C552_GripperChannel channel,
                                   C552_GripperState state, uint32_t now)
{
    uint8_t parameters[2];
    if ((channel > C552_GRIPPER_PWM2) || (state > C552_GRIPPER_CLOSED)) {
        return C552_REQUEST_INVALID_ARGUMENT;
    }
    parameters[0] = (uint8_t)channel;
    parameters[1] = (uint8_t)state;
    return c552_queue_command(C552_ID_GRIPPER,
                              C552_COMMAND_SET_GRIPPER,
                              parameters, sizeof(parameters),
                              (uint8_t)state, now);
}

void C552_GetCommandStatus(C552_CommandStatus *status)
{
    uint32_t primask;
    if (status == NULL) return;
    primask = c552_enter_critical();
    *status = g_command_status;
    c552_exit_critical(primask);
}

uint8_t C552_CommandIsActive(void)
{
    uint8_t active;
    uint32_t primask = c552_enter_critical();
    active = c552_command_is_active_state(g_command_status.state);
    c552_exit_critical(primask);
    return active;
}

const char *C552_CommandStateString(C552_CommandState state)
{
    static const char *const names[] = {
        "IDLE", "TX_PENDING", "WAIT_ACCEPTED", "WAIT_APPLIED",
        "APPLIED", "FAILED", "TIMEOUT"
    };
    return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
           names[state] : "UNKNOWN";
}

const char *C552_AckResultString(uint8_t result)
{
    switch (result) {
    case C552_ACK_APPLIED: return "APPLIED";
    case C552_ACK_ACCEPTED: return "ACCEPTED";
    case C552_ACK_INVALID_COMMAND: return "INVALID_COMMAND";
    case C552_ACK_FORWARD_FAILED: return "FORWARD_FAILED";
    case C552_ACK_K230_TIMEOUT: return "K230_TIMEOUT";
    case C552_ACK_K230_REJECTED: return "K230_REJECTED";
    case C552_ACK_SEQUENCE_MISMATCH: return "SEQUENCE_MISMATCH";
    case C552_ACK_BUSY: return "BUSY";
    case C552_ACK_LOCAL_TIMEOUT: return "LOCAL_TIMEOUT";
    case C552_ACK_LOCAL_TX_ERROR: return "LOCAL_TX_ERROR";
    default: return "UNKNOWN";
    }
}

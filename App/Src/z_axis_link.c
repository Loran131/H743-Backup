#include "z_axis_link.h"

#include "main.h"
#include "usart.h"
#include <limits.h>
#include <stddef.h>
#include <string.h>

#define Z_AXIS_HEADER_0             0xAAU
#define Z_AXIS_HEADER_1             0x55U
#define Z_AXIS_PAYLOAD_SIZE         10U
#define Z_AXIS_COMMAND_MOVE         0x01U
#define Z_AXIS_COMMAND_STOP         0x02U
#define Z_AXIS_RESPONSE_MOVE        0x81U
#define Z_AXIS_RESPONSE_STOP        0x82U
#define Z_AXIS_STATUS_ACCEPTED      0x00U
#define Z_AXIS_STATUS_COMPLETE      0x01U
#define Z_AXIS_STATUS_STOPPED       0x07U
#define Z_AXIS_ACK_TIMEOUT_MS       250U
#define Z_AXIS_COMPLETION_MARGIN_MS 2000U
#define Z_AXIS_MAX_TIMEOUT_MS       0x7FFFFFFFU

static uint8_t g_rx_frame[Z_AXIS_FRAME_SIZE];
static uint8_t g_rx_index;
static uint8_t g_tx_frame[Z_AXIS_FRAME_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
static volatile ZAxisStatus g_status;
static volatile uint32_t g_deadline;
static volatile uint8_t g_deadline_active;
static volatile uint8_t g_active_direction;
static volatile uint8_t g_stop_ack_received;

static void publish_motion_result(uint8_t status, uint32_t steps)
{
    if (g_status.move_active == 0U) return;
    if (steps > (uint32_t)INT32_MAX) {
        g_status.motion_result_signed_steps = 0;
        g_status.motion_result_status = 0xFFU;
        ++g_status.motion_result_seq;
        ++g_status.unexpected_frames;
        g_status.move_active = 0U;
        return;
    }
    g_status.motion_result_signed_steps = (g_active_direction == 0U) ?
        (int32_t)steps : -(int32_t)steps;
    g_status.motion_result_status = status;
    ++g_status.motion_result_seq;
    g_status.move_active = 0U;
}

static uint16_t crc16_modbus(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < length; ++i) {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = ((crc & 1U) != 0U) ?
                (uint16_t)((crc >> 1U) ^ 0xA001U) : (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static void build_frame(uint8_t command, uint8_t direction,
                        uint32_t speed_hz, uint32_t steps)
{
    uint16_t crc;

    g_tx_frame[0] = Z_AXIS_HEADER_0;
    g_tx_frame[1] = Z_AXIS_HEADER_1;
    g_tx_frame[2] = Z_AXIS_PAYLOAD_SIZE;
    g_tx_frame[3] = command;
    g_tx_frame[4] = direction;
    write_u32_le(&g_tx_frame[5], speed_hz);
    write_u32_le(&g_tx_frame[9], steps);
    crc = crc16_modbus(&g_tx_frame[2], 11U);
    g_tx_frame[13] = (uint8_t)crc;
    g_tx_frame[14] = (uint8_t)(crc >> 8U);
    g_tx_frame[15] = 0x0DU;
    g_tx_frame[16] = 0x0AU;
}

static void set_deadline(uint32_t now, uint32_t delay_ms)
{
    g_deadline = now + delay_ms;
    g_deadline_active = 1U;
}

static uint32_t completion_timeout(uint32_t steps, uint32_t speed_hz)
{
    uint64_t duration = (((uint64_t)steps * 1000ULL) + speed_hz - 1U) /
                        speed_hz;
    duration += Z_AXIS_COMPLETION_MARGIN_MS;
    return (duration > Z_AXIS_MAX_TIMEOUT_MS) ? Z_AXIS_MAX_TIMEOUT_MS :
                                                (uint32_t)duration;
}

static void handle_response(const uint8_t *frame, uint32_t now)
{
    uint8_t command = frame[3];
    uint8_t status = frame[4];
    ZAxisState state = g_status.state;

    g_status.last_response_command = command;
    g_status.last_status = status;
    g_status.actual_speed_hz = read_u32_le(&frame[5]);
    g_status.completed_steps = read_u32_le(&frame[9]);
    g_status.last_response_tick = now;
    ++g_status.valid_frames;

    if ((command != Z_AXIS_RESPONSE_MOVE) &&
        (command != Z_AXIS_RESPONSE_STOP)) {
        ++g_status.unexpected_frames;
        return;
    }

    if (command == Z_AXIS_RESPONSE_STOP) {
        if (state != Z_AXIS_STATE_STOPPING) {
            ++g_status.unexpected_frames;
            return;
        }
        if (status == Z_AXIS_STATUS_COMPLETE) {
            g_stop_ack_received = 1U;
            if (g_status.move_active == 0U) {
                g_status.state = Z_AXIS_STATE_IDLE;
                g_deadline_active = 0U;
            } else {
                set_deadline(now, Z_AXIS_COMPLETION_MARGIN_MS);
            }
        } else {
            g_status.state = Z_AXIS_STATE_FAULT;
            g_deadline_active = 0U;
        }
        return;
    }

    if ((state != Z_AXIS_STATE_WAIT_ACCEPT) &&
        (state != Z_AXIS_STATE_MOVING) &&
        (state != Z_AXIS_STATE_STOPPING)) {
        if ((state == Z_AXIS_STATE_IDLE) &&
            (status == Z_AXIS_STATUS_STOPPED)) {
            publish_motion_result(status, g_status.completed_steps);
            return;
        }
        ++g_status.unexpected_frames;
        return;
    }
    if ((status == Z_AXIS_STATUS_ACCEPTED) &&
        (state == Z_AXIS_STATE_WAIT_ACCEPT)) {
        if ((g_status.actual_speed_hz < Z_AXIS_MIN_SPEED_HZ) ||
            (g_status.actual_speed_hz > Z_AXIS_MAX_SPEED_HZ) ||
            (g_status.completed_steps == 0U)) {
            g_status.state = Z_AXIS_STATE_FAULT;
            g_deadline_active = 0U;
            ++g_status.unexpected_frames;
            return;
        }
        uint32_t timeout = completion_timeout(g_status.completed_steps,
                                              g_status.actual_speed_hz);
        g_status.state = Z_AXIS_STATE_MOVING;
        set_deadline(now, timeout);
    } else if ((status == Z_AXIS_STATUS_COMPLETE) ||
               (status == Z_AXIS_STATUS_STOPPED)) {
        publish_motion_result(status, g_status.completed_steps);
        if ((state != Z_AXIS_STATE_STOPPING) ||
            (g_stop_ack_received != 0U)) {
            g_status.state = Z_AXIS_STATE_IDLE;
            g_deadline_active = 0U;
        }
    } else {
        g_status.state = Z_AXIS_STATE_FAULT;
        g_deadline_active = 0U;
    }
}

void ZAxisLink_Init(uint32_t now)
{
    memset((void *)&g_status, 0, sizeof(g_status));
    g_status.state = Z_AXIS_STATE_IDLE;
    g_status.rx_ready = 0U;
    g_status.last_response_tick = now;
    g_deadline_active = 0U;
    g_active_direction = 0U;
    g_stop_ack_received = 0U;
    ZAxisLink_ResetStream();
}

ZAxisRequestResult ZAxisLink_MoveRelative(int32_t pulses,
                                          uint32_t speed_hz,
                                          uint32_t now)
{
    uint32_t steps;

    if ((pulses == 0) ||
        (speed_hz < Z_AXIS_MIN_SPEED_HZ) ||
        (speed_hz > Z_AXIS_MAX_SPEED_HZ)) {
        return Z_AXIS_REQUEST_INVALID_ARGUMENT;
    }
    if ((g_status.rx_ready == 0U) ||
        (g_status.state != Z_AXIS_STATE_IDLE) ||
        (huart4.gState != HAL_UART_STATE_READY)) {
        return Z_AXIS_REQUEST_BUSY;
    }

    steps = (pulses < 0) ? (uint32_t)(-(int64_t)pulses) : (uint32_t)pulses;
    /* Mechanical mapping: protocol DIR=1 moves away from the zero point. */
    build_frame(Z_AXIS_COMMAND_MOVE, (pulses > 0) ? 1U : 0U,
                speed_hz, steps);
    g_active_direction = (pulses < 0) ? 1U : 0U;
    g_status.move_active = 1U;
    g_status.state = Z_AXIS_STATE_WAIT_ACCEPT;
    set_deadline(now, Z_AXIS_ACK_TIMEOUT_MS);
    if (UART4_TransmitDMA(g_tx_frame, sizeof(g_tx_frame)) != HAL_OK) {
        g_status.state = Z_AXIS_STATE_IDLE;
        g_status.move_active = 0U;
        g_deadline_active = 0U;
        return Z_AXIS_REQUEST_IO_ERROR;
    }
    return Z_AXIS_REQUEST_OK;
}

ZAxisRequestResult ZAxisLink_Stop(uint32_t now)
{
    if ((g_status.rx_ready == 0U) ||
        (huart4.gState != HAL_UART_STATE_READY)) {
        return Z_AXIS_REQUEST_BUSY;
    }
    build_frame(Z_AXIS_COMMAND_STOP, 0U, 0U, 0U);
    g_stop_ack_received = 0U;
    g_status.state = Z_AXIS_STATE_STOPPING;
    set_deadline(now, Z_AXIS_ACK_TIMEOUT_MS);
    if (UART4_TransmitDMA(g_tx_frame, sizeof(g_tx_frame)) != HAL_OK) {
        g_status.state = Z_AXIS_STATE_FAULT;
        g_deadline_active = 0U;
        return Z_AXIS_REQUEST_IO_ERROR;
    }
    return Z_AXIS_REQUEST_OK;
}

void ZAxisLink_ProcessBytes(const uint8_t *data, uint16_t length,
                            uint32_t now)
{
    uint16_t i;

    if (data == NULL) return;
    for (i = 0U; i < length; ++i) {
        uint8_t byte = data[i];
        if (g_rx_index == 0U) {
            if (byte == Z_AXIS_HEADER_0) g_rx_frame[g_rx_index++] = byte;
            continue;
        }
        if (g_rx_index == 1U) {
            if (byte == Z_AXIS_HEADER_1) {
                g_rx_frame[g_rx_index++] = byte;
            } else if (byte != Z_AXIS_HEADER_0) {
                g_rx_index = 0U;
            }
            continue;
        }
        g_rx_frame[g_rx_index++] = byte;
        if ((g_rx_index == 3U) && (byte != Z_AXIS_PAYLOAD_SIZE)) {
            ++g_status.frame_errors;
            g_rx_index = 0U;
            continue;
        }
        if (g_rx_index == Z_AXIS_FRAME_SIZE) {
            uint16_t received_crc = (uint16_t)g_rx_frame[13] |
                                    ((uint16_t)g_rx_frame[14] << 8U);
            g_rx_index = 0U;
            if ((g_rx_frame[15] != 0x0DU) || (g_rx_frame[16] != 0x0AU)) {
                ++g_status.frame_errors;
            } else if (received_crc != crc16_modbus(&g_rx_frame[2], 11U)) {
                ++g_status.crc_errors;
            } else {
                handle_response(g_rx_frame, now);
            }
        }
    }
}

void ZAxisLink_Poll(uint32_t now)
{
    if ((g_deadline_active != 0U) &&
        ((int32_t)(now - g_deadline) >= 0)) {
        g_deadline_active = 0U;
        g_status.state = Z_AXIS_STATE_FAULT;
        ++g_status.timeouts;
    }
}

void ZAxisLink_GetStatus(ZAxisStatus *status)
{
    uint32_t primask;
    if (status == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    *status = g_status;
    if (primask == 0U) __enable_irq();
}

void ZAxisLink_OnUartError(uint32_t error_code, uint32_t now)
{
    (void)error_code;
    ++g_status.uart_errors;
    g_status.rx_ready = 0U;
    g_status.last_response_tick = now;
    if (g_status.state != Z_AXIS_STATE_IDLE) {
        g_status.state = Z_AXIS_STATE_FAULT;
    }
    g_deadline_active = 0U;
    ZAxisLink_ResetStream();
}

void ZAxisLink_ClearFault(void)
{
    if ((g_status.rx_ready != 0U) &&
        (g_status.state == Z_AXIS_STATE_FAULT)) {
        g_status.state = Z_AXIS_STATE_IDLE;
        g_status.move_active = 0U;
        g_stop_ack_received = 0U;
        g_stop_ack_received = 0U;
        g_deadline_active = 0U;
    }
}

void ZAxisLink_SetRxReady(uint8_t ready)
{
    g_status.rx_ready = (ready != 0U) ? 1U : 0U;
}

void ZAxisLink_ResetStream(void)
{
    g_rx_index = 0U;
}

const char *ZAxisLink_StateString(ZAxisState state)
{
    switch (state) {
        case Z_AXIS_STATE_IDLE: return "IDLE";
        case Z_AXIS_STATE_WAIT_ACCEPT: return "WAIT_ACCEPT";
        case Z_AXIS_STATE_MOVING: return "MOVING";
        case Z_AXIS_STATE_STOPPING: return "STOPPING";
        case Z_AXIS_STATE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

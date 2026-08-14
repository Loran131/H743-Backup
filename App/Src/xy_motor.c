#include "xy_motor.h"
#include "smd.h"
#include <limits.h>
#include <stddef.h>
#include <string.h>

#define XY_QUERY_PERIOD_MS          50U
#define XY_START_ACK_TIMEOUT_MS     250U
#define XY_STOP_ACK_TIMEOUT_MS      250U
#define XY_QUERY_RESPONSE_TIMEOUT_MS 20U
#define XY_BUS_GUARD_MS             10U
#define XY_HOME_BREAK_SETTLE_MS     50U
#define XY_HOME_MAX_RETRIES         2U
#define XY_HOME_ZERO_SETTLE_MS      300U
#define XY_MOVE_STATIC_GUARD_MS     100U
#define XY_X_SETTLE_TOLERANCE_PULSES 1024
#define XY_X_SETTLE_POSITION_SAMPLES 3U

typedef struct {
    XY_AxisStatus status;
    uint32_t last_query_tick;
    uint8_t query_phase;
    uint8_t waiting_move_ack;
    uint8_t waiting_stop_ack;
    uint8_t waiting_home_ack;
    uint8_t waiting_zero_ack;
    uint8_t home_stage;
    int32_t raw_position_pulses;
    int32_t zero_offset_pulses;
    uint32_t home_start_tick;
    uint32_t home_next_action_tick;
    uint8_t settle_position_count;
} XY_AxisRuntime;

enum {
    XY_HOME_STAGE_IDLE = 0,
    XY_HOME_STAGE_BREAK,
    XY_HOME_STAGE_CLEAR,
    XY_HOME_STAGE_ENABLE,
    XY_HOME_STAGE_PARAMS,
    XY_HOME_STAGE_CLOG_OFF,
    XY_HOME_STAGE_START,
    XY_HOME_STAGE_RUNNING,
    XY_HOME_STAGE_CAPTURE_ZERO,
    XY_HOME_STAGE_CLOG_ON
};

static XY_AxisConfig g_xy_config[XY_AXIS_COUNT] = {
    {
        .motor_address = XY_X_MOTOR_ADDRESS,
        .positive_direction = XY_POSITIVE_DIRECTION,
        .acceleration = 100U,
        .default_speed_rpm = 3000U,
        .max_speed_rpm = 3000U,
        .soft_min_pulses = 0,
        .soft_max_pulses = 2425263,
        .home_mode = 0U,
        .home_direction = XY_NEGATIVE_DIRECTION,
        .home_speed_rpm = 100U,
        .home_current_ma = 600,
        .home_timeout_ms = 30000U
    },
    {
        .motor_address = XY_Y_MOTOR_ADDRESS,
        .positive_direction = XY_POSITIVE_DIRECTION,
        .acceleration = 200U,
        .default_speed_rpm = 5U,
        .max_speed_rpm = 15U,
        .soft_min_pulses = 0,
        .soft_max_pulses = 102400,
        .home_mode = 0U,
        .home_direction = XY_NEGATIVE_DIRECTION,
        /* Y mechanics require a deliberately low sensorless-homing speed. */
        .home_speed_rpm = 10U,
        .home_current_ma = 400,
        .home_timeout_ms = 30000U
    }
};

static XY_AxisRuntime g_xy_runtime[XY_AXIS_COUNT];
static XY_StartupStatus g_xy_startup;
static uint32_t g_next_query_tick;
static uint32_t g_query_sent_tick;
static uint8_t g_query_outstanding;
static uint8_t g_query_address;
static uint8_t g_query_function;
static XY_Axis g_query_axis;
static uint8_t g_emergency_stop_pending;
static uint32_t g_emergency_stop_tick;
static XY_StopDiagnostics g_stop_diagnostics;

static uint8_t xy_command_ack_pending(void);

static XY_Axis xy_axis_from_address(uint8_t address)
{
    if (address == XY_X_MOTOR_ADDRESS) return XY_AXIS_X;
    if (address == XY_Y_MOTOR_ADDRESS) return XY_AXIS_Y;
    return XY_AXIS_COUNT;
}

static int32_t xy_read_be_i32(const uint8_t *data)
{
    uint32_t value = ((uint32_t)data[0] << 24) |
                     ((uint32_t)data[1] << 16) |
                     ((uint32_t)data[2] << 8) |
                     (uint32_t)data[3];
    return (int32_t)value;
}

static int16_t xy_read_be_i16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static void xy_set_fault(XY_Axis axis, XY_Fault fault)
{
    XY_AxisRuntime *runtime = &g_xy_runtime[axis];
    uint32_t now = HAL_GetTick();

    if (runtime->status.state == XY_STATE_FAULT) return;
    runtime->status.fault_function = runtime->status.last_command_function;
    runtime->status.fault_home_stage = runtime->home_stage;
    runtime->status.fault = fault;
    runtime->status.state = XY_STATE_FAULT;
    runtime->waiting_move_ack = 0U;
    runtime->waiting_stop_ack = 0U;
    runtime->waiting_home_ack = 0U;
    runtime->waiting_zero_ack = 0U;
    runtime->settle_position_count = 0U;
    runtime->home_stage = XY_HOME_STAGE_IDLE;
    g_query_outstanding = 0U;
    g_emergency_stop_pending = 1U;
    g_emergency_stop_tick = now + XY_BUS_GUARD_MS;
}

static uint8_t xy_response_callback(const SmdResponse *response)
{
    XY_Axis axis = xy_axis_from_address(response->address);
    XY_AxisRuntime *runtime;

    if (axis >= XY_AXIS_COUNT) return 0U;
    runtime = &g_xy_runtime[axis];
    runtime->status.last_feedback_tick = response->rx_tick;
    runtime->status.response_seen = 1U;
    g_xy_startup.response_mask |= (uint8_t)(1U << axis);
    if ((g_query_outstanding != 0U) &&
        (response->address == g_query_address) &&
        (response->function == g_query_function)) {
        g_query_outstanding = 0U;
    }

    if (response->result != ACK_SUCCEED) {
        if ((runtime->waiting_move_ack != 0U) ||
            (runtime->waiting_home_ack != 0U) ||
            (runtime->waiting_zero_ack != 0U)) {
            xy_set_fault(axis, XY_FAULT_DRIVER_REJECTED);
            return 1U;
        }
        return 0U;
    }

    switch (response->function) {
    case FCT_POS_REL_MODE:
        runtime->waiting_move_ack = 0U;
        if (runtime->status.state == XY_STATE_STARTING) {
            runtime->status.state = XY_STATE_MOVING;
        }
        return 1U;

    case FCT_STOP_NOW:
        if (runtime->waiting_stop_ack == 0U) return 1U;
        runtime->waiting_stop_ack = 0U;
        runtime->status.target_pulses = runtime->status.position_pulses;
        runtime->status.state = runtime->status.position_valid ?
                                XY_STATE_IDLE : XY_STATE_UNREFERENCED;
        runtime->status.completion_tick = response->rx_tick;
        runtime->status.completion_source = XY_COMPLETION_STOP_ACK;
        runtime->settle_position_count = 0U;
        return 1U;

    case FCT_ANGLE_ZERO:
        if (runtime->waiting_zero_ack != 0U) {
            runtime->waiting_zero_ack = 0U;
            runtime->raw_position_pulses = 0;
            runtime->zero_offset_pulses = 0;
            runtime->status.position_pulses = 0;
            runtime->status.target_pulses = 0;
            runtime->status.position_valid = 1U;
            runtime->status.fault = XY_FAULT_NONE;
            runtime->status.state = XY_STATE_IDLE;
            runtime->status.completion_tick = response->rx_tick;
            runtime->status.completion_source = XY_COMPLETION_ZERO_ACK;
            return 1U;
        }
        return 0U;

    case FCT_READ_POS:
        if (response->data_length >= 4U) {
            runtime->raw_position_pulses = xy_read_be_i32(response->data);
            if ((runtime->status.state == XY_STATE_HOMING) &&
                (runtime->home_stage == XY_HOME_STAGE_CAPTURE_ZERO)) {
                runtime->zero_offset_pulses = runtime->raw_position_pulses;
                runtime->status.position_pulses = 0;
                runtime->status.target_pulses = 0;
                runtime->status.position_valid = 1U;
                runtime->waiting_home_ack = 0U;
                runtime->status.home_retry_count = 0U;
                runtime->home_stage = XY_HOME_STAGE_CLOG_ON;
                runtime->home_next_action_tick = response->rx_tick +
                                                 XY_BUS_GUARD_MS;
            } else if (runtime->status.position_valid != 0U) {
                int64_t position_error;
                runtime->status.position_pulses =
                    runtime->raw_position_pulses - runtime->zero_offset_pulses;
                position_error = (int64_t)runtime->status.position_pulses -
                                 runtime->status.target_pulses;
                if ((axis == XY_AXIS_X) &&
                    (runtime->status.state == XY_STATE_MOVING) &&
                    (position_error >= -XY_X_SETTLE_TOLERANCE_PULSES) &&
                    (position_error <= XY_X_SETTLE_TOLERANCE_PULSES)) {
                    if (runtime->settle_position_count <
                        XY_X_SETTLE_POSITION_SAMPLES) {
                        ++runtime->settle_position_count;
                    }
                    if (runtime->settle_position_count >=
                        XY_X_SETTLE_POSITION_SAMPLES) {
                        runtime->status.target_pulses =
                            runtime->status.position_pulses;
                        runtime->status.state = XY_STATE_IDLE;
                        runtime->status.completion_tick = response->rx_tick;
                        runtime->status.completion_source =
                            XY_COMPLETION_TOLERANCE;
                        ++runtime->status.tolerance_release_count;
                        runtime->settle_position_count = 0U;
                    }
                } else {
                    runtime->settle_position_count = 0U;
                }
            }
        }
        return 1U;

    case FCT_READ_ROTATE_SPEED:
        if (response->data_length >= 2U) {
            runtime->status.speed_rpm = xy_read_be_i16(response->data);
        }
        return 1U;

    case FCT_READ_MOTOR_STA:
        if (response->data_length >= 1U) {
            runtime->status.motor_status = response->data[0];
            if (response->data[0] == MOTOR_STA_STATIC) {
                runtime->status.last_static_reply_tick = response->rx_tick;
            }
            if (response->data[0] == MOTOR_STA_STALL) {
                xy_set_fault(axis, XY_FAULT_STALL);
            } else if (response->data[0] == MOTOR_STA_UNDERVOLTAGE) {
                xy_set_fault(axis, XY_FAULT_UNDERVOLTAGE);
            } else if ((response->data[0] == MOTOR_STA_STATIC) &&
                       (runtime->status.state == XY_STATE_STOPPING)) {
                runtime->status.target_pulses = runtime->status.position_pulses;
                runtime->status.state = runtime->status.position_valid ?
                                        XY_STATE_IDLE : XY_STATE_UNREFERENCED;
            } else if ((response->data[0] == MOTOR_STA_STATIC) &&
                       (runtime->status.state == XY_STATE_MOVING) &&
                       ((uint32_t)(response->rx_tick -
                                   runtime->status.command_tick) >=
                        XY_MOVE_STATIC_GUARD_MS)) {
                /* Some drivers leave ARRIVED low after a completed relative
                 * move. Static plus a fresh position safely releases BUSY. */
                runtime->status.target_pulses = runtime->status.position_pulses;
                runtime->status.state = XY_STATE_IDLE;
                runtime->status.completion_tick = response->rx_tick;
                runtime->status.completion_source = XY_COMPLETION_STATIC;
                ++runtime->status.static_release_count;
            }
        }
        return 1U;

    case FCT_READ_ARRIVED_STA:
        if (response->data_length >= 1U) {
            runtime->status.arrived = response->data[0] ? 1U : 0U;
            runtime->status.last_arrived_reply_tick = response->rx_tick;
            if ((runtime->status.arrived != 0U) &&
                (runtime->status.state == XY_STATE_MOVING)) {
                runtime->status.position_pulses = runtime->status.target_pulses;
                runtime->status.state = XY_STATE_IDLE;
                runtime->status.completion_tick = response->rx_tick;
                runtime->status.completion_source = XY_COMPLETION_ARRIVED;
                ++runtime->status.arrived_release_count;
            }
        }
        return 1U;

    case FCT_ORIGIN_LIMIT_HOME:
        if (runtime->home_stage == XY_HOME_STAGE_START) {
            runtime->home_stage = XY_HOME_STAGE_RUNNING;
            runtime->waiting_home_ack = 0U;
            return 1U;
        }
        return 0U;

    case FCT_ORIGIN_BREAK:
        if (runtime->home_stage == XY_HOME_STAGE_BREAK) {
            runtime->home_stage = XY_HOME_STAGE_CLEAR;
            runtime->waiting_home_ack = 0U;
            runtime->status.home_retry_count = 0U;
            runtime->home_next_action_tick = response->rx_tick +
                                             XY_HOME_BREAK_SETTLE_MS;
            return 1U;
        }
        return 0U;

    case FCT_CLEAR_STATE:
        if (runtime->home_stage == XY_HOME_STAGE_CLEAR) {
            runtime->home_stage = XY_HOME_STAGE_ENABLE;
            runtime->waiting_home_ack = 0U;
            runtime->status.home_retry_count = 0U;
            runtime->home_next_action_tick = response->rx_tick + XY_BUS_GUARD_MS;
            return 1U;
        }
        return 0U;

    case FCT_MOTOR_ENABLE:
        if (runtime->home_stage == XY_HOME_STAGE_ENABLE) {
            runtime->home_stage = XY_HOME_STAGE_PARAMS;
            runtime->waiting_home_ack = 0U;
            runtime->status.home_retry_count = 0U;
            runtime->home_next_action_tick = response->rx_tick + XY_BUS_GUARD_MS;
            return 1U;
        }
        return 0U;

    case FCT_ORIGIN_SET_PARAMS:
        if (runtime->home_stage == XY_HOME_STAGE_PARAMS) {
            runtime->home_stage = XY_HOME_STAGE_CLOG_OFF;
            runtime->waiting_home_ack = 0U;
            runtime->status.home_retry_count = 0U;
            runtime->home_next_action_tick = response->rx_tick + XY_BUS_GUARD_MS;
            return 1U;
        }
        return 0U;

    case FCT_SET_CLOG_PRO:
        if (runtime->home_stage == XY_HOME_STAGE_CLOG_OFF) {
            runtime->home_stage = XY_HOME_STAGE_START;
            runtime->waiting_home_ack = 0U;
            runtime->status.home_retry_count = 0U;
            runtime->home_next_action_tick = response->rx_tick + XY_BUS_GUARD_MS;
            return 1U;
        }
        if (runtime->home_stage == XY_HOME_STAGE_CLOG_ON) {
            runtime->waiting_home_ack = 0U;
            runtime->status.home_retry_count = 0U;
            runtime->status.state = XY_STATE_IDLE;
            runtime->status.completion_tick = response->rx_tick;
            runtime->status.completion_source = XY_COMPLETION_HOME;
            runtime->home_stage = XY_HOME_STAGE_IDLE;
            return 1U;
        }
        return 0U;

    case FCT_ORIGIN_READ_STA:
        if ((response->data_length >= 1U) &&
            (runtime->status.state == XY_STATE_HOMING)) {
            if (response->data[0] == 2U) {
                runtime->status.fault = XY_FAULT_NONE;
                runtime->status.position_valid = 0U;
                runtime->home_stage = XY_HOME_STAGE_CAPTURE_ZERO;
                runtime->home_next_action_tick = response->rx_tick +
                                                 XY_HOME_ZERO_SETTLE_MS;
            } else if (response->data[0] == 3U) {
                xy_set_fault(axis, XY_FAULT_HOME_FAILED);
            }
        }
        return 1U;

    default:
        return 0U;
    }
}

void XY_Motor_Init(uint32_t now)
{
    memset(g_xy_runtime, 0, sizeof(g_xy_runtime));
    for (uint8_t i = 0U; i < XY_AXIS_COUNT; ++i) {
        g_xy_runtime[i].status.state = XY_STATE_UNREFERENCED;
        g_xy_runtime[i].status.fault = XY_FAULT_NONE;
        g_xy_runtime[i].status.command_tick = now;
        g_xy_runtime[i].status.last_feedback_tick = now;
    }
    smd_set_response_callback(xy_response_callback);
    g_next_query_tick = now;
    g_query_sent_tick = now;
    g_query_outstanding = 0U;
    g_query_axis = XY_AXIS_X;
    g_emergency_stop_pending = 0U;
    memset(&g_stop_diagnostics, 0, sizeof(g_stop_diagnostics));
    g_emergency_stop_tick = now;
    memset(&g_xy_startup, 0, sizeof(g_xy_startup));
    g_xy_startup.state = XY_STARTUP_WAITING_REPLIES;
    g_xy_startup.start_tick = now;
}

static void xy_poll_startup_homing(uint32_t now)
{
    XY_Result result;

    switch (g_xy_startup.state) {
    case XY_STARTUP_WAITING_REPLIES:
        if ((g_xy_runtime[XY_AXIS_X].status.state == XY_STATE_HOMING) ||
            (g_xy_runtime[XY_AXIS_Y].status.state == XY_STATE_HOMING)) {
            g_xy_startup.state = XY_STARTUP_FAILED;
            g_xy_startup.finish_tick = now;
            break;
        }
        if (g_xy_startup.response_mask ==
            ((1U << XY_AXIS_X) | (1U << XY_AXIS_Y))) {
            g_xy_startup.state = XY_STARTUP_WAITING_TO_HOME_X;
        } else if ((uint32_t)(now - g_xy_startup.start_tick) >
                   XY_STARTUP_REPLY_TIMEOUT_MS) {
            g_xy_startup.state = XY_STARTUP_SKIPPED_NO_REPLY;
            g_xy_startup.finish_tick = now;
        }
        break;

    case XY_STARTUP_WAITING_TO_HOME_X:
        if ((g_query_outstanding == 0U) &&
            (xy_command_ack_pending() == 0U)) {
            result = XY_HomeSensorless(XY_AXIS_X);
            if (result == XY_RESULT_OK) {
                g_xy_startup.state = XY_STARTUP_HOMING_X;
            } else if (result != XY_RESULT_BUSY) {
                g_xy_startup.state = XY_STARTUP_FAILED;
                g_xy_startup.finish_tick = now;
            }
        }
        break;

    case XY_STARTUP_HOMING_X:
        if (g_xy_runtime[XY_AXIS_X].status.state == XY_STATE_IDLE) {
            g_xy_startup.state = XY_STARTUP_WAITING_TO_HOME_Y;
        } else if (g_xy_runtime[XY_AXIS_X].status.state == XY_STATE_FAULT) {
            g_xy_startup.state = XY_STARTUP_FAILED;
            g_xy_startup.finish_tick = now;
        }
        break;

    case XY_STARTUP_WAITING_TO_HOME_Y:
        if ((g_query_outstanding == 0U) &&
            (xy_command_ack_pending() == 0U)) {
            result = XY_HomeSensorless(XY_AXIS_Y);
            if (result == XY_RESULT_OK) {
                g_xy_startup.state = XY_STARTUP_HOMING_Y;
            } else if (result != XY_RESULT_BUSY) {
                g_xy_startup.state = XY_STARTUP_FAILED;
                g_xy_startup.finish_tick = now;
            }
        }
        break;

    case XY_STARTUP_HOMING_Y:
        if (g_xy_runtime[XY_AXIS_Y].status.state == XY_STATE_IDLE) {
            g_xy_startup.state = XY_STARTUP_COMPLETE;
            g_xy_startup.finish_tick = now;
        } else if (g_xy_runtime[XY_AXIS_Y].status.state == XY_STATE_FAULT) {
            g_xy_startup.state = XY_STARTUP_FAILED;
            g_xy_startup.finish_tick = now;
        }
        break;

    default:
        break;
    }
}

static uint8_t xy_command_ack_pending(void)
{
    for (uint8_t i = 0U; i < XY_AXIS_COUNT; ++i) {
        if ((g_xy_runtime[i].waiting_move_ack != 0U) ||
            (g_xy_runtime[i].waiting_stop_ack != 0U) ||
            (g_xy_runtime[i].waiting_home_ack != 0U) ||
            (g_xy_runtime[i].waiting_zero_ack != 0U)) return 1U;
    }
    return 0U;
}

static uint8_t xy_poll_home_action(XY_Axis axis, uint32_t now)
{
    XY_AxisRuntime *runtime = &g_xy_runtime[axis];
    const XY_AxisConfig *config = &g_xy_config[axis];
    uint8_t data[8];
    uint8_t length;
    uint8_t function;

    if ((runtime->status.state != XY_STATE_HOMING) ||
        (runtime->waiting_home_ack != 0U) ||
        (runtime->home_stage == XY_HOME_STAGE_IDLE) ||
        (runtime->home_stage == XY_HOME_STAGE_RUNNING) ||
        ((int32_t)(now - runtime->home_next_action_tick) < 0)) {
        return 0U;
    }

    switch (runtime->home_stage) {
    case XY_HOME_STAGE_BREAK:
        function = FCT_ORIGIN_BREAK;
        length = 0U;
        break;
    case XY_HOME_STAGE_CLEAR:
        function = FCT_CLEAR_STATE;
        length = 0U;
        break;
    case XY_HOME_STAGE_ENABLE:
        function = FCT_MOTOR_ENABLE;
        data[0] = 0U;
        length = 1U;
        break;
    case XY_HOME_STAGE_PARAMS:
        function = FCT_ORIGIN_SET_PARAMS;
        data[0] = (uint8_t)(config->home_timeout_ms >> 24);
        data[1] = (uint8_t)(config->home_timeout_ms >> 16);
        data[2] = (uint8_t)(config->home_timeout_ms >> 8);
        data[3] = (uint8_t)config->home_timeout_ms;
        length = 4U;
        break;
    case XY_HOME_STAGE_CLOG_OFF:
        function = FCT_SET_CLOG_PRO;
        data[0] = 0U;
        length = 1U;
        break;
    case XY_HOME_STAGE_CLOG_ON:
        function = FCT_SET_CLOG_PRO;
        data[0] = 1U;
        length = 1U;
        break;
    case XY_HOME_STAGE_CAPTURE_ZERO:
        function = FCT_READ_POS;
        length = 0U;
        break;
    case XY_HOME_STAGE_START:
        function = FCT_ORIGIN_LIMIT_HOME;
        data[0] = config->home_mode;
        data[1] = config->home_direction;
        data[2] = (uint8_t)(config->home_speed_rpm >> 24);
        data[3] = (uint8_t)(config->home_speed_rpm >> 16);
        data[4] = (uint8_t)(config->home_speed_rpm >> 8);
        data[5] = (uint8_t)config->home_speed_rpm;
        data[6] = (uint8_t)((uint16_t)config->home_current_ma >> 8);
        data[7] = (uint8_t)config->home_current_ma;
        length = 8U;
        break;
    default:
        return 0U;
    }

    if (smd_send_cmd(config->motor_address, function,
                     (length != 0U) ? data : NULL, length) != 0U) {
        xy_set_fault(axis, XY_FAULT_DRIVER_REJECTED);
        return 0U;
    }
    runtime->waiting_home_ack = 1U;
    runtime->status.last_command_function = function;
    runtime->status.command_tick = now;
    return 1U;
}

static void xy_send_next_query(uint32_t now)
{
    XY_Axis axis;
    XY_AxisRuntime *runtime;
    uint8_t address;
    uint8_t function;

    if (g_query_outstanding != 0U) {
        if ((uint32_t)(now - g_query_sent_tick) <=
            XY_QUERY_RESPONSE_TIMEOUT_MS) return;
        g_query_outstanding = 0U;
    }
    if ((int32_t)(now - g_next_query_tick) < 0) return;
    if (xy_command_ack_pending() != 0U) return;

    axis = g_query_axis;
    runtime = &g_xy_runtime[axis];
    address = g_xy_config[axis].motor_address;
    runtime->last_query_tick = now;

    if (runtime->status.state == XY_STATE_HOMING) {
        if (runtime->home_stage == XY_HOME_STAGE_RUNNING) {
            function = FCT_ORIGIN_READ_STA;
        } else {
            return;
        }
    } else {
        switch (runtime->query_phase) {
        case 0U: function = FCT_READ_POS; break;
        case 1U: function = FCT_READ_ARRIVED_STA; break;
        case 2U: function = FCT_READ_MOTOR_STA; break;
        default:
            function = ((runtime->status.state == XY_STATE_STARTING) ||
                        (runtime->status.state == XY_STATE_MOVING)) ?
                       FCT_READ_ARRIVED_STA : FCT_READ_ROTATE_SPEED;
            break;
        }
        runtime->query_phase = (uint8_t)((runtime->query_phase + 1U) & 3U);
    }

    if (smd_send_cmd(address, function, NULL, 0U) == 0U) {
        g_query_address = address;
        g_query_function = function;
        g_query_sent_tick = now;
        g_query_outstanding = 1U;
        g_query_axis = (axis == XY_AXIS_X) ? XY_AXIS_Y : XY_AXIS_X;
        g_next_query_tick = now + (XY_QUERY_PERIOD_MS / 2U);
    }
}

void XY_Motor_Poll(uint32_t now)
{
    for (uint8_t i = 0U; i < XY_AXIS_COUNT; ++i) {
        XY_AxisRuntime *runtime = &g_xy_runtime[i];
        XY_AxisConfig *config = &g_xy_config[i];

        if (((runtime->waiting_move_ack != 0U) ||
             (runtime->waiting_home_ack != 0U) ||
             (runtime->waiting_zero_ack != 0U)) &&
            ((uint32_t)(now - runtime->status.command_tick) >
             XY_START_ACK_TIMEOUT_MS)) {
            if ((runtime->waiting_home_ack != 0U) &&
                (runtime->home_stage != XY_HOME_STAGE_START) &&
                (runtime->home_stage != XY_HOME_STAGE_RUNNING) &&
                (runtime->status.home_retry_count < XY_HOME_MAX_RETRIES)) {
                runtime->waiting_home_ack = 0U;
                ++runtime->status.home_retry_count;
                runtime->home_next_action_tick = now + XY_BUS_GUARD_MS;
            } else {
                xy_set_fault((XY_Axis)i, XY_FAULT_FEEDBACK_TIMEOUT);
            }
        }
        if ((runtime->waiting_stop_ack != 0U) &&
            ((uint32_t)(now - runtime->status.command_tick) >
             XY_STOP_ACK_TIMEOUT_MS)) {
            xy_set_fault((XY_Axis)i, XY_FAULT_STOP_UNCONFIRMED);
        }
        if ((runtime->status.state == XY_STATE_STOPPING) &&
            ((uint32_t)(now - runtime->status.command_tick) >
             XY_STOP_ACK_TIMEOUT_MS)) {
            xy_set_fault((XY_Axis)i, XY_FAULT_STOP_UNCONFIRMED);
        }
        if ((runtime->status.state == XY_STATE_HOMING) &&
            ((uint32_t)(now - runtime->home_start_tick) >
             config->home_timeout_ms)) {
            xy_set_fault((XY_Axis)i, XY_FAULT_HOME_FAILED);
        }
        if (((runtime->status.state == XY_STATE_STARTING) ||
             (runtime->status.state == XY_STATE_MOVING)) &&
            ((uint32_t)(now - runtime->status.command_tick) >
             XY_MOVE_TIMEOUT_MS)) {
            xy_set_fault((XY_Axis)i, XY_FAULT_COMMAND_TIMEOUT);
        }
        if (((runtime->status.state == XY_STATE_STARTING) ||
             (runtime->status.state == XY_STATE_MOVING)) &&
            ((uint32_t)(now - runtime->status.last_feedback_tick) >
             XY_FEEDBACK_TIMEOUT_MS)) {
            xy_set_fault((XY_Axis)i, XY_FAULT_FEEDBACK_TIMEOUT);
        }
    }
    if ((g_emergency_stop_pending != 0U) &&
        ((int32_t)(now - g_emergency_stop_tick) >= 0)) {
        (void)smd_send_cmd(SMD_BROADCAST_ADDR, FCT_STOP_NOW, NULL, 0U);
        ++g_stop_diagnostics.fault_broadcast_count;
        g_emergency_stop_pending = 0U;
        return;
    }
    if (xy_command_ack_pending() == 0U) {
        for (uint8_t i = 0U; i < XY_AXIS_COUNT; ++i) {
            if (xy_poll_home_action((XY_Axis)i, now) != 0U) return;
        }
    }
    xy_send_next_query(now);
    xy_poll_startup_homing(now);
}

static XY_Result xy_validate_move(XY_Axis axis, int32_t target_pulses,
                                  uint32_t magnitude, uint16_t speed_rpm,
                                  uint8_t acceleration)
{
    const XY_AxisConfig *config;
    const XY_AxisStatus *status;

    if (axis >= XY_AXIS_COUNT) return XY_RESULT_INVALID_AXIS;
    if (g_query_outstanding != 0U) return XY_RESULT_BUSY;
    if (xy_command_ack_pending() != 0U) return XY_RESULT_BUSY;
    config = &g_xy_config[axis];
    status = &g_xy_runtime[axis].status;
    if (status->state == XY_STATE_FAULT) return XY_RESULT_FAULT;
    if ((status->position_valid == 0U) ||
        (status->state == XY_STATE_UNREFERENCED)) return XY_RESULT_NOT_REFERENCED;
    if (status->state != XY_STATE_IDLE) return XY_RESULT_BUSY;
    if ((speed_rpm == 0U) || (speed_rpm > config->max_speed_rpm)) {
        return XY_RESULT_INVALID_SPEED;
    }
    if (acceleration > 200U) return XY_RESULT_INVALID_ACCELERATION;
    if (magnitude == 0U) return XY_RESULT_INVALID_PULSES;
    if ((target_pulses < config->soft_min_pulses) ||
        (target_pulses > config->soft_max_pulses)) return XY_RESULT_SOFT_LIMIT;
    return XY_RESULT_OK;
}

XY_Result XY_MoveRelative(XY_Axis axis, int32_t delta_pulses,
                          uint16_t speed_rpm, uint8_t acceleration)
{
    XY_AxisRuntime *runtime;
    const XY_AxisConfig *config;
    int64_t target64;
    uint32_t magnitude;
    uint8_t direction;
    XY_Result result;

    if (axis >= XY_AXIS_COUNT) return XY_RESULT_INVALID_AXIS;
    runtime = &g_xy_runtime[axis];
    config = &g_xy_config[axis];
    if (delta_pulses == 0) return XY_RESULT_INVALID_PULSES;
    target64 = (int64_t)runtime->status.position_pulses + delta_pulses;
    if ((target64 < INT32_MIN) || (target64 > INT32_MAX)) {
        return XY_RESULT_SOFT_LIMIT;
    }
    magnitude = (delta_pulses > 0) ? (uint32_t)delta_pulses :
                (uint32_t)(-(int64_t)delta_pulses);
    result = xy_validate_move(axis, (int32_t)target64, magnitude,
                              speed_rpm, acceleration);
    if (result != XY_RESULT_OK) return result;

    direction = (delta_pulses > 0) ? config->positive_direction :
                (uint8_t)(config->positive_direction ^ 1U);
    if (smd_pos_rel_move(config->motor_address, direction, acceleration,
                         speed_rpm, magnitude) != 0U) {
        return XY_RESULT_CAN_REJECTED;
    }
    runtime->status.target_pulses = (int32_t)target64;
    runtime->status.arrived = 0U;
    runtime->status.completion_tick = 0U;
    runtime->status.completion_source = XY_COMPLETION_NONE;
    runtime->status.command_tick = HAL_GetTick();
    runtime->status.last_command_function = FCT_POS_REL_MODE;
    runtime->status.state = XY_STATE_STARTING;
    runtime->waiting_move_ack = 1U;
    runtime->settle_position_count = 0U;
    return XY_RESULT_OK;
}

XY_Result XY_MoveAbsolute(XY_Axis axis, int32_t target_pulses,
                          uint16_t speed_rpm, uint8_t acceleration)
{
    int64_t delta;
    if (axis >= XY_AXIS_COUNT) return XY_RESULT_INVALID_AXIS;
    delta = (int64_t)target_pulses -
            g_xy_runtime[axis].status.position_pulses;
    if (delta == 0) return XY_RESULT_OK;
    if ((delta < INT32_MIN) || (delta > INT32_MAX)) return XY_RESULT_SOFT_LIMIT;
    return XY_MoveRelative(axis, (int32_t)delta, speed_rpm, acceleration);
}

void XY_Stop(XY_Axis axis)
{
    XY_AxisRuntime *runtime;
    uint8_t address;
    if (axis >= XY_AXIS_COUNT) return;
    runtime = &g_xy_runtime[axis];
    g_query_outstanding = 0U;
    address = g_xy_config[axis].motor_address;
    (void)smd_send_cmd(address, FCT_STOP_NOW, NULL, 0U);
    runtime->status.command_tick = HAL_GetTick();
    runtime->status.last_command_function = FCT_STOP_NOW;
    runtime->status.state = XY_STATE_STOPPING;
    runtime->waiting_stop_ack = 1U;
    runtime->waiting_move_ack = 0U;
    runtime->waiting_home_ack = 0U;
    runtime->settle_position_count = 0U;
    ++g_stop_diagnostics.stop_api_count;
}

uint8_t xy_stop_all(void)
{
    uint32_t now = HAL_GetTick();

    /* One broadcast avoids simultaneous X/Y replies in the shared RX stream. */
    g_query_outstanding = 0U;
    if (smd_send_cmd(SMD_BROADCAST_ADDR, FCT_STOP_NOW, NULL, 0U) != 0U) {
        return 0U;
    }
    ++g_stop_diagnostics.stop_all_count;
    for (uint8_t i = 0U; i < XY_AXIS_COUNT; ++i) {
        XY_AxisRuntime *runtime = &g_xy_runtime[i];
        runtime->status.command_tick = now;
        runtime->status.position_valid = 0U;
        runtime->status.state = XY_STATE_UNREFERENCED;
        runtime->waiting_move_ack = 0U;
        runtime->waiting_stop_ack = 0U;
        runtime->waiting_home_ack = 0U;
        runtime->waiting_zero_ack = 0U;
        runtime->home_stage = XY_HOME_STAGE_IDLE;
        runtime->settle_position_count = 0U;
    }
    return 1U;
}

XY_Result XY_HomeSensorless(XY_Axis axis)
{
    XY_AxisRuntime *runtime;
    XY_AxisConfig *config;
    if (axis >= XY_AXIS_COUNT) return XY_RESULT_INVALID_AXIS;
    runtime = &g_xy_runtime[axis];
    config = &g_xy_config[axis];
    if ((runtime->status.state != XY_STATE_IDLE) &&
        (runtime->status.state != XY_STATE_UNREFERENCED)) return XY_RESULT_BUSY;
    if (g_query_outstanding != 0U) return XY_RESULT_BUSY;
    if (xy_command_ack_pending() != 0U) return XY_RESULT_BUSY;

    if (smd_send_cmd(config->motor_address, FCT_ORIGIN_BREAK, NULL, 0U) != 0U) {
        return XY_RESULT_CAN_REJECTED;
    }
    runtime->status.position_valid = 0U;
    runtime->status.fault = XY_FAULT_NONE;
    runtime->status.state = XY_STATE_HOMING;
    runtime->status.command_tick = HAL_GetTick();
    runtime->status.last_command_function = FCT_ORIGIN_BREAK;
    runtime->status.home_retry_count = 0U;
    runtime->home_start_tick = runtime->status.command_tick;
    runtime->home_next_action_tick = runtime->status.command_tick;
    runtime->status.last_feedback_tick = runtime->status.command_tick;
    runtime->waiting_home_ack = 1U;
    runtime->home_stage = XY_HOME_STAGE_BREAK;
    return XY_RESULT_OK;
}

XY_Result XY_SetCurrentPositionAsZero(XY_Axis axis)
{
    XY_AxisRuntime *runtime;
    if (axis >= XY_AXIS_COUNT) return XY_RESULT_INVALID_AXIS;
    runtime = &g_xy_runtime[axis];
    if ((runtime->status.state != XY_STATE_IDLE) &&
        (runtime->status.state != XY_STATE_UNREFERENCED)) return XY_RESULT_BUSY;
    if (g_query_outstanding != 0U) return XY_RESULT_BUSY;
    if (xy_command_ack_pending() != 0U) return XY_RESULT_BUSY;
    if (smd_send_cmd(g_xy_config[axis].motor_address, FCT_ANGLE_ZERO,
                     NULL, 0U) != 0U) {
        return XY_RESULT_CAN_REJECTED;
    }
    runtime->status.position_valid = 0U;
    runtime->status.state = XY_STATE_STARTING;
    runtime->status.command_tick = HAL_GetTick();
    runtime->status.last_command_function = FCT_ANGLE_ZERO;
    runtime->waiting_zero_ack = 1U;
    return XY_RESULT_OK;
}

XY_Result XY_ClearFault(XY_Axis axis)
{
    XY_AxisRuntime *runtime;
    if (axis >= XY_AXIS_COUNT) return XY_RESULT_INVALID_AXIS;
    runtime = &g_xy_runtime[axis];
    if (runtime->status.state != XY_STATE_FAULT) return XY_RESULT_OK;
    if (smd_send_cmd(g_xy_config[axis].motor_address, FCT_CLEAR_STATE,
                     NULL, 0U) != 0U) {
        return XY_RESULT_CAN_REJECTED;
    }
    runtime->status.fault = XY_FAULT_NONE;
    runtime->status.state = runtime->status.position_valid ?
                            XY_STATE_IDLE : XY_STATE_UNREFERENCED;
    runtime->status.command_tick = HAL_GetTick();
    return XY_RESULT_OK;
}

XY_Result XY_SetSoftLimits(XY_Axis axis, int32_t min_pulses,
                           int32_t max_pulses)
{
    if (axis >= XY_AXIS_COUNT) return XY_RESULT_INVALID_AXIS;
    if (min_pulses >= max_pulses) return XY_RESULT_INVALID_PULSES;
    if ((g_xy_runtime[axis].status.position_valid != 0U) &&
        ((g_xy_runtime[axis].status.position_pulses < min_pulses) ||
         (g_xy_runtime[axis].status.position_pulses > max_pulses))) {
        return XY_RESULT_SOFT_LIMIT;
    }
    g_xy_config[axis].soft_min_pulses = min_pulses;
    g_xy_config[axis].soft_max_pulses = max_pulses;
    return XY_RESULT_OK;
}

XY_Result XY_SetDefaultSpeed(XY_Axis axis, uint16_t speed_rpm)
{
    if (axis >= XY_AXIS_COUNT) return XY_RESULT_INVALID_AXIS;
    if ((speed_rpm == 0U) || (speed_rpm > g_xy_config[axis].max_speed_rpm)) {
        return XY_RESULT_INVALID_SPEED;
    }
    g_xy_config[axis].default_speed_rpm = speed_rpm;
    return XY_RESULT_OK;
}

const XY_AxisConfig *XY_GetConfig(XY_Axis axis)
{
    return (axis < XY_AXIS_COUNT) ? &g_xy_config[axis] : NULL;
}

uint8_t XY_GetStatus(XY_Axis axis, XY_AxisStatus *status)
{
    if ((axis >= XY_AXIS_COUNT) || (status == NULL)) return 0U;
    *status = g_xy_runtime[axis].status;
    return 1U;
}

uint8_t XY_AllIdle(void)
{
    return ((g_xy_runtime[XY_AXIS_X].status.state == XY_STATE_IDLE) &&
            (g_xy_runtime[XY_AXIS_Y].status.state == XY_STATE_IDLE)) ? 1U : 0U;
}

void XY_GetStartupStatus(XY_StartupStatus *status)
{
    if (status != NULL) *status = g_xy_startup;
}

void XY_GetStopDiagnostics(XY_StopDiagnostics *diagnostics)
{
    if (diagnostics != NULL) *diagnostics = g_stop_diagnostics;
}

const char *XY_ResultString(XY_Result result)
{
    static const char *const names[] = {
        "OK", "INVALID_AXIS", "NOT_REFERENCED", "BUSY", "INVALID_SPEED",
        "INVALID_ACCELERATION", "INVALID_PULSES", "SOFT_LIMIT",
        "CAN_REJECTED", "FAULT"
    };
    return ((uint32_t)result < (sizeof(names) / sizeof(names[0]))) ?
           names[result] : "UNKNOWN";
}

const char *XY_StateString(XY_State state)
{
    static const char *const names[] = {
        "UNREFERENCED", "IDLE", "STARTING", "MOVING", "STOPPING",
        "HOMING", "FAULT"
    };
    return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
           names[state] : "UNKNOWN";
}

const char *XY_FaultString(XY_Fault fault)
{
    static const char *const names[] = {
        "NONE", "COMMAND_TIMEOUT", "FEEDBACK_TIMEOUT", "DRIVER_REJECTED",
        "STALL", "UNDERVOLTAGE", "HOME_FAILED", "STOP_UNCONFIRMED"
    };
    return ((uint32_t)fault < (sizeof(names) / sizeof(names[0]))) ?
           names[fault] : "UNKNOWN";
}

const char *XY_CompletionSourceString(XY_CompletionSource source)
{
    static const char *const names[] = {
        "NONE", "ARRIVED", "STATIC", "TOLERANCE", "STOP_ACK",
        "ZERO_ACK", "HOME"
    };
    return ((uint32_t)source < (sizeof(names) / sizeof(names[0]))) ?
           names[source] : "UNKNOWN";
}

const char *XY_StartupStateString(XY_StartupState state)
{
    static const char *const names[] = {
        "WAITING_REPLIES", "WAITING_HOME_X", "HOMING_X",
        "WAITING_HOME_Y", "HOMING_Y", "COMPLETE",
        "SKIPPED_NO_REPLY", "FAILED"
    };
    return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
           names[state] : "UNKNOWN";
}

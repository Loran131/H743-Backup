#include "z_axis.h"

#include "main.h"
#include "z_axis_link.h"
#include <limits.h>
#include <stddef.h>
#include <string.h>

static ZAxisControlStatus g_z;
static uint32_t g_last_motion_result_seq;
static uint32_t g_last_uart_errors;
static uint32_t g_last_timeouts;
static uint8_t g_track_active_move;

static void z_invalidate(ZAxisControlFault fault)
{
    g_z.position_valid = 0U;
    g_z.fault = fault;
    g_z.state = Z_STATE_FAULT;
}

void ZAxis_Init(uint32_t now)
{
    ZAxisStatus link;
    memset(&g_z, 0, sizeof(g_z));
    ZAxisLink_GetStatus(&link);
    g_z.state = Z_STATE_UNREFERENCED;
    g_z.fault = Z_FAULT_NONE;
    g_z.rx_ready = link.rx_ready;
    g_z.command_tick = now;
    g_last_motion_result_seq = link.motion_result_seq;
    g_last_uart_errors = link.uart_errors;
    g_last_timeouts = link.timeouts;
    g_track_active_move = 0U;
}

void ZAxis_Poll(uint32_t now)
{
    ZAxisStatus link;
    int64_t position;

    ZAxisLink_GetStatus(&link);
    g_z.rx_ready = link.rx_ready;
    g_z.last_controller_status = link.last_status;

    if (link.motion_result_seq != g_last_motion_result_seq) {
        g_last_motion_result_seq = link.motion_result_seq;
        if (g_track_active_move == 0U) {
            ++g_z.completed_moves;
            g_z.completion_tick = now;
            g_z.target_pulses = g_z.position_pulses;
            if ((link.motion_result_status == 0x01U) ||
                (link.motion_result_status == 0x07U)) {
                g_z.state = (link.state == Z_AXIS_STATE_STOPPING) ?
                            Z_STATE_STOPPING : Z_STATE_UNREFERENCED;
                g_z.fault = Z_FAULT_NONE;
            } else {
                z_invalidate(Z_FAULT_CONTROLLER_REJECTED);
            }
        } else {
            position = (int64_t)g_z.position_pulses +
                       link.motion_result_signed_steps;
            if ((position < Z_AXIS_SOFT_MIN_PULSES) ||
                (position > Z_AXIS_SOFT_MAX_PULSES)) {
                z_invalidate(Z_FAULT_POSITION_UNCERTAIN);
            } else {
                g_z.position_pulses = (int32_t)position;
                ++g_z.completed_moves;
                g_z.completion_tick = now;
                if ((link.motion_result_status == 0x01U) ||
                    (link.motion_result_status == 0x07U)) {
                    g_z.target_pulses = g_z.position_pulses;
                    g_z.state = (link.state == Z_AXIS_STATE_STOPPING) ?
                                Z_STATE_STOPPING : Z_STATE_IDLE;
                    g_z.fault = Z_FAULT_NONE;
                } else {
                    z_invalidate(Z_FAULT_CONTROLLER_REJECTED);
                }
            }
        }
        g_track_active_move = 0U;
    }

    if (link.uart_errors != g_last_uart_errors) {
        g_last_uart_errors = link.uart_errors;
        z_invalidate(Z_FAULT_LINK);
    }
    if (link.timeouts != g_last_timeouts) {
        g_last_timeouts = link.timeouts;
        z_invalidate(Z_FAULT_TIMEOUT);
    }
    if ((link.state == Z_AXIS_STATE_FAULT) &&
        (g_z.state != Z_STATE_FAULT)) {
        z_invalidate(Z_FAULT_CONTROLLER_REJECTED);
    }

    if (g_z.state == Z_STATE_STARTING) {
        if (link.state == Z_AXIS_STATE_MOVING) g_z.state = Z_STATE_MOVING;
    } else if ((g_z.state == Z_STATE_STOPPING) &&
               (link.state == Z_AXIS_STATE_IDLE) &&
               (link.move_active == 0U)) {
        g_z.target_pulses = g_z.position_pulses;
        g_z.state = g_z.position_valid ? Z_STATE_IDLE : Z_STATE_UNREFERENCED;
    }
}

ZAxisControlResult ZAxisControl_MoveRelative(int32_t delta_pulses,
                                             uint32_t speed_hz)
{
    int64_t target;
    ZAxisRequestResult link_result;

    if (g_z.state == Z_STATE_FAULT) return Z_RESULT_FAULT;
    if ((g_z.state != Z_STATE_IDLE) &&
        (g_z.state != Z_STATE_UNREFERENCED)) return Z_RESULT_BUSY;
    if ((speed_hz < Z_AXIS_MIN_SPEED_HZ) ||
        (speed_hz > Z_AXIS_MAX_SPEED_HZ)) return Z_RESULT_INVALID_SPEED;
    if (delta_pulses == 0) return Z_RESULT_INVALID_PULSES;

    if (g_z.position_valid != 0U) {
        target = (int64_t)g_z.position_pulses + delta_pulses;
        if ((target < Z_AXIS_SOFT_MIN_PULSES) ||
            (target > Z_AXIS_SOFT_MAX_PULSES)) return Z_RESULT_SOFT_LIMIT;
    } else {
        uint32_t magnitude = (delta_pulses > 0) ? (uint32_t)delta_pulses :
                             (uint32_t)(-(int64_t)delta_pulses);
        if (magnitude > (uint32_t)Z_AXIS_SOFT_MAX_PULSES) {
            return Z_RESULT_SOFT_LIMIT;
        }
        target = g_z.position_pulses;
    }

    link_result = ZAxisLink_MoveRelative(delta_pulses, speed_hz,
                                          HAL_GetTick());
    if (link_result == Z_AXIS_REQUEST_BUSY) return Z_RESULT_BUSY;
    if (link_result != Z_AXIS_REQUEST_OK) return Z_RESULT_LINK_ERROR;

    g_z.target_pulses = (int32_t)target;
    g_z.command_speed_hz = speed_hz;
    g_z.command_tick = HAL_GetTick();
    g_track_active_move = g_z.position_valid;
    g_z.state = Z_STATE_STARTING;
    return Z_RESULT_OK;
}

ZAxisControlResult ZAxisControl_MoveAbsolute(int32_t target_pulses,
                                             uint32_t speed_hz)
{
    int64_t delta;
    if ((target_pulses < Z_AXIS_SOFT_MIN_PULSES) ||
        (target_pulses > Z_AXIS_SOFT_MAX_PULSES)) return Z_RESULT_SOFT_LIMIT;
    if ((g_z.position_valid == 0U) ||
        (g_z.state == Z_STATE_UNREFERENCED)) return Z_RESULT_NOT_REFERENCED;
    delta = (int64_t)target_pulses - g_z.position_pulses;
    if (delta == 0) return Z_RESULT_OK;
    if ((delta < INT32_MIN) || (delta > INT32_MAX)) return Z_RESULT_SOFT_LIMIT;
    return ZAxisControl_MoveRelative((int32_t)delta, speed_hz);
}

ZAxisControlResult ZAxisControl_Stop(void)
{
    ZAxisRequestResult result = ZAxisLink_Stop(HAL_GetTick());
    if (result == Z_AXIS_REQUEST_BUSY) return Z_RESULT_BUSY;
    if (result != Z_AXIS_REQUEST_OK) return Z_RESULT_LINK_ERROR;
    g_z.state = Z_STATE_STOPPING;
    g_z.command_tick = HAL_GetTick();
    return Z_RESULT_OK;
}

ZAxisControlResult ZAxisControl_SetZero(void)
{
    ZAxisStatus link;
    ZAxisLink_GetStatus(&link);
    if ((link.rx_ready == 0U) || (link.state != Z_AXIS_STATE_IDLE)) {
        return Z_RESULT_BUSY;
    }
    if ((g_z.state != Z_STATE_UNREFERENCED) &&
        (g_z.state != Z_STATE_IDLE)) {
        return Z_RESULT_BUSY;
    }
    g_z.position_pulses = 0;
    g_z.target_pulses = 0;
    g_z.position_valid = 1U;
    g_z.fault = Z_FAULT_NONE;
    g_z.state = Z_STATE_IDLE;
    return Z_RESULT_OK;
}

ZAxisControlResult ZAxisControl_ClearFault(void)
{
    ZAxisStatus link;
    if (g_z.state != Z_STATE_FAULT) return Z_RESULT_OK;
    ZAxisLink_GetStatus(&link);
    if (link.rx_ready == 0U) return Z_RESULT_LINK_ERROR;
    ZAxisLink_ClearFault();
    g_z.fault = Z_FAULT_NONE;
    g_z.position_valid = 0U;
    g_z.state = Z_STATE_UNREFERENCED;
    return Z_RESULT_OK;
}

void ZAxis_GetControlStatus(ZAxisControlStatus *status)
{
    uint32_t primask;
    if (status == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    *status = g_z;
    if (primask == 0U) __enable_irq();
}

const char *ZAxis_StateString(ZAxisControlState state)
{
    static const char *const names[] = {
        "UNREFERENCED", "IDLE", "STARTING", "MOVING", "STOPPING", "FAULT"
    };
    return (state <= Z_STATE_FAULT) ? names[state] : "UNKNOWN";
}

const char *ZAxis_ResultString(ZAxisControlResult result)
{
    static const char *const names[] = {
        "OK", "NOT_REFERENCED", "BUSY", "INVALID_SPEED",
        "INVALID_PULSES", "SOFT_LIMIT", "LINK_ERROR", "FAULT"
    };
    return (result <= Z_RESULT_FAULT) ? names[result] : "UNKNOWN";
}

const char *ZAxis_FaultString(ZAxisControlFault fault)
{
    static const char *const names[] = {
        "NONE", "LINK", "TIMEOUT", "CONTROLLER_REJECTED",
        "POSITION_UNCERTAIN"
    };
    return (fault <= Z_FAULT_POSITION_UNCERTAIN) ? names[fault] : "UNKNOWN";
}

#include "xy_vision_align.h"
#include "c552.h"
#include "motion_interfaces.h"
#include "motion_coordinator.h"
#include "xy_motor.h"
#include "xz_vision_align.h"
#include "xz_vision_calibration.h"
#include <limits.h>
#include <stddef.h>
#include <string.h>

#define XY_ALIGN_PIXEL_DEADZONE       3
#define XY_ALIGN_STABLE_SAMPLES       3U
#define XY_ALIGN_VISION_TIMEOUT_MS    500U
#define XY_ALIGN_CONTROL_TIMEOUT_MS   60000U
#define XY_ALIGN_MAX_CORRECTIONS      50U
#define XY_ALIGN_PROPORTIONAL_GAIN    0.5f
#define XY_ALIGN_X_MAX_STEP_PULSES    512000L
#define XY_ALIGN_Y_MAX_STEP_PULSES    10000L
#define XY_ALIGN_X_SPEED_RPM          300U
#define XY_ALIGN_Y_SPEED_RPM          5U
#define XY_ALIGN_ACCELERATION         200U

static XY_VisionAlignStatus g_align;
static uint32_t g_next_decision_tick;
static uint16_t g_last_decision_seq;

static int32_t align_round_float(float value)
{
    if (value >= (float)INT32_MAX) return INT32_MAX;
    if (value <= (float)INT32_MIN) return INT32_MIN;
    return (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static int32_t align_clamp(int32_t value, int32_t limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static uint8_t align_in_deadzone(int16_t value)
{
    return ((value >= -XY_ALIGN_PIXEL_DEADZONE) &&
            (value <= XY_ALIGN_PIXEL_DEADZONE)) ? 1U : 0U;
}

static uint8_t align_axes_ready(void)
{
    XY_AxisStatus x;
    XY_AxisStatus y;

    if ((XY_GetStatus(XY_AXIS_X, &x) == 0U) ||
        (XY_GetStatus(XY_AXIS_Y, &y) == 0U)) return 0U;
    return ((x.position_valid != 0U) && (y.position_valid != 0U) &&
            (x.state == XY_STATE_IDLE) && (y.state == XY_STATE_IDLE)) ? 1U : 0U;
}

static void align_record_attempt(XY_Axis axis, int32_t delta,
                                 XY_Result result)
{
    XY_AxisStatus status;
    int64_t target;

    g_align.failed_axis = axis;
    g_align.last_move_result = result;
    if (XY_GetStatus(axis, &status) != 0U) {
        g_align.axis_position[axis] = status.position_pulses;
        target = (int64_t)status.position_pulses + delta;
        g_align.attempted_target[axis] =
            (target > INT32_MAX) ? INT32_MAX :
            ((target < INT32_MIN) ? INT32_MIN : (int32_t)target);
    }
}

static uint8_t align_get_sample(C552_K230Data *sensor, uint8_t *ready)
{
    C552_Data data;
    C552_Health health;
    uint8_t mask;

    *ready = 0U;
    if (C552_GetSnapshot(&data, &health) == 0U) return 0U;
    if (g_xy_vision_calibration.k230_id == C552_ID_K230_1) {
        mask = C552_DEVICE_K230_1;
        *sensor = data.k230_1;
    } else if (g_xy_vision_calibration.k230_id == C552_ID_K230_2) {
        mask = C552_DEVICE_K230_2;
        *sensor = data.k230_2;
    } else {
        return 0U;
    }
    *ready = ((health.ready_mask & mask) != 0U) ? 1U : 0U;
    return 1U;
}

static void align_fault(XY_VisionAlignFault fault)
{
    XY_AxisStatus x;
    XY_AxisStatus y;

    g_align.fault = fault;
    g_align.state = XY_VISION_ALIGN_FAULT;
    if ((XY_GetStatus(XY_AXIS_X, &x) != 0U) &&
        ((x.state == XY_STATE_STARTING) || (x.state == XY_STATE_MOVING))) {
        XY_Stop(XY_AXIS_X);
    }
    if ((XY_GetStatus(XY_AXIS_Y, &y) != 0U) &&
        ((y.state == XY_STATE_STARTING) || (y.state == XY_STATE_MOVING))) {
        XY_Stop(XY_AXIS_Y);
    }
}

static uint8_t align_axis_faulted(void)
{
    XY_AxisStatus x;
    XY_AxisStatus y;

    if ((XY_GetStatus(XY_AXIS_X, &x) == 0U) ||
        (XY_GetStatus(XY_AXIS_Y, &y) == 0U)) return 1U;
    return ((x.state == XY_STATE_FAULT) || (y.state == XY_STATE_FAULT)) ? 1U : 0U;
}

void XY_VisionAlign_Init(uint32_t now)
{
    memset(&g_align, 0, sizeof(g_align));
    g_align.state = XY_VISION_ALIGN_IDLE;
    g_align.failed_axis = XY_AXIS_COUNT;
    g_align.last_move_result = XY_RESULT_OK;
    g_next_decision_tick = now;
    g_last_decision_seq = 0U;
}

uint8_t XY_VisionAlign_Start(uint32_t now)
{
    return XY_VisionAlign_StartOwned(MOTION_OWNER_XY_ALIGN,
        g_xy_vision_calibration.reference_pixel[0],
        g_xy_vision_calibration.reference_pixel[1], now);
}

uint8_t XY_VisionAlign_StartOwned(MotionOwner owner, int16_t target_x,
                                  int16_t target_y, uint32_t now)
{
    uint8_t required_mask =
        (g_xy_vision_calibration.k230_id == C552_ID_K230_2) ?
        C552_DEVICE_K230_2 : C552_DEVICE_K230_1;
    if ((owner != MOTION_OWNER_XY_ALIGN) &&
        (owner != MOTION_OWNER_MISSION)) return 0U;
    if (MotionCoordinator_Acquire(owner,
                                  required_mask, now) == 0U) return 0U;
    if ((g_align.state == XY_VISION_ALIGN_WAIT_SAMPLE) ||
        (g_align.state == XY_VISION_ALIGN_MOVE_X) ||
        (g_align.state == XY_VISION_ALIGN_MOVE_Y)) goto reject;
    if ((VisionCalibration_IsActive() != 0U) ||
        (XZCalibration_IsActive() != 0U) ||
        (XZVisionAlign_IsActive() != 0U)) goto reject;
    if (g_xy_vision_calibration.calibrated == 0U) {
        g_align.fault = XY_VISION_ALIGN_FAULT_NOT_CALIBRATED;
        g_align.state = XY_VISION_ALIGN_FAULT;
        goto reject;
    }
    if (align_axes_ready() == 0U) {
        g_align.fault = XY_VISION_ALIGN_FAULT_AXIS_NOT_READY;
        g_align.state = XY_VISION_ALIGN_FAULT;
        goto reject;
    }

    memset(&g_align, 0, sizeof(g_align));
    g_align.state = XY_VISION_ALIGN_WAIT_SAMPLE;
    g_align.start_tick = now;
    g_align.last_sample_tick = now;
    g_align.failed_axis = XY_AXIS_COUNT;
    g_align.last_move_result = XY_RESULT_OK;
    g_align.owner = owner;
    g_align.target_pixel[0] = target_x;
    g_align.target_pixel[1] = target_y;
    g_next_decision_tick = now;
    g_last_decision_seq = 0U;
    return 1U;

reject:
    if (owner != MOTION_OWNER_MISSION) MotionCoordinator_Release(owner, now);
    return 0U;
}

void XY_VisionAlign_Abort(void)
{
    MotionOwner owner = g_align.owner;
    if (XY_VisionAlign_IsActive() != 0U) {
        XY_AxisStatus x;
        XY_AxisStatus y;
        if ((XY_GetStatus(XY_AXIS_X, &x) != 0U) &&
            ((x.state == XY_STATE_STARTING) || (x.state == XY_STATE_MOVING))) {
            XY_Stop(XY_AXIS_X);
        }
        if ((XY_GetStatus(XY_AXIS_Y, &y) != 0U) &&
            ((y.state == XY_STATE_STARTING) || (y.state == XY_STATE_MOVING))) {
            XY_Stop(XY_AXIS_Y);
        }
    }
    g_align.state = XY_VISION_ALIGN_IDLE;
    g_align.fault = XY_VISION_ALIGN_FAULT_NONE;
    if ((owner != MOTION_OWNER_NONE) &&
        (owner != MOTION_OWNER_MISSION)) {
        MotionCoordinator_Release(owner, HAL_GetTick());
    }
}

void XY_VisionAlign_Poll(uint32_t now)
{
    C552_K230Data sensor;
    XY_Result result;
    uint8_t ready;
    int16_t error_x;
    int16_t error_y;

    if (XY_VisionAlign_IsActive() == 0U) return;
    if (((uint32_t)(now - g_align.start_tick) >
         XY_ALIGN_CONTROL_TIMEOUT_MS) ||
        (g_align.corrections >= XY_ALIGN_MAX_CORRECTIONS)) {
        align_fault(XY_VISION_ALIGN_FAULT_CONTROL_TIMEOUT);
        return;
    }
    if (align_axis_faulted() != 0U) {
        align_fault(XY_VISION_ALIGN_FAULT_AXIS);
        return;
    }
    if (align_get_sample(&sensor, &ready) == 0U) ready = 0U;
    if (ready == 0U) {
        if ((uint32_t)(now - g_align.last_sample_tick) >
            XY_ALIGN_VISION_TIMEOUT_MS) {
            align_fault(XY_VISION_ALIGN_FAULT_VISION_TIMEOUT);
        }
        return;
    }
    if (sensor.sample_seq != g_align.last_sample_seq) {
        g_align.last_sample_seq = sensor.sample_seq;
        g_align.last_sample_tick = now;
        g_align.pixel[0] = sensor.center_x;
        g_align.pixel[1] = sensor.center_y;
    }

    if (g_align.state == XY_VISION_ALIGN_MOVE_X) {
        if (align_axes_ready() == 0U) return;
        if (g_align.requested_pulses[1] != 0) {
            result = XY_MoveRelative(XY_AXIS_Y, g_align.requested_pulses[1],
                                     XY_ALIGN_Y_SPEED_RPM,
                                     XY_ALIGN_ACCELERATION);
            align_record_attempt(XY_AXIS_Y, g_align.requested_pulses[1], result);
            if (result == XY_RESULT_OK) {
                g_align.failed_axis = XY_AXIS_COUNT;
                g_align.state = XY_VISION_ALIGN_MOVE_Y;
                return;
            }
            if ((result == XY_RESULT_BUSY) ||
                (result == XY_RESULT_CAN_REJECTED)) return;
            align_fault(XY_VISION_ALIGN_FAULT_MOVE);
            return;
        }
        g_align.state = XY_VISION_ALIGN_WAIT_SAMPLE;
        g_next_decision_tick = now + XY_VISION_ALIGN_PERIOD_MS;
        return;
    }
    if (g_align.state == XY_VISION_ALIGN_MOVE_Y) {
        if (align_axes_ready() == 0U) return;
        g_align.state = XY_VISION_ALIGN_WAIT_SAMPLE;
        g_next_decision_tick = now + XY_VISION_ALIGN_PERIOD_MS;
        return;
    }
    if ((int32_t)(now - g_next_decision_tick) < 0) return;
    if ((sensor.sample_seq == 0U) ||
        (sensor.sample_seq == g_last_decision_seq)) return;
    g_last_decision_seq = sensor.sample_seq;

    error_x = (int16_t)(g_align.target_pixel[0] - sensor.center_x);
    error_y = (int16_t)(g_align.target_pixel[1] - sensor.center_y);
    g_align.error_pixel[0] = error_x;
    g_align.error_pixel[1] = error_y;
    if ((align_in_deadzone(error_x) != 0U) &&
        (align_in_deadzone(error_y) != 0U)) {
        ++g_align.stable_samples;
        g_next_decision_tick = now + XY_VISION_ALIGN_PERIOD_MS;
        if (g_align.stable_samples >= XY_ALIGN_STABLE_SAMPLES) {
            g_align.state = XY_VISION_ALIGN_COMPLETE;
        }
        return;
    }
    g_align.stable_samples = 0U;
    g_align.requested_pulses[0] = align_clamp(align_round_float(
        XY_ALIGN_PROPORTIONAL_GAIN *
        (g_xy_vision_calibration.pulse_per_pixel[0][0] * error_x +
         g_xy_vision_calibration.pulse_per_pixel[0][1] * error_y)),
        XY_ALIGN_X_MAX_STEP_PULSES);
    g_align.requested_pulses[1] = align_clamp(align_round_float(
        XY_ALIGN_PROPORTIONAL_GAIN *
        (g_xy_vision_calibration.pulse_per_pixel[1][0] * error_x +
         g_xy_vision_calibration.pulse_per_pixel[1][1] * error_y)),
        XY_ALIGN_Y_MAX_STEP_PULSES);

    if (g_align.requested_pulses[0] != 0) {
        result = XY_MoveRelative(XY_AXIS_X, g_align.requested_pulses[0],
                                 XY_ALIGN_X_SPEED_RPM,
                                 XY_ALIGN_ACCELERATION);
        align_record_attempt(XY_AXIS_X, g_align.requested_pulses[0], result);
        if (result == XY_RESULT_OK) {
            g_align.failed_axis = XY_AXIS_COUNT;
            ++g_align.corrections;
            g_align.state = XY_VISION_ALIGN_MOVE_X;
            return;
        }
        if ((result == XY_RESULT_BUSY) ||
            (result == XY_RESULT_CAN_REJECTED)) {
            g_next_decision_tick = now + XY_VISION_ALIGN_PERIOD_MS;
            g_last_decision_seq = 0U;
            return;
        }
        align_fault(XY_VISION_ALIGN_FAULT_MOVE);
        return;
    }
    if (g_align.requested_pulses[1] != 0) {
        result = XY_MoveRelative(XY_AXIS_Y, g_align.requested_pulses[1],
                                 XY_ALIGN_Y_SPEED_RPM,
                                 XY_ALIGN_ACCELERATION);
        align_record_attempt(XY_AXIS_Y, g_align.requested_pulses[1], result);
        if (result == XY_RESULT_OK) {
            g_align.failed_axis = XY_AXIS_COUNT;
            ++g_align.corrections;
            g_align.state = XY_VISION_ALIGN_MOVE_Y;
            return;
        }
        if ((result == XY_RESULT_BUSY) ||
            (result == XY_RESULT_CAN_REJECTED)) {
            g_next_decision_tick = now + XY_VISION_ALIGN_PERIOD_MS;
            g_last_decision_seq = 0U;
            return;
        }
        align_fault(XY_VISION_ALIGN_FAULT_MOVE);
        return;
    }
    g_next_decision_tick = now + XY_VISION_ALIGN_PERIOD_MS;
}

void XY_VisionAlign_GetStatus(XY_VisionAlignStatus *status)
{
    if (status != NULL) *status = g_align;
}

uint8_t XY_VisionAlign_IsActive(void)
{
    return ((g_align.state == XY_VISION_ALIGN_WAIT_SAMPLE) ||
            (g_align.state == XY_VISION_ALIGN_MOVE_X) ||
            (g_align.state == XY_VISION_ALIGN_MOVE_Y)) ? 1U : 0U;
}

const char *XY_VisionAlign_StateString(XY_VisionAlignState state)
{
    static const char *const names[] = {
        "IDLE", "WAIT_SAMPLE", "MOVE_X", "MOVE_Y", "COMPLETE", "FAULT"
    };
    return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
           names[state] : "UNKNOWN";
}

const char *XY_VisionAlign_FaultString(XY_VisionAlignFault fault)
{
    static const char *const names[] = {
        "NONE", "NOT_CALIBRATED", "AXIS_NOT_READY", "VISION_TIMEOUT",
        "CONTROL_TIMEOUT", "MOVE", "AXIS"
    };
    return ((uint32_t)fault < (sizeof(names) / sizeof(names[0]))) ?
           names[fault] : "UNKNOWN";
}

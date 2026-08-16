#include "xz_vision_align.h"

#include "c552.h"
#include "motion_coordinator.h"
#include "vision_calibration.h"
#include "xy_vision_align.h"
#include "xz_vision_calibration.h"
#include <limits.h>
#include <stddef.h>
#include <string.h>

#define XZ_ALIGN_PIXEL_DEADZONE       3
#define XZ_ALIGN_STABLE_SAMPLES       3U
#define XZ_ALIGN_MODE_TIMEOUT_MS      1000U
#define XZ_ALIGN_VISION_TIMEOUT_MS    500U
#define XZ_ALIGN_CONTROL_TIMEOUT_MS   60000U
#define XZ_ALIGN_MAX_CORRECTIONS      50U
#define XZ_ALIGN_PROPORTIONAL_GAIN    0.5f
#define XZ_ALIGN_X_MAX_STEP_PULSES    24000L
#define XZ_ALIGN_Z_MAX_STEP_PULSES    6400L
#define XZ_ALIGN_X_SPEED_RPM          300U
#define XZ_ALIGN_Z_SPEED_HZ           10000U
#define XZ_ALIGN_ACCELERATION         200U

static XZVisionAlignStatus g_align;
static uint16_t g_wait_sample_seq;
static uint8_t g_wait_sample_initialized;
static uint32_t g_next_decision_tick;
static uint32_t g_mode_deadline;
static C552_K230Mode g_k230_mode;

static int32_t align_round_float(float value)
{
    if (value >= (float)INT32_MAX) return INT32_MAX;
    if (value <= (float)INT32_MIN) return INT32_MIN;
    return (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static uint8_t align_in_deadzone(int32_t value)
{
    return ((value >= -XZ_ALIGN_PIXEL_DEADZONE) &&
            (value <= XZ_ALIGN_PIXEL_DEADZONE)) ? 1U : 0U;
}

static uint8_t align_get_axes(XY_AxisStatus *x, ZAxisControlStatus *z)
{
    if ((x == NULL) || (z == NULL) ||
        (XY_GetStatus(XY_AXIS_X, x) == 0U)) return 0U;
    ZAxis_GetControlStatus(z);
    g_align.axis_position[0] = x->position_pulses;
    g_align.axis_position[1] = z->position_pulses;
    return 1U;
}

static uint8_t align_axes_ready(void)
{
    XY_AxisStatus x;
    ZAxisControlStatus z;
    if (align_get_axes(&x, &z) == 0U) return 0U;
    return ((x.position_valid != 0U) && (x.state == XY_STATE_IDLE) &&
            (z.position_valid != 0U) && (z.state == Z_STATE_IDLE)) ? 1U : 0U;
}

static uint8_t align_axes_faulted(void)
{
    XY_AxisStatus x;
    ZAxisControlStatus z;
    if (align_get_axes(&x, &z) == 0U) return 1U;
    return ((x.state == XY_STATE_FAULT) ||
            (z.state == Z_STATE_FAULT)) ? 1U : 0U;
}

static uint8_t align_get_red_sample(C552_K230Data *sensor)
{
    C552_Data data;
    C552_Health health;
    if ((sensor == NULL) ||
        (C552_GetSnapshot(&data, &health) == 0U) ||
        ((health.ready_mask & C552_DEVICE_K230_2) == 0U)) return 0U;
    *sensor = data.k230_2;
    return 1U;
}

static void align_stop_axes(void)
{
    XY_AxisStatus x;
    ZAxisControlStatus z;
    if ((XY_GetStatus(XY_AXIS_X, &x) != 0U) &&
        ((x.state == XY_STATE_STARTING) || (x.state == XY_STATE_MOVING))) {
        XY_Stop(XY_AXIS_X);
    }
    ZAxis_GetControlStatus(&z);
    if ((z.state == Z_STATE_STARTING) || (z.state == Z_STATE_MOVING)) {
        (void)ZAxisControl_Stop();
    }
}

static void align_fault(XZVisionAlignFault fault)
{
    align_stop_axes();
    g_align.fault = fault;
    g_align.state = XZ_VISION_ALIGN_FAULT;
}

static float align_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void align_reduce_scale(float raw, float available, float *scale)
{
    float candidate;
    if (align_absf(raw) < 0.5f) return;
    if (available <= 0.0f) {
        *scale = 0.0f;
        return;
    }
    candidate = available / align_absf(raw);
    if (candidate < *scale) *scale = candidate;
}

static uint8_t align_scale_vector(float raw_x, float raw_z)
{
    const XY_AxisConfig *x_config = XY_GetConfig(XY_AXIS_X);
    float x_travel;
    float z_travel;
    float scale = 1.0f;

    if (x_config == NULL) return 0U;
    x_travel = (raw_x >= 0.0f) ?
        (float)(x_config->soft_max_pulses - g_align.axis_position[0]) :
        (float)(g_align.axis_position[0] - x_config->soft_min_pulses);
    z_travel = (raw_z >= 0.0f) ?
        (float)(Z_AXIS_SOFT_MAX_PULSES - g_align.axis_position[1]) :
        (float)(g_align.axis_position[1] - Z_AXIS_SOFT_MIN_PULSES);

    /* One scale preserves the calibrated pixel-space correction direction. */
    align_reduce_scale(raw_x, (float)XZ_ALIGN_X_MAX_STEP_PULSES, &scale);
    align_reduce_scale(raw_z, (float)XZ_ALIGN_Z_MAX_STEP_PULSES, &scale);
    align_reduce_scale(raw_x, x_travel, &scale);
    align_reduce_scale(raw_z, z_travel, &scale);
    if (scale <= 0.0f) return 0U;
    if (scale > 1.0f) scale = 1.0f;

    g_align.raw_pulses[0] = align_round_float(raw_x);
    g_align.raw_pulses[1] = align_round_float(raw_z);
    g_align.requested_pulses[0] = align_round_float(raw_x * scale);
    g_align.requested_pulses[1] = align_round_float(raw_z * scale);
    g_align.vector_scale_permille =
        (uint16_t)((scale * 1000.0f) + 0.5f);
    g_align.attempted_target[0] = g_align.axis_position[0] +
                                  g_align.requested_pulses[0];
    g_align.attempted_target[1] = g_align.axis_position[1] +
                                  g_align.requested_pulses[1];
    return ((g_align.requested_pulses[0] != 0) ||
            (g_align.requested_pulses[1] != 0)) ? 1U : 0U;
}

void XZVisionAlign_Init(uint32_t now)
{
    memset(&g_align, 0, sizeof(g_align));
    g_align.state = XZ_VISION_ALIGN_IDLE;
    g_align.last_x_result = XY_RESULT_OK;
    g_align.last_z_result = Z_RESULT_OK;
    g_next_decision_tick = now;
    g_mode_deadline = now;
    g_k230_mode = C552_K230_MODE_RED_BLOCK;
    g_wait_sample_seq = 0U;
    g_wait_sample_initialized = 0U;
}

uint8_t XZVisionAlign_Start(uint32_t now)
{
    return XZVisionAlign_StartOwned(MOTION_OWNER_P4_XZ_ALIGN,
        g_xz_vision_calibration.reference_pixel[0],
        g_xz_vision_calibration.reference_pixel[1],
        C552_K230_MODE_RED_BLOCK, now);
}

uint8_t XZVisionAlign_StartOwned(MotionOwner owner, int16_t target_x,
                                 int16_t target_y, uint8_t k230_mode,
                                 uint32_t now)
{
    C552_CommandStatus command;
    C552_K230Data sensor;
    C552_RequestResult request;
    uint8_t mode_applied;
    uint8_t sample_ready;
    if ((owner != MOTION_OWNER_P4_XZ_ALIGN) &&
        (owner != MOTION_OWNER_MISSION)) return 0U;
    if ((k230_mode != C552_K230_MODE_APRILTAG) &&
        (k230_mode != C552_K230_MODE_RED_BLOCK)) return 0U;
    if (MotionCoordinator_Acquire(owner,
                                  C552_DEVICE_K230_2, now) == 0U) return 0U;
    if ((XZVisionAlign_IsActive() != 0U) ||
        (VisionCalibration_IsActive() != 0U) ||
        (XY_VisionAlign_IsActive() != 0U) ||
        (XZCalibration_IsActive() != 0U)) {
        g_align.fault = XZ_VISION_ALIGN_FAULT_BUSY;
        g_align.state = XZ_VISION_ALIGN_FAULT;
        goto reject;
    }
    if (g_xz_vision_calibration.valid == 0U) {
        g_align.fault = XZ_VISION_ALIGN_FAULT_NOT_CALIBRATED;
        g_align.state = XZ_VISION_ALIGN_FAULT;
        goto reject;
    }
    if (align_axes_ready() == 0U) {
        g_align.fault = XZ_VISION_ALIGN_FAULT_AXIS_NOT_READY;
        g_align.state = XZ_VISION_ALIGN_FAULT;
        goto reject;
    }

    C552_GetCommandStatus(&command);
    mode_applied = ((command.id == C552_ID_K230_2) &&
                    (command.command == C552_COMMAND_SET_K230_MODE) &&
                    (command.requested_value == k230_mode) &&
                    (command.state == C552_COMMAND_APPLIED)) ? 1U : 0U;
    if (mode_applied == 0U) {
        request = C552_SetK230Mode(C552_ID_K230_2,
                                   (C552_K230Mode)k230_mode, now);
        if (request != C552_REQUEST_OK) {
            g_align.fault = (request == C552_REQUEST_BUSY) ?
                            XZ_VISION_ALIGN_FAULT_BUSY :
                            XZ_VISION_ALIGN_FAULT_MODE;
            g_align.state = XZ_VISION_ALIGN_FAULT;
            goto reject;
        }
    }

    memset(&g_align, 0, sizeof(g_align));
    g_align.state = (mode_applied != 0U) ?
                    XZ_VISION_ALIGN_WAIT_RED_SAMPLE :
                    XZ_VISION_ALIGN_WAIT_RED_MODE;
    g_align.start_tick = now;
    g_align.last_sample_tick = now;
    g_align.last_x_result = XY_RESULT_OK;
    g_align.last_z_result = Z_RESULT_OK;
    g_align.owner = owner;
    g_align.target_pixel[0] = target_x;
    g_align.target_pixel[1] = target_y;
    g_k230_mode = (C552_K230Mode)k230_mode;
    g_next_decision_tick = now;
    g_mode_deadline = now + XZ_ALIGN_MODE_TIMEOUT_MS;
    g_wait_sample_seq = 0U;
    g_wait_sample_initialized = 0U;
    if (mode_applied != 0U) {
        sample_ready = align_get_red_sample(&sensor);
        g_wait_sample_seq = (sample_ready != 0U) ? sensor.sample_seq : 0U;
        g_wait_sample_initialized = sample_ready;
    }
    return 1U;

reject:
    if (owner != MOTION_OWNER_MISSION) MotionCoordinator_Release(owner, now);
    return 0U;
}

void XZVisionAlign_Abort(void)
{
    MotionOwner owner = g_align.owner;
    if (XZVisionAlign_IsActive() != 0U) align_stop_axes();
    g_align.state = XZ_VISION_ALIGN_IDLE;
    g_align.fault = XZ_VISION_ALIGN_FAULT_NONE;
    if ((owner != MOTION_OWNER_NONE) &&
        (owner != MOTION_OWNER_MISSION)) {
        MotionCoordinator_Release(owner, HAL_GetTick());
    }
}

void XZVisionAlign_Poll(uint32_t now)
{
    C552_CommandStatus command;
    C552_K230Data sensor;
    XY_AxisStatus x;
    ZAxisControlStatus z;
    uint8_t sample_ready;

    if (XZVisionAlign_IsActive() == 0U) return;
    if ((uint32_t)(now - g_align.start_tick) >
        XZ_ALIGN_CONTROL_TIMEOUT_MS) {
        align_fault(XZ_VISION_ALIGN_FAULT_CONTROL_TIMEOUT);
        return;
    }
    if (align_axes_faulted() != 0U) {
        align_fault(XZ_VISION_ALIGN_FAULT_AXIS);
        return;
    }

    sample_ready = align_get_red_sample(&sensor);
    if ((sample_ready != 0U) &&
        (sensor.sample_seq != g_align.last_sample_seq)) {
        g_align.last_sample_seq = sensor.sample_seq;
        g_align.last_sample_tick = now;
        g_align.pixel[0] = sensor.center_x;
        g_align.pixel[1] = sensor.center_y;
    }

    if (g_align.state == XZ_VISION_ALIGN_WAIT_RED_MODE) {
        C552_GetCommandStatus(&command);
        if ((command.id == C552_ID_K230_2) &&
            (command.command == C552_COMMAND_SET_K230_MODE) &&
            (command.requested_value == (uint8_t)g_k230_mode) &&
            (command.state == C552_COMMAND_APPLIED)) {
            g_wait_sample_seq = (sample_ready != 0U) ? sensor.sample_seq : 0U;
            g_wait_sample_initialized = sample_ready;
            g_align.last_sample_tick = now;
            g_align.state = XZ_VISION_ALIGN_WAIT_RED_SAMPLE;
        } else if ((command.state == C552_COMMAND_FAILED) ||
                   (command.state == C552_COMMAND_TIMEOUT) ||
                   ((int32_t)(now - g_mode_deadline) >= 0)) {
            align_fault(XZ_VISION_ALIGN_FAULT_MODE);
        }
        return;
    }

    if ((sample_ready == 0U) ||
        ((uint32_t)(now - g_align.last_sample_tick) >
         XZ_ALIGN_VISION_TIMEOUT_MS)) {
        if ((uint32_t)(now - g_align.last_sample_tick) >
            XZ_ALIGN_VISION_TIMEOUT_MS) {
            align_fault(XZ_VISION_ALIGN_FAULT_VISION_TIMEOUT);
        }
        return;
    }

    switch (g_align.state) {
    case XZ_VISION_ALIGN_WAIT_RED_SAMPLE:
    case XZ_VISION_ALIGN_WAIT_NEW_SAMPLE:
        if ((int32_t)(now - g_next_decision_tick) < 0) return;
        if ((g_wait_sample_initialized != 0U) &&
            (sensor.sample_seq == g_wait_sample_seq)) return;
        g_align.decision_sample_seq = sensor.sample_seq;
        g_align.pixel[0] = sensor.center_x;
        g_align.pixel[1] = sensor.center_y;
        g_align.decision_pixel[0] = sensor.center_x;
        g_align.decision_pixel[1] = sensor.center_y;
        g_align.state = XZ_VISION_ALIGN_CALCULATE_ERROR;
        break;

    case XZ_VISION_ALIGN_CALCULATE_ERROR: {
        int32_t error_x = (int32_t)g_align.target_pixel[0] -
                          g_align.decision_pixel[0];
        int32_t error_y = (int32_t)g_align.target_pixel[1] -
                          g_align.decision_pixel[1];
        float raw_x;
        float raw_z;

        g_align.error_pixel[0] = error_x;
        g_align.error_pixel[1] = error_y;
        if ((align_in_deadzone(error_x) != 0U) &&
            (align_in_deadzone(error_y) != 0U)) {
            ++g_align.stable_samples;
            if (g_align.stable_samples >= XZ_ALIGN_STABLE_SAMPLES) {
                g_align.state = XZ_VISION_ALIGN_COMPLETE;
            } else {
                g_wait_sample_seq = g_align.decision_sample_seq;
                g_wait_sample_initialized = 1U;
                g_next_decision_tick = now + XZ_VISION_ALIGN_PERIOD_MS;
                g_align.state = XZ_VISION_ALIGN_WAIT_NEW_SAMPLE;
            }
            break;
        }
        g_align.stable_samples = 0U;
        if (g_align.corrections >= XZ_ALIGN_MAX_CORRECTIONS) {
            align_fault(XZ_VISION_ALIGN_FAULT_CONTROL_TIMEOUT);
            break;
        }
        raw_x = XZ_ALIGN_PROPORTIONAL_GAIN *
            (g_xz_vision_calibration.pulse_per_pixel[0][0] * error_x +
             g_xz_vision_calibration.pulse_per_pixel[0][1] * error_y);
        raw_z = XZ_ALIGN_PROPORTIONAL_GAIN *
            (g_xz_vision_calibration.pulse_per_pixel[1][0] * error_x +
             g_xz_vision_calibration.pulse_per_pixel[1][1] * error_y);
        if (align_get_axes(&x, &z) == 0U) {
            align_fault(XZ_VISION_ALIGN_FAULT_AXIS);
            break;
        }
        if (align_scale_vector(raw_x, raw_z) == 0U) {
            align_fault(XZ_VISION_ALIGN_FAULT_SOFT_LIMIT);
            break;
        }
        ++g_align.corrections;
        g_align.state = XZ_VISION_ALIGN_MOVE_X;
        break;
    }

    case XZ_VISION_ALIGN_MOVE_X:
        if (g_align.requested_pulses[0] == 0) {
            g_align.state = XZ_VISION_ALIGN_MOVE_Z;
            break;
        }
        g_align.last_x_result = XY_MoveRelative(
            XY_AXIS_X, g_align.requested_pulses[0], XZ_ALIGN_X_SPEED_RPM,
            XZ_ALIGN_ACCELERATION);
        if (g_align.last_x_result == XY_RESULT_OK) {
            g_align.state = XZ_VISION_ALIGN_WAIT_X_IDLE;
        } else if ((g_align.last_x_result != XY_RESULT_BUSY) &&
                   (g_align.last_x_result != XY_RESULT_CAN_REJECTED)) {
            align_fault((g_align.last_x_result == XY_RESULT_SOFT_LIMIT) ?
                        XZ_VISION_ALIGN_FAULT_SOFT_LIMIT :
                        XZ_VISION_ALIGN_FAULT_MOVE_X);
        }
        break;

    case XZ_VISION_ALIGN_WAIT_X_IDLE:
        if (align_get_axes(&x, &z) == 0U) {
            align_fault(XZ_VISION_ALIGN_FAULT_AXIS);
        } else if (x.state == XY_STATE_IDLE) {
            g_align.state = XZ_VISION_ALIGN_MOVE_Z;
        }
        break;

    case XZ_VISION_ALIGN_MOVE_Z:
        if (g_align.requested_pulses[1] == 0) {
            g_wait_sample_seq = sensor.sample_seq;
            g_wait_sample_initialized = 1U;
            g_next_decision_tick = now + XZ_VISION_ALIGN_PERIOD_MS;
            g_align.state = XZ_VISION_ALIGN_WAIT_NEW_SAMPLE;
            break;
        }
        g_align.last_z_result = ZAxisControl_MoveRelative(
            g_align.requested_pulses[1], XZ_ALIGN_Z_SPEED_HZ);
        if (g_align.last_z_result == Z_RESULT_OK) {
            g_align.state = XZ_VISION_ALIGN_WAIT_Z_IDLE;
        } else if (g_align.last_z_result != Z_RESULT_BUSY) {
            align_fault((g_align.last_z_result == Z_RESULT_SOFT_LIMIT) ?
                        XZ_VISION_ALIGN_FAULT_SOFT_LIMIT :
                        XZ_VISION_ALIGN_FAULT_MOVE_Z);
        }
        break;

    case XZ_VISION_ALIGN_WAIT_Z_IDLE:
        if (align_get_axes(&x, &z) == 0U) {
            align_fault(XZ_VISION_ALIGN_FAULT_AXIS);
        } else if (z.state == Z_STATE_IDLE) {
            g_wait_sample_seq = sensor.sample_seq;
            g_wait_sample_initialized = 1U;
            g_next_decision_tick = now + XZ_VISION_ALIGN_PERIOD_MS;
            g_align.state = XZ_VISION_ALIGN_WAIT_NEW_SAMPLE;
        }
        break;

    default:
        break;
    }
}

void XZVisionAlign_GetStatus(XZVisionAlignStatus *status)
{
    if (status != NULL) *status = g_align;
}

uint8_t XZVisionAlign_IsActive(void)
{
    return ((g_align.state != XZ_VISION_ALIGN_IDLE) &&
            (g_align.state != XZ_VISION_ALIGN_COMPLETE) &&
            (g_align.state != XZ_VISION_ALIGN_FAULT)) ? 1U : 0U;
}

const char *XZVisionAlign_StateString(XZVisionAlignState state)
{
    static const char *const names[] = {
        "IDLE", "WAIT_RED_MODE", "WAIT_RED_SAMPLE", "CALCULATE_ERROR",
        "MOVE_X", "WAIT_X_IDLE", "MOVE_Z", "WAIT_Z_IDLE",
        "WAIT_NEW_SAMPLE", "COMPLETE", "FAULT"
    };
    return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
           names[state] : "UNKNOWN";
}

const char *XZVisionAlign_FaultString(XZVisionAlignFault fault)
{
    static const char *const names[] = {
        "NONE", "BUSY", "NOT_CALIBRATED", "AXIS_NOT_READY", "MODE",
        "VISION_TIMEOUT", "CONTROL_TIMEOUT", "MOVE_X", "MOVE_Z",
        "SOFT_LIMIT", "AXIS"
    };
    return ((uint32_t)fault < (sizeof(names) / sizeof(names[0]))) ?
           names[fault] : "UNKNOWN";
}

#include "mission_subflow.h"

#include "c552.h"
#include "motion_interfaces.h"
#include "xy_motor.h"
#include "xy_vision_align.h"
#include "xz_vision_align.h"
#include "z_axis.h"
#include <limits.h>
#include <stddef.h>
#include <string.h>

#define SUBFLOW_MODE_TIMEOUT_MS       1000U
#define SUBFLOW_SAMPLE_TIMEOUT_MS     500U
#define SUBFLOW_OBSERVE_ATTEMPT_MS    2000U
#define SUBFLOW_RETRY_DELAY_MS        50U
#define SUBFLOW_BUSY_RETRY_DELAY_MS   50U
#define SUBFLOW_AXIS_RECOVERY_DELAY_MS 50U
#define SUBFLOW_MAX_ATTEMPTS          10U
#define SUBFLOW_Z_MAX_RECOVERIES      10U
#define SUBFLOW_POSE_RETRY_MS         50U
#define SUBFLOW_MOVE_TIMEOUT_MS       60000U
#define SUBFLOW_STABLE_SAMPLES        3U
#define SUBFLOW_PIXEL_STABILITY       4
#define SUBFLOW_TOF3_STABLE_SAMPLES   10U
#define SUBFLOW_TOF3_SEGMENT_PULSES   8000L
#define SUBFLOW_Y_SPEED_RPM           5U
#define SUBFLOW_X_SPEED_RPM           300U
#define SUBFLOW_Z_SPEED_HZ            10000U
#define SUBFLOW_ACCELERATION          200U

typedef struct {
    MissionSubflowStatus status;
    uint8_t k230_id;
    C552_K230Mode k230_mode;
    uint16_t applied_sample_seq;
    int16_t stable_min[2];
    int16_t stable_max[2];
    uint16_t stop_mm;
    uint32_t pulses_per_mm;
    int8_t direction;
    uint8_t tof_id;
    int32_t max_descent_pulses;
    int32_t descent_commanded;
    uint8_t return_axis;
    uint8_t command_issued;
    uint32_t attempt_tick;
    uint32_t last_sensor_tick;
    uint32_t z_fault_count_seen;
} MissionSubflowRuntime;

static MissionSubflowRuntime g_subflow;
static MotionPositionSnapshot g_recorded_pose;
static MotionPositionSnapshot g_safe_pose;
static uint8_t g_recorded_pose_valid;
static uint8_t g_safe_pose_valid;
static uint8_t g_retain_owner;
static volatile uint8_t g_cancel_request;

static uint8_t tick_due(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

static uint8_t subflow_active_state(MissionSubflowState state)
{
    return ((state != MISSION_SUBFLOW_IDLE) &&
            (state != MISSION_SUBFLOW_COMPLETE) &&
            (state != MISSION_SUBFLOW_FAULT)) ? 1U : 0U;
}

static uint8_t k230_sample_blank(const C552_K230Data *sample)
{
    return ((sample->center_x == 0) && (sample->center_y == 0) &&
            (sample->x_rotation_cdeg == 0U) &&
            (sample->y_rotation_cdeg == 0U) &&
            (sample->sample_seq == 0U) &&
            (sample->sample_age_ms == 0U)) ? 1U : 0U;
}

static uint8_t axes_faulted(MissionFailure *failure)
{
    XY_AxisStatus x;
    XY_AxisStatus y;
    ZAxisControlStatus z;
    (void)XY_GetStatus(XY_AXIS_X, &x);
    (void)XY_GetStatus(XY_AXIS_Y, &y);
    ZAxis_GetControlStatus(&z);
    if (x.state == XY_STATE_FAULT) {
        failure->source = MISSION_FAIL_SOURCE_AXIS_X;
        failure->reason = MISSION_FAIL_AXIS_FAULT;
        failure->detail = (int32_t)x.fault;
        return 1U;
    }
    if (y.state == XY_STATE_FAULT) {
        failure->source = MISSION_FAIL_SOURCE_AXIS_Y;
        failure->reason = MISSION_FAIL_AXIS_FAULT;
        failure->detail = (int32_t)y.fault;
        return 1U;
    }
    if (z.state == Z_STATE_FAULT) {
        g_subflow.z_fault_count_seen = z.fault_count;
        failure->source = MISSION_FAIL_SOURCE_AXIS_Z;
        failure->reason = MISSION_FAIL_AXIS_FAULT;
        failure->detail = (int32_t)z.fault;
        return 1U;
    }
    if ((g_subflow.status.type != MISSION_SUBFLOW_NONE) &&
        (z.fault_count != g_subflow.z_fault_count_seen)) {
        g_subflow.z_fault_count_seen = z.fault_count;
        failure->source = MISSION_FAIL_SOURCE_AXIS_Z;
        failure->reason = MISSION_FAIL_AXIS_FAULT;
        failure->detail = (int32_t)z.last_fault;
        return 1U;
    }
    return 0U;
}

static uint8_t align_axes_idle(void)
{
    XY_AxisStatus x;
    XY_AxisStatus y;
    ZAxisControlStatus z;
    (void)XY_GetStatus(XY_AXIS_X, &x);
    (void)XY_GetStatus(XY_AXIS_Y, &y);
    ZAxis_GetControlStatus(&z);
    if (g_subflow.status.type == MISSION_SUBFLOW_ALIGN_XZ)
        return ((x.state == XY_STATE_IDLE) &&
                (z.state == Z_STATE_IDLE)) ? 1U : 0U;
    return ((x.state == XY_STATE_IDLE) &&
            (y.state == XY_STATE_IDLE)) ? 1U : 0U;
}

static void recover_z_fault(const MissionFailure *failure)
{
    ZAxisControlStatus z;
    if ((failure == NULL) ||
        (failure->source != MISSION_FAIL_SOURCE_AXIS_Z)) return;
    ZAxis_GetControlStatus(&z);
    if (z.state == Z_STATE_FAULT) (void)ZAxisControl_ClearFault();
}

/* 0: invalid/fault, 1: valid and idle, 2: transition still in progress. */
static uint8_t xyz_ready(void)
{
    XY_AxisStatus x;
    XY_AxisStatus y;
    ZAxisControlStatus z;
    (void)XY_GetStatus(XY_AXIS_X, &x);
    (void)XY_GetStatus(XY_AXIS_Y, &y);
    ZAxis_GetControlStatus(&z);
    if ((x.state == XY_STATE_STARTING) || (x.state == XY_STATE_MOVING) ||
        (x.state == XY_STATE_STOPPING) || (x.state == XY_STATE_HOMING) ||
        (y.state == XY_STATE_STARTING) || (y.state == XY_STATE_MOVING) ||
        (y.state == XY_STATE_STOPPING) || (y.state == XY_STATE_HOMING) ||
        (z.state == Z_STATE_STARTING) || (z.state == Z_STATE_MOVING) ||
        (z.state == Z_STATE_STOPPING) ||
        (z.state == Z_STATE_RECOVERING)) return 2U;
    if ((x.state != XY_STATE_IDLE) || (y.state != XY_STATE_IDLE) ||
        (z.state != Z_STATE_IDLE) || (x.position_valid == 0U) ||
        (y.position_valid == 0U) || (z.position_valid == 0U)) return 0U;
    return 1U;
}

static void stop_axes(void)
{
    XY_AxisStatus x;
    XY_AxisStatus y;
    ZAxisControlStatus z;
    (void)XY_GetStatus(XY_AXIS_X, &x);
    (void)XY_GetStatus(XY_AXIS_Y, &y);
    ZAxis_GetControlStatus(&z);
    if ((x.state == XY_STATE_STARTING) || (x.state == XY_STATE_MOVING))
        XY_Stop(XY_AXIS_X);
    if ((y.state == XY_STATE_STARTING) || (y.state == XY_STATE_MOVING))
        XY_Stop(XY_AXIS_Y);
    if ((z.state == Z_STATE_STARTING) || (z.state == Z_STATE_MOVING))
        (void)ZAxisControl_Stop();
}

static void finish(MissionSubflowState state, MissionFailure failure,
                   uint32_t now)
{
    g_subflow.status.state = state;
    g_subflow.status.failure = failure;
    g_subflow.status.state_tick = now;
    if (g_retain_owner == 0U)
        MotionCoordinator_Release(MOTION_OWNER_MISSION, now);
}

static void fail(MissionFailureSource source, MissionFailureReason reason,
                 int32_t detail, uint32_t now)
{
    MissionFailure failure = {source, reason, detail};
    stop_axes();
    finish(MISSION_SUBFLOW_FAULT, failure, now);
}

static uint8_t begin(MissionTaskName task, MissionSubflowType type,
                     uint8_t required_mask, uint8_t max_attempts,
                     uint32_t now)
{
    if ((task >= MISSION_TASK_COUNT) ||
        (MissionSubflow_IsActive() != 0U) ||
        (MotionCoordinator_Acquire(MOTION_OWNER_MISSION, required_mask,
                                   now) == 0U)) return 0U;
    memset(&g_subflow, 0, sizeof(g_subflow));
    g_cancel_request = 0U;
    {
        ZAxisControlStatus z;
        ZAxis_GetControlStatus(&z);
        g_subflow.z_fault_count_seen = z.fault_count;
    }
    g_subflow.status.task = task;
    g_subflow.status.type = type;
    g_subflow.status.state = MISSION_SUBFLOW_STARTING;
    g_subflow.status.max_attempts = max_attempts;
    g_subflow.status.max_axis_recoveries = SUBFLOW_Z_MAX_RECOVERIES;
    g_subflow.status.start_tick = now;
    g_subflow.status.state_tick = now;
    return 1U;
}

static uint8_t get_k230(uint8_t id, C552_K230Data *sample,
                        C552_Health *health)
{
    C552_Data data;
    uint8_t mask;
    if (C552_GetSnapshot(&data, health) == 0U) return 0U;
    if (id == C552_ID_K230_1) {
        *sample = data.k230_1;
        mask = C552_DEVICE_K230_1;
    } else {
        *sample = data.k230_2;
        mask = C552_DEVICE_K230_2;
    }
    return (((health->ready_mask & mask) != 0U) &&
            (k230_sample_blank(sample) == 0U)) ? 1U : 0U;
}

static void retry_or_fail(MissionFailureSource source,
                          MissionFailureReason reason, int32_t detail,
                          uint32_t now)
{
    if (g_subflow.status.attempt < g_subflow.status.max_attempts) {
        g_subflow.status.failure.source = source;
        g_subflow.status.failure.reason = reason;
        g_subflow.status.failure.detail = detail;
        g_subflow.status.state = MISSION_SUBFLOW_RETRY_WAIT;
        g_subflow.status.state_tick = now + SUBFLOW_RETRY_DELAY_MS;
        g_subflow.status.stable_samples = 0U;
        g_subflow.command_issued = 0U;
    } else {
        fail(source, MISSION_FAIL_RETRY_EXHAUSTED, detail, now);
    }
}

static void retry_axis_fault(MissionFailure failure, uint8_t refund_attempt,
                             uint32_t now)
{
    if ((failure.source == MISSION_FAIL_SOURCE_AXIS_Z) &&
        (g_subflow.status.axis_recoveries <
         g_subflow.status.max_axis_recoveries)) {
        ++g_subflow.status.axis_recoveries;
        if ((refund_attempt != 0U) && (g_subflow.status.attempt != 0U))
            --g_subflow.status.attempt;
        g_subflow.status.failure = failure;
        g_subflow.status.state = MISSION_SUBFLOW_RETRY_WAIT;
        g_subflow.status.state_tick = now + SUBFLOW_AXIS_RECOVERY_DELAY_MS;
        g_subflow.status.stable_samples = 0U;
        g_subflow.command_issued = 0U;
    } else if (failure.source == MISSION_FAIL_SOURCE_AXIS_Z) {
        fail(failure.source, MISSION_FAIL_RETRY_EXHAUSTED,
             failure.detail, now);
    } else {
        fail(failure.source, failure.reason, failure.detail, now);
    }
}

static void start_observe_attempt(uint32_t now)
{
    C552_RequestResult result;
    if (g_subflow.status.attempt >= g_subflow.status.max_attempts) {
        fail(MISSION_FAIL_SOURCE_K230_MODE, MISSION_FAIL_RETRY_EXHAUSTED,
             0, now);
        return;
    }
    ++g_subflow.status.attempt;
    g_subflow.attempt_tick = now;
    result = C552_SetK230Mode(g_subflow.k230_id, g_subflow.k230_mode, now);
    if (result == C552_REQUEST_OK) {
        g_subflow.status.state = MISSION_SUBFLOW_WAIT_APPLIED;
        g_subflow.status.state_tick = now;
    } else {
        retry_or_fail(MISSION_FAIL_SOURCE_K230_MODE,
                      (result == C552_REQUEST_BUSY) ? MISSION_FAIL_BUSY :
                      MISSION_FAIL_REJECTED, (int32_t)result, now);
    }
}

static void poll_observe(uint32_t now)
{
    C552_CommandStatus command;
    C552_K230Data sample;
    C552_Health health;
    uint8_t sample_valid;
    if (g_subflow.status.state == MISSION_SUBFLOW_STARTING) {
        start_observe_attempt(now);
        return;
    }
    if (g_subflow.status.state == MISSION_SUBFLOW_RETRY_WAIT) {
        if (tick_due(now, g_subflow.status.state_tick) != 0U)
            start_observe_attempt(now);
        return;
    }
    sample_valid = get_k230(g_subflow.k230_id, &sample, &health);
    if (g_subflow.status.state == MISSION_SUBFLOW_WAIT_APPLIED) {
        C552_GetCommandStatus(&command);
        if ((command.id == g_subflow.k230_id) &&
            (command.command == C552_COMMAND_SET_K230_MODE) &&
            (command.requested_value == (uint8_t)g_subflow.k230_mode) &&
            (command.state == C552_COMMAND_APPLIED)) {
            g_subflow.applied_sample_seq = sample_valid ? sample.sample_seq : 0U;
            g_subflow.status.state = MISSION_SUBFLOW_WAIT_SAMPLE;
            g_subflow.status.state_tick = now;
        } else if ((command.state == C552_COMMAND_FAILED) ||
                   (command.state == C552_COMMAND_TIMEOUT) ||
                   ((uint32_t)(now - g_subflow.status.state_tick) >
                    SUBFLOW_MODE_TIMEOUT_MS)) {
            retry_or_fail(MISSION_FAIL_SOURCE_K230_MODE,
                          MISSION_FAIL_TIMEOUT, (int32_t)command.result, now);
        }
        return;
    }
    if (g_subflow.status.state != MISSION_SUBFLOW_WAIT_SAMPLE) return;
    if ((uint32_t)(now - g_subflow.attempt_tick) >
        SUBFLOW_OBSERVE_ATTEMPT_MS) {
        retry_or_fail(MISSION_FAIL_SOURCE_K230_DATA,
                      MISSION_FAIL_TIMEOUT, 0, now);
        return;
    }
    if ((sample_valid != 0U) &&
        (sample.sample_seq != g_subflow.applied_sample_seq) &&
        (sample.sample_seq != g_subflow.status.last_sample_seq)) {
        int16_t next_min_x;
        int16_t next_max_x;
        int16_t next_min_y;
        int16_t next_max_y;
        g_subflow.status.last_sample_seq = sample.sample_seq;
        g_subflow.status.target_pixel[0] = sample.center_x;
        g_subflow.status.target_pixel[1] = sample.center_y;
        if (g_subflow.status.stable_samples == 0U) {
            g_subflow.stable_min[0] = sample.center_x;
            g_subflow.stable_max[0] = sample.center_x;
            g_subflow.stable_min[1] = sample.center_y;
            g_subflow.stable_max[1] = sample.center_y;
            g_subflow.status.stable_samples = 1U;
        } else {
            next_min_x = (sample.center_x < g_subflow.stable_min[0]) ?
                         sample.center_x : g_subflow.stable_min[0];
            next_max_x = (sample.center_x > g_subflow.stable_max[0]) ?
                         sample.center_x : g_subflow.stable_max[0];
            next_min_y = (sample.center_y < g_subflow.stable_min[1]) ?
                         sample.center_y : g_subflow.stable_min[1];
            next_max_y = (sample.center_y > g_subflow.stable_max[1]) ?
                         sample.center_y : g_subflow.stable_max[1];
            if (((int32_t)next_max_x - next_min_x <=
                 SUBFLOW_PIXEL_STABILITY) &&
                ((int32_t)next_max_y - next_min_y <=
                 SUBFLOW_PIXEL_STABILITY)) {
                g_subflow.stable_min[0] = next_min_x;
                g_subflow.stable_max[0] = next_max_x;
                g_subflow.stable_min[1] = next_min_y;
                g_subflow.stable_max[1] = next_max_y;
                ++g_subflow.status.stable_samples;
            } else {
                g_subflow.stable_min[0] = sample.center_x;
                g_subflow.stable_max[0] = sample.center_x;
                g_subflow.stable_min[1] = sample.center_y;
                g_subflow.stable_max[1] = sample.center_y;
                g_subflow.status.stable_samples = 1U;
            }
        }
        g_subflow.status.state_tick = now;
        if (g_subflow.status.stable_samples >= SUBFLOW_STABLE_SAMPLES) {
            MissionFailure none = {MISSION_FAIL_SOURCE_NONE,
                                   MISSION_FAIL_NONE, 0};
            finish(MISSION_SUBFLOW_COMPLETE, none, now);
        }
    } else if ((uint32_t)(now - g_subflow.status.state_tick) >
               SUBFLOW_SAMPLE_TIMEOUT_MS) {
        retry_or_fail((health.link_online != 0U) ?
                      MISSION_FAIL_SOURCE_K230_DATA :
                      MISSION_FAIL_SOURCE_C552,
                      (health.link_online == 0U) ? MISSION_FAIL_LINK :
                      ((sample_valid != 0U) ? MISSION_FAIL_TIMEOUT :
                       MISSION_FAIL_EMPTY_SAMPLE), 0, now);
    }
}

static void start_align_attempt(uint32_t now)
{
    uint8_t started;
    MissionFailure axis_failure;
    if (axes_faulted(&axis_failure) != 0U) {
        stop_axes();
        recover_z_fault(&axis_failure);
        retry_axis_fault(axis_failure, 0U, now);
        return;
    }
    if (align_axes_idle() == 0U) {
        g_subflow.status.state = MISSION_SUBFLOW_RETRY_WAIT;
        g_subflow.status.state_tick = now + SUBFLOW_RETRY_DELAY_MS;
        return;
    }
    if (g_subflow.status.attempt >= g_subflow.status.max_attempts) {
        fail(MISSION_FAIL_SOURCE_COORDINATOR, MISSION_FAIL_RETRY_EXHAUSTED,
             0, now);
        return;
    }
    ++g_subflow.status.attempt;
    if (g_subflow.status.type == MISSION_SUBFLOW_ALIGN_XZ) {
        uint8_t mode = (g_subflow.status.task == MISSION_TASK_TAG_PUT) ?
                       C552_K230_MODE_APRILTAG :
                       C552_K230_MODE_RED_BLOCK;
        started = XZVisionAlign_StartOwned(MOTION_OWNER_MISSION,
            g_subflow.status.target_pixel[0],
            g_subflow.status.target_pixel[1], mode, now);
    } else {
        started = XY_VisionAlign_StartOwned(MOTION_OWNER_MISSION,
            g_subflow.status.target_pixel[0],
            g_subflow.status.target_pixel[1], now);
    }
    if (started != 0U) {
        g_subflow.status.state = MISSION_SUBFLOW_RUNNING;
        g_subflow.status.state_tick = now;
    } else {
        retry_or_fail(MISSION_FAIL_SOURCE_COORDINATOR,
                      MISSION_FAIL_REJECTED, 0, now);
    }
}

static void poll_align(uint32_t now)
{
    MissionFailure axis_failure;
    if (g_subflow.status.state == MISSION_SUBFLOW_STARTING) {
        start_align_attempt(now);
        return;
    }
    if (g_subflow.status.state == MISSION_SUBFLOW_RETRY_WAIT) {
        if (tick_due(now, g_subflow.status.state_tick) != 0U)
            start_align_attempt(now);
        return;
    }
    if (g_subflow.status.state != MISSION_SUBFLOW_RUNNING) return;
    if (axes_faulted(&axis_failure) != 0U) {
        if (g_subflow.status.type == MISSION_SUBFLOW_ALIGN_XZ)
            XZVisionAlign_Abort();
        else
            XY_VisionAlign_Abort();
        stop_axes();
        recover_z_fault(&axis_failure);
        retry_axis_fault(axis_failure, 1U, now);
        return;
    }
    if (g_subflow.status.type == MISSION_SUBFLOW_ALIGN_XZ) {
        XZVisionAlignStatus align;
        XZVisionAlign_GetStatus(&align);
        if (align.state == XZ_VISION_ALIGN_COMPLETE) {
            MissionFailure none = {MISSION_FAIL_SOURCE_NONE,
                                   MISSION_FAIL_NONE, 0};
            finish(MISSION_SUBFLOW_COMPLETE, none, now);
        } else if (align.state == XZ_VISION_ALIGN_FAULT) {
            XZVisionAlign_Abort();
            if (align.fault == XZ_VISION_ALIGN_FAULT_AXIS) {
                MissionFailure failure = {MISSION_FAIL_SOURCE_AXIS_Z,
                    MISSION_FAIL_AXIS_FAULT, (int32_t)align.fault};
                recover_z_fault(&failure);
                retry_axis_fault(failure, 1U, now);
            } else if (align.fault == XZ_VISION_ALIGN_FAULT_MODE) {
                retry_or_fail(MISSION_FAIL_SOURCE_K230_MODE,
                              MISSION_FAIL_TIMEOUT,
                              (int32_t)align.fault, now);
            } else if (align.fault == XZ_VISION_ALIGN_FAULT_SOFT_LIMIT) {
                fail(MISSION_FAIL_SOURCE_POSE, MISSION_FAIL_SOFT_LIMIT,
                     (int32_t)align.fault, now);
            } else if (align.fault == XZ_VISION_ALIGN_FAULT_MOVE_X) {
                retry_or_fail(MISSION_FAIL_SOURCE_AXIS_X,
                              MISSION_FAIL_REJECTED,
                              (int32_t)align.fault, now);
            } else if (align.fault == XZ_VISION_ALIGN_FAULT_MOVE_Z) {
                MissionFailure failure = {MISSION_FAIL_SOURCE_AXIS_Z,
                    MISSION_FAIL_AXIS_FAULT, (int32_t)align.last_z_result};
                recover_z_fault(&failure);
                retry_axis_fault(failure, 1U, now);
            } else {
                retry_or_fail(MISSION_FAIL_SOURCE_K230_DATA,
                              MISSION_FAIL_TIMEOUT,
                              (int32_t)align.fault, now);
            }
        }
    } else {
        XY_VisionAlignStatus align;
        XY_VisionAlign_GetStatus(&align);
        if (align.state == XY_VISION_ALIGN_COMPLETE) {
            MissionFailure none = {MISSION_FAIL_SOURCE_NONE,
                                   MISSION_FAIL_NONE, 0};
            finish(MISSION_SUBFLOW_COMPLETE, none, now);
        } else if (align.state == XY_VISION_ALIGN_FAULT) {
            XY_VisionAlign_Abort();
            if (align.fault == XY_VISION_ALIGN_FAULT_AXIS) {
                fail(MISSION_FAIL_SOURCE_AXIS_X, MISSION_FAIL_AXIS_FAULT,
                     (int32_t)align.fault, now);
            } else {
                retry_or_fail(MISSION_FAIL_SOURCE_K230_DATA,
                              MISSION_FAIL_TIMEOUT,
                              (int32_t)align.fault, now);
            }
        }
    }
}

static uint8_t get_tof(uint8_t id, C552_TofData *tof)
{
    C552_Data data;
    C552_Health health;
    uint8_t mask;
    if (C552_GetSnapshot(&data, &health) == 0U) return 0U;
    if (id == C552_ID_TOF1) {
        *tof = data.tof1;
        mask = C552_DEVICE_TOF1;
    } else if (id == C552_ID_TOF2) {
        *tof = data.tof2;
        mask = C552_DEVICE_TOF2;
    } else {
        *tof = data.tof3;
        mask = C552_DEVICE_TOF3;
    }
    return ((health.ready_mask & mask) != 0U) ? 1U : 0U;
}

static void poll_blind_y(uint32_t now)
{
    C552_TofData tof;
    XY_AxisStatus y;
    MissionFailure failure;
    if (g_subflow.status.state == MISSION_SUBFLOW_RETRY_WAIT) {
        if (tick_due(now, g_subflow.status.state_tick) != 0U)
            g_subflow.status.state = MISSION_SUBFLOW_STARTING;
        return;
    }
    if (g_subflow.status.state == MISSION_SUBFLOW_STARTING) {
        int64_t pulses;
        XY_Result result;
        uint8_t ready;
        if (axes_faulted(&failure) != 0U) {
            stop_axes();
            recover_z_fault(&failure);
            retry_axis_fault(failure, 0U, now);
            return;
        }
        ready = xyz_ready();
        if (ready == 2U) {
            g_subflow.status.state = MISSION_SUBFLOW_RETRY_WAIT;
            g_subflow.status.state_tick = now + SUBFLOW_RETRY_DELAY_MS;
            return;
        }
        if (ready == 0U) {
            fail(MISSION_FAIL_SOURCE_POSE,
                 MISSION_FAIL_POSITION_INVALID, 0, now);
            return;
        }
        if (g_subflow.status.attempt >= g_subflow.status.max_attempts) {
            fail(MISSION_FAIL_SOURCE_AXIS_Y, MISSION_FAIL_RETRY_EXHAUSTED,
                 0, now);
            return;
        }
        ++g_subflow.status.attempt;
        if (get_tof(g_subflow.tof_id, &tof) == 0U) {
            retry_or_fail(MISSION_FAIL_SOURCE_TOF, MISSION_FAIL_LINK, 0, now);
            return;
        }
        g_subflow.status.distance_mm = tof.filtered_mm;
        if (tof.filtered_mm <= g_subflow.stop_mm) {
            MissionFailure none = {MISSION_FAIL_SOURCE_NONE,
                                   MISSION_FAIL_NONE, 0};
            finish(MISSION_SUBFLOW_COMPLETE, none, now);
            return;
        }
        pulses = (int64_t)(tof.filtered_mm - g_subflow.stop_mm) *
                 g_subflow.pulses_per_mm * g_subflow.direction;
        if ((pulses < INT32_MIN) || (pulses > INT32_MAX)) {
            fail(MISSION_FAIL_SOURCE_AXIS_Y, MISSION_FAIL_INVALID_ARGUMENT,
                 0, now);
            return;
        }
        g_subflow.status.requested_pulses = (int32_t)pulses;
        result = XY_MoveRelative(XY_AXIS_Y, (int32_t)pulses,
                                 SUBFLOW_Y_SPEED_RPM,
                                 SUBFLOW_ACCELERATION);
        if (result == XY_RESULT_OK) {
            g_subflow.status.state = MISSION_SUBFLOW_WAIT_IDLE;
            g_subflow.status.state_tick = now;
        } else if (result == XY_RESULT_BUSY) {
            /* Status queries are transient bus users and must not consume a
             * BlindMoveY motion attempt. */
            if (g_subflow.status.attempt != 0U)
                --g_subflow.status.attempt;
            g_subflow.status.state = MISSION_SUBFLOW_RETRY_WAIT;
            g_subflow.status.state_tick = now + SUBFLOW_BUSY_RETRY_DELAY_MS;
        } else if (result == XY_RESULT_SOFT_LIMIT) {
            fail(MISSION_FAIL_SOURCE_AXIS_Y, MISSION_FAIL_SOFT_LIMIT,
                 (int32_t)result, now);
        } else {
            retry_or_fail(MISSION_FAIL_SOURCE_AXIS_Y, MISSION_FAIL_REJECTED,
                          (int32_t)result, now);
        }
        return;
    }
    if (g_subflow.status.state != MISSION_SUBFLOW_WAIT_IDLE) return;
    (void)XY_GetStatus(XY_AXIS_Y, &y);
    if (y.state == XY_STATE_IDLE) {
        MissionFailure none = {MISSION_FAIL_SOURCE_NONE, MISSION_FAIL_NONE, 0};
        finish(MISSION_SUBFLOW_COMPLETE, none, now);
    } else if (axes_faulted(&failure) != 0U) {
        stop_axes();
        recover_z_fault(&failure);
        retry_axis_fault(failure, 1U, now);
    } else if ((uint32_t)(now - g_subflow.status.state_tick) >
               SUBFLOW_MOVE_TIMEOUT_MS) {
        stop_axes();
        retry_or_fail(MISSION_FAIL_SOURCE_AXIS_Y, MISSION_FAIL_TIMEOUT, 0, now);
    }
}

static void poll_tof3_descend(uint32_t now)
{
    C552_TofData tof;
    ZAxisControlStatus z;
    MissionFailure failure;
    uint8_t new_sample = 0U;
    if ((g_subflow.status.state == MISSION_SUBFLOW_STARTING) &&
        (g_subflow.command_issued == 0U)) {
        uint8_t ready;
        if (axes_faulted(&failure) != 0U) {
            stop_axes();
            recover_z_fault(&failure);
            retry_axis_fault(failure, 1U, now);
            return;
        }
        ready = xyz_ready();
        if (ready == 2U) {
            g_subflow.status.state = MISSION_SUBFLOW_RETRY_WAIT;
            g_subflow.status.state_tick = now + SUBFLOW_RETRY_DELAY_MS;
            return;
        }
        if (ready == 0U) {
            fail(MISSION_FAIL_SOURCE_POSE,
                 MISSION_FAIL_POSITION_INVALID, 0, now);
            return;
        }
        if (g_subflow.status.attempt >= g_subflow.status.max_attempts) {
            fail(MISSION_FAIL_SOURCE_AXIS_Z,
                 MISSION_FAIL_RETRY_EXHAUSTED, 0, now);
            return;
        }
        ++g_subflow.status.attempt;
        g_subflow.command_issued = 1U;
    }
    if (get_tof(C552_ID_TOF3, &tof) != 0U) {
        g_subflow.status.distance_mm = tof.filtered_mm;
        if (tof.sample_seq != g_subflow.status.last_sample_seq) {
            g_subflow.status.last_sample_seq = tof.sample_seq;
            new_sample = 1U;
            g_subflow.last_sensor_tick = now;
            if (tof.filtered_mm <= g_subflow.stop_mm) {
                if (g_subflow.status.stable_samples < UINT8_MAX)
                    ++g_subflow.status.stable_samples;
            } else {
                g_subflow.status.stable_samples = 0U;
            }
        }
    }
    ZAxis_GetControlStatus(&z);
    if ((g_subflow.status.state != MISSION_SUBFLOW_RETRY_WAIT) &&
        ((uint32_t)(now - g_subflow.last_sensor_tick) >
         SUBFLOW_SAMPLE_TIMEOUT_MS)) {
        stop_axes();
        retry_or_fail(MISSION_FAIL_SOURCE_TOF, MISSION_FAIL_TIMEOUT, 0, now);
        return;
    }
    if (g_subflow.status.stable_samples >= SUBFLOW_TOF3_STABLE_SAMPLES) {
        if ((z.state == Z_STATE_STARTING) || (z.state == Z_STATE_MOVING)) {
            (void)ZAxisControl_Stop();
            g_subflow.status.state = MISSION_SUBFLOW_WAIT_IDLE;
        } else if (z.state == Z_STATE_IDLE) {
            MissionFailure none = {MISSION_FAIL_SOURCE_NONE,
                                   MISSION_FAIL_NONE, 0};
            finish(MISSION_SUBFLOW_COMPLETE, none, now);
        }
        return;
    }
    if (g_subflow.status.stable_samples != 0U) {
        if ((z.state == Z_STATE_STARTING) || (z.state == Z_STATE_MOVING)) {
            (void)ZAxisControl_Stop();
            g_subflow.status.state = MISSION_SUBFLOW_WAIT_IDLE;
        }
        return;
    }
    if ((g_subflow.status.state == MISSION_SUBFLOW_WAIT_IDLE) &&
        (z.state == Z_STATE_IDLE)) {
        g_subflow.status.state = MISSION_SUBFLOW_RUNNING;
    }
    if (g_subflow.status.state == MISSION_SUBFLOW_RETRY_WAIT) {
        if (tick_due(now, g_subflow.status.state_tick) != 0U) {
            g_subflow.status.state = MISSION_SUBFLOW_STARTING;
            g_subflow.last_sensor_tick = now;
        }
        return;
    }
    if ((g_subflow.status.state == MISSION_SUBFLOW_STARTING) ||
        ((g_subflow.status.state == MISSION_SUBFLOW_RUNNING) &&
         (z.state == Z_STATE_IDLE))) {
        int32_t remaining = g_subflow.max_descent_pulses -
                            g_subflow.descent_commanded;
        int32_t segment;
        ZAxisControlResult result;
        if (axes_faulted(&failure) != 0U) {
            stop_axes();
            recover_z_fault(&failure);
            retry_axis_fault(failure, 1U, now);
            return;
        }
        if ((z.state == Z_STATE_STARTING) || (z.state == Z_STATE_MOVING) ||
            (z.state == Z_STATE_STOPPING)) return;
        if (new_sample == 0U) {
            if ((uint32_t)(now - g_subflow.status.state_tick) >
                SUBFLOW_SAMPLE_TIMEOUT_MS)
                retry_or_fail(MISSION_FAIL_SOURCE_TOF,
                              MISSION_FAIL_TIMEOUT, 0, now);
            return;
        }
        if (remaining <= 0) {
            fail(MISSION_FAIL_SOURCE_AXIS_Z, MISSION_FAIL_SOFT_LIMIT, 0, now);
            return;
        }
        segment = (remaining < SUBFLOW_TOF3_SEGMENT_PULSES) ?
                  remaining : SUBFLOW_TOF3_SEGMENT_PULSES;
        segment *= g_subflow.direction;
        result = ZAxisControl_MoveRelative(segment, SUBFLOW_Z_SPEED_HZ);
        if (result == Z_RESULT_OK) {
            g_subflow.descent_commanded +=
                (segment < 0) ? -segment : segment;
            g_subflow.status.requested_pulses = segment;
            g_subflow.status.state = MISSION_SUBFLOW_RUNNING;
            g_subflow.status.state_tick = now;
        } else if (result == Z_RESULT_SOFT_LIMIT) {
            fail(MISSION_FAIL_SOURCE_AXIS_Z, MISSION_FAIL_SOFT_LIMIT,
                 (int32_t)result, now);
        } else if (result != Z_RESULT_BUSY) {
            MissionFailure move_failure = {MISSION_FAIL_SOURCE_AXIS_Z,
                MISSION_FAIL_AXIS_FAULT, (int32_t)result};
            recover_z_fault(&move_failure);
            retry_axis_fault(move_failure, 1U, now);
        }
    } else if (axes_faulted(&failure) != 0U) {
        stop_axes();
        recover_z_fault(&failure);
        retry_axis_fault(failure, 1U, now);
    } else if ((uint32_t)(now - g_subflow.status.state_tick) >
               SUBFLOW_MOVE_TIMEOUT_MS) {
        stop_axes();
        retry_or_fail(MISSION_FAIL_SOURCE_AXIS_Z, MISSION_FAIL_TIMEOUT, 0, now);
    }
}

static void poll_grip(uint32_t now)
{
    C552_CommandStatus command;
    C552_RequestResult result;
    uint8_t requested = (g_subflow.status.type ==
                         MISSION_SUBFLOW_GRIP_CLOSE) ?
                        C552_GRIPPER_CLOSED : C552_GRIPPER_OPEN;
    if (g_subflow.status.state == MISSION_SUBFLOW_RETRY_WAIT) {
        if (tick_due(now, g_subflow.status.state_tick) != 0U)
            g_subflow.status.state = MISSION_SUBFLOW_STARTING;
        return;
    }
    if (g_subflow.status.state == MISSION_SUBFLOW_STARTING) {
        if (g_subflow.status.attempt >= g_subflow.status.max_attempts) {
            fail(MISSION_FAIL_SOURCE_GRIPPER,
                 MISSION_FAIL_RETRY_EXHAUSTED, 0, now);
            return;
        }
        ++g_subflow.status.attempt;
        result = C552_SetGripper(C552_GRIPPER_BOTH,
                                 (C552_GripperState)requested, now);
        if (result == C552_REQUEST_OK) {
            g_subflow.status.state = MISSION_SUBFLOW_WAIT_APPLIED;
            g_subflow.status.state_tick = now;
        } else {
            retry_or_fail(MISSION_FAIL_SOURCE_GRIPPER,
                          MISSION_FAIL_BUSY, (int32_t)result, now);
        }
        return;
    }
    C552_GetCommandStatus(&command);
    if ((command.id == C552_ID_GRIPPER) &&
        (command.command == C552_COMMAND_SET_GRIPPER) &&
        (command.requested_value == requested) &&
        (command.state == C552_COMMAND_APPLIED)) {
        MissionFailure none = {MISSION_FAIL_SOURCE_NONE, MISSION_FAIL_NONE, 0};
        finish(MISSION_SUBFLOW_COMPLETE, none, now);
    } else if ((command.state == C552_COMMAND_FAILED) ||
               (command.state == C552_COMMAND_TIMEOUT) ||
               ((uint32_t)(now - g_subflow.status.state_tick) >
                SUBFLOW_MODE_TIMEOUT_MS)) {
        retry_or_fail(MISSION_FAIL_SOURCE_GRIPPER, MISSION_FAIL_TIMEOUT,
                      (int32_t)command.result, now);
    }
}

static uint8_t start_axis_absolute(uint8_t axis,
                                   const MotionPositionSnapshot *pose)
{
    if (axis == 0U)
        return (XY_MoveAbsolute(XY_AXIS_X, pose->x_pulses,
                SUBFLOW_X_SPEED_RPM, SUBFLOW_ACCELERATION) == XY_RESULT_OK);
    if (axis == 1U)
        return (XY_MoveAbsolute(XY_AXIS_Y, pose->y_pulses,
                SUBFLOW_Y_SPEED_RPM, SUBFLOW_ACCELERATION) == XY_RESULT_OK);
    return (ZAxisControl_MoveAbsolute(pose->z_pulses,
                                      SUBFLOW_Z_SPEED_HZ) == Z_RESULT_OK);
}

static uint8_t axis_idle(uint8_t axis)
{
    XY_AxisStatus xy;
    ZAxisControlStatus z;
    if (axis < 2U) {
        (void)XY_GetStatus((XY_Axis)axis, &xy);
        return (xy.state == XY_STATE_IDLE) ? 1U : 0U;
    }
    ZAxis_GetControlStatus(&z);
    return (z.state == Z_STATE_IDLE) ? 1U : 0U;
}

static uint8_t pose_axis_for_stage(void)
{
    static const uint8_t zxy_order[3] = {2U, 0U, 1U};
    static const uint8_t tag_retreat_order[3] = {1U, 2U, 0U};
    if ((g_subflow.status.type == MISSION_SUBFLOW_SAFE_RETREAT) &&
        (g_subflow.status.task == MISSION_TASK_TAG_PUT)) {
        return tag_retreat_order[g_subflow.return_axis];
    }
    if ((g_subflow.status.type == MISSION_SUBFLOW_SAFE_RETREAT) ||
        (g_subflow.status.type == MISSION_SUBFLOW_PRESET_POSE)) {
        return zxy_order[g_subflow.return_axis];
    }
    return g_subflow.return_axis;
}

static void poll_pose(uint32_t now)
{
    const MotionPositionSnapshot *pose;
    if (g_subflow.status.type == MISSION_SUBFLOW_RETURN_POSE)
        pose = &g_recorded_pose;
    else if (g_subflow.status.type == MISSION_SUBFLOW_PRESET_POSE)
        pose = &g_subflow.status.pose;
    else
        pose = &g_safe_pose;
    if (g_subflow.status.type == MISSION_SUBFLOW_RECORD_POSE) {
        if ((g_subflow.status.state == MISSION_SUBFLOW_STARTING) ||
            ((g_subflow.status.state == MISSION_SUBFLOW_RETRY_WAIT) &&
             (tick_due(now, g_subflow.status.state_tick) != 0U))) {
            ++g_subflow.status.attempt;
            if (MotionCoordinator_CaptureSnapshot(&g_recorded_pose, 1U,
                                                  now) != 0U) {
                MissionFailure none = {MISSION_FAIL_SOURCE_NONE,
                                       MISSION_FAIL_NONE, 0};
                g_recorded_pose_valid = 1U;
                g_subflow.status.pose = g_recorded_pose;
                finish(MISSION_SUBFLOW_COMPLETE, none, now);
            } else if (g_subflow.status.attempt >=
                       g_subflow.status.max_attempts) {
                fail(MISSION_FAIL_SOURCE_POSE,
                     MISSION_FAIL_POSITION_INVALID, 0, now);
            } else {
                g_subflow.status.state = MISSION_SUBFLOW_RETRY_WAIT;
                g_subflow.status.state_tick = now + SUBFLOW_POSE_RETRY_MS;
            }
        }
        return;
    }
    if (g_subflow.status.state == MISSION_SUBFLOW_RETRY_WAIT) {
        if (tick_due(now, g_subflow.status.state_tick) != 0U)
            g_subflow.status.state = MISSION_SUBFLOW_STARTING;
        return;
    }
    if (g_subflow.status.state == MISSION_SUBFLOW_STARTING) {
        uint8_t axis = pose_axis_for_stage();
        if (g_subflow.status.attempt >= g_subflow.status.max_attempts) {
            fail(MISSION_FAIL_SOURCE_POSE, MISSION_FAIL_RETRY_EXHAUSTED,
                 axis, now);
            return;
        }
        ++g_subflow.status.attempt;
        if (start_axis_absolute(axis, pose) != 0U) {
            g_subflow.status.state = MISSION_SUBFLOW_WAIT_IDLE;
            g_subflow.status.state_tick = now;
        } else {
            retry_or_fail(MISSION_FAIL_SOURCE_POSE, MISSION_FAIL_REJECTED,
                          axis, now);
        }
    } else if (g_subflow.status.state == MISSION_SUBFLOW_WAIT_IDLE) {
        MissionFailure failure;
        uint8_t axis = pose_axis_for_stage();
        if (axis_idle(axis) != 0U) {
            ++g_subflow.return_axis;
            if (g_subflow.return_axis >= 3U) {
                MissionFailure none = {MISSION_FAIL_SOURCE_NONE,
                                       MISSION_FAIL_NONE, 0};
                finish(MISSION_SUBFLOW_COMPLETE, none, now);
            } else {
                g_subflow.status.state = MISSION_SUBFLOW_STARTING;
            }
        } else if (axes_faulted(&failure) != 0U) {
            fail(failure.source, failure.reason, failure.detail, now);
        } else if ((uint32_t)(now - g_subflow.status.state_tick) >
                   SUBFLOW_MOVE_TIMEOUT_MS) {
            fail(MISSION_FAIL_SOURCE_POSE, MISSION_FAIL_TIMEOUT,
                 axis, now);
        }
    }
}

void MissionSubflow_Init(uint32_t now)
{
    memset(&g_subflow, 0, sizeof(g_subflow));
    memset(&g_recorded_pose, 0, sizeof(g_recorded_pose));
    memset(&g_safe_pose, 0, sizeof(g_safe_pose));
    g_subflow.status.state = MISSION_SUBFLOW_IDLE;
    g_subflow.status.state_tick = now;
    g_recorded_pose_valid = 0U;
    g_safe_pose_valid = 0U;
    g_retain_owner = 0U;
    g_cancel_request = 0U;
}

uint8_t MissionSubflow_SetOwnerRetention(uint8_t retain)
{
    if (MissionSubflow_IsActive() != 0U) return 0U;
    g_retain_owner = (retain != 0U) ? 1U : 0U;
    return 1U;
}

static void cancel_now(uint32_t now)
{
    MissionFailure failure = {MISSION_FAIL_SOURCE_COORDINATOR,
                              MISSION_FAIL_CANCELLED, 0};
    if (g_subflow.status.type == MISSION_SUBFLOW_ALIGN_XZ)
        XZVisionAlign_Abort();
    if (g_subflow.status.type == MISSION_SUBFLOW_ALIGN_XY)
        XY_VisionAlign_Abort();
    stop_axes();
    finish(MISSION_SUBFLOW_FAULT, failure, now);
}

void MissionSubflow_Poll(uint32_t now)
{
    MissionFailure axis_failure;
    if (MissionSubflow_IsActive() == 0U) return;
    if (g_cancel_request != 0U) {
        g_cancel_request = 0U;
        cancel_now(now);
        return;
    }
    if (MotionCoordinator_GetOwner() != MOTION_OWNER_MISSION) {
        MissionFailure failure = {MISSION_FAIL_SOURCE_COORDINATOR,
                                  MISSION_FAIL_CANCELLED, 0};
        stop_axes();
        g_subflow.status.state = MISSION_SUBFLOW_FAULT;
        g_subflow.status.failure = failure;
        return;
    }
    if ((g_subflow.status.type != MISSION_SUBFLOW_ALIGN_XZ) &&
        (g_subflow.status.type != MISSION_SUBFLOW_ALIGN_XY) &&
        (g_subflow.status.type != MISSION_SUBFLOW_BLIND_MOVE_Y) &&
        (g_subflow.status.type != MISSION_SUBFLOW_TOF3_DESCEND) &&
        (axes_faulted(&axis_failure) != 0U)) {
        fail(axis_failure.source, axis_failure.reason,
             axis_failure.detail, now);
        return;
    }
    switch (g_subflow.status.type) {
    case MISSION_SUBFLOW_OBSERVE_RED_FRONT:
    case MISSION_SUBFLOW_OBSERVE_TAG_FRONT:
    case MISSION_SUBFLOW_OBSERVE_FRAME_DOWN:
        poll_observe(now);
        break;
    case MISSION_SUBFLOW_ALIGN_XZ:
    case MISSION_SUBFLOW_ALIGN_XY:
        poll_align(now);
        break;
    case MISSION_SUBFLOW_BLIND_MOVE_Y:
        poll_blind_y(now);
        break;
    case MISSION_SUBFLOW_TOF3_DESCEND:
        poll_tof3_descend(now);
        break;
    case MISSION_SUBFLOW_GRIP_OPEN:
    case MISSION_SUBFLOW_GRIP_CLOSE:
        poll_grip(now);
        break;
    case MISSION_SUBFLOW_RECORD_POSE:
    case MISSION_SUBFLOW_RETURN_POSE:
    case MISSION_SUBFLOW_SAFE_RETREAT:
    case MISSION_SUBFLOW_PRESET_POSE:
        poll_pose(now);
        break;
    default:
        fail(MISSION_FAIL_SOURCE_COORDINATOR,
             MISSION_FAIL_INVALID_ARGUMENT, 0, now);
        break;
    }
}

static uint8_t start_observe(MissionTaskName task, MissionSubflowType type,
                             uint8_t id, C552_K230Mode mode, uint32_t now)
{
    uint8_t mask = (id == C552_ID_K230_1) ?
                   C552_DEVICE_K230_1 : C552_DEVICE_K230_2;
    if (begin(task, type, mask, SUBFLOW_MAX_ATTEMPTS, now) == 0U) return 0U;
    g_subflow.k230_id = id;
    g_subflow.k230_mode = mode;
    return 1U;
}

uint8_t MissionSubflow_StartObserveRedFront(MissionTaskName task,
                                            uint32_t now)
{
    return start_observe(task, MISSION_SUBFLOW_OBSERVE_RED_FRONT,
                         C552_ID_K230_2, C552_K230_MODE_RED_BLOCK, now);
}

uint8_t MissionSubflow_StartObserveTagFront(MissionTaskName task,
                                            uint32_t now)
{
    return start_observe(task, MISSION_SUBFLOW_OBSERVE_TAG_FRONT,
                         C552_ID_K230_2, C552_K230_MODE_APRILTAG, now);
}

uint8_t MissionSubflow_StartObserveFrameDown(MissionTaskName task,
                                             uint32_t now)
{
    return start_observe(task, MISSION_SUBFLOW_OBSERVE_FRAME_DOWN,
                         C552_ID_K230_1, C552_K230_MODE_RED_BLOCK, now);
}

static uint8_t start_align(MissionTaskName task, MissionSubflowType type,
                           int16_t target_x, int16_t target_y, uint32_t now)
{
    uint8_t mask = (type == MISSION_SUBFLOW_ALIGN_XZ) ?
                   C552_DEVICE_K230_2 : C552_DEVICE_K230_1;
    if (begin(task, type, mask, SUBFLOW_MAX_ATTEMPTS, now) == 0U) return 0U;
    g_subflow.status.target_pixel[0] = target_x;
    g_subflow.status.target_pixel[1] = target_y;
    return 1U;
}

uint8_t MissionSubflow_StartAlignXZ(MissionTaskName task, int16_t target_x,
                                    int16_t target_y, uint32_t now)
{
    return start_align(task, MISSION_SUBFLOW_ALIGN_XZ,
                       target_x, target_y, now);
}

uint8_t MissionSubflow_StartAlignXY(MissionTaskName task, int16_t target_x,
                                    int16_t target_y, uint32_t now)
{
    return start_align(task, MISSION_SUBFLOW_ALIGN_XY,
                       target_x, target_y, now);
}

uint8_t MissionSubflow_StartBlindMoveY(MissionTaskName task, uint8_t tof_id,
                                       uint16_t stop_mm,
                                       uint32_t pulses_per_mm,
                                       int8_t direction, uint32_t now)
{
    uint8_t mask;
    if (((tof_id != C552_ID_TOF1) && (tof_id != C552_ID_TOF2)) ||
        ((task != MISSION_TASK_RED_PICK) &&
         (task != MISSION_TASK_RED_FIND) &&
         (task != MISSION_TASK_TAG_PUT)) ||
        (((task == MISSION_TASK_RED_PICK) ||
          (task == MISSION_TASK_RED_FIND)) &&
         (tof_id != C552_ID_TOF2)) ||
        ((task == MISSION_TASK_TAG_PUT) &&
         (tof_id != C552_ID_TOF1)) ||
        (pulses_per_mm == 0U) ||
        ((direction != -1) && (direction != 1))) return 0U;
    mask = (tof_id == C552_ID_TOF1) ? C552_DEVICE_TOF1 : C552_DEVICE_TOF2;
    if (begin(task, MISSION_SUBFLOW_BLIND_MOVE_Y, mask,
              SUBFLOW_MAX_ATTEMPTS, now) == 0U)
        return 0U;
    g_subflow.tof_id = tof_id;
    g_subflow.stop_mm = stop_mm;
    g_subflow.pulses_per_mm = pulses_per_mm;
    g_subflow.direction = direction;
    return 1U;
}

uint8_t MissionSubflow_StartTof3Descend(MissionTaskName task,
                                        uint16_t stop_mm,
                                        uint32_t max_descent_pulses,
                                        int8_t direction, uint32_t now)
{
    if ((max_descent_pulses == 0U) ||
        (max_descent_pulses > INT32_MAX) ||
        ((direction != -1) && (direction != 1))) return 0U;
    if (begin(task, MISSION_SUBFLOW_TOF3_DESCEND, C552_DEVICE_TOF3,
              SUBFLOW_MAX_ATTEMPTS, now) == 0U) return 0U;
    g_subflow.stop_mm = stop_mm;
    g_subflow.max_descent_pulses = (int32_t)max_descent_pulses;
    g_subflow.direction = direction;
    g_subflow.last_sensor_tick = now;
    return 1U;
}

uint8_t MissionSubflow_StartGrip(MissionTaskName task, uint8_t close,
                                 uint32_t now)
{
    return begin(task, close ? MISSION_SUBFLOW_GRIP_CLOSE :
                              MISSION_SUBFLOW_GRIP_OPEN,
                 0U, SUBFLOW_MAX_ATTEMPTS, now);
}

uint8_t MissionSubflow_StartRecordPose(MissionTaskName task, uint32_t now)
{
    return begin(task, MISSION_SUBFLOW_RECORD_POSE, 0U,
                 SUBFLOW_MAX_ATTEMPTS, now);
}

uint8_t MissionSubflow_StartReturnPose(MissionTaskName task, uint32_t now)
{
    MotionPositionSnapshot current;
    if ((g_recorded_pose_valid == 0U) ||
        (MotionCoordinator_CaptureSnapshot(&current, 1U, now) == 0U))
        return 0U;
    if (begin(task, MISSION_SUBFLOW_RETURN_POSE, 0U,
              SUBFLOW_MAX_ATTEMPTS, now) == 0U)
        return 0U;
    g_subflow.status.pose = g_recorded_pose;
    return 1U;
}

static uint8_t pose_in_limits(const MotionPositionSnapshot *pose)
{
    const XY_AxisConfig *x = XY_GetConfig(XY_AXIS_X);
    const XY_AxisConfig *y = XY_GetConfig(XY_AXIS_Y);
    return ((pose != NULL) && (x != NULL) && (y != NULL) &&
            (pose->x_pulses >= x->soft_min_pulses) &&
            (pose->x_pulses <= x->soft_max_pulses) &&
            (pose->y_pulses >= y->soft_min_pulses) &&
            (pose->y_pulses <= y->soft_max_pulses) &&
            (pose->z_pulses >= Z_AXIS_SOFT_MIN_PULSES) &&
            (pose->z_pulses <= Z_AXIS_SOFT_MAX_PULSES)) ? 1U : 0U;
}

uint8_t MissionSubflow_StartPresetPose(MissionTaskName task,
                                       const MotionPositionSnapshot *pose,
                                       uint32_t now)
{
    MotionPositionSnapshot current;
    if ((pose_in_limits(pose) == 0U) ||
        (MotionCoordinator_CaptureSnapshot(&current, 1U, now) == 0U))
        return 0U;
    if (begin(task, MISSION_SUBFLOW_PRESET_POSE, 0U,
              SUBFLOW_MAX_ATTEMPTS, now) == 0U)
        return 0U;
    g_subflow.status.pose = *pose;
    return 1U;
}

uint8_t MissionSubflow_SetSafePose(const MotionPositionSnapshot *pose)
{
    if (pose_in_limits(pose) == 0U) return 0U;
    g_safe_pose = *pose;
    g_safe_pose_valid = 1U;
    return 1U;
}

uint8_t MissionSubflow_HasSafePose(void)
{
    return g_safe_pose_valid;
}

uint8_t MissionSubflow_GetSafePose(MotionPositionSnapshot *pose)
{
    if ((pose == NULL) || (g_safe_pose_valid == 0U)) return 0U;
    *pose = g_safe_pose;
    return 1U;
}

uint8_t MissionSubflow_StartSafeRetreat(MissionTaskName task, uint32_t now)
{
    MotionPositionSnapshot current;
    if ((g_safe_pose_valid == 0U) ||
        (MotionCoordinator_CaptureSnapshot(&current, 1U, now) == 0U))
        return 0U;
    if (begin(task, MISSION_SUBFLOW_SAFE_RETREAT, 0U,
              SUBFLOW_MAX_ATTEMPTS, now) == 0U)
        return 0U;
    g_subflow.status.pose = g_safe_pose;
    return 1U;
}

void MissionSubflow_Cancel(uint32_t now)
{
    (void)now;
    g_cancel_request = MissionSubflow_IsActive();
}

void MissionSubflow_AbortAll(MissionFailure failure, uint32_t now)
{
    if (g_subflow.status.type == MISSION_SUBFLOW_ALIGN_XZ)
        XZVisionAlign_Abort();
    if (g_subflow.status.type == MISSION_SUBFLOW_ALIGN_XY)
        XY_VisionAlign_Abort();
    stop_axes();
    g_subflow.status.type = MISSION_SUBFLOW_ABORT_ALL;
    finish(MISSION_SUBFLOW_FAULT, failure, now);
}

void MissionSubflow_GetStatus(MissionSubflowStatus *status)
{
    if (status != NULL) *status = g_subflow.status;
}

uint8_t MissionSubflow_IsActive(void)
{
    return subflow_active_state(g_subflow.status.state);
}

const char *MissionSubflow_TaskString(MissionTaskName task)
{
    static const char *const names[] = {
        "red_pick", "tag_put", "red_find", "frame_put"
    };
    return ((uint32_t)task < MISSION_TASK_COUNT) ? names[task] : "unknown";
}

const char *MissionSubflow_TypeString(MissionSubflowType type)
{
    static const char *const names[] = {
        "NONE", "ObserveRedFront", "ObserveTagFront", "ObserveFrameDown",
        "AlignXZ", "AlignXY", "BlindMoveY", "Tof3Descend", "GripOpen",
        "GripClose", "RecordPose", "ReturnPose", "SafeRetreat",
        "PresetPose", "AbortAll"
    };
    return ((uint32_t)type < (sizeof(names) / sizeof(names[0]))) ?
           names[type] : "UNKNOWN";
}

const char *MissionSubflow_StateString(MissionSubflowState state)
{
    static const char *const names[] = {
        "IDLE", "STARTING", "WAIT_APPLIED", "WAIT_SAMPLE", "RUNNING",
        "WAIT_IDLE", "RETRY_WAIT", "COMPLETE", "FAULT"
    };
    return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
           names[state] : "UNKNOWN";
}

const char *MissionSubflow_SourceString(MissionFailureSource source)
{
    static const char *const names[] = {
        "NONE", "AXIS_X", "AXIS_Y", "AXIS_Z", "K230_MODE", "K230_DATA",
        "C552", "TOF", "GRIPPER", "POSE", "COORDINATOR"
    };
    return ((uint32_t)source < (sizeof(names) / sizeof(names[0]))) ?
           names[source] : "UNKNOWN";
}

const char *MissionSubflow_ReasonString(MissionFailureReason reason)
{
    static const char *const names[] = {
        "NONE", "BUSY", "INVALID_ARGUMENT", "LINK", "TIMEOUT", "REJECTED",
        "EMPTY_SAMPLE", "AXIS_FAULT", "POSITION_INVALID", "SOFT_LIMIT",
        "RETRY_EXHAUSTED", "CANCELLED"
    };
    return ((uint32_t)reason < (sizeof(names) / sizeof(names[0]))) ?
           names[reason] : "UNKNOWN";
}

#include "motion_coordinator.h"

#include "c552.h"
#include "main.h"
#include "vision_calibration.h"
#include "xy_motor.h"
#include "xy_vision_align.h"
#include "xz_vision_align.h"
#include "xz_vision_calibration.h"
#include "z_axis.h"
#include <string.h>

static MotionCoordinatorStatus g_motion;
static volatile uint8_t g_abort_request;
static uint8_t g_idle_required_mask = C552_DEVICE_REQUIRED_DEFAULT;
static uint8_t g_xy_stop_sent;
static uint32_t g_z_fault_count_at_acquire;

static uint8_t owner_is_automatic(MotionOwner owner)
{
    return ((owner == MOTION_OWNER_P2_CALIBRATION) ||
            (owner == MOTION_OWNER_P3_CALIBRATION) ||
            (owner == MOTION_OWNER_XY_ALIGN) ||
            (owner == MOTION_OWNER_P4_XZ_ALIGN) ||
            (owner == MOTION_OWNER_MISSION)) ? 1U : 0U;
}

static uint8_t owner_task_active(MotionOwner owner)
{
    switch (owner) {
    case MOTION_OWNER_P2_CALIBRATION:
        return VisionCalibration_IsActive();
    case MOTION_OWNER_P3_CALIBRATION:
        return XZCalibration_IsActive();
    case MOTION_OWNER_XY_ALIGN:
        return XY_VisionAlign_IsActive();
    case MOTION_OWNER_P4_XZ_ALIGN:
        return XZVisionAlign_IsActive();
    case MOTION_OWNER_MISSION:
        return 1U;
    default:
        return 0U;
    }
}

static uint8_t axes_faulted(void)
{
    XY_AxisStatus x;
    XY_AxisStatus y;
    ZAxisControlStatus z;
    if ((XY_GetStatus(XY_AXIS_X, &x) == 0U) ||
        (XY_GetStatus(XY_AXIS_Y, &y) == 0U)) return 1U;
    ZAxis_GetControlStatus(&z);
    return ((x.state == XY_STATE_FAULT) || (y.state == XY_STATE_FAULT) ||
            (z.state == Z_STATE_FAULT)) ? 1U : 0U;
}

static uint8_t monitored_axis_faulted(void)
{
    ZAxisControlStatus z;
    ZAxis_GetControlStatus(&z);
    return ((axes_faulted() != 0U) ||
            (z.fault_count != g_z_fault_count_at_acquire)) ? 1U : 0U;
}

static uint8_t axes_stopped(void)
{
    XY_AxisStatus x;
    XY_AxisStatus y;
    ZAxisControlStatus z;
    if ((XY_GetStatus(XY_AXIS_X, &x) == 0U) ||
        (XY_GetStatus(XY_AXIS_Y, &y) == 0U)) return 0U;
    ZAxis_GetControlStatus(&z);
    return (((x.state != XY_STATE_STARTING) &&
             (x.state != XY_STATE_MOVING) &&
             (x.state != XY_STATE_STOPPING) &&
             (x.state != XY_STATE_HOMING)) &&
            ((y.state != XY_STATE_STARTING) &&
             (y.state != XY_STATE_MOVING) &&
             (y.state != XY_STATE_STOPPING) &&
             (y.state != XY_STATE_HOMING)) &&
            ((z.state != Z_STATE_STARTING) &&
             (z.state != Z_STATE_MOVING) &&
             (z.state != Z_STATE_STOPPING))) ? 1U : 0U;
}

static void abort_all_tasks(void)
{
    if (VisionCalibration_IsActive() != 0U) VisionCalibration_Abort();
    if (XZCalibration_IsActive() != 0U) XZCalibration_Abort();
    if (XY_VisionAlign_IsActive() != 0U) XY_VisionAlign_Abort();
    if (XZVisionAlign_IsActive() != 0U) XZVisionAlign_Abort();
}

static void begin_stop(uint32_t now)
{
    g_motion.stop_pending = 1U;
    g_motion.gripper_frozen = 1U;
    g_motion.latch_tick = now;
    g_xy_stop_sent = 0U;
    g_z_fault_count_at_acquire = 0U;
    ++g_motion.stop_count;
}

static void poll_stop(void)
{
    ZAxisControlStatus z;
    if (g_xy_stop_sent == 0U) {
        g_xy_stop_sent = xy_stop_all();
    }
    ZAxis_GetControlStatus(&z);
    if ((z.state == Z_STATE_STARTING) || (z.state == Z_STATE_MOVING)) {
        (void)ZAxisControl_Stop();
    }
    if ((g_xy_stop_sent != 0U) && (axes_stopped() != 0U)) {
        g_motion.stop_pending = 0U;
    }
}

void MotionCoordinator_Init(uint32_t now)
{
    memset(&g_motion, 0, sizeof(g_motion));
    g_abort_request = 0U;
    g_motion.owner = MOTION_OWNER_NONE;
    g_motion.owner_since_tick = now;
    g_motion.required_mask = g_idle_required_mask;
    g_xy_stop_sent = 0U;
    (void)C552_SetRequiredMask(g_idle_required_mask);
}

void MotionCoordinator_PollEmergency(uint32_t now)
{
    if ((g_abort_request != 0U) &&
        (g_motion.latch_reason != MOTION_LATCH_ABORT)) {
        g_motion.latch_reason = MOTION_LATCH_ABORT;
        begin_stop(now);
    }
    if ((g_abort_request != 0U) &&
        (g_motion.stop_pending != 0U)) poll_stop();
}

void MotionCoordinator_Poll(uint32_t now)
{
    MotionOwner owner = g_motion.owner;
    if (g_abort_request != 0U) {
        g_abort_request = 0U;
        g_motion.latch_reason = MOTION_LATCH_ABORT;
        ++g_motion.abort_count;
        abort_all_tasks();
        g_motion.owner = MOTION_OWNER_NONE;
        g_motion.owner_since_tick = now;
        g_motion.required_mask = g_idle_required_mask;
        (void)C552_SetRequiredMask(g_idle_required_mask);
        if (g_motion.stop_pending == 0U) begin_stop(now);
    } else if ((g_motion.latch_reason == MOTION_LATCH_NONE) &&
               (owner != MOTION_OWNER_NONE) &&
               (monitored_axis_faulted() != 0U)) {
        g_motion.latch_reason = MOTION_LATCH_AXIS_FAULT;
        ++g_motion.axis_fault_count;
        abort_all_tasks();
        g_motion.owner = MOTION_OWNER_NONE;
        g_motion.owner_since_tick = now;
        g_motion.required_mask = g_idle_required_mask;
        (void)C552_SetRequiredMask(g_idle_required_mask);
        begin_stop(now);
    }

    if (g_motion.stop_pending != 0U) poll_stop();

    owner = g_motion.owner;
    if ((owner_is_automatic(owner) != 0U) &&
        (owner_task_active(owner) == 0U)) {
        MotionCoordinator_Release(owner, now);
    } else if ((owner == MOTION_OWNER_MANUAL) &&
               (g_motion.manual_hold == 0U) &&
               (g_motion.stop_pending == 0U) &&
               (axes_stopped() != 0U) &&
               (C552_CommandIsActive() == 0U)) {
        MotionCoordinator_Release(owner, now);
    }
}

uint8_t MotionCoordinator_Acquire(MotionOwner owner, uint8_t required_mask,
                                  uint32_t now)
{
    uint32_t primask;
    uint8_t acquired = 0U;
    if ((owner == MOTION_OWNER_NONE) ||
        ((required_mask & (uint8_t)~C552_DEVICE_ALL) != 0U)) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    if ((g_abort_request == 0U) &&
        (g_motion.latch_reason == MOTION_LATCH_NONE) &&
        ((g_motion.owner == MOTION_OWNER_NONE) ||
         (g_motion.owner == owner))) {
        if (g_motion.owner == MOTION_OWNER_NONE) {
            ZAxisControlStatus z;
            g_motion.owner = owner;
            g_motion.owner_since_tick = now;
            ZAxis_GetControlStatus(&z);
            g_z_fault_count_at_acquire = z.fault_count;
        }
        g_motion.required_mask = required_mask;
        acquired = 1U;
    }
    if (primask == 0U) __enable_irq();
    if (acquired != 0U) (void)C552_SetRequiredMask(required_mask);
    return acquired;
}

void MotionCoordinator_Release(MotionOwner owner, uint32_t now)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t released = 0U;
    __disable_irq();
    if (g_motion.owner == owner) {
        g_motion.owner = MOTION_OWNER_NONE;
        g_motion.owner_since_tick = now;
        g_motion.required_mask = g_idle_required_mask;
        released = 1U;
    }
    if (primask == 0U) __enable_irq();
    if (released != 0U) (void)C552_SetRequiredMask(g_idle_required_mask);
}

uint8_t MotionCoordinator_SetRequiredMask(MotionOwner owner,
                                          uint8_t required_mask)
{
    if ((g_motion.owner != owner) ||
        ((required_mask & (uint8_t)~C552_DEVICE_ALL) != 0U)) return 0U;
    g_motion.required_mask = required_mask;
    return C552_SetRequiredMask(required_mask);
}

uint8_t MotionCoordinator_SetIdleRequiredMask(uint8_t required_mask)
{
    if ((g_motion.owner != MOTION_OWNER_NONE) ||
        ((required_mask & (uint8_t)~C552_DEVICE_ALL) != 0U)) return 0U;
    g_idle_required_mask = required_mask;
    g_motion.required_mask = required_mask;
    return C552_SetRequiredMask(required_mask);
}

void MotionCoordinator_SetManualHold(uint8_t hold)
{
    if (g_motion.owner == MOTION_OWNER_MANUAL) {
        g_motion.manual_hold = (hold != 0U) ? 1U : 0U;
    }
}

void MotionCoordinator_RequestAbort(void)
{
    g_abort_request = 1U;
}

uint8_t MotionCoordinator_Resume(uint32_t now)
{
    if ((g_abort_request != 0U) || (g_motion.stop_pending != 0U) ||
        (axes_faulted() != 0U) ||
        (axes_stopped() == 0U)) return 0U;
    g_motion.latch_reason = MOTION_LATCH_NONE;
    g_motion.gripper_frozen = 0U;
    g_motion.manual_hold = 0U;
    g_motion.latch_tick = now;
    return 1U;
}

uint8_t MotionCoordinator_CaptureSnapshot(MotionPositionSnapshot *snapshot,
                                          uint8_t require_idle,
                                          uint32_t now)
{
    XY_AxisStatus x;
    XY_AxisStatus y;
    ZAxisControlStatus z;
    if ((snapshot == NULL) || (XY_GetStatus(XY_AXIS_X, &x) == 0U) ||
        (XY_GetStatus(XY_AXIS_Y, &y) == 0U)) return 0U;
    ZAxis_GetControlStatus(&z);
    if ((x.position_valid == 0U) || (y.position_valid == 0U) ||
        (z.position_valid == 0U) || (x.state == XY_STATE_FAULT) ||
        (y.state == XY_STATE_FAULT) || (z.state == Z_STATE_FAULT)) return 0U;
    if ((require_idle != 0U) && ((x.state != XY_STATE_IDLE) ||
        (y.state != XY_STATE_IDLE) || (z.state != Z_STATE_IDLE))) return 0U;
    snapshot->x_pulses = x.position_pulses;
    snapshot->y_pulses = y.position_pulses;
    snapshot->z_pulses = z.position_pulses;
    snapshot->capture_tick = now;
    return 1U;
}

void MotionCoordinator_GetStatus(MotionCoordinatorStatus *status)
{
    if (status != NULL) {
        *status = g_motion;
        status->abort_pending = g_abort_request;
    }
}

MotionOwner MotionCoordinator_GetOwner(void)
{
    return g_motion.owner;
}

uint8_t MotionCoordinator_IsGripperFrozen(void)
{
    return g_motion.gripper_frozen;
}

const char *MotionCoordinator_OwnerString(MotionOwner owner)
{
    static const char *const names[] = {
        "NONE", "MANUAL", "P2_CALIBRATION", "P3_CALIBRATION",
        "XY_ALIGN", "P4_XZ_ALIGN", "MISSION"
    };
    return ((uint32_t)owner < (sizeof(names) / sizeof(names[0]))) ?
           names[owner] : "UNKNOWN";
}

const char *MotionCoordinator_LatchString(MotionLatchReason reason)
{
    static const char *const names[] = {"NONE", "ABORT", "AXIS_FAULT"};
    return ((uint32_t)reason < (sizeof(names) / sizeof(names[0]))) ?
           names[reason] : "UNKNOWN";
}

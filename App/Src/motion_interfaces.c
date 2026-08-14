#include "motion_interfaces.h"
#include "motion_coordinator.h"
#include "c552.h"
#include "main.h"
#include "z_axis_link.h"
#include "z_axis.h"

XY_VisionCalibration g_xy_vision_calibration = {
    .k230_id = 0U,
    .axes = {XY_AXIS_X, XY_AXIS_Y},
    .reference_pixel = {0, 0},
    .pixel_per_pulse = {{0.0f, 0.0f}, {0.0f, 0.0f}},
    .pulse_per_pixel = {{0.0f, 0.0f}, {0.0f, 0.0f}},
    .calibrated = 0U
};

MotionAuxResult ZAxis_MoveRelative(int32_t pulses, uint32_t speed_hz)
{
    ZAxisControlResult result = ZAxisControl_MoveRelative(pulses, speed_hz);
    if (result == Z_RESULT_OK) return MOTION_AUX_OK;
    if (result == Z_RESULT_BUSY) return MOTION_AUX_BUSY;
    if ((result == Z_RESULT_INVALID_PULSES) ||
        (result == Z_RESULT_INVALID_SPEED) ||
        (result == Z_RESULT_SOFT_LIMIT) ||
        (result == Z_RESULT_NOT_REFERENCED)) {
        return MOTION_AUX_INVALID_ARGUMENT;
    }
    return MOTION_AUX_NOT_AVAILABLE;
}

MotionAuxResult ZAxis_MoveAbsolute(int32_t target_pulses, uint32_t speed_hz)
{
    ZAxisControlResult result =
        ZAxisControl_MoveAbsolute(target_pulses, speed_hz);
    if (result == Z_RESULT_OK) return MOTION_AUX_OK;
    if (result == Z_RESULT_BUSY) return MOTION_AUX_BUSY;
    if ((result == Z_RESULT_INVALID_PULSES) ||
        (result == Z_RESULT_INVALID_SPEED) ||
        (result == Z_RESULT_SOFT_LIMIT) ||
        (result == Z_RESULT_NOT_REFERENCED)) {
        return MOTION_AUX_INVALID_ARGUMENT;
    }
    return MOTION_AUX_NOT_AVAILABLE;
}

MotionAuxResult ZAxis_Stop(void)
{
    ZAxisControlResult result = ZAxisControl_Stop();
    if (result == Z_RESULT_OK) return MOTION_AUX_OK;
    if (result == Z_RESULT_BUSY) return MOTION_AUX_BUSY;
    return MOTION_AUX_NOT_AVAILABLE;
}

MotionAuxResult ZAxis_SetZero(void)
{
    ZAxisControlResult result = ZAxisControl_SetZero();
    if (result == Z_RESULT_OK) return MOTION_AUX_OK;
    if (result == Z_RESULT_BUSY) return MOTION_AUX_BUSY;
    return MOTION_AUX_NOT_AVAILABLE;
}

MotionAuxResult ZAxis_ClearFault(void)
{
    ZAxisControlResult result = ZAxisControl_ClearFault();
    if (result == Z_RESULT_OK) return MOTION_AUX_OK;
    if (result == Z_RESULT_BUSY) return MOTION_AUX_BUSY;
    return MOTION_AUX_NOT_AVAILABLE;
}

MotionAuxResult Gripper_SetPosition(GripperPosition position)
{
    C552_RequestResult result;
    if ((position != GRIPPER_OPEN) && (position != GRIPPER_CLOSED)) {
        return MOTION_AUX_INVALID_ARGUMENT;
    }
    if (MotionCoordinator_IsGripperFrozen() != 0U) {
        return MOTION_AUX_NOT_AVAILABLE;
    }
    result = C552_SetGripper(C552_GRIPPER_BOTH,
                             (position == GRIPPER_OPEN) ?
                                 C552_GRIPPER_OPEN : C552_GRIPPER_CLOSED,
                             HAL_GetTick());
    if (result == C552_REQUEST_OK) return MOTION_AUX_OK;
    if (result == C552_REQUEST_BUSY) return MOTION_AUX_BUSY;
    return MOTION_AUX_INVALID_ARGUMENT;
}

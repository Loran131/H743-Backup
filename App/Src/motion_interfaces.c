#include "motion_interfaces.h"
#include "c552.h"
#include "main.h"

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
    (void)pulses;
    (void)speed_hz;
    return MOTION_AUX_NOT_AVAILABLE;
}

MotionAuxResult ZAxis_Stop(void)
{
    return MOTION_AUX_NOT_AVAILABLE;
}

MotionAuxResult Gripper_SetPosition(GripperPosition position)
{
    C552_RequestResult result;
    if ((position != GRIPPER_OPEN) && (position != GRIPPER_CLOSED)) {
        return MOTION_AUX_INVALID_ARGUMENT;
    }
    result = C552_SetGripper(C552_GRIPPER_BOTH,
                             (position == GRIPPER_OPEN) ?
                                 C552_GRIPPER_OPEN : C552_GRIPPER_CLOSED,
                             HAL_GetTick());
    if (result == C552_REQUEST_OK) return MOTION_AUX_OK;
    if (result == C552_REQUEST_BUSY) return MOTION_AUX_BUSY;
    return MOTION_AUX_INVALID_ARGUMENT;
}

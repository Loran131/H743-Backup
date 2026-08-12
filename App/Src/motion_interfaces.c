#include "motion_interfaces.h"

XY_VisionCalibration g_xy_vision_calibration = {
    .center_x_target = 0,
    .center_y_target = 0,
    .center_x_axis = XY_AXIS_X,
    .center_y_axis = XY_AXIS_Y,
    .center_x_error_to_positive_axis = 0,
    .center_y_error_to_positive_axis = 0,
    .center_x_pixels_per_mm = 0.0f,
    .center_y_pixels_per_mm = 0.0f,
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
    if ((position != GRIPPER_OPEN) && (position != GRIPPER_CLOSED)) {
        return MOTION_AUX_INVALID_ARGUMENT;
    }
    return MOTION_AUX_NOT_AVAILABLE;
}

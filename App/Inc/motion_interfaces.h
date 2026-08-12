#ifndef MOTION_INTERFACES_H
#define MOTION_INTERFACES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "xy_motor.h"
#include <stdint.h>

typedef struct {
    int16_t center_x_target;
    int16_t center_y_target;
    XY_Axis center_x_axis;
    XY_Axis center_y_axis;
    int8_t center_x_error_to_positive_axis;
    int8_t center_y_error_to_positive_axis;
    float center_x_pixels_per_mm;
    float center_y_pixels_per_mm;
    uint8_t calibrated;
} XY_VisionCalibration;

/* Runtime-modifiable calibration. Defaults are deliberately uncalibrated. */
extern XY_VisionCalibration g_xy_vision_calibration;

typedef enum {
    MOTION_AUX_OK = 0,
    MOTION_AUX_NOT_AVAILABLE,
    MOTION_AUX_INVALID_ARGUMENT
} MotionAuxResult;

typedef enum {
    GRIPPER_OPEN = 0,
    GRIPPER_CLOSED
} GripperPosition;

MotionAuxResult ZAxis_MoveRelative(int32_t pulses, uint32_t speed_hz);
MotionAuxResult ZAxis_Stop(void);
MotionAuxResult Gripper_SetPosition(GripperPosition position);

#ifdef __cplusplus
}
#endif

#endif

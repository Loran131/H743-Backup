#ifndef MOTION_INTERFACES_H
#define MOTION_INTERFACES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "xy_motor.h"
#include <stdint.h>

typedef struct {
    uint8_t k230_id;
    XY_Axis axes[2];
    int16_t reference_pixel[2];
    float pixel_per_pulse[2][2];
    float pulse_per_pixel[2][2];
    uint8_t calibrated;
} XY_VisionCalibration;

/* Runtime-modifiable calibration. Defaults are deliberately uncalibrated. */
extern XY_VisionCalibration g_xy_vision_calibration;

typedef enum {
    MOTION_AUX_OK = 0,
    MOTION_AUX_NOT_AVAILABLE,
    MOTION_AUX_INVALID_ARGUMENT,
    MOTION_AUX_BUSY
} MotionAuxResult;

typedef enum {
    GRIPPER_OPEN = 0,
    GRIPPER_CLOSED
} GripperPosition;

MotionAuxResult ZAxis_MoveRelative(int32_t pulses, uint32_t speed_hz);
MotionAuxResult ZAxis_MoveAbsolute(int32_t target_pulses, uint32_t speed_hz);
MotionAuxResult ZAxis_Stop(void);
MotionAuxResult ZAxis_SetZero(void);
MotionAuxResult ZAxis_ClearFault(void);
MotionAuxResult Gripper_SetPosition(GripperPosition position);

#ifdef __cplusplus
}
#endif

#endif

#ifndef XZ_VISION_ALIGN_H
#define XZ_VISION_ALIGN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "xy_motor.h"
#include "z_axis.h"
#include <stdint.h>

#define XZ_VISION_ALIGN_PERIOD_MS 470U

typedef enum {
    XZ_VISION_ALIGN_IDLE = 0,
    XZ_VISION_ALIGN_WAIT_RED_MODE,
    XZ_VISION_ALIGN_WAIT_RED_SAMPLE,
    XZ_VISION_ALIGN_CALCULATE_ERROR,
    XZ_VISION_ALIGN_MOVE_X,
    XZ_VISION_ALIGN_WAIT_X_IDLE,
    XZ_VISION_ALIGN_MOVE_Z,
    XZ_VISION_ALIGN_WAIT_Z_IDLE,
    XZ_VISION_ALIGN_WAIT_NEW_SAMPLE,
    XZ_VISION_ALIGN_COMPLETE,
    XZ_VISION_ALIGN_FAULT
} XZVisionAlignState;

typedef enum {
    XZ_VISION_ALIGN_FAULT_NONE = 0,
    XZ_VISION_ALIGN_FAULT_BUSY,
    XZ_VISION_ALIGN_FAULT_NOT_CALIBRATED,
    XZ_VISION_ALIGN_FAULT_AXIS_NOT_READY,
    XZ_VISION_ALIGN_FAULT_MODE,
    XZ_VISION_ALIGN_FAULT_VISION_TIMEOUT,
    XZ_VISION_ALIGN_FAULT_CONTROL_TIMEOUT,
    XZ_VISION_ALIGN_FAULT_MOVE_X,
    XZ_VISION_ALIGN_FAULT_MOVE_Z,
    XZ_VISION_ALIGN_FAULT_SOFT_LIMIT,
    XZ_VISION_ALIGN_FAULT_AXIS
} XZVisionAlignFault;

typedef struct {
    XZVisionAlignState state;
    XZVisionAlignFault fault;
    uint16_t last_sample_seq;
    uint16_t decision_sample_seq;
    int16_t pixel[2];
    int16_t decision_pixel[2];
    int32_t error_pixel[2];
    int32_t raw_pulses[2];
    int32_t requested_pulses[2];
    int32_t axis_position[2];
    int32_t attempted_target[2];
    XY_Result last_x_result;
    ZAxisControlResult last_z_result;
    uint8_t stable_samples;
    uint16_t vector_scale_permille;
    uint32_t start_tick;
    uint32_t last_sample_tick;
    uint32_t corrections;
} XZVisionAlignStatus;

void XZVisionAlign_Init(uint32_t now);
void XZVisionAlign_Poll(uint32_t now);
uint8_t XZVisionAlign_Start(uint32_t now);
void XZVisionAlign_Abort(void);
void XZVisionAlign_GetStatus(XZVisionAlignStatus *status);
uint8_t XZVisionAlign_IsActive(void);
const char *XZVisionAlign_StateString(XZVisionAlignState state);
const char *XZVisionAlign_FaultString(XZVisionAlignFault fault);

#ifdef __cplusplus
}
#endif

#endif

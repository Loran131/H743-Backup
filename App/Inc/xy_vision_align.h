#ifndef XY_VISION_ALIGN_H
#define XY_VISION_ALIGN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "xy_motor.h"
#include "motion_coordinator.h"
#include <stdint.h>

#define XY_VISION_ALIGN_PERIOD_MS 470U

typedef enum {
    XY_VISION_ALIGN_IDLE = 0,
    XY_VISION_ALIGN_WAIT_SAMPLE,
    XY_VISION_ALIGN_MOVE_X,
    XY_VISION_ALIGN_MOVE_Y,
    XY_VISION_ALIGN_COMPLETE,
    XY_VISION_ALIGN_FAULT
} XY_VisionAlignState;

typedef enum {
    XY_VISION_ALIGN_FAULT_NONE = 0,
    XY_VISION_ALIGN_FAULT_NOT_CALIBRATED,
    XY_VISION_ALIGN_FAULT_AXIS_NOT_READY,
    XY_VISION_ALIGN_FAULT_VISION_TIMEOUT,
    XY_VISION_ALIGN_FAULT_CONTROL_TIMEOUT,
    XY_VISION_ALIGN_FAULT_MOVE,
    XY_VISION_ALIGN_FAULT_AXIS
} XY_VisionAlignFault;

typedef struct {
    XY_VisionAlignState state;
    XY_VisionAlignFault fault;
    uint16_t last_sample_seq;
    int16_t pixel[2];
    int16_t error_pixel[2];
    int32_t requested_pulses[2];
    int32_t axis_position[2];
    int32_t attempted_target[2];
    XY_Result last_move_result;
    XY_Axis failed_axis;
    uint8_t stable_samples;
    uint32_t start_tick;
    uint32_t last_sample_tick;
    uint32_t corrections;
    MotionOwner owner;
    int16_t target_pixel[2];
} XY_VisionAlignStatus;

void XY_VisionAlign_Init(uint32_t now);
void XY_VisionAlign_Poll(uint32_t now);
uint8_t XY_VisionAlign_Start(uint32_t now);
uint8_t XY_VisionAlign_StartOwned(MotionOwner owner, int16_t target_x,
                                  int16_t target_y, uint32_t now);
void XY_VisionAlign_Abort(void);
void XY_VisionAlign_GetStatus(XY_VisionAlignStatus *status);
uint8_t XY_VisionAlign_IsActive(void);
const char *XY_VisionAlign_StateString(XY_VisionAlignState state);
const char *XY_VisionAlign_FaultString(XY_VisionAlignFault fault);

#ifdef __cplusplus
}
#endif

#endif

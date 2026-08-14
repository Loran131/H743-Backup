#ifndef MOTION_COORDINATOR_H
#define MOTION_COORDINATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    MOTION_OWNER_NONE = 0,
    MOTION_OWNER_MANUAL,
    MOTION_OWNER_P2_CALIBRATION,
    MOTION_OWNER_P3_CALIBRATION,
    MOTION_OWNER_XY_ALIGN,
    MOTION_OWNER_P4_XZ_ALIGN,
    MOTION_OWNER_MISSION
} MotionOwner;

typedef enum {
    MOTION_LATCH_NONE = 0,
    MOTION_LATCH_ABORT,
    MOTION_LATCH_AXIS_FAULT
} MotionLatchReason;

typedef struct {
    int32_t x_pulses;
    int32_t y_pulses;
    int32_t z_pulses;
    uint32_t capture_tick;
} MotionPositionSnapshot;

typedef struct {
    MotionOwner owner;
    MotionLatchReason latch_reason;
    uint8_t required_mask;
    uint8_t abort_pending;
    uint8_t stop_pending;
    uint8_t gripper_frozen;
    uint8_t manual_hold;
    uint32_t owner_since_tick;
    uint32_t latch_tick;
    uint32_t abort_count;
    uint32_t axis_fault_count;
    uint32_t stop_count;
} MotionCoordinatorStatus;

void MotionCoordinator_Init(uint32_t now);
void MotionCoordinator_PollEmergency(uint32_t now);
void MotionCoordinator_Poll(uint32_t now);
uint8_t MotionCoordinator_Acquire(MotionOwner owner, uint8_t required_mask,
                                  uint32_t now);
void MotionCoordinator_Release(MotionOwner owner, uint32_t now);
uint8_t MotionCoordinator_SetRequiredMask(MotionOwner owner,
                                          uint8_t required_mask);
uint8_t MotionCoordinator_SetIdleRequiredMask(uint8_t required_mask);
void MotionCoordinator_SetManualHold(uint8_t hold);
void MotionCoordinator_RequestAbort(void);
uint8_t MotionCoordinator_Resume(uint32_t now);
uint8_t MotionCoordinator_CaptureSnapshot(MotionPositionSnapshot *snapshot,
                                          uint8_t require_idle,
                                          uint32_t now);
void MotionCoordinator_GetStatus(MotionCoordinatorStatus *status);
MotionOwner MotionCoordinator_GetOwner(void);
uint8_t MotionCoordinator_IsGripperFrozen(void);
const char *MotionCoordinator_OwnerString(MotionOwner owner);
const char *MotionCoordinator_LatchString(MotionLatchReason reason);

#ifdef __cplusplus
}
#endif

#endif

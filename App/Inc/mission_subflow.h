#ifndef MISSION_SUBFLOW_H
#define MISSION_SUBFLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motion_coordinator.h"
#include <stdint.h>

typedef enum {
    MISSION_TASK_RED_PICK = 0,
    MISSION_TASK_TAG_PUT,
    MISSION_TASK_RED_FIND,
    MISSION_TASK_FRAME_PUT,
    MISSION_TASK_COUNT
} MissionTaskName;

typedef enum {
    MISSION_SUBFLOW_NONE = 0,
    MISSION_SUBFLOW_OBSERVE_RED_FRONT,
    MISSION_SUBFLOW_OBSERVE_TAG_FRONT,
    MISSION_SUBFLOW_OBSERVE_FRAME_DOWN,
    MISSION_SUBFLOW_ALIGN_XZ,
    MISSION_SUBFLOW_ALIGN_XY,
    MISSION_SUBFLOW_BLIND_MOVE_Y,
    MISSION_SUBFLOW_TOF3_DESCEND,
    MISSION_SUBFLOW_GRIP_OPEN,
    MISSION_SUBFLOW_GRIP_CLOSE,
    MISSION_SUBFLOW_RECORD_POSE,
    MISSION_SUBFLOW_RETURN_POSE,
    MISSION_SUBFLOW_SAFE_RETREAT,
    MISSION_SUBFLOW_PRESET_POSE,
    MISSION_SUBFLOW_ABORT_ALL
} MissionSubflowType;

typedef enum {
    MISSION_SUBFLOW_IDLE = 0,
    MISSION_SUBFLOW_STARTING,
    MISSION_SUBFLOW_WAIT_APPLIED,
    MISSION_SUBFLOW_WAIT_SAMPLE,
    MISSION_SUBFLOW_RUNNING,
    MISSION_SUBFLOW_WAIT_IDLE,
    MISSION_SUBFLOW_RETRY_WAIT,
    MISSION_SUBFLOW_COMPLETE,
    MISSION_SUBFLOW_FAULT
} MissionSubflowState;

typedef enum {
    MISSION_FAIL_SOURCE_NONE = 0,
    MISSION_FAIL_SOURCE_AXIS_X,
    MISSION_FAIL_SOURCE_AXIS_Y,
    MISSION_FAIL_SOURCE_AXIS_Z,
    MISSION_FAIL_SOURCE_K230_MODE,
    MISSION_FAIL_SOURCE_K230_DATA,
    MISSION_FAIL_SOURCE_C552,
    MISSION_FAIL_SOURCE_TOF,
    MISSION_FAIL_SOURCE_GRIPPER,
    MISSION_FAIL_SOURCE_POSE,
    MISSION_FAIL_SOURCE_COORDINATOR
} MissionFailureSource;

typedef enum {
    MISSION_FAIL_NONE = 0,
    MISSION_FAIL_BUSY,
    MISSION_FAIL_INVALID_ARGUMENT,
    MISSION_FAIL_LINK,
    MISSION_FAIL_TIMEOUT,
    MISSION_FAIL_REJECTED,
    MISSION_FAIL_EMPTY_SAMPLE,
    MISSION_FAIL_AXIS_FAULT,
    MISSION_FAIL_POSITION_INVALID,
    MISSION_FAIL_SOFT_LIMIT,
    MISSION_FAIL_RETRY_EXHAUSTED,
    MISSION_FAIL_CANCELLED
} MissionFailureReason;

typedef struct {
    MissionFailureSource source;
    MissionFailureReason reason;
    int32_t detail;
} MissionFailure;

typedef struct {
    MissionTaskName task;
    MissionSubflowType type;
    MissionSubflowState state;
    MissionFailure failure;
    uint8_t attempt;
    uint8_t max_attempts;
    uint8_t axis_recoveries;
    uint8_t max_axis_recoveries;
    uint8_t stable_samples;
    uint16_t last_sample_seq;
    uint16_t distance_mm;
    int16_t target_pixel[2];
    int32_t requested_pulses;
    MotionPositionSnapshot pose;
    uint32_t start_tick;
    uint32_t state_tick;
} MissionSubflowStatus;

void MissionSubflow_Init(uint32_t now);
void MissionSubflow_Poll(uint32_t now);
uint8_t MissionSubflow_SetOwnerRetention(uint8_t retain);
uint8_t MissionSubflow_StartObserveRedFront(MissionTaskName task,
                                            uint32_t now);
uint8_t MissionSubflow_StartObserveTagFront(MissionTaskName task,
                                            uint32_t now);
uint8_t MissionSubflow_StartObserveFrameDown(MissionTaskName task,
                                             uint32_t now);
uint8_t MissionSubflow_StartAlignXZ(MissionTaskName task, int16_t target_x,
                                    int16_t target_y, uint32_t now);
uint8_t MissionSubflow_StartAlignXY(MissionTaskName task, int16_t target_x,
                                    int16_t target_y, uint32_t now);
uint8_t MissionSubflow_StartBlindMoveY(MissionTaskName task, uint8_t tof_id,
                                       uint16_t stop_mm,
                                       uint32_t pulses_per_mm,
                                       int8_t direction, uint32_t now);
uint8_t MissionSubflow_StartTof3Descend(MissionTaskName task,
                                        uint16_t stop_mm,
                                        uint32_t max_descent_pulses,
                                        int8_t direction, uint32_t now);
uint8_t MissionSubflow_StartGrip(MissionTaskName task, uint8_t close,
                                 uint32_t now);
uint8_t MissionSubflow_StartRecordPose(MissionTaskName task, uint32_t now);
uint8_t MissionSubflow_StartReturnPose(MissionTaskName task, uint32_t now);
uint8_t MissionSubflow_StartPresetPose(MissionTaskName task,
                                       const MotionPositionSnapshot *pose,
                                       uint32_t now);
uint8_t MissionSubflow_SetSafePose(const MotionPositionSnapshot *pose);
uint8_t MissionSubflow_HasSafePose(void);
uint8_t MissionSubflow_GetSafePose(MotionPositionSnapshot *pose);
uint8_t MissionSubflow_StartSafeRetreat(MissionTaskName task, uint32_t now);
void MissionSubflow_Cancel(uint32_t now);
void MissionSubflow_AbortAll(MissionFailure failure, uint32_t now);
void MissionSubflow_GetStatus(MissionSubflowStatus *status);
uint8_t MissionSubflow_IsActive(void);
const char *MissionSubflow_TaskString(MissionTaskName task);
const char *MissionSubflow_TypeString(MissionSubflowType type);
const char *MissionSubflow_StateString(MissionSubflowState state);
const char *MissionSubflow_SourceString(MissionFailureSource source);
const char *MissionSubflow_ReasonString(MissionFailureReason reason);

#ifdef __cplusplus
}
#endif

#endif

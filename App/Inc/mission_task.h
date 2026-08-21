#ifndef MISSION_TASK_H
#define MISSION_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mission_subflow.h"
#include <stdint.h>

typedef enum {
    MISSION_PAYLOAD_EMPTY = 0,
    MISSION_PAYLOAD_HELD
} MissionPayloadState;

typedef enum {
    MISSION_STORAGE_NOT_LOADED = 0,
    MISSION_STORAGE_EMPTY,
    MISSION_STORAGE_VALID,
    MISSION_STORAGE_INVALID,
    MISSION_STORAGE_IO_ERROR
} MissionStorageState;

typedef struct {
    int32_t x_pulses;
    int32_t y_pulses;
    int32_t z_pulses;
} MissionTaskPose;

typedef enum {
    MISSION_STATE_IDLE = 0,
    MISSION_STATE_PRECHECK,
    MISSION_STATE_PRECHECK_HELD,
    MISSION_STATE_PREPARE_GRIP_OPEN,
    MISSION_STATE_PRESET_POSE,
    MISSION_STATE_RED_OBSERVE,
    MISSION_STATE_TAG_OBSERVE,
    MISSION_STATE_FRAME_OBSERVE,
    MISSION_STATE_FRONT_RED_READY,
    MISSION_STATE_FRONT_TAG_READY,
    MISSION_STATE_DOWN_FRAME_READY,
    MISSION_STATE_ALIGN_XZ,
    MISSION_STATE_ALIGN_XY,
    MISSION_STATE_STABLE,
    MISSION_STATE_RECORD_XYZ,
    MISSION_STATE_BLIND_Y,
    MISSION_STATE_OPTIONAL_Z_DROP,
    MISSION_STATE_TOF3_Z_DESCEND,
    MISSION_STATE_GRIP_CLOSE,
    MISSION_STATE_GRIP_OPEN,
    MISSION_STATE_RETURN_RECORDED_POSE,
    MISSION_STATE_VERIFY,
    MISSION_STATE_SAFE_RETREAT,
    MISSION_STATE_RECOVERY_WAIT_IDLE,
    MISSION_STATE_RECOVER_PRESET,
    MISSION_STATE_ABORTING,
    MISSION_STATE_COMPLETE,
    MISSION_STATE_FAULT
} MissionTaskState;

typedef struct {
    int16_t target_x;
    int16_t target_y;
    uint16_t blind_stop_mm;
    uint32_t blind_pulses_per_mm;
    int8_t blind_direction;
    uint16_t z_stop_mm;
    uint32_t z_max_pulses;
    int8_t z_direction;
    uint8_t align_configured;
    uint8_t blind_configured;
    uint8_t z_enabled;
    uint8_t z_configured;
    MissionTaskPose preset_pose;
    MissionTaskPose safe_pose;
} MissionTaskConfig;

typedef struct {
    MissionTaskName task;
    MissionTaskState state;
    MissionFailure failure;
    MissionPayloadState payload;
    MissionTaskConfig config;
    MissionSubflowType subflow_type;
    MissionSubflowState subflow_state;
    uint8_t subflow_attempt;
    uint8_t subflow_max_attempts;
    uint8_t axis_recoveries;
    uint8_t max_axis_recoveries;
    uint8_t restart_count;
    uint8_t max_restarts;
    uint16_t subflow_distance_mm;
    int32_t subflow_requested_pulses;
    uint8_t start_pending;
    uint8_t cancel_pending;
    uint32_t start_tick;
    uint32_t state_tick;
    uint32_t completed_count;
    uint32_t failed_count;
    MissionStorageState storage_state;
    uint32_t storage_generation;
} MissionTaskStatus;

void MissionTask_Init(uint32_t now);
void MissionTask_Poll(uint32_t now);
uint8_t MissionTask_RequestStart(MissionTaskName task);
void MissionTask_RequestCancel(void);
uint8_t MissionTask_SetAlignTarget(MissionTaskName task, int16_t target_x,
                                   int16_t target_y);
uint8_t MissionTask_SetBlindY(MissionTaskName task, uint16_t stop_mm,
                              uint32_t pulses_per_mm, int8_t direction);
uint8_t MissionTask_SetZDrop(MissionTaskName task, uint8_t enabled,
                             uint16_t stop_mm, uint32_t max_pulses,
                             int8_t direction);
uint8_t MissionTask_SetPresetPose(MissionTaskName task, int32_t x_pulses,
                                  int32_t y_pulses, int32_t z_pulses);
uint8_t MissionTask_SetSafePose(MissionTaskName task, int32_t x_pulses,
                                int32_t y_pulses, int32_t z_pulses);
uint8_t MissionTask_SetPayload(MissionPayloadState payload);
uint8_t MissionTask_SaveConfiguration(void);
void MissionTask_GetStatus(MissionTaskStatus *status);
void MissionTask_GetConfig(MissionTaskName task, MissionTaskConfig *config);
uint8_t MissionTask_IsActive(void);
const char *MissionTask_StateString(MissionTaskState state);
const char *MissionTask_PayloadString(MissionPayloadState payload);
const char *MissionTask_StorageString(MissionStorageState state);

#ifdef __cplusplus
}
#endif

#endif

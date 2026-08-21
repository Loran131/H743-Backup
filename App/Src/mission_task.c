#include "mission_task.h"

#include "c552.h"
#include "eeprom.h"
#include "motion_coordinator.h"
#include "xy_motor.h"
#include "z_axis.h"
#include <limits.h>
#include <string.h>

#define MISSION_MARKER_HOLD_MS       20U
#define MISSION_TOTAL_TIMEOUT_MS     600000U
#define MISSION_OBSERVE_TIMEOUT_MS   30000U
#define MISSION_ALIGN_TIMEOUT_MS     260000U
#define MISSION_BLIND_TIMEOUT_MS     260000U
#define MISSION_Z_TIMEOUT_MS         260000U
#define MISSION_GRIP_TIMEOUT_MS      15000U
#define MISSION_POSE_TIMEOUT_MS      240000U
#define MISSION_RECOVERY_IDLE_MS     5000U
#define MISSION_MAX_RESTARTS         10U
#define MISSION_DEFAULT_PRESET_X     1012000L
#define MISSION_DEFAULT_PRESET_Y     0L
#define MISSION_DEFAULT_PRESET_Z     187600L
#define MISSION_DEFAULT_SAFE_X       1012000L
#define MISSION_DEFAULT_SAFE_Y       0L
#define MISSION_DEFAULT_SAFE_Z       0L
#define MISSION_STORAGE_ADDRESS      128U
#define MISSION_STORAGE_MAGIC        0x37474643UL
#define MISSION_STORAGE_VERSION      2U
#define MISSION_STORAGE_LENGTH       128U
#define MISSION_STORAGE_POSE_OFFSET  7U
#define MISSION_STORAGE_POSE_SIZE    18U
#define MISSION_STORAGE_CONFIG_OFFSET 79U
#define MISSION_STORAGE_CONFIG_BYTES 45U
#define MISSION_STORAGE_CRC_OFFSET   124U

#define MISSION_STORAGE_V1_VERSION   1U
#define MISSION_STORAGE_V1_LENGTH    112U
#define MISSION_STORAGE_V1_CONFIG_OFFSET 28U
#define MISSION_STORAGE_V1_CONFIG_SIZE 20U
#define MISSION_STORAGE_V1_CRC_OFFSET 108U
#define MISSION_UINT24_MAX           0xFFFFFFUL

#define MISSION_CONFIG_ALIGN         0x01U
#define MISSION_CONFIG_BLIND         0x02U
#define MISSION_CONFIG_Z_ENABLED     0x04U
#define MISSION_CONFIG_Z_CONFIGURED  0x08U

#if (MISSION_STORAGE_ADDRESS + MISSION_STORAGE_LENGTH) > EEPROM_SIZE
#error "P7 mission storage exceeds EEPROM capacity"
#endif
#if (MISSION_STORAGE_CONFIG_OFFSET + MISSION_STORAGE_CONFIG_BYTES) != \
    MISSION_STORAGE_CRC_OFFSET
#error "P7 mission config storage layout is inconsistent"
#endif

_Static_assert((MISSION_STORAGE_POSE_OFFSET +
                MISSION_TASK_COUNT * MISSION_STORAGE_POSE_SIZE) ==
               MISSION_STORAGE_CONFIG_OFFSET,
               "P7 mission pose storage layout is inconsistent");

typedef struct {
    MissionTaskStatus status;
    MissionTaskConfig configs[MISSION_TASK_COUNT];
    uint8_t subflow_started;
    uint8_t cancel_issued;
} MissionTaskRuntime;

static MissionTaskRuntime g_task;
static volatile uint8_t g_start_request;
static volatile uint8_t g_requested_task;
static volatile uint8_t g_cancel_request;

static void storage_write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void storage_write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static uint16_t storage_read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t storage_read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static void storage_write_u24(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
}

static uint32_t storage_read_u24(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U);
}

static uint32_t storage_crc32(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint16_t i = 0U; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

static void set_default_poses(MissionTaskConfig *config)
{
    config->preset_pose.x_pulses = MISSION_DEFAULT_PRESET_X;
    config->preset_pose.y_pulses = MISSION_DEFAULT_PRESET_Y;
    config->preset_pose.z_pulses = MISSION_DEFAULT_PRESET_Z;
    config->safe_pose.x_pulses = MISSION_DEFAULT_SAFE_X;
    config->safe_pose.y_pulses = MISSION_DEFAULT_SAFE_Y;
    config->safe_pose.z_pulses = MISSION_DEFAULT_SAFE_Z;
}

static uint8_t pose_sane(const MissionTaskPose *pose)
{
    const XY_AxisConfig *x = XY_GetConfig(XY_AXIS_X);
    const XY_AxisConfig *y = XY_GetConfig(XY_AXIS_Y);
    return ((pose != NULL) && (x != NULL) && (y != NULL) &&
            (pose->x_pulses >= x->soft_min_pulses) &&
            (pose->x_pulses <= x->soft_max_pulses) &&
            (pose->y_pulses >= y->soft_min_pulses) &&
            (pose->y_pulses <= y->soft_max_pulses) &&
            (pose->z_pulses >= Z_AXIS_SOFT_MIN_PULSES) &&
            (pose->z_pulses <= Z_AXIS_SOFT_MAX_PULSES) &&
            ((uint32_t)pose->x_pulses <= MISSION_UINT24_MAX) &&
            ((uint32_t)pose->y_pulses <= MISSION_UINT24_MAX) &&
            ((uint32_t)pose->z_pulses <= MISSION_UINT24_MAX)) ? 1U : 0U;
}

static uint8_t config_sane(MissionTaskName task,
                           const MissionTaskConfig *config)
{
    if ((config->align_configured > 1U) ||
        (config->blind_configured > 1U) ||
        (config->z_enabled > 1U) || (config->z_configured > 1U)) return 0U;
    if (config->blind_configured != 0U) {
        if ((task == MISSION_TASK_FRAME_PUT) ||
            (config->blind_pulses_per_mm == 0U) ||
            (config->blind_pulses_per_mm > MISSION_UINT24_MAX) ||
            ((config->blind_direction != -1) &&
             (config->blind_direction != 1))) return 0U;
    }
    if ((config->z_enabled != 0U) || (config->z_configured != 0U)) {
        if ((task != MISSION_TASK_TAG_PUT) &&
            (task != MISSION_TASK_FRAME_PUT)) return 0U;
    }
    if (config->z_configured != 0U) {
        if ((config->z_max_pulses == 0U) ||
            (config->z_max_pulses > MISSION_UINT24_MAX) ||
            ((config->z_direction != -1) &&
             (config->z_direction != 1))) return 0U;
    }
    if ((task == MISSION_TASK_FRAME_PUT) &&
        (config->z_configured != 0U) &&
        (config->z_enabled == 0U)) return 0U;
    return ((pose_sane(&config->preset_pose) != 0U) &&
            (pose_sane(&config->safe_pose) != 0U)) ? 1U : 0U;
}

static uint8_t load_configuration(uint32_t now)
{
    uint8_t record[MISSION_STORAGE_LENGTH];
    MissionTaskConfig loaded[MISSION_TASK_COUNT];
    uint8_t version;

    (void)now;

    if (EEPROM_Read(MISSION_STORAGE_ADDRESS, record, sizeof(record)) != HAL_OK) {
        g_task.status.storage_state = MISSION_STORAGE_IO_ERROR;
        return 0U;
    }
    if (storage_read_u32(record) != MISSION_STORAGE_MAGIC) {
        g_task.status.storage_state = MISSION_STORAGE_EMPTY;
        return 0U;
    }

    memset(loaded, 0, sizeof(loaded));
    for (uint8_t task = 0U; task < MISSION_TASK_COUNT; ++task)
        set_default_poses(&loaded[task]);

    version = record[4];
    if (version == MISSION_STORAGE_VERSION) {
        uint16_t offset = MISSION_STORAGE_CONFIG_OFFSET;
        if (storage_read_u32(&record[MISSION_STORAGE_CRC_OFFSET]) !=
            storage_crc32(&record[4], MISSION_STORAGE_CRC_OFFSET - 4U)) {
            g_task.status.storage_state = MISSION_STORAGE_INVALID;
            return 0U;
        }

        for (uint8_t task = 0U; task < MISSION_TASK_COUNT; ++task) {
            const uint8_t *source = &record[MISSION_STORAGE_POSE_OFFSET +
                task * MISSION_STORAGE_POSE_SIZE];
            loaded[task].preset_pose.x_pulses =
                (int32_t)storage_read_u24(&source[0]);
            loaded[task].preset_pose.y_pulses =
                (int32_t)storage_read_u24(&source[3]);
            loaded[task].preset_pose.z_pulses =
                (int32_t)storage_read_u24(&source[6]);
            loaded[task].safe_pose.x_pulses =
                (int32_t)storage_read_u24(&source[9]);
            loaded[task].safe_pose.y_pulses =
                (int32_t)storage_read_u24(&source[12]);
            loaded[task].safe_pose.z_pulses =
                (int32_t)storage_read_u24(&source[15]);
        }

        for (uint8_t task = 0U; task < MISSION_TASK_COUNT; ++task) {
            const uint8_t *source = &record[offset];
            uint8_t flags;
            loaded[task].target_x = (int16_t)storage_read_u16(&source[0]);
            loaded[task].target_y = (int16_t)storage_read_u16(&source[2]);
            if (task != MISSION_TASK_FRAME_PUT) {
                loaded[task].blind_stop_mm = storage_read_u16(&source[4]);
                loaded[task].blind_pulses_per_mm = storage_read_u24(&source[6]);
                flags = source[9];
                offset += 10U;
            } else {
                loaded[task].z_stop_mm = storage_read_u16(&source[4]);
                loaded[task].z_max_pulses = storage_read_u24(&source[6]);
                flags = source[9];
                offset += 10U;
            }
            if (task == MISSION_TASK_TAG_PUT) {
                loaded[task].z_stop_mm = storage_read_u16(&source[10]);
                loaded[task].z_max_pulses = storage_read_u24(&source[12]);
                offset += 5U;
            }
            if ((flags & 0xC0U) != 0U) {
                g_task.status.storage_state = MISSION_STORAGE_INVALID;
                return 0U;
            }
            loaded[task].align_configured =
                ((flags & MISSION_CONFIG_ALIGN) != 0U) ? 1U : 0U;
            loaded[task].blind_configured =
                ((flags & MISSION_CONFIG_BLIND) != 0U) ? 1U : 0U;
            loaded[task].z_enabled =
                ((flags & MISSION_CONFIG_Z_ENABLED) != 0U) ? 1U : 0U;
            loaded[task].z_configured =
                ((flags & MISSION_CONFIG_Z_CONFIGURED) != 0U) ? 1U : 0U;
            loaded[task].blind_direction =
                ((flags & 0x10U) != 0U) ? -1 : 1;
            loaded[task].z_direction =
                ((flags & 0x20U) != 0U) ? -1 : 1;
        }
        g_task.status.storage_generation = storage_read_u16(&record[5]);
    } else if ((storage_read_u16(&record[4]) ==
                MISSION_STORAGE_V1_VERSION) &&
               (storage_read_u16(&record[6]) ==
                MISSION_STORAGE_V1_LENGTH) &&
               (record[12] == MISSION_TASK_COUNT) &&
               (storage_read_u32(&record[MISSION_STORAGE_V1_CRC_OFFSET]) ==
                storage_crc32(&record[4],
                    MISSION_STORAGE_V1_CRC_OFFSET - 4U))) {
        for (uint8_t task = 0U; task < MISSION_TASK_COUNT; ++task) {
            const uint8_t *source = &record[MISSION_STORAGE_V1_CONFIG_OFFSET +
                task * MISSION_STORAGE_V1_CONFIG_SIZE];
            uint8_t flags = source[6];
            loaded[task].target_x = (int16_t)storage_read_u16(&source[0]);
            loaded[task].target_y = (int16_t)storage_read_u16(&source[2]);
            loaded[task].blind_stop_mm = storage_read_u16(&source[4]);
            loaded[task].blind_direction = (int8_t)source[7];
            loaded[task].blind_pulses_per_mm = storage_read_u32(&source[8]);
            loaded[task].z_stop_mm = storage_read_u16(&source[12]);
            loaded[task].z_direction = (int8_t)source[14];
            loaded[task].z_max_pulses = storage_read_u32(&source[16]);
            loaded[task].align_configured =
                ((flags & MISSION_CONFIG_ALIGN) != 0U) ? 1U : 0U;
            loaded[task].blind_configured =
                ((flags & MISSION_CONFIG_BLIND) != 0U) ? 1U : 0U;
            loaded[task].z_enabled =
                ((flags & MISSION_CONFIG_Z_ENABLED) != 0U) ? 1U : 0U;
            loaded[task].z_configured =
                ((flags & MISSION_CONFIG_Z_CONFIGURED) != 0U) ? 1U : 0U;
        }
        g_task.status.storage_generation = storage_read_u32(&record[8]);
    } else {
        g_task.status.storage_state = MISSION_STORAGE_INVALID;
        return 0U;
    }

    for (uint8_t task = 0U; task < MISSION_TASK_COUNT; ++task) {
        if (config_sane((MissionTaskName)task, &loaded[task]) == 0U) {
            g_task.status.storage_state = MISSION_STORAGE_INVALID;
            return 0U;
        }
    }
    memcpy(g_task.configs, loaded, sizeof(loaded));
    g_task.status.storage_state = MISSION_STORAGE_VALID;
    if (version != MISSION_STORAGE_VERSION)
        return MissionTask_SaveConfiguration();
    return 1U;
}

static uint8_t terminal_state(MissionTaskState state)
{
    return ((state == MISSION_STATE_IDLE) ||
            (state == MISSION_STATE_COMPLETE) ||
            (state == MISSION_STATE_FAULT)) ? 1U : 0U;
}

static void enter_state(MissionTaskState state, uint32_t now)
{
    g_task.status.state = state;
    g_task.status.state_tick = now;
    g_task.subflow_started = 0U;
}

static void release_mission(uint32_t now)
{
    (void)MissionSubflow_SetOwnerRetention(0U);
    MotionCoordinator_Release(MOTION_OWNER_MISSION, now);
}

static void finish_complete(uint32_t now)
{
    MissionFailure none = {MISSION_FAIL_SOURCE_NONE, MISSION_FAIL_NONE, 0};
    g_task.status.failure = none;
    ++g_task.status.completed_count;
    enter_state(MISSION_STATE_COMPLETE, now);
    release_mission(now);
}

static void finish_terminal_fault(MissionFailure failure, uint32_t now)
{
    if (MissionSubflow_IsActive() != 0U)
        MissionSubflow_AbortAll(failure, now);
    g_task.status.failure = failure;
    ++g_task.status.failed_count;
    enter_state(MISSION_STATE_FAULT, now);
    release_mission(now);
}

static uint8_t fault_retryable(MissionFailure failure)
{
    MotionCoordinatorStatus coordinator;
    if ((g_task.status.restart_count >= g_task.status.max_restarts) ||
        (failure.reason == MISSION_FAIL_BUSY) ||
        (failure.reason == MISSION_FAIL_INVALID_ARGUMENT) ||
        (failure.reason == MISSION_FAIL_POSITION_INVALID) ||
        (failure.reason == MISSION_FAIL_SOFT_LIMIT) ||
        (failure.reason == MISSION_FAIL_CANCELLED) ||
        (g_task.status.state == MISSION_STATE_PRECHECK) ||
        (g_task.status.state == MISSION_STATE_PRECHECK_HELD) ||
        (g_task.status.state == MISSION_STATE_VERIFY) ||
        (((g_task.status.task == MISSION_TASK_TAG_PUT) ||
          (g_task.status.task == MISSION_TASK_FRAME_PUT)) &&
         (g_task.status.payload != MISSION_PAYLOAD_HELD))) return 0U;
    MotionCoordinator_GetStatus(&coordinator);
    return ((coordinator.owner == MOTION_OWNER_MISSION) &&
            (coordinator.latch_reason == MOTION_LATCH_NONE) &&
            (coordinator.gripper_frozen == 0U)) ? 1U : 0U;
}

static void handle_fault(MissionFailure failure, uint32_t now)
{
    if (fault_retryable(failure) == 0U) {
        finish_terminal_fault(failure, now);
        return;
    }
    if (MissionSubflow_IsActive() != 0U)
        MissionSubflow_AbortAll(failure, now);
    g_task.status.failure = failure;
    ++g_task.status.restart_count;
    enter_state(MISSION_STATE_RECOVERY_WAIT_IDLE, now);
}

static uint8_t config_valid(MissionTaskName task, int32_t *detail)
{
    const MissionTaskConfig *config = &g_task.configs[task];
    int32_t missing = 0;
    if (config->align_configured == 0U) missing |= 1;
    if ((task != MISSION_TASK_FRAME_PUT) &&
        (config->blind_configured == 0U)) missing |= 2;
    if ((task == MISSION_TASK_FRAME_PUT) &&
        (config->z_configured == 0U)) missing |= 4;
    if ((task == MISSION_TASK_TAG_PUT) &&
        (config->z_enabled != 0U) &&
        (config->z_configured == 0U)) missing |= 4;
    if ((pose_sane(&config->preset_pose) == 0U) ||
        (pose_sane(&config->safe_pose) == 0U)) missing |= 8;
    *detail = missing;
    return (missing == 0) ? 1U : 0U;
}

static void poll_precheck(uint32_t now)
{
    MotionCoordinatorStatus coordinator;
    MotionPositionSnapshot pose;
    int32_t detail;
    MotionCoordinator_GetStatus(&coordinator);
    if ((coordinator.latch_reason != MOTION_LATCH_NONE) ||
        (coordinator.gripper_frozen != 0U)) {
        MissionFailure failure = {MISSION_FAIL_SOURCE_COORDINATOR,
                                  MISSION_FAIL_REJECTED,
                                  (int32_t)coordinator.latch_reason};
        handle_fault(failure, now);
        return;
    }
    if (config_valid(g_task.status.task, &detail) == 0U) {
        MissionFailure failure = {MISSION_FAIL_SOURCE_COORDINATOR,
                                  MISSION_FAIL_INVALID_ARGUMENT, detail};
        handle_fault(failure, now);
        return;
    }
    if ((g_task.status.state == MISSION_STATE_PRECHECK_HELD) &&
        (g_task.status.payload != MISSION_PAYLOAD_HELD)) {
        MissionFailure failure = {MISSION_FAIL_SOURCE_GRIPPER,
                                  MISSION_FAIL_REJECTED,
                                  (int32_t)g_task.status.payload};
        handle_fault(failure, now);
        return;
    }
    if (MotionCoordinator_CaptureSnapshot(&pose, 1U, now) == 0U) {
        MissionFailure failure = {MISSION_FAIL_SOURCE_POSE,
                                  MISSION_FAIL_POSITION_INVALID, 0};
        handle_fault(failure, now);
        return;
    }
    enter_state(((g_task.status.task == MISSION_TASK_RED_PICK) ||
                 (g_task.status.task == MISSION_TASK_RED_FIND)) ?
                MISSION_STATE_PREPARE_GRIP_OPEN :
                MISSION_STATE_PRESET_POSE, now);
}

static MissionSubflowType expected_subflow(MissionTaskState state)
{
    switch (state) {
    case MISSION_STATE_PREPARE_GRIP_OPEN:
        return MISSION_SUBFLOW_GRIP_OPEN;
    case MISSION_STATE_PRESET_POSE:
        return MISSION_SUBFLOW_PRESET_POSE;
    case MISSION_STATE_RED_OBSERVE:
        return MISSION_SUBFLOW_OBSERVE_RED_FRONT;
    case MISSION_STATE_TAG_OBSERVE:
        return MISSION_SUBFLOW_OBSERVE_TAG_FRONT;
    case MISSION_STATE_FRAME_OBSERVE:
        return MISSION_SUBFLOW_OBSERVE_FRAME_DOWN;
    case MISSION_STATE_ALIGN_XZ:
        return MISSION_SUBFLOW_ALIGN_XZ;
    case MISSION_STATE_ALIGN_XY:
        return MISSION_SUBFLOW_ALIGN_XY;
    case MISSION_STATE_RECORD_XYZ:
        return MISSION_SUBFLOW_RECORD_POSE;
    case MISSION_STATE_BLIND_Y:
        return MISSION_SUBFLOW_BLIND_MOVE_Y;
    case MISSION_STATE_OPTIONAL_Z_DROP:
    case MISSION_STATE_TOF3_Z_DESCEND:
        return MISSION_SUBFLOW_TOF3_DESCEND;
    case MISSION_STATE_GRIP_CLOSE:
        return MISSION_SUBFLOW_GRIP_CLOSE;
    case MISSION_STATE_GRIP_OPEN:
        return MISSION_SUBFLOW_GRIP_OPEN;
    case MISSION_STATE_RETURN_RECORDED_POSE:
        return MISSION_SUBFLOW_RETURN_POSE;
    case MISSION_STATE_SAFE_RETREAT:
        return MISSION_SUBFLOW_SAFE_RETREAT;
    case MISSION_STATE_RECOVER_PRESET:
        return MISSION_SUBFLOW_PRESET_POSE;
    default:
        return MISSION_SUBFLOW_NONE;
    }
}

static uint32_t state_timeout(MissionTaskState state)
{
    switch (state) {
    case MISSION_STATE_PREPARE_GRIP_OPEN:
        return MISSION_GRIP_TIMEOUT_MS;
    case MISSION_STATE_PRESET_POSE:
        return MISSION_POSE_TIMEOUT_MS;
    case MISSION_STATE_RED_OBSERVE:
    case MISSION_STATE_TAG_OBSERVE:
    case MISSION_STATE_FRAME_OBSERVE:
        return MISSION_OBSERVE_TIMEOUT_MS;
    case MISSION_STATE_ALIGN_XZ:
    case MISSION_STATE_ALIGN_XY:
        return MISSION_ALIGN_TIMEOUT_MS;
    case MISSION_STATE_BLIND_Y:
        return MISSION_BLIND_TIMEOUT_MS;
    case MISSION_STATE_OPTIONAL_Z_DROP:
    case MISSION_STATE_TOF3_Z_DESCEND:
        return MISSION_Z_TIMEOUT_MS;
    case MISSION_STATE_GRIP_CLOSE:
    case MISSION_STATE_GRIP_OPEN:
        return MISSION_GRIP_TIMEOUT_MS;
    case MISSION_STATE_RECORD_XYZ:
    case MISSION_STATE_RETURN_RECORDED_POSE:
    case MISSION_STATE_SAFE_RETREAT:
    case MISSION_STATE_RECOVER_PRESET:
        return MISSION_POSE_TIMEOUT_MS;
    default:
        return 1000U;
    }
}

static uint8_t start_subflow(uint32_t now)
{
    MissionTaskName task = g_task.status.task;
    const MissionTaskConfig *config = &g_task.configs[task];
    switch (g_task.status.state) {
    case MISSION_STATE_PREPARE_GRIP_OPEN:
        return MissionSubflow_StartGrip(task, 0U, now);
    case MISSION_STATE_PRESET_POSE:
    case MISSION_STATE_RECOVER_PRESET: {
        MotionPositionSnapshot pose = {
            config->preset_pose.x_pulses,
            config->preset_pose.y_pulses,
            config->preset_pose.z_pulses,
            now
        };
        return MissionSubflow_StartPresetPose(task, &pose, now);
    }
    case MISSION_STATE_RED_OBSERVE:
        return MissionSubflow_StartObserveRedFront(task, now);
    case MISSION_STATE_TAG_OBSERVE:
        return MissionSubflow_StartObserveTagFront(task, now);
    case MISSION_STATE_FRAME_OBSERVE:
        return MissionSubflow_StartObserveFrameDown(task, now);
    case MISSION_STATE_ALIGN_XZ:
        return MissionSubflow_StartAlignXZ(task, config->target_x,
                                           config->target_y, now);
    case MISSION_STATE_ALIGN_XY:
        return MissionSubflow_StartAlignXY(task, config->target_x,
                                           config->target_y, now);
    case MISSION_STATE_RECORD_XYZ:
        return MissionSubflow_StartRecordPose(task, now);
    case MISSION_STATE_BLIND_Y:
        return MissionSubflow_StartBlindMoveY(task,
            (task == MISSION_TASK_TAG_PUT) ?
                C552_ID_TOF1 : C552_ID_TOF2,
            config->blind_stop_mm, config->blind_pulses_per_mm,
            config->blind_direction, now);
    case MISSION_STATE_OPTIONAL_Z_DROP:
    case MISSION_STATE_TOF3_Z_DESCEND:
        return MissionSubflow_StartTof3Descend(task, config->z_stop_mm,
            config->z_max_pulses, config->z_direction, now);
    case MISSION_STATE_GRIP_CLOSE:
        return MissionSubflow_StartGrip(task, 1U, now);
    case MISSION_STATE_GRIP_OPEN:
        return MissionSubflow_StartGrip(task, 0U, now);
    case MISSION_STATE_RETURN_RECORDED_POSE:
        return MissionSubflow_StartReturnPose(task, now);
    case MISSION_STATE_SAFE_RETREAT:
    {
        MotionPositionSnapshot pose = {
            config->safe_pose.x_pulses,
            config->safe_pose.y_pulses,
            config->safe_pose.z_pulses,
            now
        };
        return MissionSubflow_StartSafeRetreat(task, &pose, now);
    }
    default:
        return 0U;
    }
}

static void advance_after_subflow(uint32_t now)
{
    switch (g_task.status.state) {
    case MISSION_STATE_PREPARE_GRIP_OPEN:
        g_task.status.payload = MISSION_PAYLOAD_EMPTY;
        enter_state(MISSION_STATE_PRESET_POSE, now);
        break;
    case MISSION_STATE_PRESET_POSE:
        if ((g_task.status.task == MISSION_TASK_RED_PICK) ||
            (g_task.status.task == MISSION_TASK_RED_FIND))
            enter_state(MISSION_STATE_RED_OBSERVE, now);
        else if (g_task.status.task == MISSION_TASK_TAG_PUT)
            enter_state(MISSION_STATE_TAG_OBSERVE, now);
        else
            enter_state(MISSION_STATE_FRAME_OBSERVE, now);
        break;
    case MISSION_STATE_RED_OBSERVE:
        enter_state(MISSION_STATE_FRONT_RED_READY, now);
        break;
    case MISSION_STATE_TAG_OBSERVE:
        enter_state(MISSION_STATE_FRONT_TAG_READY, now);
        break;
    case MISSION_STATE_FRAME_OBSERVE:
        enter_state(MISSION_STATE_DOWN_FRAME_READY, now);
        break;
    case MISSION_STATE_ALIGN_XZ:
    case MISSION_STATE_ALIGN_XY:
        enter_state(MISSION_STATE_STABLE, now);
        break;
    case MISSION_STATE_RECORD_XYZ:
        enter_state(MISSION_STATE_BLIND_Y, now);
        break;
    case MISSION_STATE_BLIND_Y:
        enter_state((g_task.status.task == MISSION_TASK_TAG_PUT) ?
                    MISSION_STATE_OPTIONAL_Z_DROP :
                    MISSION_STATE_GRIP_CLOSE, now);
        break;
    case MISSION_STATE_OPTIONAL_Z_DROP:
    case MISSION_STATE_TOF3_Z_DESCEND:
        enter_state(MISSION_STATE_GRIP_OPEN, now);
        break;
    case MISSION_STATE_GRIP_CLOSE:
        g_task.status.payload = MISSION_PAYLOAD_HELD;
        enter_state(MISSION_STATE_RETURN_RECORDED_POSE, now);
        break;
    case MISSION_STATE_GRIP_OPEN:
        g_task.status.payload = MISSION_PAYLOAD_EMPTY;
        enter_state(MISSION_STATE_SAFE_RETREAT, now);
        break;
    case MISSION_STATE_RETURN_RECORDED_POSE:
        enter_state(MISSION_STATE_VERIFY, now);
        break;
    case MISSION_STATE_SAFE_RETREAT:
        finish_complete(now);
        break;
    case MISSION_STATE_RECOVER_PRESET:
        enter_state(((g_task.status.task == MISSION_TASK_TAG_PUT) ||
                     (g_task.status.task == MISSION_TASK_FRAME_PUT)) ?
                    MISSION_STATE_PRECHECK_HELD : MISSION_STATE_PRECHECK,
                    now);
        break;
    default: {
        MissionFailure failure = {MISSION_FAIL_SOURCE_COORDINATOR,
                                  MISSION_FAIL_INVALID_ARGUMENT,
                                  (int32_t)g_task.status.state};
        handle_fault(failure, now);
        break;
    }
    }
}

static void poll_subflow_state(uint32_t now)
{
    MissionSubflowStatus subflow;
    MissionSubflowType expected = expected_subflow(g_task.status.state);
    if (g_task.subflow_started == 0U) {
        if (start_subflow(now) == 0U) {
            MissionFailure failure = {MISSION_FAIL_SOURCE_COORDINATOR,
                                      MISSION_FAIL_REJECTED,
                                      (int32_t)expected};
            handle_fault(failure, now);
        } else {
            g_task.subflow_started = 1U;
        }
        return;
    }
    MissionSubflow_GetStatus(&subflow);
    if ((subflow.task != g_task.status.task) ||
        (subflow.type != expected)) {
        MissionFailure failure = {MISSION_FAIL_SOURCE_COORDINATOR,
                                  MISSION_FAIL_REJECTED,
                                  (int32_t)subflow.type};
        handle_fault(failure, now);
        return;
    }
    if (subflow.state == MISSION_SUBFLOW_COMPLETE) {
        advance_after_subflow(now);
    } else if (subflow.state == MISSION_SUBFLOW_FAULT) {
        handle_fault(subflow.failure, now);
    } else if ((uint32_t)(now - g_task.status.state_tick) >
               state_timeout(g_task.status.state)) {
        MissionFailure failure = {MISSION_FAIL_SOURCE_COORDINATOR,
                                  MISSION_FAIL_TIMEOUT,
                                  (int32_t)g_task.status.state};
        handle_fault(failure, now);
    }
}

static void process_start_request(uint32_t now)
{
    MotionCoordinatorStatus coordinator;
    MissionTaskName task = (MissionTaskName)g_requested_task;
    MissionFailure failure;
    g_start_request = 0U;
    MotionCoordinator_GetStatus(&coordinator);
    if ((task >= MISSION_TASK_COUNT) ||
        (MissionSubflow_IsActive() != 0U) ||
        (coordinator.owner != MOTION_OWNER_NONE) ||
        (coordinator.latch_reason != MOTION_LATCH_NONE)) {
        failure.source = MISSION_FAIL_SOURCE_COORDINATOR;
        failure.reason = MISSION_FAIL_BUSY;
        failure.detail = (int32_t)coordinator.owner;
        g_task.status.task = task;
        g_task.status.failure = failure;
        ++g_task.status.failed_count;
        enter_state(MISSION_STATE_FAULT, now);
        return;
    }
    if (MotionCoordinator_Acquire(MOTION_OWNER_MISSION, 0U, now) == 0U) {
        failure.source = MISSION_FAIL_SOURCE_COORDINATOR;
        failure.reason = MISSION_FAIL_BUSY;
        failure.detail = 0;
        g_task.status.task = task;
        g_task.status.failure = failure;
        ++g_task.status.failed_count;
        enter_state(MISSION_STATE_FAULT, now);
        return;
    }
    if (MissionSubflow_SetOwnerRetention(1U) == 0U) {
        failure.source = MISSION_FAIL_SOURCE_COORDINATOR;
        failure.reason = MISSION_FAIL_REJECTED;
        failure.detail = 0;
        g_task.status.task = task;
        g_task.status.failure = failure;
        ++g_task.status.failed_count;
        enter_state(MISSION_STATE_FAULT, now);
        MotionCoordinator_Release(MOTION_OWNER_MISSION, now);
        return;
    }
    g_task.status.task = task;
    g_task.status.failure.source = MISSION_FAIL_SOURCE_NONE;
    g_task.status.failure.reason = MISSION_FAIL_NONE;
    g_task.status.failure.detail = 0;
    g_task.status.start_tick = now;
    g_task.status.restart_count = 0U;
    g_task.status.max_restarts = MISSION_MAX_RESTARTS;
    g_task.cancel_issued = 0U;
    enter_state(((task == MISSION_TASK_TAG_PUT) ||
                 (task == MISSION_TASK_FRAME_PUT)) ?
                MISSION_STATE_PRECHECK_HELD : MISSION_STATE_PRECHECK, now);
}

void MissionTask_Init(uint32_t now)
{
    memset(&g_task, 0, sizeof(g_task));
    for (uint8_t task = 0U; task < MISSION_TASK_COUNT; ++task)
        set_default_poses(&g_task.configs[task]);
    g_task.status.task = MISSION_TASK_RED_PICK;
    g_task.status.state = MISSION_STATE_IDLE;
    g_task.status.payload = MISSION_PAYLOAD_EMPTY;
    g_task.status.max_restarts = MISSION_MAX_RESTARTS;
    g_task.status.state_tick = now;
    g_start_request = 0U;
    g_requested_task = 0U;
    g_cancel_request = 0U;
    g_task.status.storage_state = MISSION_STORAGE_NOT_LOADED;
    (void)load_configuration(now);
}

void MissionTask_Poll(uint32_t now)
{
    MotionCoordinatorStatus coordinator;
    if ((g_start_request != 0U) &&
        (terminal_state(g_task.status.state) != 0U))
        process_start_request(now);
    if (MissionTask_IsActive() == 0U) return;

    if (g_cancel_request != 0U) {
        g_cancel_request = 0U;
        g_task.cancel_issued = 0U;
        enter_state(MISSION_STATE_ABORTING, now);
    }
    if (g_task.status.state == MISSION_STATE_ABORTING) {
        if ((g_task.cancel_issued == 0U) &&
            (MissionSubflow_IsActive() != 0U)) {
            MissionSubflow_Cancel(now);
            g_task.cancel_issued = 1U;
            return;
        }
        if (MissionSubflow_IsActive() == 0U) {
            MissionFailure failure = {MISSION_FAIL_SOURCE_COORDINATOR,
                                      MISSION_FAIL_CANCELLED, 0};
            finish_terminal_fault(failure, now);
        }
        return;
    }

    MotionCoordinator_GetStatus(&coordinator);
    if ((coordinator.latch_reason != MOTION_LATCH_NONE) ||
        (coordinator.owner != MOTION_OWNER_MISSION)) {
        MissionFailure failure = {MISSION_FAIL_SOURCE_COORDINATOR,
                                  MISSION_FAIL_CANCELLED,
                                  (int32_t)coordinator.latch_reason};
        finish_terminal_fault(failure, now);
        return;
    }
    if ((uint32_t)(now - g_task.status.start_tick) >
        MISSION_TOTAL_TIMEOUT_MS) {
        MissionFailure failure = {MISSION_FAIL_SOURCE_COORDINATOR,
                                  MISSION_FAIL_TIMEOUT,
                                  (int32_t)g_task.status.state};
        finish_terminal_fault(failure, now);
        return;
    }

    if (g_task.status.state == MISSION_STATE_RECOVERY_WAIT_IDLE) {
        XY_AxisStatus x;
        XY_AxisStatus y;
        ZAxisControlStatus z;
        (void)XY_GetStatus(XY_AXIS_X, &x);
        (void)XY_GetStatus(XY_AXIS_Y, &y);
        ZAxis_GetControlStatus(&z);
        if ((x.position_valid == 0U) || (y.position_valid == 0U) ||
            (z.position_valid == 0U) || (x.state == XY_STATE_FAULT) ||
            (y.state == XY_STATE_FAULT) || (z.state == Z_STATE_FAULT)) {
            finish_terminal_fault(g_task.status.failure, now);
        } else if ((x.state == XY_STATE_IDLE) &&
                   (y.state == XY_STATE_IDLE) &&
                   (z.state == Z_STATE_IDLE)) {
            enter_state(MISSION_STATE_RECOVER_PRESET, now);
        } else if ((uint32_t)(now - g_task.status.state_tick) >
                   MISSION_RECOVERY_IDLE_MS) {
            finish_terminal_fault(g_task.status.failure, now);
        }
        return;
    }

    switch (g_task.status.state) {
    case MISSION_STATE_PRECHECK:
    case MISSION_STATE_PRECHECK_HELD:
        poll_precheck(now);
        break;
    case MISSION_STATE_FRONT_RED_READY:
    case MISSION_STATE_FRONT_TAG_READY:
        if ((uint32_t)(now - g_task.status.state_tick) >=
            MISSION_MARKER_HOLD_MS)
            enter_state(MISSION_STATE_ALIGN_XZ, now);
        break;
    case MISSION_STATE_DOWN_FRAME_READY:
        if ((uint32_t)(now - g_task.status.state_tick) >=
            MISSION_MARKER_HOLD_MS)
            enter_state(MISSION_STATE_ALIGN_XY, now);
        break;
    case MISSION_STATE_STABLE:
        if ((uint32_t)(now - g_task.status.state_tick) <
            MISSION_MARKER_HOLD_MS) break;
        if ((g_task.status.task == MISSION_TASK_RED_PICK) ||
            (g_task.status.task == MISSION_TASK_RED_FIND))
            enter_state(MISSION_STATE_RECORD_XYZ, now);
        else if (g_task.status.task == MISSION_TASK_TAG_PUT)
            enter_state(MISSION_STATE_BLIND_Y, now);
        else
            enter_state(MISSION_STATE_TOF3_Z_DESCEND, now);
        break;
    case MISSION_STATE_OPTIONAL_Z_DROP:
        if (g_task.configs[g_task.status.task].z_enabled == 0U) {
            enter_state(MISSION_STATE_GRIP_OPEN, now);
            break;
        }
        poll_subflow_state(now);
        break;
    case MISSION_STATE_VERIFY:
        if (g_task.status.payload == MISSION_PAYLOAD_HELD)
            enter_state(MISSION_STATE_SAFE_RETREAT, now);
        else {
            MissionFailure failure = {MISSION_FAIL_SOURCE_GRIPPER,
                                      MISSION_FAIL_REJECTED, 0};
            handle_fault(failure, now);
        }
        break;
    case MISSION_STATE_PREPARE_GRIP_OPEN:
    case MISSION_STATE_PRESET_POSE:
    case MISSION_STATE_RED_OBSERVE:
    case MISSION_STATE_TAG_OBSERVE:
    case MISSION_STATE_FRAME_OBSERVE:
    case MISSION_STATE_ALIGN_XZ:
    case MISSION_STATE_ALIGN_XY:
    case MISSION_STATE_RECORD_XYZ:
    case MISSION_STATE_BLIND_Y:
    case MISSION_STATE_TOF3_Z_DESCEND:
    case MISSION_STATE_GRIP_CLOSE:
    case MISSION_STATE_GRIP_OPEN:
    case MISSION_STATE_RETURN_RECORDED_POSE:
    case MISSION_STATE_SAFE_RETREAT:
    case MISSION_STATE_RECOVER_PRESET:
        poll_subflow_state(now);
        break;
    default: {
        MissionFailure failure = {MISSION_FAIL_SOURCE_COORDINATOR,
                                  MISSION_FAIL_INVALID_ARGUMENT,
                                  (int32_t)g_task.status.state};
        handle_fault(failure, now);
        break;
    }
    }
}

uint8_t MissionTask_RequestStart(MissionTaskName task)
{
    MotionCoordinatorStatus coordinator;
    MotionCoordinator_GetStatus(&coordinator);
    if ((task >= MISSION_TASK_COUNT) ||
        (MissionTask_IsActive() != 0U) ||
        (MissionSubflow_IsActive() != 0U) ||
        (coordinator.owner != MOTION_OWNER_NONE) ||
        (coordinator.latch_reason != MOTION_LATCH_NONE)) return 0U;
    g_requested_task = (uint8_t)task;
    g_start_request = 1U;
    return 1U;
}

void MissionTask_RequestCancel(void)
{
    if (MissionTask_IsActive() != 0U) g_cancel_request = 1U;
}

uint8_t MissionTask_SetAlignTarget(MissionTaskName task, int16_t target_x,
                                   int16_t target_y)
{
    MissionTaskConfig previous;
    if ((task >= MISSION_TASK_COUNT) ||
        (MissionTask_IsActive() != 0U) ||
        (g_start_request != 0U)) return 0U;
    previous = g_task.configs[task];
    g_task.configs[task].target_x = target_x;
    g_task.configs[task].target_y = target_y;
    g_task.configs[task].align_configured = 1U;
    if (MissionTask_SaveConfiguration() == 0U) {
        g_task.configs[task] = previous;
        return 0U;
    }
    return 1U;
}

uint8_t MissionTask_SetBlindY(MissionTaskName task, uint16_t stop_mm,
                              uint32_t pulses_per_mm, int8_t direction)
{
    MissionTaskConfig previous;
    if ((task >= MISSION_TASK_COUNT) ||
        (task == MISSION_TASK_FRAME_PUT) ||
        (pulses_per_mm == 0U) ||
        (pulses_per_mm > MISSION_UINT24_MAX) ||
        ((direction != -1) && (direction != 1)) ||
        (MissionTask_IsActive() != 0U) ||
        (g_start_request != 0U)) return 0U;
    previous = g_task.configs[task];
    g_task.configs[task].blind_stop_mm = stop_mm;
    g_task.configs[task].blind_pulses_per_mm = pulses_per_mm;
    g_task.configs[task].blind_direction = direction;
    g_task.configs[task].blind_configured = 1U;
    if (MissionTask_SaveConfiguration() == 0U) {
        g_task.configs[task] = previous;
        return 0U;
    }
    return 1U;
}

uint8_t MissionTask_SetZDrop(MissionTaskName task, uint8_t enabled,
                             uint16_t stop_mm, uint32_t max_pulses,
                             int8_t direction)
{
    MissionTaskConfig previous;
    if (((task != MISSION_TASK_TAG_PUT) &&
         (task != MISSION_TASK_FRAME_PUT)) ||
        (MissionTask_IsActive() != 0U) ||
        (g_start_request != 0U)) return 0U;
    previous = g_task.configs[task];
    if (enabled == 0U) {
        if (task != MISSION_TASK_TAG_PUT) return 0U;
        g_task.configs[task].z_enabled = 0U;
        if (MissionTask_SaveConfiguration() == 0U) {
            g_task.configs[task] = previous;
            return 0U;
        }
        return 1U;
    }
    if ((max_pulses == 0U) || (max_pulses > MISSION_UINT24_MAX) ||
        ((direction != -1) && (direction != 1))) return 0U;
    g_task.configs[task].z_stop_mm = stop_mm;
    g_task.configs[task].z_max_pulses = max_pulses;
    g_task.configs[task].z_direction = direction;
    g_task.configs[task].z_enabled = 1U;
    g_task.configs[task].z_configured = 1U;
    if (MissionTask_SaveConfiguration() == 0U) {
        g_task.configs[task] = previous;
        return 0U;
    }
    return 1U;
}

static uint8_t set_task_pose(MissionTaskName task, MissionTaskPose *target,
                             int32_t x_pulses, int32_t y_pulses,
                             int32_t z_pulses)
{
    MissionTaskPose previous;
    MissionTaskPose pose = {x_pulses, y_pulses, z_pulses};
    if ((task >= MISSION_TASK_COUNT) || (target == NULL) ||
        (pose_sane(&pose) == 0U) || (MissionTask_IsActive() != 0U) ||
        (g_start_request != 0U)) return 0U;
    previous = *target;
    *target = pose;
    if (MissionTask_SaveConfiguration() == 0U) {
        *target = previous;
        return 0U;
    }
    return 1U;
}

uint8_t MissionTask_SetPresetPose(MissionTaskName task, int32_t x_pulses,
                                  int32_t y_pulses, int32_t z_pulses)
{
    if (task >= MISSION_TASK_COUNT) return 0U;
    return set_task_pose(task, &g_task.configs[task].preset_pose,
                         x_pulses, y_pulses, z_pulses);
}

uint8_t MissionTask_SetSafePose(MissionTaskName task, int32_t x_pulses,
                                int32_t y_pulses, int32_t z_pulses)
{
    if (task >= MISSION_TASK_COUNT) return 0U;
    return set_task_pose(task, &g_task.configs[task].safe_pose,
                         x_pulses, y_pulses, z_pulses);
}

uint8_t MissionTask_SetPayload(MissionPayloadState payload)
{
    if ((payload > MISSION_PAYLOAD_HELD) ||
        (MissionTask_IsActive() != 0U) ||
        (g_start_request != 0U)) return 0U;
    g_task.status.payload = payload;
    return 1U;
}

uint8_t MissionTask_SaveConfiguration(void)
{
    uint8_t record[MISSION_STORAGE_LENGTH];
    uint16_t offset = MISSION_STORAGE_CONFIG_OFFSET;
    uint16_t generation =
        (uint16_t)(g_task.status.storage_generation + 1U);

    if (generation == 0U) generation = 1U;

    for (uint8_t task = 0U; task < MISSION_TASK_COUNT; ++task) {
        if (config_sane((MissionTaskName)task, &g_task.configs[task]) == 0U)
            return 0U;
    }

    memset(record, 0, sizeof(record));
    record[4] = MISSION_STORAGE_VERSION;
    storage_write_u16(&record[5], generation);

    for (uint8_t task = 0U; task < MISSION_TASK_COUNT; ++task) {
        const MissionTaskConfig *config = &g_task.configs[task];
        uint8_t *target = &record[MISSION_STORAGE_POSE_OFFSET +
                                  task * MISSION_STORAGE_POSE_SIZE];
        storage_write_u24(&target[0],
                          (uint32_t)config->preset_pose.x_pulses);
        storage_write_u24(&target[3],
                          (uint32_t)config->preset_pose.y_pulses);
        storage_write_u24(&target[6],
                          (uint32_t)config->preset_pose.z_pulses);
        storage_write_u24(&target[9],
                          (uint32_t)config->safe_pose.x_pulses);
        storage_write_u24(&target[12],
                          (uint32_t)config->safe_pose.y_pulses);
        storage_write_u24(&target[15],
                          (uint32_t)config->safe_pose.z_pulses);
    }

    for (uint8_t task = 0U; task < MISSION_TASK_COUNT; ++task) {
        const MissionTaskConfig *config = &g_task.configs[task];
        uint8_t *target = &record[offset];
        uint8_t flags = 0U;
        if (config->align_configured != 0U) flags |= MISSION_CONFIG_ALIGN;
        if (config->blind_configured != 0U) flags |= MISSION_CONFIG_BLIND;
        if (config->z_enabled != 0U) flags |= MISSION_CONFIG_Z_ENABLED;
        if (config->z_configured != 0U) flags |= MISSION_CONFIG_Z_CONFIGURED;
        if (config->blind_direction < 0) flags |= 0x10U;
        if (config->z_direction < 0) flags |= 0x20U;
        storage_write_u16(&target[0], (uint16_t)config->target_x);
        storage_write_u16(&target[2], (uint16_t)config->target_y);
        if (task != MISSION_TASK_FRAME_PUT) {
            storage_write_u16(&target[4], config->blind_stop_mm);
            storage_write_u24(&target[6], config->blind_pulses_per_mm);
            target[9] = flags;
            offset += 10U;
        } else {
            storage_write_u16(&target[4], config->z_stop_mm);
            storage_write_u24(&target[6], config->z_max_pulses);
            target[9] = flags;
            offset += 10U;
        }
        if (task == MISSION_TASK_TAG_PUT) {
            storage_write_u16(&target[10], config->z_stop_mm);
            storage_write_u24(&target[12], config->z_max_pulses);
            offset += 5U;
        }
    }
    if (offset != MISSION_STORAGE_CRC_OFFSET) return 0U;
    storage_write_u32(&record[MISSION_STORAGE_CRC_OFFSET],
        storage_crc32(&record[4], MISSION_STORAGE_CRC_OFFSET - 4U));

    if ((EEPROM_Write(MISSION_STORAGE_ADDRESS, record, 4U) != HAL_OK) ||
        (EEPROM_Write(MISSION_STORAGE_ADDRESS + 4U, &record[4],
                      MISSION_STORAGE_LENGTH - 4U) != HAL_OK)) {
        g_task.status.storage_state = MISSION_STORAGE_IO_ERROR;
        return 0U;
    }
    storage_write_u32(record, MISSION_STORAGE_MAGIC);
    if (EEPROM_Write(MISSION_STORAGE_ADDRESS, record, 4U) != HAL_OK) {
        g_task.status.storage_state = MISSION_STORAGE_IO_ERROR;
        return 0U;
    }
    g_task.status.storage_generation = (uint32_t)generation;
    g_task.status.storage_state = MISSION_STORAGE_VALID;
    return 1U;
}

void MissionTask_GetStatus(MissionTaskStatus *status)
{
    MissionSubflowStatus subflow;
    if (status == NULL) return;
    *status = g_task.status;
    status->config = g_task.configs[g_task.status.task];
    status->start_pending = g_start_request;
    status->cancel_pending = g_cancel_request;
    MissionSubflow_GetStatus(&subflow);
    status->subflow_type = subflow.type;
    status->subflow_state = subflow.state;
    status->subflow_attempt = subflow.attempt;
    status->subflow_max_attempts = subflow.max_attempts;
    status->axis_recoveries = subflow.axis_recoveries;
    status->max_axis_recoveries = subflow.max_axis_recoveries;
    status->subflow_distance_mm = subflow.distance_mm;
    status->subflow_requested_pulses = subflow.requested_pulses;
}

void MissionTask_GetConfig(MissionTaskName task, MissionTaskConfig *config)
{
    if ((config != NULL) && (task < MISSION_TASK_COUNT))
        *config = g_task.configs[task];
}

uint8_t MissionTask_IsActive(void)
{
    return ((terminal_state(g_task.status.state) == 0U) ||
            (g_start_request != 0U)) ? 1U : 0U;
}

const char *MissionTask_StateString(MissionTaskState state)
{
    static const char *const names[] = {
        "IDLE", "PRECHECK", "PRECHECK_HELD", "PREPARE_GRIP_OPEN",
        "PRESET_POSE", "RED_OBSERVE",
        "TAG_OBSERVE", "FRAME_OBSERVE", "FRONT_RED_READY",
        "FRONT_TAG_READY", "DOWN_FRAME_READY", "ALIGN_XZ", "ALIGN_XY",
        "STABLE", "RECORD_XYZ", "BLIND_Y", "OPTIONAL_Z_DROP",
        "TOF3_Z_DESCEND", "GRIP_CLOSE", "GRIP_OPEN",
        "RETURN_RECORDED_POSE", "VERIFY", "SAFE_RETREAT",
        "RECOVERY_WAIT_IDLE", "RECOVER_PRESET", "ABORTING", "COMPLETE",
        "FAULT"
    };
    return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
           names[state] : "UNKNOWN";
}

const char *MissionTask_PayloadString(MissionPayloadState payload)
{
    static const char *const names[] = {"EMPTY", "HELD"};
    return ((uint32_t)payload < (sizeof(names) / sizeof(names[0]))) ?
           names[payload] : "UNKNOWN";
}

const char *MissionTask_StorageString(MissionStorageState state)
{
    static const char *const names[] = {
        "NOT_LOADED", "EMPTY", "VALID", "INVALID", "IO_ERROR"
    };
    return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
           names[state] : "UNKNOWN";
}

#include "remote_control.h"

#include "c552.h"
#include "cmsis_os2.h"
#include "jsmn.h"
#include "main.h"
#include "mission_subflow.h"
#include "mission_task.h"
#include "motion_coordinator.h"
#include "network_config.h"
#include "xy_motor.h"
#include "z_axis.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REMOTE_REQUEST_QUEUE_DEPTH  4U
#define REMOTE_JSON_TOKENS          64U
#define REMOTE_AXIS_NONE            0xFFU
#define REMOTE_CONFIG_CURRENT       0xFFU
#define REMOTE_X_MAX_STEP           512000L
#define REMOTE_Y_MAX_STEP           10000L
#define REMOTE_Z_MAX_STEP           57600L

typedef enum {
    REMOTE_COMMAND_MISSION_START = 0,
    REMOTE_COMMAND_MISSION_CANCEL,
    REMOTE_COMMAND_AXIS_MOVE,
    REMOTE_COMMAND_AXIS_STOP,
    REMOTE_COMMAND_AXIS_STATUS,
    REMOTE_COMMAND_EMERGENCY_STOP,
    REMOTE_COMMAND_PAYLOAD_SET,
    REMOTE_COMMAND_STATUS_QUERY,
    REMOTE_COMMAND_CONFIG_QUERY,
    REMOTE_COMMAND_Z_SET_ZERO,
    REMOTE_COMMAND_Z_CLEAR_FAULT
} RemoteCommandKind;

typedef enum {
    REMOTE_RECORD_QUEUED = 0,
    REMOTE_RECORD_RUNNING,
    REMOTE_RECORD_COMPLETED,
    REMOTE_RECORD_FAILED,
    REMOTE_RECORD_REJECTED
} RemoteRecordState;

typedef struct {
    uint8_t record_index;
    uint32_t generation;
} RemoteRequest;

typedef struct {
    uint8_t valid;
    uint8_t event_pending;
    uint8_t payload_held;
    uint8_t axis;
    uint8_t release_manual;
    RemoteCommandKind kind;
    RemoteRecordState state;
    MissionTaskName task;
    MissionFailure failure;
    int32_t delta_pulses;
    uint32_t generation;
    uint32_t sequence;
    uint32_t completed_at_start;
    uint32_t failed_at_start;
    char command_id[REMOTE_COMMAND_ID_SIZE];
    const char *reason;
} RemoteCommandRecord;

static osMessageQueueId_t g_request_queue;
static osMutexId_t g_record_mutex;
static RemoteCommandRecord g_records[REMOTE_COMMAND_HISTORY];
static uint32_t g_record_generation;
static uint32_t g_record_sequence;
static uint32_t g_message_sequence;
static int8_t g_active_mission_record;
static int8_t g_active_axis_record;
static int8_t g_peeked_event_record;
static uint32_t g_peeked_event_generation;
static uint8_t g_status_requested;
static uint8_t g_status_config_task;
static jsmntok_t g_tokens[REMOTE_JSON_TOKENS];

static void copy_text(char *destination, size_t size, const char *source)
{
    size_t length;
    if ((destination == NULL) || (size == 0U)) return;
    if (source == NULL) source = "";
    length = strlen(source);
    if (length >= size) length = size - 1U;
    memcpy(destination, source, length);
    destination[length] = '\0';
}

static void format_message_fields(char *msg_id, size_t msg_size,
                                  char *timestamp, size_t timestamp_size,
                                  uint32_t now)
{
    uint32_t sequence = ++g_message_sequence;
    if (sequence == 0U) sequence = ++g_message_sequence;
    (void)snprintf(msg_id, msg_size, "g-%08lX-%08lX",
                   (unsigned long)now, (unsigned long)sequence);
    (void)snprintf(timestamp, timestamp_size, "uptime:%lu",
                   (unsigned long)now);
}

static uint8_t token_equals(const char *json, const jsmntok_t *token,
                            const char *text)
{
    size_t length = strlen(text);
    return ((token != NULL) && (token->type == JSMN_STRING) &&
            ((size_t)(token->end - token->start) == length) &&
            (memcmp(&json[token->start], text, length) == 0)) ? 1U : 0U;
}

static uint8_t token_copy_safe(const char *json, const jsmntok_t *token,
                               char *destination, size_t size)
{
    size_t length;
    if ((token == NULL) || (token->type != JSMN_STRING) ||
        (destination == NULL) || (size == 0U) ||
        (token->end <= token->start)) return 0U;
    length = (size_t)(token->end - token->start);
    if (length >= size) return 0U;
    for (size_t i = 0U; i < length; ++i) {
        unsigned char c = (unsigned char)json[token->start + (int)i];
        if ((c == '\\') || (c < 0x20U) || (c > 0x7EU)) return 0U;
    }
    memcpy(destination, &json[token->start], length);
    destination[length] = '\0';
    return 1U;
}

static uint8_t token_is_nonempty_string(const jsmntok_t *token)
{
    return ((token != NULL) && (token->type == JSMN_STRING) &&
            (token->end > token->start)) ? 1U : 0U;
}

static uint8_t token_to_int32(const char *json, const jsmntok_t *token,
                              int32_t *value)
{
    char text[24];
    char *end;
    long parsed;
    size_t length;
    if ((token == NULL) || (token->type != JSMN_PRIMITIVE) ||
        (value == NULL) || (token->end <= token->start)) return 0U;
    length = (size_t)(token->end - token->start);
    if (length >= sizeof(text)) return 0U;
    memcpy(text, &json[token->start], length);
    text[length] = '\0';
    parsed = strtol(text, &end, 10);
    if ((*end != '\0') || (parsed < INT32_MIN) || (parsed > INT32_MAX))
        return 0U;
    *value = (int32_t)parsed;
    return 1U;
}

static int token_after(const jsmntok_t *tokens, int count, int index)
{
    int next = index + 1;
    while ((next < count) && (tokens[next].start < tokens[index].end)) ++next;
    return next;
}

static int object_value(const char *json, const jsmntok_t *tokens, int count,
                        int object_index, const char *key)
{
    int index;
    if ((object_index < 0) || (object_index >= count) ||
        (tokens[object_index].type != JSMN_OBJECT)) return -1;
    index = object_index + 1;
    for (int pair = 0; pair < tokens[object_index].size; ++pair) {
        int value = index + 1;
        if ((value >= count) || (tokens[index].type != JSMN_STRING)) return -1;
        if (token_equals(json, &tokens[index], key) != 0U) return value;
        index = token_after(tokens, count, value);
    }
    return -1;
}

static uint8_t command_id_valid(const char *id)
{
    size_t length = strlen(id);
    if (length == 0U) return 0U;
    for (size_t i = 0U; i < length; ++i) {
        char c = id[i];
        if (!(((c >= 'a') && (c <= 'z')) ||
              ((c >= 'A') && (c <= 'Z')) ||
              ((c >= '0') && (c <= '9')) ||
              (c == '-') || (c == '_') || (c == '.') || (c == ':')))
            return 0U;
    }
    return 1U;
}

static uint8_t parse_task(const char *name, MissionTaskName *task)
{
    if (strcmp(name, "red_pick") == 0) *task = MISSION_TASK_RED_PICK;
    else if (strcmp(name, "tag_put") == 0) *task = MISSION_TASK_TAG_PUT;
    else if (strcmp(name, "red_find") == 0) *task = MISSION_TASK_RED_FIND;
    else if (strcmp(name, "frame_put") == 0) *task = MISSION_TASK_FRAME_PUT;
    else return 0U;
    return 1U;
}

static uint8_t parse_axis(const char *name, uint8_t *axis)
{
    if (strcmp(name, "x") == 0) *axis = XY_AXIS_X;
    else if (strcmp(name, "y") == 0) *axis = XY_AXIS_Y;
    else if (strcmp(name, "z") == 0) *axis = 2U;
    else return 0U;
    return 1U;
}

static uint8_t parse_kind(const char *name, RemoteCommandKind *kind)
{
    static const char *const names[] = {
        "mission_start", "mission_cancel", "axis_move", "axis_stop",
        "axis_status", "emergency_stop", "payload_set", "status_query",
        "config_query", "z_set_zero", "z_clear_fault"
    };
    for (uint8_t i = 0U; i < (sizeof(names) / sizeof(names[0])); ++i) {
        if (strcmp(name, names[i]) == 0) {
            *kind = (RemoteCommandKind)i;
            return 1U;
        }
    }
    return 0U;
}

static const char *parse_target(const char *json, int count, int target_token,
                                RemoteCommandRecord *record)
{
    int token;
    char value[24];
    if ((target_token < 0) || (g_tokens[target_token].type != JSMN_OBJECT))
        return "INVALID_TARGET";
    record->axis = REMOTE_AXIS_NONE;
    switch (record->kind) {
    case REMOTE_COMMAND_MISSION_START:
        if (g_tokens[target_token].size != 1) return "INVALID_TARGET";
        token = object_value(json, g_tokens, count, target_token, "task");
        if ((token_copy_safe(json, (token >= 0) ? &g_tokens[token] : NULL,
                             value, sizeof(value)) == 0U) ||
            (parse_task(value, &record->task) == 0U)) return "INVALID_TASK";
        break;
    case REMOTE_COMMAND_AXIS_MOVE:
        if (g_tokens[target_token].size != 2) return "INVALID_TARGET";
        token = object_value(json, g_tokens, count, target_token, "axis");
        if ((token_copy_safe(json, (token >= 0) ? &g_tokens[token] : NULL,
                             value, sizeof(value)) == 0U) ||
            (parse_axis(value, &record->axis) == 0U)) return "INVALID_AXIS";
        token = object_value(json, g_tokens, count, target_token,
                             "delta_pulses");
        if (token_to_int32(json, (token >= 0) ? &g_tokens[token] : NULL,
                           &record->delta_pulses) == 0U)
            return "INVALID_DELTA";
        break;
    case REMOTE_COMMAND_AXIS_STOP:
        if (g_tokens[target_token].size != 1) return "INVALID_TARGET";
        token = object_value(json, g_tokens, count, target_token, "axis");
        if ((token_copy_safe(json, (token >= 0) ? &g_tokens[token] : NULL,
                             value, sizeof(value)) == 0U) ||
            (parse_axis(value, &record->axis) == 0U)) return "INVALID_AXIS";
        break;
    case REMOTE_COMMAND_AXIS_STATUS:
        if (g_tokens[target_token].size > 1) return "INVALID_TARGET";
        if (g_tokens[target_token].size == 1) {
            token = object_value(json, g_tokens, count, target_token, "axis");
            if ((token_copy_safe(json, (token >= 0) ? &g_tokens[token] : NULL,
                                 value, sizeof(value)) == 0U) ||
                (parse_axis(value, &record->axis) == 0U)) return "INVALID_AXIS";
        }
        break;
    case REMOTE_COMMAND_PAYLOAD_SET:
        if (g_tokens[target_token].size != 1) return "INVALID_TARGET";
        token = object_value(json, g_tokens, count, target_token, "state");
        if (token_copy_safe(json, (token >= 0) ? &g_tokens[token] : NULL,
                            value, sizeof(value)) == 0U)
            return "INVALID_PAYLOAD";
        if (strcmp(value, "held") == 0) record->payload_held = 1U;
        else if (strcmp(value, "empty") == 0) record->payload_held = 0U;
        else return "INVALID_PAYLOAD";
        break;
    case REMOTE_COMMAND_CONFIG_QUERY:
        record->task = MISSION_TASK_COUNT;
        if (g_tokens[target_token].size > 1) return "INVALID_TARGET";
        if (g_tokens[target_token].size == 1) {
            token = object_value(json, g_tokens, count, target_token, "task");
            if ((token_copy_safe(json, (token >= 0) ? &g_tokens[token] : NULL,
                                 value, sizeof(value)) == 0U) ||
                (parse_task(value, &record->task) == 0U)) return "INVALID_TASK";
        }
        break;
    default:
        if (g_tokens[target_token].size != 0) return "INVALID_TARGET";
        break;
    }
    return NULL;
}

static int find_record(const char *command_id)
{
    for (uint8_t i = 0U; i < REMOTE_COMMAND_HISTORY; ++i) {
        if ((g_records[i].valid != 0U) &&
            (strcmp(g_records[i].command_id, command_id) == 0)) return i;
    }
    return -1;
}

static int allocate_record(void)
{
    int candidate = -1;
    uint32_t oldest = UINT32_MAX;
    for (uint8_t i = 0U; i < REMOTE_COMMAND_HISTORY; ++i) {
        if (g_records[i].valid == 0U) return i;
        if ((g_records[i].state != REMOTE_RECORD_QUEUED) &&
            (g_records[i].state != REMOTE_RECORD_RUNNING) &&
            (g_records[i].sequence < oldest)) {
            oldest = g_records[i].sequence;
            candidate = i;
        }
    }
    return candidate;
}

static uint8_t records_match(const RemoteCommandRecord *left,
                             const RemoteCommandRecord *right)
{
    return ((left->kind == right->kind) && (left->task == right->task) &&
            (left->axis == right->axis) &&
            (left->delta_pulses == right->delta_pulses) &&
            (left->payload_held == right->payload_held)) ? 1U : 0U;
}

static uint8_t mission_config_valid(MissionTaskName task, int32_t *detail)
{
    MissionTaskConfig config;
    int32_t missing = 0;
    MissionTask_GetConfig(task, &config);
    if (config.align_configured == 0U) missing |= 1;
    if ((task != MISSION_TASK_FRAME_PUT) &&
        (config.blind_configured == 0U)) missing |= 2;
    if ((task == MISSION_TASK_FRAME_PUT) &&
        (config.z_configured == 0U)) missing |= 4;
    if ((task == MISSION_TASK_TAG_PUT) && (config.z_enabled != 0U) &&
        (config.z_configured == 0U)) missing |= 4;
    *detail = missing;
    return (missing == 0) ? 1U : 0U;
}

static const char *precheck_command(RemoteCommandRecord *record, uint32_t now)
{
    MotionCoordinatorStatus motion;
    MissionTaskStatus mission;
    MotionPositionSnapshot pose;
    XY_AxisStatus xy;
    ZAxisControlStatus z;
    int64_t target;
    int32_t detail;
    MotionCoordinator_GetStatus(&motion);
    MissionTask_GetStatus(&mission);
    if ((record->kind != REMOTE_COMMAND_EMERGENCY_STOP) &&
        (motion.latch_reason != MOTION_LATCH_NONE)) return "GLOBAL_ABORT";
    switch (record->kind) {
    case REMOTE_COMMAND_MISSION_START:
        if ((MissionTask_IsActive() != 0U) ||
            (motion.owner != MOTION_OWNER_NONE)) return "BUSY";
        if (mission_config_valid(record->task, &detail) == 0U) {
            record->failure.detail = detail;
            return "CONFIG_MISSING";
        }
        if (MotionCoordinator_CaptureSnapshot(&pose, 1U, now) == 0U)
            return "POSITION_INVALID";
        if (((record->task == MISSION_TASK_TAG_PUT) ||
             (record->task == MISSION_TASK_FRAME_PUT)) &&
            (mission.payload != MISSION_PAYLOAD_HELD)) return "PAYLOAD_REQUIRED";
        break;
    case REMOTE_COMMAND_AXIS_MOVE:
        if ((record->delta_pulses == 0) ||
            ((record->axis == XY_AXIS_X) &&
             (((int64_t)record->delta_pulses > REMOTE_X_MAX_STEP) ||
              ((int64_t)record->delta_pulses < -REMOTE_X_MAX_STEP))) ||
            ((record->axis == XY_AXIS_Y) &&
             (((int64_t)record->delta_pulses > REMOTE_Y_MAX_STEP) ||
              ((int64_t)record->delta_pulses < -REMOTE_Y_MAX_STEP))) ||
            ((record->axis == 2U) &&
             (((int64_t)record->delta_pulses > REMOTE_Z_MAX_STEP) ||
              ((int64_t)record->delta_pulses < -REMOTE_Z_MAX_STEP))))
            return "STEP_TOO_LARGE";
        if ((motion.owner != MOTION_OWNER_NONE) ||
            (MissionTask_IsActive() != 0U) || (g_active_axis_record >= 0))
            return "BUSY";
        if (record->axis < 2U) {
            const XY_AxisConfig *config =
                XY_GetConfig((XY_Axis)record->axis);
            (void)XY_GetStatus((XY_Axis)record->axis, &xy);
            if ((xy.position_valid == 0U) || (xy.state != XY_STATE_IDLE) ||
                (xy.fault != XY_FAULT_NONE)) return "AXIS_NOT_READY";
            target = (int64_t)xy.position_pulses + record->delta_pulses;
            if ((target < config->soft_min_pulses) ||
                (target > config->soft_max_pulses)) return "SOFT_LIMIT";
        } else {
            ZAxis_GetControlStatus(&z);
            if ((z.position_valid == 0U) || (z.state != Z_STATE_IDLE) ||
                (z.fault != Z_FAULT_NONE)) return "AXIS_NOT_READY";
            target = (int64_t)z.position_pulses + record->delta_pulses;
            if ((target < Z_AXIS_SOFT_MIN_PULSES) ||
                (target > Z_AXIS_SOFT_MAX_PULSES)) return "SOFT_LIMIT";
        }
        break;
    case REMOTE_COMMAND_PAYLOAD_SET:
        if (MissionTask_IsActive() != 0U) return "BUSY";
        break;
    case REMOTE_COMMAND_Z_SET_ZERO:
        ZAxis_GetControlStatus(&z);
        if ((motion.owner != MOTION_OWNER_NONE) ||
            (MissionTask_IsActive() != 0U)) return "BUSY";
        if ((z.state != Z_STATE_IDLE) &&
            (z.state != Z_STATE_UNREFERENCED)) return "AXIS_NOT_READY";
        break;
    case REMOTE_COMMAND_Z_CLEAR_FAULT:
        if ((motion.owner != MOTION_OWNER_NONE) ||
            (MissionTask_IsActive() != 0U)) return "BUSY";
        break;
    default:
        break;
    }
    return NULL;
}

static void format_ack(const RemoteCommandRecord *record, char *response,
                       size_t size, uint32_t now)
{
    char msg_id[32];
    char timestamp[32];
    format_message_fields(msg_id, sizeof(msg_id), timestamp,
                          sizeof(timestamp), now);
    if (record->state == REMOTE_RECORD_REJECTED) {
        (void)snprintf(response, size,
            "{\"type\":\"ack\",\"device_id\":\"%s\"," 
            "\"msg_id\":\"%s\",\"timestamp\":\"%s\"," 
            "\"payload\":{\"command_id\":\"%s\"," 
            "\"status\":\"rejected\",\"reason\":\"%s\"," 
            "\"detail\":%ld}}",
            NETWORK_DEVICE_ID, msg_id, timestamp, record->command_id,
            (record->reason != NULL) ? record->reason : "REJECTED",
            (long)record->failure.detail);
    } else {
        (void)snprintf(response, size,
            "{\"type\":\"ack\",\"device_id\":\"%s\"," 
            "\"msg_id\":\"%s\",\"timestamp\":\"%s\"," 
            "\"payload\":{\"command_id\":\"%s\"," 
            "\"status\":\"accepted\"}}",
            NETWORK_DEVICE_ID, msg_id, timestamp, record->command_id);
    }
}

static void format_event(const RemoteCommandRecord *record, char *response,
                         size_t size, uint32_t now)
{
    char msg_id[32];
    char timestamp[32];
    char failure[64];
    format_message_fields(msg_id, sizeof(msg_id), timestamp,
                          sizeof(timestamp), now);
    if (record->state == REMOTE_RECORD_COMPLETED) {
        (void)snprintf(response, size,
            "{\"type\":\"event\",\"device_id\":\"%s\"," 
            "\"msg_id\":\"%s\",\"timestamp\":\"%s\"," 
            "\"payload\":{\"command_id\":\"%s\"," 
            "\"status\":\"completed\"}}",
            NETWORK_DEVICE_ID, msg_id, timestamp, record->command_id);
    } else {
        if (record->reason != NULL) {
            copy_text(failure, sizeof(failure), record->reason);
        } else {
            (void)snprintf(failure, sizeof(failure), "%s:%s",
                MissionSubflow_SourceString(record->failure.source),
                MissionSubflow_ReasonString(record->failure.reason));
        }
        (void)snprintf(response, size,
            "{\"type\":\"event\",\"device_id\":\"%s\"," 
            "\"msg_id\":\"%s\",\"timestamp\":\"%s\"," 
            "\"payload\":{\"command_id\":\"%s\"," 
            "\"status\":\"failed\",\"reason\":\"%s\"," 
            "\"detail\":%ld}}",
            NETWORK_DEVICE_ID, msg_id, timestamp, record->command_id,
            failure, (long)record->failure.detail);
    }
}

void RemoteControl_FormatHello(char *response, size_t size, uint32_t now)
{
    char msg_id[32];
    char timestamp[32];
    format_message_fields(msg_id, sizeof(msg_id), timestamp,
                          sizeof(timestamp), now);
    (void)snprintf(response, size,
        "{\"type\":\"hello\",\"device_id\":\"%s\"," 
        "\"msg_id\":\"%s\",\"timestamp\":\"%s\",\"payload\":{" 
        "\"protocol_version\":%u,\"firmware\":\"TestH743\"," 
        "\"role\":\"gantry\",\"max_frame_bytes\":%u," 
        "\"capabilities\":[\"mission_start\",\"mission_cancel\"," 
        "\"axis_move\",\"axis_stop\",\"axis_status\"," 
        "\"emergency_stop\",\"payload_set\",\"status_query\"," 
        "\"config_query\",\"z_set_zero\",\"z_clear_fault\"]," 
        "\"tasks\":[\"red_pick\",\"red_find\",\"tag_put\"," 
        "\"frame_put\"],\"axes\":[\"x\",\"y\",\"z\"]}}",
        NETWORK_DEVICE_ID, msg_id, timestamp,
        (unsigned int)NETWORK_PROTOCOL_VERSION,
        (unsigned int)REMOTE_JSON_LINE_SIZE);
}

void RemoteControl_FormatHeartbeat(char *response, size_t size, uint32_t now)
{
    char msg_id[32];
    char timestamp[32];
    format_message_fields(msg_id, sizeof(msg_id), timestamp,
                          sizeof(timestamp), now);
    (void)snprintf(response, size,
        "{\"type\":\"heartbeat\",\"device_id\":\"%s\"," 
        "\"msg_id\":\"%s\",\"timestamp\":\"%s\"," 
        "\"payload\":{\"status\":\"online\",\"uptime_ms\":%lu}}",
        NETWORK_DEVICE_ID, msg_id, timestamp, (unsigned long)now);
}

void RemoteControl_FormatStatus(char *response, size_t size,
                                uint8_t config_task, uint32_t now)
{
    char msg_id[32];
    char timestamp[32];
    MissionTaskStatus mission;
    MissionTaskConfig config;
    MotionCoordinatorStatus motion;
    XY_AxisStatus x;
    XY_AxisStatus y;
    ZAxisControlStatus z;
    C552_Data data;
    C552_Health health;
    uint8_t has_c552;
    MissionTaskName task;
    char mission_command_id[REMOTE_COMMAND_ID_SIZE] = "";
    char x_command_id[REMOTE_COMMAND_ID_SIZE] = "";
    char y_command_id[REMOTE_COMMAND_ID_SIZE] = "";
    char z_command_id[REMOTE_COMMAND_ID_SIZE] = "";
    MissionTask_GetStatus(&mission);
    MotionCoordinator_GetStatus(&motion);
    (void)XY_GetStatus(XY_AXIS_X, &x);
    (void)XY_GetStatus(XY_AXIS_Y, &y);
    ZAxis_GetControlStatus(&z);
    memset(&data, 0, sizeof(data));
    memset(&health, 0, sizeof(health));
    has_c552 = C552_GetSnapshot(&data, &health);
    task = (config_task < MISSION_TASK_COUNT) ?
           (MissionTaskName)config_task : mission.task;
    MissionTask_GetConfig(task, &config);
    (void)osMutexAcquire(g_record_mutex, osWaitForever);
    if ((g_active_mission_record >= 0) &&
        (g_active_mission_record < (int8_t)REMOTE_COMMAND_HISTORY)) {
        copy_text(mission_command_id, sizeof(mission_command_id),
                  g_records[g_active_mission_record].command_id);
    }
    if ((g_active_axis_record >= 0) &&
        (g_active_axis_record < (int8_t)REMOTE_COMMAND_HISTORY)) {
        RemoteCommandRecord *active = &g_records[g_active_axis_record];
        char *destination = (active->axis == XY_AXIS_X) ? x_command_id :
                            (active->axis == XY_AXIS_Y) ? y_command_id :
                                                        z_command_id;
        copy_text(destination, REMOTE_COMMAND_ID_SIZE, active->command_id);
    }
    (void)osMutexRelease(g_record_mutex);
    format_message_fields(msg_id, sizeof(msg_id), timestamp,
                          sizeof(timestamp), now);
    (void)snprintf(response, size,
        "{\"type\":\"status\",\"device_id\":\"%s\"," 
        "\"msg_id\":\"%s\",\"timestamp\":\"%s\",\"payload\":{" 
        "\"system\":{\"state\":\"%s\",\"uptime_ms\":%lu," 
        "\"firmware\":\"TestH743\"},"
        "\"network\":{\"link\":\"up\",\"ip\":\"%u.%u.%u.%u\"," 
        "\"center\":\"online\"},"
        "\"motion\":{\"owner\":\"%s\",\"latch\":\"%s\"," 
        "\"stop_pending\":%u},"
        "\"mission\":{\"active\":%s,\"command_id\":\"%s\"," 
        "\"task\":\"%s\"," 
        "\"phase\":\"%s\",\"attempt\":%u,\"z_recovery\":%u," 
        "\"restart\":%u,\"elapsed_ms\":%lu," 
        "\"fault\":{\"source\":\"%s\",\"reason\":\"%s\"," 
        "\"detail\":%ld}},"
        "\"axes\":{" 
        "\"x\":{\"state\":\"%s\",\"position\":%ld,\"target\":%ld," 
        "\"valid\":%s,\"fault\":\"%s\",\"completion\":\"%s\"," 
        "\"active_command_id\":\"%s\"},"
        "\"y\":{\"state\":\"%s\",\"position\":%ld,\"target\":%ld," 
        "\"valid\":%s,\"fault\":\"%s\",\"completion\":\"%s\"," 
        "\"active_command_id\":\"%s\"},"
        "\"z\":{\"state\":\"%s\",\"position\":%ld,\"target\":%ld," 
        "\"valid\":%s,\"fault\":\"%s\"," 
        "\"active_command_id\":\"%s\"}},"
        "\"payload_state\":\"%s\"," 
        "\"devices\":{\"required_mask\":%u,\"healthy_mask\":%u," 
        "\"c552_online\":%s},"
        "\"storage\":{\"state\":\"%s\",\"generation\":%lu},"
        "\"config\":{\"task\":\"%s\",\"align_configured\":%u," 
        "\"blind_configured\":%u,\"z_enabled\":%u," 
        "\"z_configured\":%u,\"preset\":[%ld,%ld,%ld]," 
        "\"safe\":[%ld,%ld,%ld]}}}",
        NETWORK_DEVICE_ID, msg_id, timestamp,
        (motion.latch_reason == MOTION_LATCH_NONE) ? "ready" : "aborted",
        (unsigned long)now, NETWORK_IP_0, NETWORK_IP_1, NETWORK_IP_2,
        NETWORK_IP_3, MotionCoordinator_OwnerString(motion.owner),
        MotionCoordinator_LatchString(motion.latch_reason),
        (unsigned int)motion.stop_pending,
        (MissionTask_IsActive() != 0U) ? "true" : "false",
        mission_command_id,
        MissionSubflow_TaskString(mission.task),
        MissionTask_StateString(mission.state),
        (unsigned int)mission.subflow_attempt,
        (unsigned int)mission.axis_recoveries,
        (unsigned int)mission.restart_count,
        (unsigned long)(now - mission.start_tick),
        MissionSubflow_SourceString(mission.failure.source),
        MissionSubflow_ReasonString(mission.failure.reason),
        (long)mission.failure.detail,
        XY_StateString(x.state), (long)x.position_pulses,
        (long)x.target_pulses, x.position_valid ? "true" : "false",
        XY_FaultString(x.fault), XY_CompletionSourceString(x.completion_source),
        x_command_id,
        XY_StateString(y.state), (long)y.position_pulses,
        (long)y.target_pulses, y.position_valid ? "true" : "false",
        XY_FaultString(y.fault), XY_CompletionSourceString(y.completion_source),
        y_command_id,
        ZAxis_StateString(z.state), (long)z.position_pulses,
        (long)z.target_pulses, z.position_valid ? "true" : "false",
        ZAxis_FaultString(z.fault),
        z_command_id,
        MissionTask_PayloadString(mission.payload),
        (unsigned int)motion.required_mask, (unsigned int)health.ready_mask,
        (has_c552 != 0U) ? "true" : "false",
        MissionTask_StorageString(mission.storage_state),
        (unsigned long)mission.storage_generation,
        MissionSubflow_TaskString(task),
        (unsigned int)config.align_configured,
        (unsigned int)config.blind_configured,
        (unsigned int)config.z_enabled, (unsigned int)config.z_configured,
        (long)config.preset_pose.x_pulses,
        (long)config.preset_pose.y_pulses,
        (long)config.preset_pose.z_pulses,
        (long)config.safe_pose.x_pulses,
        (long)config.safe_pose.y_pulses,
        (long)config.safe_pose.z_pulses);
}

void RemoteControl_FormatError(char *response, size_t size,
                               const char *reason, uint32_t now)
{
    char msg_id[32];
    char timestamp[32];
    format_message_fields(msg_id, sizeof(msg_id), timestamp,
                          sizeof(timestamp), now);
    (void)snprintf(response, size,
        "{\"type\":\"error\",\"device_id\":\"%s\"," 
        "\"msg_id\":\"%s\",\"timestamp\":\"%s\"," 
        "\"payload\":{\"reason\":\"%s\"}}",
        NETWORK_DEVICE_ID, msg_id, timestamp,
        (reason != NULL) ? reason : "invalid_message");
}

void RemoteControl_Init(void)
{
    memset(g_records, 0, sizeof(g_records));
    g_record_generation = 0U;
    g_record_sequence = 0U;
    g_message_sequence = 0U;
    g_active_mission_record = -1;
    g_active_axis_record = -1;
    g_peeked_event_record = -1;
    g_peeked_event_generation = 0U;
    g_status_requested = 0U;
    g_status_config_task = REMOTE_CONFIG_CURRENT;
    g_request_queue = osMessageQueueNew(REMOTE_REQUEST_QUEUE_DEPTH,
                                         sizeof(RemoteRequest), NULL);
    g_record_mutex = osMutexNew(NULL);
    if ((g_request_queue == NULL) || (g_record_mutex == NULL)) Error_Handler();
}

RemoteSubmitResult RemoteControl_SubmitJson(const char *json, char *response,
                                            size_t response_size,
                                            uint32_t now)
{
    jsmn_parser parser;
    int count;
    int type_token;
    int device_token;
    int msg_token;
    int timestamp_token;
    int payload_token;
    int command_id_token;
    int name_token;
    int target_token;
    char type[16];
    char device_id[16];
    char command_id[REMOTE_COMMAND_ID_SIZE];
    char name[32];
    RemoteCommandRecord incoming;
    int record_index;
    const char *reject_reason;
    RemoteRequest request;

    if ((json == NULL) || (response == NULL) || (response_size == 0U))
        return REMOTE_SUBMIT_INVALID;
    jsmn_init(&parser);
    count = jsmn_parse(&parser, json, strlen(json), g_tokens,
                       REMOTE_JSON_TOKENS);
    if ((count <= 0) || (g_tokens[0].type != JSMN_OBJECT) ||
        (token_after(g_tokens, count, 0) != count)) {
        RemoteControl_FormatError(response, response_size, "invalid_json", now);
        return REMOTE_SUBMIT_RESPONSE;
    }
    type_token = object_value(json, g_tokens, count, 0, "type");
    device_token = object_value(json, g_tokens, count, 0, "device_id");
    msg_token = object_value(json, g_tokens, count, 0, "msg_id");
    timestamp_token = object_value(json, g_tokens, count, 0, "timestamp");
    payload_token = object_value(json, g_tokens, count, 0, "payload");
    if ((token_copy_safe(json, (type_token >= 0) ? &g_tokens[type_token] : NULL,
                         type, sizeof(type)) == 0U) ||
        (token_copy_safe(json, (device_token >= 0) ? &g_tokens[device_token] : NULL,
                         device_id, sizeof(device_id)) == 0U) ||
        (token_is_nonempty_string((msg_token >= 0) ?
                                  &g_tokens[msg_token] : NULL) == 0U) ||
        (token_is_nonempty_string((timestamp_token >= 0) ?
                                  &g_tokens[timestamp_token] : NULL) == 0U) ||
        (payload_token < 0) ||
        (g_tokens[payload_token].type != JSMN_OBJECT)) {
        RemoteControl_FormatError(response, response_size,
                                  "missing_required_field", now);
        return REMOTE_SUBMIT_RESPONSE;
    }
    if (strcmp(device_id, NETWORK_CENTER_ID) != 0) {
        RemoteControl_FormatError(response, response_size,
                                  "device_id_mismatch", now);
        return REMOTE_SUBMIT_RESPONSE;
    }
    if (strcmp(type, "heartbeat") == 0) return REMOTE_SUBMIT_NO_RESPONSE;
    if (strcmp(type, "command") != 0) {
        RemoteControl_FormatError(response, response_size,
                                  "unsupported_message_type", now);
        return REMOTE_SUBMIT_RESPONSE;
    }

    command_id_token = object_value(json, g_tokens, count, payload_token,
                                    "command_id");
    name_token = object_value(json, g_tokens, count, payload_token, "name");
    target_token = object_value(json, g_tokens, count, payload_token, "target");
    if ((token_copy_safe(json, (command_id_token >= 0) ?
                         &g_tokens[command_id_token] : NULL,
                         command_id, sizeof(command_id)) == 0U) ||
        (command_id_valid(command_id) == 0U) ||
        (token_copy_safe(json, (name_token >= 0) ? &g_tokens[name_token] : NULL,
                         name, sizeof(name)) == 0U)) {
        RemoteControl_FormatError(response, response_size,
                                  "invalid_command_fields", now);
        return REMOTE_SUBMIT_RESPONSE;
    }
    memset(&incoming, 0, sizeof(incoming));
    incoming.axis = REMOTE_AXIS_NONE;
    incoming.task = MISSION_TASK_COUNT;
    copy_text(incoming.command_id, sizeof(incoming.command_id), command_id);
    if (parse_kind(name, &incoming.kind) == 0U) {
        incoming.state = REMOTE_RECORD_REJECTED;
        incoming.reason = "UNSUPPORTED_COMMAND";
        format_ack(&incoming, response, response_size, now);
        return REMOTE_SUBMIT_RESPONSE;
    }
    reject_reason = parse_target(json, count, target_token, &incoming);
    if (reject_reason != NULL) {
        incoming.state = REMOTE_RECORD_REJECTED;
        incoming.reason = reject_reason;
        format_ack(&incoming, response, response_size, now);
        return REMOTE_SUBMIT_RESPONSE;
    }

    (void)osMutexAcquire(g_record_mutex, osWaitForever);
    record_index = find_record(command_id);
    if (record_index >= 0) {
        RemoteCommandRecord *record = &g_records[record_index];
        if (records_match(record, &incoming) == 0U) {
            RemoteCommandRecord conflict = *record;
            conflict.state = REMOTE_RECORD_REJECTED;
            conflict.reason = "command_id_conflict";
            format_ack(&conflict, response, response_size, now);
        } else {
            format_ack(record, response, response_size, now);
            if ((record->state == REMOTE_RECORD_COMPLETED) ||
                (record->state == REMOTE_RECORD_FAILED))
                record->event_pending = 1U;
        }
        (void)osMutexRelease(g_record_mutex);
        return REMOTE_SUBMIT_RESPONSE;
    }
    record_index = allocate_record();
    if (record_index < 0) {
        (void)osMutexRelease(g_record_mutex);
        RemoteControl_FormatError(response, response_size,
                                  "command_history_busy", now);
        return REMOTE_SUBMIT_RESPONSE;
    }
    reject_reason = precheck_command(&incoming, now);
    incoming.valid = 1U;
    incoming.generation = ++g_record_generation;
    incoming.sequence = ++g_record_sequence;
    if (reject_reason != NULL) {
        incoming.state = REMOTE_RECORD_REJECTED;
        incoming.reason = reject_reason;
        g_records[record_index] = incoming;
        format_ack(&g_records[record_index], response, response_size, now);
        (void)osMutexRelease(g_record_mutex);
        return REMOTE_SUBMIT_RESPONSE;
    }

    g_records[record_index] = incoming;
    if (incoming.kind == REMOTE_COMMAND_EMERGENCY_STOP) {
        g_records[record_index].state = REMOTE_RECORD_RUNNING;
        MotionCoordinator_RequestAbort();
    } else if (incoming.kind == REMOTE_COMMAND_AXIS_STOP) {
        g_records[record_index].state = REMOTE_RECORD_QUEUED;
    } else {
        request.record_index = (uint8_t)record_index;
        request.generation = g_records[record_index].generation;
        if (osMessageQueuePut(g_request_queue, &request, 0U, 0U) != osOK) {
            g_records[record_index].state = REMOTE_RECORD_REJECTED;
            g_records[record_index].reason = "COMMAND_QUEUE_FULL";
        } else {
            g_records[record_index].state = REMOTE_RECORD_QUEUED;
        }
    }
    format_ack(&g_records[record_index], response, response_size, now);
    (void)osMutexRelease(g_record_mutex);
    return REMOTE_SUBMIT_RESPONSE;
}

static void complete_record(RemoteCommandRecord *record, uint8_t success,
                            const char *reason)
{
    record->state = (success != 0U) ? REMOTE_RECORD_COMPLETED :
                                      REMOTE_RECORD_FAILED;
    record->reason = reason;
    record->event_pending = 1U;
}

static uint8_t axis_is_stopped(uint8_t axis)
{
    XY_AxisStatus xy;
    ZAxisControlStatus z;
    if (axis < 2U) {
        (void)XY_GetStatus((XY_Axis)axis, &xy);
        return ((xy.state != XY_STATE_STARTING) &&
                (xy.state != XY_STATE_MOVING) &&
                (xy.state != XY_STATE_STOPPING) &&
                (xy.state != XY_STATE_HOMING)) ? 1U : 0U;
    }
    ZAxis_GetControlStatus(&z);
    return ((z.state != Z_STATE_STARTING) && (z.state != Z_STATE_MOVING) &&
            (z.state != Z_STATE_STOPPING) &&
            (z.state != Z_STATE_RECOVERING)) ? 1U : 0U;
}

static void execute_request(const RemoteRequest *request, uint32_t now)
{
    RemoteCommandRecord snapshot;
    RemoteCommandRecord *record;
    MissionTaskStatus mission;
    uint8_t success = 0U;
    const char *reason = NULL;
    if (request->record_index >= REMOTE_COMMAND_HISTORY) return;
    (void)osMutexAcquire(g_record_mutex, osWaitForever);
    record = &g_records[request->record_index];
    if ((record->valid == 0U) ||
        (record->generation != request->generation) ||
        (record->state != REMOTE_RECORD_QUEUED)) {
        (void)osMutexRelease(g_record_mutex);
        return;
    }
    record->state = REMOTE_RECORD_RUNNING;
    snapshot = *record;
    (void)osMutexRelease(g_record_mutex);

    MissionTask_GetStatus(&mission);
    switch (snapshot.kind) {
    case REMOTE_COMMAND_MISSION_START:
        success = MissionTask_RequestStart(snapshot.task);
        break;
    case REMOTE_COMMAND_MISSION_CANCEL:
        MissionTask_RequestCancel();
        success = 1U;
        break;
    case REMOTE_COMMAND_AXIS_MOVE:
        if (MotionCoordinator_Acquire(MOTION_OWNER_MANUAL, 0U, now) == 0U) {
            reason = "BUSY";
            break;
        }
        MotionCoordinator_SetManualHold(1U);
        if (snapshot.axis < 2U) {
            const XY_AxisConfig *config = XY_GetConfig((XY_Axis)snapshot.axis);
            success = (XY_MoveRelative((XY_Axis)snapshot.axis,
                snapshot.delta_pulses, config->default_speed_rpm,
                config->acceleration) == XY_RESULT_OK) ? 1U : 0U;
        } else {
            success = (ZAxisControl_MoveRelative(snapshot.delta_pulses,
                Z_AXIS_DEFAULT_SPEED_HZ) == Z_RESULT_OK) ? 1U : 0U;
        }
        if (success == 0U) {
            MotionCoordinator_SetManualHold(0U);
            MotionCoordinator_Release(MOTION_OWNER_MANUAL, now);
            reason = "AXIS_MOVE_REJECTED";
        }
        break;
    case REMOTE_COMMAND_PAYLOAD_SET:
        success = MissionTask_SetPayload(snapshot.payload_held ?
            MISSION_PAYLOAD_HELD : MISSION_PAYLOAD_EMPTY);
        if (success == 0U) reason = "PAYLOAD_REJECTED";
        break;
    case REMOTE_COMMAND_AXIS_STATUS:
    case REMOTE_COMMAND_STATUS_QUERY:
        success = 1U;
        break;
    case REMOTE_COMMAND_CONFIG_QUERY:
        success = 1U;
        break;
    case REMOTE_COMMAND_Z_SET_ZERO:
        success = (ZAxisControl_SetZero() == Z_RESULT_OK) ? 1U : 0U;
        if (success == 0U) reason = "Z_SET_ZERO_REJECTED";
        break;
    case REMOTE_COMMAND_Z_CLEAR_FAULT: {
        ZAxisControlResult result = ZAxisControl_ClearFault();
        success = (result == Z_RESULT_OK) ? 1U : 0U;
        if (success == 0U) reason = "Z_CLEAR_FAULT_REJECTED";
        break;
    }
    default:
        reason = "INTERNAL_COMMAND_ERROR";
        break;
    }

    (void)osMutexAcquire(g_record_mutex, osWaitForever);
    record = &g_records[request->record_index];
    if ((record->valid == 0U) ||
        (record->generation != request->generation)) {
        (void)osMutexRelease(g_record_mutex);
        return;
    }
    if (snapshot.kind == REMOTE_COMMAND_MISSION_START) {
        if (success != 0U) {
            record->completed_at_start = mission.completed_count;
            record->failed_at_start = mission.failed_count;
            g_active_mission_record = (int8_t)request->record_index;
        } else {
            complete_record(record, 0U, "MISSION_START_REJECTED");
        }
    } else if (snapshot.kind == REMOTE_COMMAND_MISSION_CANCEL) {
        if (MissionTask_IsActive() == 0U) complete_record(record, 1U, NULL);
    } else if (snapshot.kind == REMOTE_COMMAND_AXIS_MOVE) {
        if (success != 0U) g_active_axis_record = (int8_t)request->record_index;
        else complete_record(record, 0U, reason);
    } else if (snapshot.kind == REMOTE_COMMAND_Z_CLEAR_FAULT) {
        if (success == 0U) complete_record(record, 0U, reason);
        else {
            ZAxisControlStatus z;
            ZAxis_GetControlStatus(&z);
            if ((z.state == Z_STATE_IDLE) && (z.fault == Z_FAULT_NONE))
                complete_record(record, 1U, NULL);
        }
    } else {
        complete_record(record, success, reason);
        if ((snapshot.kind == REMOTE_COMMAND_AXIS_STATUS) ||
            (snapshot.kind == REMOTE_COMMAND_STATUS_QUERY) ||
            (snapshot.kind == REMOTE_COMMAND_CONFIG_QUERY)) {
            g_status_requested = 1U;
            g_status_config_task =
                (snapshot.kind == REMOTE_COMMAND_CONFIG_QUERY) ?
                (uint8_t)snapshot.task : REMOTE_CONFIG_CURRENT;
        }
    }
    (void)osMutexRelease(g_record_mutex);
}

static void execute_axis_stop(uint8_t index, uint32_t now)
{
    RemoteCommandRecord snapshot;
    uint8_t issued;
    MotionOwner owner;
    (void)osMutexAcquire(g_record_mutex, osWaitForever);
    if ((g_records[index].valid == 0U) ||
        (g_records[index].state != REMOTE_RECORD_QUEUED) ||
        (g_records[index].kind != REMOTE_COMMAND_AXIS_STOP)) {
        (void)osMutexRelease(g_record_mutex);
        return;
    }
    g_records[index].state = REMOTE_RECORD_RUNNING;
    snapshot = g_records[index];
    (void)osMutexRelease(g_record_mutex);

    owner = MotionCoordinator_GetOwner();
    if (owner == MOTION_OWNER_MISSION) MissionTask_RequestCancel();
    else if ((owner != MOTION_OWNER_NONE) && (owner != MOTION_OWNER_MANUAL))
        MotionCoordinator_RequestCancel(owner);
    if (axis_is_stopped(snapshot.axis) != 0U) {
        issued = 1U;
    } else if (snapshot.axis < 2U) {
        issued = XY_Stop((XY_Axis)snapshot.axis);
    } else {
        issued = (ZAxisControl_Stop() == Z_RESULT_OK) ? 1U : 0U;
    }

    (void)osMutexAcquire(g_record_mutex, osWaitForever);
    if (issued == 0U) {
        complete_record(&g_records[index], 0U, "AXIS_STOP_REJECTED");
    } else {
        if ((g_active_axis_record >= 0) &&
            (g_records[g_active_axis_record].axis == snapshot.axis)) {
            complete_record(&g_records[g_active_axis_record], 0U,
                            "REMOTE_AXIS_STOP");
            g_records[index].release_manual = 1U;
            g_active_axis_record = -1;
        }
        if (axis_is_stopped(snapshot.axis) != 0U)
            complete_record(&g_records[index], 1U, NULL);
    }
    (void)osMutexRelease(g_record_mutex);
    (void)now;
}

static void poll_running_records(uint32_t now)
{
    MissionTaskStatus mission;
    MotionCoordinatorStatus motion;
    MissionTask_GetStatus(&mission);
    MotionCoordinator_GetStatus(&motion);
    (void)osMutexAcquire(g_record_mutex, osWaitForever);
    if ((g_active_mission_record >= 0) &&
        (g_active_mission_record < (int8_t)REMOTE_COMMAND_HISTORY)) {
        RemoteCommandRecord *record = &g_records[g_active_mission_record];
        if ((record->state == REMOTE_RECORD_RUNNING) &&
            (mission.start_pending == 0U) && (mission.task == record->task)) {
            if ((mission.state == MISSION_STATE_COMPLETE) &&
                (mission.completed_count != record->completed_at_start)) {
                complete_record(record, 1U, NULL);
                g_active_mission_record = -1;
            } else if ((mission.state == MISSION_STATE_FAULT) &&
                       (mission.failed_count != record->failed_at_start)) {
                record->failure = mission.failure;
                complete_record(record, 0U, NULL);
                g_active_mission_record = -1;
            }
        }
    }
    if ((g_active_axis_record >= 0) &&
        (g_active_axis_record < (int8_t)REMOTE_COMMAND_HISTORY)) {
        RemoteCommandRecord *record = &g_records[g_active_axis_record];
        if (record->state == REMOTE_RECORD_RUNNING) {
            if (record->axis < 2U) {
                XY_AxisStatus xy;
                (void)XY_GetStatus((XY_Axis)record->axis, &xy);
                if ((xy.state == XY_STATE_IDLE) &&
                    (xy.position_valid != 0U) && (xy.fault == XY_FAULT_NONE)) {
                    complete_record(record, 1U, NULL);
                } else if ((xy.state == XY_STATE_FAULT) ||
                           (xy.position_valid == 0U)) {
                    record->failure.detail = (int32_t)xy.fault;
                    complete_record(record, 0U, "AXIS_FAULT");
                }
            } else {
                ZAxisControlStatus z;
                ZAxis_GetControlStatus(&z);
                if ((z.state == Z_STATE_IDLE) && (z.position_valid != 0U) &&
                    (z.fault == Z_FAULT_NONE)) complete_record(record, 1U, NULL);
                else if ((z.state == Z_STATE_FAULT) ||
                         (z.position_valid == 0U)) {
                    record->failure.detail = (int32_t)z.fault;
                    complete_record(record, 0U, "AXIS_FAULT");
                }
            }
            if ((record->state == REMOTE_RECORD_COMPLETED) ||
                (record->state == REMOTE_RECORD_FAILED)) {
                MotionCoordinator_SetManualHold(0U);
                MotionCoordinator_Release(MOTION_OWNER_MANUAL, now);
                g_active_axis_record = -1;
            }
        }
    }
    for (uint8_t i = 0U; i < REMOTE_COMMAND_HISTORY; ++i) {
        RemoteCommandRecord *record = &g_records[i];
        if ((record->valid == 0U) ||
            (record->state != REMOTE_RECORD_RUNNING)) continue;
        if ((record->kind == REMOTE_COMMAND_MISSION_CANCEL) &&
            (MissionTask_IsActive() == 0U)) {
            complete_record(record, 1U, NULL);
        } else if ((record->kind == REMOTE_COMMAND_EMERGENCY_STOP) &&
                   (motion.latch_reason == MOTION_LATCH_ABORT) &&
                   (motion.stop_pending == 0U)) {
            complete_record(record, 1U, NULL);
        } else if (record->kind == REMOTE_COMMAND_AXIS_STOP) {
            if (axis_is_stopped(record->axis) != 0U) {
                if (record->release_manual != 0U) {
                    MotionCoordinator_SetManualHold(0U);
                    MotionCoordinator_Release(MOTION_OWNER_MANUAL, now);
                }
                complete_record(record, 1U, NULL);
            }
        } else if (record->kind == REMOTE_COMMAND_Z_CLEAR_FAULT) {
            ZAxisControlStatus z;
            ZAxis_GetControlStatus(&z);
            if ((z.state == Z_STATE_IDLE) && (z.fault == Z_FAULT_NONE))
                complete_record(record, 1U, NULL);
            else if (z.state == Z_STATE_FAULT) {
                record->failure.detail = (int32_t)z.fault;
                complete_record(record, 0U, "Z_CLEAR_FAULT_FAILED");
            }
        }
    }
    (void)osMutexRelease(g_record_mutex);
}

void RemoteControl_Poll(uint32_t now)
{
    RemoteRequest request;
    int8_t stop_record = -1;
    (void)osMutexAcquire(g_record_mutex, osWaitForever);
    for (uint8_t i = 0U; i < REMOTE_COMMAND_HISTORY; ++i) {
        if ((g_records[i].valid != 0U) &&
            (g_records[i].state == REMOTE_RECORD_QUEUED) &&
            (g_records[i].kind == REMOTE_COMMAND_AXIS_STOP)) {
            stop_record = (int8_t)i;
            break;
        }
    }
    (void)osMutexRelease(g_record_mutex);
    if (stop_record >= 0) execute_axis_stop((uint8_t)stop_record, now);
    if (osMessageQueueGet(g_request_queue, &request, NULL, 0U) == osOK)
        execute_request(&request, now);
    poll_running_records(now);
}

uint8_t RemoteControl_TakeStatusRequest(uint8_t *config_task)
{
    uint8_t pending;
    if (config_task == NULL) return 0U;
    (void)osMutexAcquire(g_record_mutex, osWaitForever);
    pending = g_status_requested;
    if (pending != 0U) {
        *config_task = g_status_config_task;
        g_status_requested = 0U;
        g_status_config_task = REMOTE_CONFIG_CURRENT;
    }
    (void)osMutexRelease(g_record_mutex);
    return pending;
}

uint8_t RemoteControl_PeekEvent(char *response, size_t response_size,
                                uint32_t now)
{
    uint8_t found = 0U;
    if ((response == NULL) || (response_size == 0U)) return 0U;
    (void)osMutexAcquire(g_record_mutex, osWaitForever);
    for (uint8_t i = 0U; i < REMOTE_COMMAND_HISTORY; ++i) {
        RemoteCommandRecord *record = &g_records[i];
        if ((record->valid != 0U) && (record->event_pending != 0U) &&
            ((record->state == REMOTE_RECORD_COMPLETED) ||
             (record->state == REMOTE_RECORD_FAILED))) {
            format_event(record, response, response_size, now);
            g_peeked_event_record = (int8_t)i;
            g_peeked_event_generation = record->generation;
            found = 1U;
            break;
        }
    }
    (void)osMutexRelease(g_record_mutex);
    return found;
}

void RemoteControl_ConfirmEvent(void)
{
    (void)osMutexAcquire(g_record_mutex, osWaitForever);
    if ((g_peeked_event_record >= 0) &&
        (g_peeked_event_record < (int8_t)REMOTE_COMMAND_HISTORY)) {
        RemoteCommandRecord *record = &g_records[g_peeked_event_record];
        if ((record->valid != 0U) &&
            (record->generation == g_peeked_event_generation))
            record->event_pending = 0U;
    }
    g_peeked_event_record = -1;
    g_peeked_event_generation = 0U;
    (void)osMutexRelease(g_record_mutex);
}

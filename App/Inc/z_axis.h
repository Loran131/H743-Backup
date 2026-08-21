#ifndef Z_AXIS_H
#define Z_AXIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define Z_AXIS_SOFT_MIN_PULSES       0
#define Z_AXIS_SOFT_MAX_PULSES       205080
#define Z_AXIS_DEFAULT_SPEED_HZ      90000U
#define Z_AXIS_TRAVEL_MM             36.0f
#define Z_AXIS_STEPS_PER_MM          16000.0f

typedef enum {
    Z_STATE_UNREFERENCED = 0,
    Z_STATE_IDLE,
    Z_STATE_STARTING,
    Z_STATE_MOVING,
    Z_STATE_STOPPING,
    Z_STATE_RECOVERING,
    Z_STATE_FAULT
} ZAxisControlState;

typedef enum {
    Z_RESULT_OK = 0,
    Z_RESULT_NOT_REFERENCED,
    Z_RESULT_BUSY,
    Z_RESULT_INVALID_SPEED,
    Z_RESULT_INVALID_PULSES,
    Z_RESULT_SOFT_LIMIT,
    Z_RESULT_LINK_ERROR,
    Z_RESULT_FAULT
} ZAxisControlResult;

typedef enum {
    Z_FAULT_NONE = 0,
    Z_FAULT_LINK,
    Z_FAULT_TIMEOUT,
    Z_FAULT_CONTROLLER_REJECTED,
    Z_FAULT_POSITION_UNCERTAIN
} ZAxisControlFault;

typedef struct {
    ZAxisControlState state;
    ZAxisControlFault fault;
    int32_t position_pulses;
    int32_t target_pulses;
    uint32_t command_speed_hz;
    uint8_t position_valid;
    uint8_t rx_ready;
    uint8_t last_controller_status;
    uint32_t command_tick;
    uint32_t completion_tick;
    uint32_t completed_moves;
    ZAxisControlFault last_fault;
    uint32_t last_fault_tick;
    uint32_t fault_count;
    uint32_t auto_recovery_count;
} ZAxisControlStatus;

void ZAxis_Init(uint32_t now);
void ZAxis_Poll(uint32_t now);
ZAxisControlResult ZAxisControl_MoveRelative(int32_t delta_pulses,
                                             uint32_t speed_hz);
ZAxisControlResult ZAxisControl_MoveAbsolute(int32_t target_pulses,
                                             uint32_t speed_hz);
ZAxisControlResult ZAxisControl_Stop(void);
ZAxisControlResult ZAxisControl_SetZero(void);
ZAxisControlResult ZAxisControl_ClearFault(void);
void ZAxis_GetControlStatus(ZAxisControlStatus *status);
const char *ZAxis_StateString(ZAxisControlState state);
const char *ZAxis_ResultString(ZAxisControlResult result);
const char *ZAxis_FaultString(ZAxisControlFault fault);

#ifdef __cplusplus
}
#endif

#endif

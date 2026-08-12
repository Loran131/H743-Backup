#ifndef XY_MOTOR_H
#define XY_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define XY_X_MOTOR_ADDRESS          1U
#define XY_Y_MOTOR_ADDRESS          2U
#define XY_POSITIVE_DIRECTION       0U
#define XY_NEGATIVE_DIRECTION       1U
#define XY_CONTROL_PERIOD_MS        20U
#define XY_FEEDBACK_TIMEOUT_MS      250U
#define XY_MOVE_TIMEOUT_MS          30000U

typedef enum {
    XY_AXIS_X = 0,
    XY_AXIS_Y,
    XY_AXIS_COUNT
} XY_Axis;

typedef enum {
    XY_STATE_UNREFERENCED = 0,
    XY_STATE_IDLE,
    XY_STATE_STARTING,
    XY_STATE_MOVING,
    XY_STATE_STOPPING,
    XY_STATE_HOMING,
    XY_STATE_FAULT
} XY_State;

typedef enum {
    XY_RESULT_OK = 0,
    XY_RESULT_INVALID_AXIS,
    XY_RESULT_NOT_REFERENCED,
    XY_RESULT_BUSY,
    XY_RESULT_INVALID_SPEED,
    XY_RESULT_INVALID_ACCELERATION,
    XY_RESULT_INVALID_PULSES,
    XY_RESULT_SOFT_LIMIT,
    XY_RESULT_CAN_REJECTED,
    XY_RESULT_FAULT
} XY_Result;

typedef enum {
    XY_FAULT_NONE = 0,
    XY_FAULT_COMMAND_TIMEOUT,
    XY_FAULT_FEEDBACK_TIMEOUT,
    XY_FAULT_DRIVER_REJECTED,
    XY_FAULT_STALL,
    XY_FAULT_UNDERVOLTAGE,
    XY_FAULT_HOME_FAILED,
    XY_FAULT_STOP_UNCONFIRMED
} XY_Fault;

typedef struct {
    uint8_t motor_address;
    uint8_t positive_direction;
    uint8_t acceleration;
    uint16_t default_speed_rpm;
    uint16_t max_speed_rpm;
    int32_t soft_min_pulses;
    int32_t soft_max_pulses;
    uint8_t home_mode;
    uint8_t home_direction;
    uint16_t home_speed_rpm;
    int16_t home_current_ma;
    uint32_t home_timeout_ms;
} XY_AxisConfig;

typedef struct {
    XY_State state;
    XY_Fault fault;
    int32_t position_pulses;
    int32_t target_pulses;
    int16_t speed_rpm;
    uint8_t position_valid;
    uint8_t arrived;
    uint8_t motor_status;
    uint32_t command_tick;
    uint32_t last_feedback_tick;
} XY_AxisStatus;

void XY_Motor_Init(uint32_t now);
void XY_Motor_Poll(uint32_t now);

XY_Result XY_MoveRelative(XY_Axis axis, int32_t delta_pulses,
                          uint16_t speed_rpm, uint8_t acceleration);
XY_Result XY_MoveAbsolute(XY_Axis axis, int32_t target_pulses,
                          uint16_t speed_rpm, uint8_t acceleration);

void XY_Stop(XY_Axis axis);
void xy_stop_all(void);

/* Both functions are manual-only. Neither is called by init or recovery. */
XY_Result XY_HomeSensorless(XY_Axis axis);
XY_Result XY_SetCurrentPositionAsZero(XY_Axis axis);
XY_Result XY_ClearFault(XY_Axis axis);

XY_Result XY_SetSoftLimits(XY_Axis axis, int32_t min_pulses,
                           int32_t max_pulses);
XY_Result XY_SetDefaultSpeed(XY_Axis axis, uint16_t speed_rpm);
const XY_AxisConfig *XY_GetConfig(XY_Axis axis);
uint8_t XY_GetStatus(XY_Axis axis, XY_AxisStatus *status);
uint8_t XY_AllIdle(void);
const char *XY_ResultString(XY_Result result);
const char *XY_StateString(XY_State state);

#ifdef __cplusplus
}
#endif

#endif

/**
 ******************************************************************************
 * @file    smd.h
 * @brief   PD42S1 stepper motor driver protocol definitions
 *          Serial frame protocol over CAN transport
 ******************************************************************************
 */

#ifndef __SMD_H__
#define __SMD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Frame constants -----------------------------------------------------------*/
#define FRAME_HEAD              0xC5
#define FRAME_TAIL              0x5C
#define SMD_BROADCAST_ADDR      0x00

/* Default CAN ID (29-bit extended) ------------------------------------------*/
#define CAN_EXTID_DEFAULT       0x1000

/* Frame buffer size (max response frame length) -----------------------------*/
#define SMD_FRAME_BUF_SIZE      256
#define SMD_RX_BUF_SIZE         512
#define SMD_CLOG_CURRENT_MAX_MA 3000U

/* ======================== FUNCTION CODE ENUM ============================== */

/* System commands (0x00-0x0F) -----------------------------------------------*/
#define FCT_IDLE                0x00
#define FCT_CAL_ENCODER         0x01
#define FCT_RESTART             0x02
#define FCT_RESET_FACTORY       0x03
#define FCT_PARAM_SAVE          0x04

/* Read/Query commands (0x20-0x3F) -------------------------------------------*/
#define FCT_READ_SOFT_HARD_VER  0x20
#define FCT_READ_PSI            0x21
#define FCT_READ_PHASE_RES_IND  0x22
#define FCT_READ_PHASE_MA       0x23
#define FCT_READ_VOL            0x24
#define FCT_READ_MA_PID         0x25
#define FCT_READ_SPEED_PID      0x26
#define FCT_READ_POS_PID        0x27
#define FCT_READ_TOTAL_PULSE    0x28
#define FCT_READ_ROTATE_SPEED   0x29
#define FCT_READ_POS            0x2A
#define FCT_READ_POS_ERROR      0x2B
#define FCT_READ_MOTOR_STA      0x2C
#define FCT_READ_CLOG_FLAG      0x2D
#define FCT_READ_CLOG_CUR       0x2E
#define FCT_READ_ENABLE_STA     0x2F
#define FCT_READ_ARRIVED_STA    0x30
#define FCT_READ_SYS_PARAM      0x31
#define FCT_READ_DRIVE_PARAMS   0x32

/* Configuration commands (0x60-0x7F) ----------------------------------------*/
#define FCT_SET_SLAVE_ADD       0x60
#define FCT_SET_GROUP_ADD       0x61
#define FCT_SET_MODE            0x62
#define FCT_SET_POS_PID         0x63
#define FCT_SET_POS_TORQUE      0x64
#define FCT_SET_STEP            0x65
#define FCT_SET_MA              0x66
#define FCT_SET_UART_BAUD       0x67
#define FCT_SET_CAN_BAUD        0x68
#define FCT_SET_MODBUS          0x69
#define FCT_SET_CLOG_PRO        0x6A
#define FCT_SET_CLOG_CUR        0x6B
#define FCT_SET_CAN_ID          0x6C
#define FCT_SET_DIR_LEVEL       0x6D
#define FCT_SET_EN_LEVEL        0x6E
#define FCT_SET_CMD_ECHO        0x6F
#define FCT_SET_KEY_LOCK        0x70
#define FCT_SET_AUTO_NOT_DISPLAY 0x71
#define FCT_SET_IO_START_LEVEL  0x72
#define FCT_SET_SPEED_PID       0x73

/* Homing/Origin commands (0x90-0x9F) ----------------------------------------*/
#define FCT_ORIGIN_SET_LEFT_POS 0x90
#define FCT_ORIGIN_LIMIT_HOME   0x91
#define FCT_ORIGIN_TRIG         0x92
#define FCT_ORIGIN_BREAK        0x93
#define FCT_ORIGIN_READ_PARAMS  0x94
#define FCT_ORIGIN_SET_PARAMS   0x95
#define FCT_ORIGIN_READ_STA     0x96
#define FCT_ORIGIN_AOTO_ZERO    0x97
#define FCT_ORIGIN_SET_RIGHT_POS 0x98
#define FCT_ORIGIN_SWITCH       0x99

/* Open-loop motion commands (0xE0-0xE4) -------------------------------------*/
#define FCT_OL_SPEED_MODE       0xE0
#define FCT_OL_POS_MODE         0xE1
#define FCT_OL_POS_REL_MODE     0xE2
#define FCT_OL_PULSES_MODE      0xE3
#define FCT_IO_RUN_MODE         0xE4

/* Closed-loop motion commands (0xF0-0xFC) -----------------------------------*/
#define FCT_TORQUE_MODE         0xF0
#define FCT_SPEED_MODE          0xF1
#define FCT_POS_MODE            0xF2
#define FCT_POS_REL_MODE        0xF3
#define FCT_PULSES_MODE         0xF4
#define FCT_PULSE_WIDTH_POS_MODE    0xF5
#define FCT_PULSE_WIDTH_MA_MODE     0xF6
#define FCT_PULSE_WIDTH_SPEED_MODE  0xF7
#define FCT_ANGLE_ZERO          0xF8
#define FCT_CLEAR_CLOG_PRO      0xF9
#define FCT_MOTOR_ENABLE        0xFA
#define FCT_CLEAR_STATE         0xFB
#define FCT_STOP_NOW            0xFC

/* ====================== RESPONSE STRUCTURES =============================== */

/* ACK error codes */
#define ACK_SUCCEED             0x01
#define ACK_FRAME_TOO_SHORT     0xE1
#define ACK_INVALID_HEADER      0xE2
#define ACK_INVALID_FOOTER      0xE3
#define ACK_CHECKSUM_MISMATCH   0xE4
#define ACK_UNSUPPORTED_FUNCTION 0xE5
#define ACK_ERR_ILLEGAL_VAL     0xE6

/* Motor status */
#define MOTOR_STA_STATIC        0
#define MOTOR_STA_ACCELERATING  1
#define MOTOR_STA_DECELERATING  2
#define MOTOR_STA_FULL_SPEED    3
#define MOTOR_STA_STALL         4
#define MOTOR_STA_UNDERVOLTAGE  5

/* Direction */
#define SMD_DIR_CW              0
#define SMD_DIR_CCW             1

/* ====================== RESPONSE FRAME BUFFER ============================== */

typedef struct {
    uint8_t  buf[SMD_FRAME_BUF_SIZE];
    uint16_t len;
    uint8_t  frame_done;       /* Set by timeout ISR when frame is complete */
} SmdRxFrame_t;

typedef struct {
    uint8_t address;
    uint8_t function;
    uint8_t result;
    const uint8_t *data;
    uint8_t data_length;
    uint32_t rx_tick;
} SmdResponse;

typedef uint8_t (*SmdResponseCallback)(const SmdResponse *response);

/* Extern to the global RX buffer (filled by CAN ISR) */
extern SmdRxFrame_t g_smd_rx_frame;

/* ====================== FUNCTION PROTOTYPES =============================== */

/* Frame assembly and send (transport layer called internally) */
uint8_t smd_send_cmd(uint8_t addr, uint8_t func_code,
                     const uint8_t *data, uint8_t data_len);
uint8_t smd_probe_can_id(uint8_t addr, uint32_t *found_id);
uint32_t smd_get_host_can_id(void);

/* Checksum utility */
uint8_t smd_checksum(const uint8_t *data, uint8_t length);

/* ---- Convenience API: System ---- */
void smd_cal_encoder(uint8_t addr);
void smd_restart(uint8_t addr);
void smd_reset_factory(uint8_t addr);
void smd_param_save(uint8_t addr);

/* ---- Convenience API: Read/Query ---- */
void smd_read_soft_hard_ver(uint8_t addr);
void smd_read_psi(uint8_t addr);
void smd_read_phase_res_ind(uint8_t addr);
void smd_read_phase_ma(uint8_t addr);
void smd_read_vol(uint8_t addr);
void smd_read_ma_pid(uint8_t addr);
void smd_read_speed_pid(uint8_t addr);
void smd_read_pos_pid(uint8_t addr);
void smd_read_total_pulse(uint8_t addr);
void smd_read_rotate_speed(uint8_t addr);
void smd_read_pos(uint8_t addr);
void smd_read_pos_error(uint8_t addr);
void smd_read_motor_sta(uint8_t addr);
void smd_read_clog_flag(uint8_t addr);
void smd_read_clog_cur(uint8_t addr);
void smd_read_enable_sta(uint8_t addr);
void smd_read_arrived_sta(uint8_t addr);
void smd_read_sys_param(uint8_t addr);
void smd_read_drive_params(uint8_t addr);

/* Synchronous queries. These consume the matching CAN response frame. */
uint8_t smd_read_arrived_sync(uint8_t addr, uint32_t timeout_ms);
uint8_t smd_origin_read_sta_sync(uint8_t addr, uint32_t timeout_ms);

/* ---- Convenience API: Configuration ---- */
void smd_set_slave_addr(uint8_t addr, uint8_t new_addr);
void smd_set_can_id(uint8_t addr, uint32_t can_id);
void smd_set_mode(uint8_t addr, uint8_t mode);
void smd_set_pos_pid(uint8_t addr, uint32_t kp, uint32_t ki, uint32_t kd);
void smd_set_speed_pid(uint8_t addr, uint32_t kp, uint32_t ki, uint32_t kd);
void smd_set_pos_torque(uint8_t addr, int16_t torque_ma);
void smd_set_ma(uint8_t addr, int16_t ma);
void smd_set_clog_pro(uint8_t addr, uint8_t enable);
void smd_set_clog_cur(uint8_t addr, uint16_t ma);
void smd_set_dir_level(uint8_t addr, uint8_t level);
void smd_set_en_level(uint8_t addr, uint8_t level);
void smd_set_cmd_echo(uint8_t addr, uint8_t echo);

/* ---- Convenience API: Homing/Origin ---- */
void smd_origin_homing_by_limit(uint8_t addr, uint8_t mode, uint8_t dir,
                                uint32_t speed_rpm, int16_t current_ma);
void smd_origin_break(uint8_t addr);
void smd_origin_set_params(uint8_t addr, uint32_t timeout_ms);
void smd_origin_read_sta(uint8_t addr);
void smd_origin_auto_zero(uint8_t addr, uint8_t enable);

/* ---- Convenience API: Motion Control ---- */
void smd_motor_enable(uint8_t addr, uint8_t enable);
void smd_stop_now(uint8_t addr);
void smd_clear_state(uint8_t addr);
void smd_angle_to_zero(uint8_t addr);

/* Closed-loop relative position move */
uint8_t smd_pos_rel_move(uint8_t addr, uint8_t dir, uint8_t acc,
                         uint16_t speed_rpm, uint32_t pulses);

/* Closed-loop absolute position move */
void smd_pos_abs_move(uint8_t addr, uint8_t dir, uint8_t acc,
                      uint16_t speed_rpm, uint32_t pulses);

/* Closed-loop speed mode */
void smd_speed_move(uint8_t addr, uint8_t dir, uint8_t acc, float speed_rpm);

/* Closed-loop torque mode */
void smd_torque_move(uint8_t addr, uint8_t dir, uint16_t current_ma);

/* ---- Response processing ---- */
void smd_process_response(const uint8_t *buf, uint16_t len);
void smd_set_response_callback(SmdResponseCallback callback);

#ifdef __cplusplus
}
#endif

#endif /* __SMD_H__ */

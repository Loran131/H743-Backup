/**
 ******************************************************************************
 * @file    smd.c
 * @brief   PD42S1 motor protocol implementation
 *          Serial frame assembly and motor command convenience APIs
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "smd.h"
#include "fdcan.h"
#include <string.h>
#include <stdio.h>

/* ====================== GLOBAL VARIABLES ================================== */

/* CAN RX reassembly buffer */
SmdRxFrame_t g_smd_rx_frame;
static uint32_t g_smd_host_can_id = CAN_EXTID_DEFAULT;
static SmdResponseCallback g_smd_response_callback;

/* ====================== INTERNAL HELPERS ================================== */

/**
 * @brief  Calculate checksum (simple byte sum)
 * @param  data   Pointer to data buffer
 * @param  length Number of bytes to sum
 * @retval Checksum value (lower 8 bits)
 */
uint8_t smd_checksum(const uint8_t *data, uint8_t length)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief  Build a protocol frame and send via CAN
 * @param  addr       Slave address (1-255, 0=broadcast)
 * @param  func_code  Function code
 * @param  data       Parameter data (can be NULL if data_len=0)
 * @param  data_len   Number of parameter bytes
 * @retval 0=success, 1=error
 *
 * Frame format: HEAD(1) + addr(1) + func(1) + data(N) + checksum(1) + TAIL(1)
 */
uint8_t smd_send_cmd(uint8_t addr, uint8_t func_code,
                     const uint8_t *data, uint8_t data_len)
{
    /* Max frame = 3 (header) + 128 (data) + 1 (checksum) + 1 (tail) = 133 */
    uint8_t frame[256];
    uint8_t pos = 0;

    /* Frame head */
    frame[pos++] = FRAME_HEAD;

    /* Address */
    frame[pos++] = addr;

    /* Function code */
    frame[pos++] = func_code;

    /* Data payload */
    if (data != NULL && data_len > 0)
    {
        memcpy(&frame[pos], data, data_len);
        pos += data_len;
    }

    /* Checksum: sum of bytes 0..pos-1 */
    frame[pos] = smd_checksum(frame, pos);
    pos++;

    /* Frame tail */
    frame[pos++] = FRAME_TAIL;

    /* Send via CAN transport */
    return can_send_long_msg(g_smd_host_can_id, frame, pos);
}

uint32_t smd_get_host_can_id(void)
{
    return g_smd_host_can_id;
}

void smd_set_response_callback(SmdResponseCallback callback)
{
    g_smd_response_callback = callback;
}

uint8_t smd_probe_can_id(uint8_t addr, uint32_t *found_id)
{
    uint8_t frame[5];

    frame[0] = FRAME_HEAD;
    frame[1] = addr;
    frame[2] = FCT_READ_SOFT_HARD_VER;
    frame[3] = smd_checksum(frame, 3U);
    frame[4] = FRAME_TAIL;

    for (uint32_t can_id = 0x1000U; can_id <= 0x100FU; ++can_id) {
        uint32_t start;

        g_can_rx_frame.len = 0U;
        g_can_rx_frame.frame_done = 0U;
        if (can_send_long_msg(can_id, frame, sizeof(frame)) != 0U) {
            continue;
        }

        start = HAL_GetTick();
        while ((HAL_GetTick() - start) < 30U) {
            can_rx_timeout_check();
            if (g_can_rx_frame.frame_done) {
                const uint8_t *buf = g_can_rx_frame.buf;
                uint16_t len = g_can_rx_frame.len;
                uint8_t valid = (len >= 6U) && (buf[0] == FRAME_HEAD) &&
                                (buf[1] == addr) &&
                                (buf[2] == FCT_READ_SOFT_HARD_VER) &&
                                (buf[3] == ACK_SUCCEED) &&
                                (buf[len - 1U] == FRAME_TAIL) &&
                                (smd_checksum(buf, (uint8_t)(len - 2U)) ==
                                 buf[len - 2U]);

                g_can_rx_frame.len = 0U;
                g_can_rx_frame.frame_done = 0U;
                if (valid) {
                    g_smd_host_can_id = can_id;
                    if (found_id != NULL) {
                        *found_id = can_id;
                    }
                    return 1U;
                }
            }
            HAL_Delay(1U);
        }
    }

    return 0U;
}

/* ====================== SYSTEM COMMANDS =================================== */

void smd_cal_encoder(uint8_t addr)
{
    uint8_t data[2] = {0x52, 0x00};
    smd_send_cmd(addr, FCT_CAL_ENCODER, data, 2);
}

void smd_restart(uint8_t addr)
{
    uint8_t data[2] = {0x52, 0x00};
    smd_send_cmd(addr, FCT_RESTART, data, 2);
}

void smd_reset_factory(uint8_t addr)
{
    smd_send_cmd(addr, FCT_RESET_FACTORY, NULL, 0);
}

void smd_param_save(uint8_t addr)
{
    uint8_t data[2] = {0x52, 0x00};
    smd_send_cmd(addr, FCT_PARAM_SAVE, data, 2);
}

/* ====================== READ / QUERY COMMANDS ============================= */

void smd_read_soft_hard_ver(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_SOFT_HARD_VER, NULL, 0);
}

void smd_read_psi(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_PSI, NULL, 0);
}

void smd_read_phase_res_ind(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_PHASE_RES_IND, NULL, 0);
}

void smd_read_phase_ma(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_PHASE_MA, NULL, 0);
}

void smd_read_vol(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_VOL, NULL, 0);
}

void smd_read_ma_pid(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_MA_PID, NULL, 0);
}

void smd_read_speed_pid(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_SPEED_PID, NULL, 0);
}

void smd_read_pos_pid(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_POS_PID, NULL, 0);
}

void smd_read_total_pulse(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_TOTAL_PULSE, NULL, 0);
}

void smd_read_rotate_speed(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_ROTATE_SPEED, NULL, 0);
}

void smd_read_pos(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_POS, NULL, 0);
}

void smd_read_pos_error(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_POS_ERROR, NULL, 0);
}

void smd_read_motor_sta(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_MOTOR_STA, NULL, 0);
}

void smd_read_clog_flag(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_CLOG_FLAG, NULL, 0);
}

void smd_read_clog_cur(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_CLOG_CUR, NULL, 0);
}

void smd_read_enable_sta(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_ENABLE_STA, NULL, 0);
}

void smd_read_arrived_sta(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_ARRIVED_STA, NULL, 0);
}

void smd_read_sys_param(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_SYS_PARAM, NULL, 0);
}

void smd_read_drive_params(uint8_t addr)
{
    smd_send_cmd(addr, FCT_READ_DRIVE_PARAMS, NULL, 0);
}

/* ====================== CONFIGURATION COMMANDS ============================ */

void smd_set_slave_addr(uint8_t addr, uint8_t new_addr)
{
    smd_send_cmd(addr, FCT_SET_SLAVE_ADD, &new_addr, 1);
}

void smd_set_can_id(uint8_t addr, uint32_t can_id)
{
    uint8_t data[4];
    data[0] = (can_id >> 24) & 0xFF;
    data[1] = (can_id >> 16) & 0xFF;
    data[2] = (can_id >> 8) & 0xFF;
    data[3] = can_id & 0xFF;
    smd_send_cmd(addr, FCT_SET_CAN_ID, data, 4);
}

void smd_set_mode(uint8_t addr, uint8_t mode)
{
    smd_send_cmd(addr, FCT_SET_MODE, &mode, 1);
}

void smd_set_pos_pid(uint8_t addr, uint32_t kp, uint32_t ki, uint32_t kd)
{
    uint8_t data[12];
    data[0]  = (kp >> 24) & 0xFF;
    data[1]  = (kp >> 16) & 0xFF;
    data[2]  = (kp >> 8) & 0xFF;
    data[3]  = kp & 0xFF;
    data[4]  = (ki >> 24) & 0xFF;
    data[5]  = (ki >> 16) & 0xFF;
    data[6]  = (ki >> 8) & 0xFF;
    data[7]  = ki & 0xFF;
    data[8]  = (kd >> 24) & 0xFF;
    data[9]  = (kd >> 16) & 0xFF;
    data[10] = (kd >> 8) & 0xFF;
    data[11] = kd & 0xFF;
    smd_send_cmd(addr, FCT_SET_POS_PID, data, 12);
}

void smd_set_speed_pid(uint8_t addr, uint32_t kp, uint32_t ki, uint32_t kd)
{
    uint8_t data[12];
    data[0]  = (kp >> 24) & 0xFF;
    data[1]  = (kp >> 16) & 0xFF;
    data[2]  = (kp >> 8) & 0xFF;
    data[3]  = kp & 0xFF;
    data[4]  = (ki >> 24) & 0xFF;
    data[5]  = (ki >> 16) & 0xFF;
    data[6]  = (ki >> 8) & 0xFF;
    data[7]  = ki & 0xFF;
    data[8]  = (kd >> 24) & 0xFF;
    data[9]  = (kd >> 16) & 0xFF;
    data[10] = (kd >> 8) & 0xFF;
    data[11] = kd & 0xFF;
    smd_send_cmd(addr, FCT_SET_SPEED_PID, data, 12);
}

void smd_set_pos_torque(uint8_t addr, int16_t torque_ma)
{
    uint8_t data[2];
    data[0] = (torque_ma >> 8) & 0xFF;
    data[1] = torque_ma & 0xFF;
    smd_send_cmd(addr, FCT_SET_POS_TORQUE, data, 2);
}

void smd_set_ma(uint8_t addr, int16_t ma)
{
    uint8_t data[2];
    data[0] = (ma >> 8) & 0xFF;
    data[1] = ma & 0xFF;
    smd_send_cmd(addr, FCT_SET_MA, data, 2);
}

void smd_set_clog_pro(uint8_t addr, uint8_t enable)
{
    smd_send_cmd(addr, FCT_SET_CLOG_PRO, &enable, 1);
}

void smd_set_clog_cur(uint8_t addr, int16_t ma)
{
    uint8_t data[2];
    data[0] = (ma >> 8) & 0xFF;
    data[1] = ma & 0xFF;
    smd_send_cmd(addr, FCT_SET_CLOG_CUR, data, 2);
}

void smd_set_dir_level(uint8_t addr, uint8_t level)
{
    smd_send_cmd(addr, FCT_SET_DIR_LEVEL, &level, 1);
}

void smd_set_en_level(uint8_t addr, uint8_t level)
{
    smd_send_cmd(addr, FCT_SET_EN_LEVEL, &level, 1);
}

void smd_set_cmd_echo(uint8_t addr, uint8_t echo)
{
    smd_send_cmd(addr, FCT_SET_CMD_ECHO, &echo, 1);
}

/* ====================== HOMING / ORIGIN COMMANDS ========================= */

void smd_origin_homing_by_limit(uint8_t addr, uint8_t mode, uint8_t dir,
                                uint32_t speed_rpm, int16_t current_ma)
{
    uint8_t data[8];
    data[0] = mode;
    data[1] = dir;
    data[2] = (uint8_t)(speed_rpm >> 24);
    data[3] = (uint8_t)(speed_rpm >> 16);
    data[4] = (uint8_t)(speed_rpm >> 8);
    data[5] = (uint8_t)speed_rpm;
    data[6] = (uint8_t)((uint16_t)current_ma >> 8);
    data[7] = (uint8_t)current_ma;
    smd_send_cmd(addr, FCT_ORIGIN_LIMIT_HOME, data, sizeof(data));
}

void smd_origin_break(uint8_t addr)
{
    smd_send_cmd(addr, FCT_ORIGIN_BREAK, NULL, 0);
}

void smd_origin_set_params(uint8_t addr, uint32_t timeout_ms)
{
    uint8_t data[4];
    data[0] = (uint8_t)(timeout_ms >> 24);
    data[1] = (uint8_t)(timeout_ms >> 16);
    data[2] = (uint8_t)(timeout_ms >> 8);
    data[3] = (uint8_t)timeout_ms;
    smd_send_cmd(addr, FCT_ORIGIN_SET_PARAMS, data, sizeof(data));
}

void smd_origin_read_sta(uint8_t addr)
{
    smd_send_cmd(addr, FCT_ORIGIN_READ_STA, NULL, 0);
}

void smd_origin_auto_zero(uint8_t addr, uint8_t enable)
{
    smd_send_cmd(addr, FCT_ORIGIN_AOTO_ZERO, &enable, 1);
}

/* ====================== MOTION CONTROL COMMANDS =========================== */

void smd_motor_enable(uint8_t addr, uint8_t enable)
{
    /* enable=0: motor enabled, enable=1: motor disabled */
    smd_send_cmd(addr, FCT_MOTOR_ENABLE, &enable, 1);
}

void smd_stop_now(uint8_t addr)
{
    smd_send_cmd(addr, FCT_STOP_NOW, NULL, 0);
}

void smd_clear_state(uint8_t addr)
{
    smd_send_cmd(addr, FCT_CLEAR_STATE, NULL, 0);
}

void smd_angle_to_zero(uint8_t addr)
{
    smd_send_cmd(addr, FCT_ANGLE_ZERO, NULL, 0);
}

/**
 * @brief  Closed-loop relative position move
 * @param  addr       Slave address
 * @param  dir        Direction: 0=CW, 1=CCW
 * @param  acc        Acceleration: 0-200 RPM/s (0=instant)
 * @param  speed_rpm  Max speed: 0-3000 RPM
 * @param  pulses     Relative pulse count (51200 = 1 revolution)
 */
uint8_t smd_pos_rel_move(uint8_t addr, uint8_t dir, uint8_t acc,
                         uint16_t speed_rpm, uint32_t pulses)
{
    uint8_t data[9];
    data[0] = dir;
    data[1] = acc;
    data[2] = (speed_rpm >> 8) & 0xFF;
    data[3] = speed_rpm & 0xFF;
    data[4] = (pulses >> 24) & 0xFF;
    data[5] = (pulses >> 16) & 0xFF;
    data[6] = (pulses >> 8) & 0xFF;
    data[7] = pulses & 0xFF;
    data[8] = 0x00;  /* Protocol-reserved byte required by PD42S1. */
    return smd_send_cmd(addr, FCT_POS_REL_MODE, data, sizeof(data));
}

/**
 * @brief  Closed-loop absolute position move
 * @param  addr       Slave address
 * @param  dir        Direction: 0=CW, 1=CCW
 * @param  acc        Acceleration: 0-200 RPM/s
 * @param  speed_rpm  Max speed: 0-3000 RPM
 * @param  pulses     Absolute pulse position
 */
void smd_pos_abs_move(uint8_t addr, uint8_t dir, uint8_t acc,
                      uint16_t speed_rpm, uint32_t pulses)
{
    uint8_t data[9];
    data[0] = dir;
    data[1] = acc;
    data[2] = (speed_rpm >> 8) & 0xFF;
    data[3] = speed_rpm & 0xFF;
    data[4] = (pulses >> 24) & 0xFF;
    data[5] = (pulses >> 16) & 0xFF;
    data[6] = (pulses >> 8) & 0xFF;
    data[7] = pulses & 0xFF;
    data[8] = 0x00;  /* Protocol-reserved byte required by PD42S1. */
    smd_send_cmd(addr, FCT_POS_MODE, data, sizeof(data));
}

/**
 * @brief  Closed-loop speed mode
 * @param  addr       Slave address
 * @param  dir        0=CW, 1=CCW
 * @param  acc        Acceleration: 0-200 RPM/s
 * @param  speed_rpm  Target speed in RPM (float)
 */
void smd_speed_move(uint8_t addr, uint8_t dir, uint8_t acc, float speed_rpm)
{
    uint8_t data[6];
    uint32_t speed_bits;
    memcpy(&speed_bits, &speed_rpm, 4);

    data[0] = dir;
    data[1] = acc;
    data[2] = (speed_bits >> 24) & 0xFF;
    data[3] = (speed_bits >> 16) & 0xFF;
    data[4] = (speed_bits >> 8) & 0xFF;
    data[5] = speed_bits & 0xFF;
    smd_send_cmd(addr, FCT_SPEED_MODE, data, 6);
}

/**
 * @brief  Closed-loop torque mode
 * @param  addr        Slave address
 * @param  dir         0=CW, 1=CCW
 * @param  current_ma  Target current in mA
 */
void smd_torque_move(uint8_t addr, uint8_t dir, uint16_t current_ma)
{
    uint8_t data[3];
    data[0] = dir;
    data[1] = (current_ma >> 8) & 0xFF;
    data[2] = current_ma & 0xFF;
    smd_send_cmd(addr, FCT_TORQUE_MODE, data, 3);
}

static void smd_clear_rx(void)
{
    g_can_rx_frame.len = 0;
    g_can_rx_frame.frame_done = 0;
}

static uint8_t smd_wait_data(uint8_t addr, uint8_t function, uint8_t *data,
                             uint8_t data_size, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms) {
        can_rx_timeout_check();
        if (g_can_rx_frame.frame_done) {
            const uint8_t *buf = g_can_rx_frame.buf;
            uint16_t len = g_can_rx_frame.len;
            uint8_t valid = (len >= 6U) && (buf[0] == FRAME_HEAD) &&
                            (buf[1] == addr) && (buf[2] == function) &&
                            (buf[3] == ACK_SUCCEED) && (buf[len - 1U] == FRAME_TAIL) &&
                            (smd_checksum(buf, (uint8_t)(len - 2U)) == buf[len - 2U]);
            if (valid && (data_size <= (uint8_t)(len - 6U))) {
                if ((data != NULL) && (data_size > 0U)) {
                    memcpy(data, &buf[4], data_size);
                }
                smd_clear_rx();
                return 1;
            }
            smd_clear_rx();
        }
        HAL_Delay(1);
    }
    return 0;
}

uint8_t smd_read_arrived_sync(uint8_t addr, uint32_t timeout_ms)
{
    uint8_t arrived = 0;
    smd_clear_rx();
    smd_read_arrived_sta(addr);
    return smd_wait_data(addr, FCT_READ_ARRIVED_STA, &arrived, 1, timeout_ms) && arrived;
}

uint8_t smd_origin_read_sta_sync(uint8_t addr, uint32_t timeout_ms)
{
    uint8_t status = 0;
    smd_clear_rx();
    smd_origin_read_sta(addr);
    return smd_wait_data(addr, FCT_ORIGIN_READ_STA, &status, 1, timeout_ms) ? status : 0;
}

/* ====================== RESPONSE PROCESSING =============================== */

/**
 * @brief  Parse a received protocol frame and print decoded info
 * @param  buf  Frame buffer (starting from HEAD byte)
 * @param  len  Total frame length
 *
 * Response frame format:
 *   HEAD(1) + addr(1) + func_code(1) + error_code(1) + [data(N)] + checksum(1) + TAIL(1)
 */
void smd_process_response(const uint8_t *buf, uint16_t len)
{
    SmdResponse response;
    uint8_t handled = 0U;
    if (len < 6)
    {
        printf("[SMD] Frame too short (%d bytes)\r\n", len);
        return;
    }

    /* Validate header and tail */
    if (buf[0] != FRAME_HEAD)
    {
        printf("[SMD] Invalid header: 0x%02X\r\n", buf[0]);
        return;
    }
    if (buf[len - 1] != FRAME_TAIL)
    {
        printf("[SMD] Invalid tail: 0x%02X\r\n", buf[len - 1]);
        return;
    }

    /* Validate checksum */
    uint8_t calc_chk = smd_checksum(buf, len - 2);
    uint8_t recv_chk  = buf[len - 2];
    if (calc_chk != recv_chk)
    {
        printf("[SMD] Checksum error: calc=0x%02X, recv=0x%02X\r\n",
               calc_chk, recv_chk);
        return;
    }

    uint8_t addr      = buf[1];
    uint8_t func_code = buf[2];
    uint8_t err_code  = buf[3];

    response.address = addr;
    response.function = func_code;
    response.result = err_code;
    response.data = &buf[4];
    response.data_length = (uint8_t)(len - 6U);
    response.rx_tick = HAL_GetTick();
    if (g_smd_response_callback != NULL) {
        handled = g_smd_response_callback(&response);
    }

    /* Check ACK */
    if (err_code != ACK_SUCCEED)
    {
        const char *err_str = "Unknown";
        switch (err_code)
        {
        case ACK_FRAME_TOO_SHORT:      err_str = "Frame too short"; break;
        case ACK_INVALID_HEADER:       err_str = "Invalid header"; break;
        case ACK_INVALID_FOOTER:       err_str = "Invalid footer"; break;
        case ACK_CHECKSUM_MISMATCH:    err_str = "Checksum mismatch"; break;
        case ACK_UNSUPPORTED_FUNCTION: err_str = "Unsupported function"; break;
        case ACK_ERR_ILLEGAL_VAL:      err_str = "Illegal value"; break;
        }
        printf("[SMD] Addr=%d Func=0x%02X ERROR: %s (0x%02X)\r\n",
               addr, func_code, err_str, err_code);
        return;
    }


    if (handled != 0U) {
        return;
    }

    printf("[SMD] Addr=%d Func=0x%02X ACK OK", addr, func_code);

    /* Parse function-specific response data */
    uint8_t data_len = len - 6; /* exclude head,addr,func,err,chk,tail */
    const uint8_t *pdata = &buf[4];

    switch (func_code)
    {
    case FCT_READ_SOFT_HARD_VER:
        if (data_len >= 2)
            printf("  SW_Ver=%d  HW_Ver=%d\r\n", pdata[0], pdata[1]);
        break;

    case FCT_READ_POS:
        if (data_len >= 4)
        {
            int32_t pos = (int32_t)((pdata[0] << 24) | (pdata[1] << 16) |
                                    (pdata[2] << 8) | pdata[3]);
            printf("  Position=%ld pulses\r\n", (long)pos);
        }
        break;

    case FCT_READ_ROTATE_SPEED:
        if (data_len >= 2)
        {
            int16_t rpm = (int16_t)((pdata[0] << 8) | pdata[1]);
            printf("  Speed=%d RPM\r\n", rpm);
        }
        break;

    case FCT_READ_MOTOR_STA:
        if (data_len >= 1)
        {
            const char *sta_str = "Unknown";
            switch (pdata[0])
            {
            case MOTOR_STA_STATIC:        sta_str = "Static"; break;
            case MOTOR_STA_ACCELERATING:  sta_str = "Accelerating"; break;
            case MOTOR_STA_DECELERATING:  sta_str = "Decelerating"; break;
            case MOTOR_STA_FULL_SPEED:    sta_str = "Full Speed"; break;
            case MOTOR_STA_STALL:         sta_str = "Stall"; break;
            case MOTOR_STA_UNDERVOLTAGE:  sta_str = "Undervoltage"; break;
            }
            printf("  Motor=%s\r\n", sta_str);
        }
        break;

    case FCT_READ_ENABLE_STA:
        if (data_len >= 1)
            printf("  Enable=%s\r\n", pdata[0] ? "Disabled" : "Enabled");
        break;

    case FCT_READ_ARRIVED_STA:
        if (data_len >= 1)
            printf("  Arrived=%s\r\n", pdata[0] ? "Yes" : "No");
        break;

    case FCT_READ_CLOG_FLAG:
        if (data_len >= 1)
            printf("  Stalled=%s\r\n", pdata[0] ? "YES" : "No");
        break;

    case FCT_READ_PHASE_MA:
        if (data_len >= 2)
        {
            int16_t ma = (int16_t)((pdata[0] << 8) | pdata[1]);
            printf("  PhaseCurrent=%d mA\r\n", ma);
        }
        break;

    case FCT_READ_VOL:
        if (data_len >= 4)
        {
            /* Float: big-endian IEEE 754 */
            union { uint8_t b[4]; float f; } u;
            u.b[0] = pdata[3]; u.b[1] = pdata[2];
            u.b[2] = pdata[1]; u.b[3] = pdata[0];
            printf("  Voltage=%.2f V\r\n", (double)u.f);
        }
        break;

    case FCT_READ_TOTAL_PULSE:
        if (data_len >= 4)
        {
            int32_t total = (int32_t)((pdata[0] << 24) | (pdata[1] << 16) |
                                      (pdata[2] << 8) | pdata[3]);
            printf("  TotalPulses=%ld\r\n", (long)total);
        }
        break;

    case FCT_READ_POS_ERROR:
        if (data_len >= 4)
        {
            int32_t err = (int32_t)((pdata[0] << 24) | (pdata[1] << 16) |
                                    (pdata[2] << 8) | pdata[3]);
            printf("  PosError=%ld\r\n", (long)err);
        }
        break;

    default:
        printf("  Data[%d]=", data_len);
        for (uint8_t i = 0; i < data_len && i < 16; i++)
            printf("%02X ", pdata[i]);
        printf("\r\n");
        break;
    }
}

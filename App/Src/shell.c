/**
 ******************************************************************************
 * @file    shell.c
 * @brief   Serial command-line shell — text command parser for motor control
 *
 * Commands entered via USART1 terminal are parsed and dispatched to the
 * PD42S1 motor driver through the smd protocol layer (CAN transport).
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "shell.h"
#include "smd.h"
#include "fdcan.h"
#include "usart.h"
#include "c552.h"
#include "xy_motor.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ====================== HELP TEXT ========================================= */

static const char *g_help_text =
    "\r\n============= Motion Control Shell =============\r\n"
    " DIAGNOSTICS:\r\n"
    "  cans             CAN state and recovery counters\r\n"
    "  canprobe [addr]  scan PD42S1 CAN ID 0x1000..0x100F\r\n"
    "  C552             show C552 link, sensors and RX counters\r\n"
    "--------------------------------------------------\r\n"
    " X/Y CONTROL (signed pulses, + = away from zero):\r\n"
    "  x_move <delta> [rpm] [acc]   safeguarded relative move\r\n"
    "  y_move <delta> [rpm] [acc]   safeguarded relative move\r\n"
    "  xy_stop                       immediately stop both axes\r\n"
    "  xy_status                     show coordinates and state\r\n"
    "  x_home / y_home               MANUAL sensorless homing\r\n"
    "  x_zero / y_zero               MANUAL current-position zero\r\n"
    "  x_clear / y_clear             clear control-layer fault\r\n"
    "--------------------------------------------------\r\n"
    " LOW-LEVEL MOTOR:\r\n"
    "  pos_rel  [addr] <dir> <acc> <speed> <pulses>\r\n"
    "  pos_abs  [addr] <dir> <acc> <speed> <pulses>\r\n"
    "  motor_speed <addr> <dir> <acc> <rpm>\r\n"
    "  torque   [addr] <dir> <ma>\r\n"
    "  motor_stop [addr]\r\n"
    "  enable   [addr] <0|1>\r\n"
    "  motor_zero [addr]\r\n"
    "  clear    [addr]\r\n"
    "------------------------------------------------\r\n"
    " QUERY:\r\n"
    "  sta      [addr]     -- motor status\r\n"
    "  motor_pos [addr]    -- current position\r\n"
    "  speed_r  [addr]     -- current speed (RPM)\r\n"
    "  en_sta   [addr]     -- enable state\r\n"
    "  arrived  [addr]     -- target arrived?\r\n"
    "  vol      [addr]     -- bus voltage\r\n"
    "  ma       [addr]     -- phase current (mA)\r\n"
    "  ver      [addr]     -- firmware version\r\n"
    "  clog     [addr]     -- stall flag\r\n"
    "  pos_err  [addr]     -- position error\r\n"
    "  total    [addr]     -- total pulses\r\n"
    "  sys      [addr]     -- system params\r\n"
    "------------------------------------------------\r\n"
    " CONFIG:\r\n"
    "  set_addr    [addr] <new_addr>\r\n"
    "  set_can_id  [addr] <can_id>\r\n"
    "  set_mode    [addr] <mode>\r\n"
    "  set_ma      [addr] <ma>\r\n"
    "  param_save  [addr]\r\n"
    "------------------------------------------------\r\n"
    " SYSTEM:\r\n"
    "  restart [addr]      -- restart driver\r\n"
    "  cal     [addr]      -- calibrate encoder\r\n"
    "  factory [addr]      -- factory reset\r\n"
    "  help / ?            -- this help\r\n"
    "================================================\r\n\r\n";

/* ====================== COMMAND TABLE ===================================== */

typedef struct {
    const char *name;
    const char *args;      /* Argument description for help */
    int min_args;          /* Minimum arguments (including addr) */
    void (*handler)(int argc, char *argv[]);
} ShellCmd_t;

/* Forward declarations */
static void cmd_pos_rel(int argc, char *argv[]);
static void cmd_pos_abs(int argc, char *argv[]);
static void cmd_speed(int argc, char *argv[]);
static void cmd_torque(int argc, char *argv[]);
static void cmd_stop(int argc, char *argv[]);
static void cmd_enable(int argc, char *argv[]);
static void cmd_zero(int argc, char *argv[]);
static void cmd_clear(int argc, char *argv[]);
static void cmd_read_sta(int argc, char *argv[]);
static void cmd_read_pos(int argc, char *argv[]);
static void cmd_read_speed(int argc, char *argv[]);
static void cmd_read_enable(int argc, char *argv[]);
static void cmd_read_arrived(int argc, char *argv[]);
static void cmd_read_vol(int argc, char *argv[]);
static void cmd_read_ma(int argc, char *argv[]);
static void cmd_read_ver(int argc, char *argv[]);
static void cmd_read_clog(int argc, char *argv[]);
static void cmd_read_pos_err(int argc, char *argv[]);
static void cmd_read_total(int argc, char *argv[]);
static void cmd_read_sys(int argc, char *argv[]);
static void cmd_set_addr(int argc, char *argv[]);
static void cmd_set_can_id(int argc, char *argv[]);
static void cmd_set_mode(int argc, char *argv[]);
static void cmd_set_ma(int argc, char *argv[]);
static void cmd_param_save(int argc, char *argv[]);
static void cmd_restart(int argc, char *argv[]);
static void cmd_cal(int argc, char *argv[]);
static void cmd_factory(int argc, char *argv[]);
static void cmd_can_status(int argc, char *argv[]);
static void cmd_can_probe(int argc, char *argv[]);
static void cmd_c552(int argc, char *argv[]);
static void cmd_x_move(int argc, char *argv[]);
static void cmd_y_move(int argc, char *argv[]);
static void cmd_xy_stop(int argc, char *argv[]);
static void cmd_xy_status(int argc, char *argv[]);
static void cmd_x_home(int argc, char *argv[]);
static void cmd_y_home(int argc, char *argv[]);
static void cmd_x_zero(int argc, char *argv[]);
static void cmd_y_zero(int argc, char *argv[]);
static void cmd_x_clear(int argc, char *argv[]);
static void cmd_y_clear(int argc, char *argv[]);

static const ShellCmd_t g_cmd_table[] = {
    {"cans",      "",           1, cmd_can_status},
    {"canprobe",  "[addr]",     1, cmd_can_probe},
    {"C552",      "",           1, cmd_c552},
    {"x_move",    "<delta_pulses> [rpm] [acc]", 2, cmd_x_move},
    {"y_move",    "<delta_pulses> [rpm] [acc]", 2, cmd_y_move},
    {"xy_stop",    "",           1, cmd_xy_stop},
    {"xy_status",  "",           1, cmd_xy_status},
    {"x_home",     "",           1, cmd_x_home},
    {"y_home",     "",           1, cmd_y_home},
    {"x_zero",     "",           1, cmd_x_zero},
    {"y_zero",     "",           1, cmd_y_zero},
    {"x_clear",    "",           1, cmd_x_clear},
    {"y_clear",    "",           1, cmd_y_clear},
    /* Motion */
    {"pos_rel",   "<addr> <dir> <acc> <speed> <pulses>", 6, cmd_pos_rel},
    {"pos_abs",   "<addr> <dir> <acc> <speed> <pulses>", 6, cmd_pos_abs},
    {"motor_speed", "<addr> <dir> <acc> <rpm>",          5, cmd_speed},
    {"torque",    "<addr> <dir> <ma>",                   4, cmd_torque},
    {"motor_stop", "[addr]",                             1, cmd_stop},
    {"enable",    "<addr> <0|1>",                        3, cmd_enable},
    {"motor_zero", "[addr]",                             1, cmd_zero},
    {"clear",     "[addr]",                              1, cmd_clear},
    /* Query */
    {"sta",       "[addr]", 1, cmd_read_sta},
    {"motor_pos", "[addr]", 1, cmd_read_pos},
    {"speed_r",   "[addr]", 1, cmd_read_speed},
    {"en_sta",    "[addr]", 1, cmd_read_enable},
    {"arrived",   "[addr]", 1, cmd_read_arrived},
    {"vol",       "[addr]", 1, cmd_read_vol},
    {"ma",        "[addr]", 1, cmd_read_ma},
    {"ver",       "[addr]", 1, cmd_read_ver},
    {"clog",      "[addr]", 1, cmd_read_clog},
    {"pos_err",   "[addr]", 1, cmd_read_pos_err},
    {"total",     "[addr]", 1, cmd_read_total},
    {"sys",       "[addr]", 1, cmd_read_sys},
    /* Config */
    {"set_addr",    "<addr> <new_addr>",  3, cmd_set_addr},
    {"set_can_id",  "<addr> <can_id>",    3, cmd_set_can_id},
    {"set_mode",    "<addr> <mode>",      3, cmd_set_mode},
    {"set_ma",      "<addr> <ma>",        3, cmd_set_ma},
    {"param_save",  "[addr]",             1, cmd_param_save},
    /* System */
    {"restart",   "[addr]", 1, cmd_restart},
    {"cal",       "[addr]", 1, cmd_cal},
    {"factory",   "[addr]", 1, cmd_factory},
    {NULL, NULL, 0, NULL}  /* Sentinel */
};

/* ====================== PARSER ============================================ */

#define MAX_ARGS    8

static void parse_and_execute(char *cmd_line)
{
    char *argv[MAX_ARGS];
    int argc = 0;

    /* Tokenize: split by spaces */
    char *token = strtok(cmd_line, " \t");
    while (token != NULL && argc < MAX_ARGS)
    {
        argv[argc++] = token;
        token = strtok(NULL, " \t");
    }

    if (argc == 0) return;

    /* Handle both documented help forms. */
    if ((strcmp(argv[0], "help") == 0) || (strcmp(argv[0], "?") == 0))
    {
        printf("%s", g_help_text);
        return;
    }

    /* Search command table */
    for (const ShellCmd_t *cmd = g_cmd_table; cmd->name != NULL; cmd++)
    {
        if (strcmp(argv[0], cmd->name) == 0)
        {
            if (argc >= cmd->min_args)
            {
                cmd->handler(argc, argv);
            }
            else
            {
                printf("Usage: %s %s\r\n", cmd->name, cmd->args);
            }
            return;
        }
    }

    /* Unknown command */
    printf("Unknown: '%s'. Type 'help' for command list.\r\n", argv[0]);
}

/* ====================== ARGUMENT HELPERS ================================== */

/* Parse optional address from argv[1]; returns 1 (default) if omitted */

static uint8_t get_addr(int argc, char *argv[], int pos)
{
    if (argc > pos)
    {
        int val = atoi(argv[pos]);
        return (val > 0 && val <= 255) ? (uint8_t)val : 1;
    }
    return 1;
}

static void cmd_can_status(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    printf("[CAN] host id=0x%08lX\r\n", (unsigned long)smd_get_host_can_id());
    can_print_status();
}

static void cmd_can_probe(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    uint32_t found_id = 0U;

    printf("[CAN] probing addr=%u, ID=0x1000..0x100F\r\n", (unsigned int)addr);
    if (smd_probe_can_id(addr, &found_id)) {
        printf("[CAN] probe found ID=0x%08lX; host ID updated\r\n",
               (unsigned long)found_id);
    } else {
        printf("[CAN] probe found no PD42S1 response\r\n");
    }
}

static void c552_print_cdeg(uint16_t value)
{
    printf("%u.%02u", (unsigned int)(value / 100U),
           (unsigned int)(value % 100U));
}

static void c552_print_tof(const char *name, uint8_t device_bit,
                           const C552_TofData *sensor, const C552_Data *data,
                           const C552_Health *health)
{
    uint8_t stale_bit = (uint8_t)(device_bit << 4);

    if ((data->status & device_bit) == 0U) {
        printf("  %s: INVALID%s (value ignored)\r\n", name,
               ((data->status & stale_bit) != 0U) ? " STALE" : "");
        return;
    }

    printf("  %s: VALID %s filtered=%u mm min3=%u mm sample_seq=%u "
           "sample_age=%u ms%s%s\r\n", name,
           ((data->status & stale_bit) != 0U) ? "STALE" : "FRESH",
           (unsigned int)sensor->filtered_mm,
           (unsigned int)sensor->min3_raw_mm,
           (unsigned int)sensor->sample_seq,
           (unsigned int)sensor->sample_age_ms,
           ((health->data_implausible_mask & device_bit) != 0U) ?
               " IMPLAUSIBLE" : "",
           ((health->ready_mask & device_bit) != 0U) ? " READY" : "");
}

static void c552_print_k230(const char *name, uint8_t device_bit,
                            const C552_K230Data *sensor,
                            const C552_Data *data,
                            const C552_Health *health)
{
    uint8_t stale_bit = (uint8_t)(device_bit << 4);

    if ((data->status & device_bit) == 0U) {
        printf("  %s: INVALID%s (value ignored)\r\n", name,
               ((data->status & stale_bit) != 0U) ? " STALE" : "");
        return;
    }

    printf("  %s: VALID %s center=(%d,%d) rotation=(", name,
           ((data->status & stale_bit) != 0U) ? "STALE" : "FRESH",
           (int)sensor->center_x, (int)sensor->center_y);
    c552_print_cdeg(sensor->x_rotation_cdeg);
    printf(",");
    c552_print_cdeg(sensor->y_rotation_cdeg);
    printf(") deg sample_seq=%u sample_age=%u ms%s%s\r\n",
           (unsigned int)sensor->sample_seq,
           (unsigned int)sensor->sample_age_ms,
           ((health->data_implausible_mask & device_bit) != 0U) ?
               " IMPLAUSIBLE" : "",
           ((health->ready_mask & device_bit) != 0U) ? " READY" : "");
}

static void cmd_c552(int argc, char *argv[])
{
    C552_Data data;
    C552_Health health;
    C552_Diagnostics diagnostics;
    uint8_t has_snapshot;
    uint32_t age = 0U;

    (void)argc;
    (void)argv;
    has_snapshot = C552_GetSnapshot(&data, &health);
    C552_GetDiagnostics(&diagnostics);
    if (has_snapshot) {
        age = HAL_GetTick() - health.last_valid_frame_tick;
    }

    printf("[C552] link=%s warning=%s motion=%s required=0x%02X ready=0x%02X\r\n",
           health.link_online ? "ONLINE" :
               (health.link_timeout_warning ? "TIMEOUT" : "RECOVERING"),
           (health.uart_error_warning || health.dma_error_warning ||
            health.link_timeout_warning || health.sensor_invalid_mask ||
            health.sensor_stale_mask || health.data_implausible_mask) ?
               "YES" : "NO",
           health.motion_allowed ? "ALLOWED" : "STOP",
           (unsigned int)health.required_mask,
           (unsigned int)health.ready_mask);

    if (!has_snapshot) {
        printf("  No valid frame received.\r\n");
    } else {
        printf("  seq=%u status=0x%02X age=%lu ms invalid=0x%02X stale=0x%02X\r\n",
               (unsigned int)data.seq, (unsigned int)data.status,
               (unsigned long)age,
               (unsigned int)health.sensor_invalid_mask,
               (unsigned int)health.sensor_stale_mask);
        c552_print_tof("TOF1", C552_DEVICE_TOF1, &data.tof1,
                       &data, &health);
        c552_print_tof("TOF2", C552_DEVICE_TOF2, &data.tof2,
                       &data, &health);
        c552_print_k230("K230_1", C552_DEVICE_K230_1, &data.k230_1,
                        &data, &health);
        c552_print_k230("K230_2", C552_DEVICE_K230_2, &data.k230_2,
                        &data, &health);
    }

    printf("  RX bytes=%lu valid=%lu ver_err=%lu len_err=%lu id_err=%lu "
           "crc_err=%lu\r\n",
           (unsigned long)diagnostics.rx_bytes,
           (unsigned long)diagnostics.valid_frames,
           (unsigned long)diagnostics.version_errors,
           (unsigned long)diagnostics.length_errors,
           (unsigned long)diagnostics.id_errors,
           (unsigned long)diagnostics.crc_errors);
    printf("  seq_gap=%lu duplicate=%lu UART[ORE=%lu FE=%lu NE=%lu PE=%lu] "
           "DMA[err=%lu restart_fail=%lu]\r\n",
           (unsigned long)diagnostics.sequence_gaps,
           (unsigned long)diagnostics.duplicate_frames,
           (unsigned long)diagnostics.uart_ore_errors,
           (unsigned long)diagnostics.uart_fe_errors,
           (unsigned long)diagnostics.uart_ne_errors,
           (unsigned long)diagnostics.uart_pe_errors,
           (unsigned long)diagnostics.dma_errors,
           (unsigned long)diagnostics.dma_restart_failures);
}

/* ====================== MOTION COMMANDS =================================== */

static void cmd_xy_move_axis(XY_Axis axis, int argc, char *argv[])
{
    const XY_AxisConfig *config = XY_GetConfig(axis);
    int32_t delta = (int32_t)strtol(argv[1], NULL, 0);
    uint16_t speed = (argc > 2) ? (uint16_t)strtoul(argv[2], NULL, 0) :
                     config->default_speed_rpm;
    uint8_t acceleration = (argc > 3) ?
                           (uint8_t)strtoul(argv[3], NULL, 0) :
                           config->acceleration;
    XY_Result result = XY_MoveRelative(axis, delta, speed, acceleration);
    printf("[%c] move delta=%ld speed=%u acc=%u: %s\r\n",
           (axis == XY_AXIS_X) ? 'X' : 'Y', (long)delta,
           (unsigned int)speed, (unsigned int)acceleration,
           XY_ResultString(result));
}

static void cmd_x_move(int argc, char *argv[])
{
    cmd_xy_move_axis(XY_AXIS_X, argc, argv);
}

static void cmd_y_move(int argc, char *argv[])
{
    cmd_xy_move_axis(XY_AXIS_Y, argc, argv);
}

static void cmd_xy_stop(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    xy_stop_all();
    printf("[XY] stop requested for both axes\r\n");
}

static void cmd_xy_status(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    for (uint8_t i = 0U; i < XY_AXIS_COUNT; ++i) {
        XY_AxisStatus status;
        const XY_AxisConfig *config = XY_GetConfig((XY_Axis)i);
        (void)XY_GetStatus((XY_Axis)i, &status);
        printf("[%c] addr=%u state=%s fault=%u ref=%u pos=%ld target=%ld "
               "rpm=%d arrived=%u limits=%ld..%ld age=%lu\r\n",
               (i == XY_AXIS_X) ? 'X' : 'Y', config->motor_address,
               XY_StateString(status.state), (unsigned int)status.fault,
               status.position_valid, (long)status.position_pulses,
               (long)status.target_pulses, (int)status.speed_rpm,
               status.arrived, (long)config->soft_min_pulses,
               (long)config->soft_max_pulses,
               (unsigned long)(HAL_GetTick() - status.last_feedback_tick));
    }
}

static void cmd_xy_manual_result(XY_Axis axis, const char *action,
                                 XY_Result result)
{
    printf("[%c] %s: %s\r\n", (axis == XY_AXIS_X) ? 'X' : 'Y',
           action, XY_ResultString(result));
}

static void cmd_x_home(int argc, char *argv[])
{
    (void)argc; (void)argv;
    cmd_xy_manual_result(XY_AXIS_X, "MANUAL sensorless home",
                         XY_HomeSensorless(XY_AXIS_X));
}

static void cmd_y_home(int argc, char *argv[])
{
    (void)argc; (void)argv;
    cmd_xy_manual_result(XY_AXIS_Y, "MANUAL sensorless home",
                         XY_HomeSensorless(XY_AXIS_Y));
}

static void cmd_x_zero(int argc, char *argv[])
{
    (void)argc; (void)argv;
    cmd_xy_manual_result(XY_AXIS_X, "MANUAL set zero",
                         XY_SetCurrentPositionAsZero(XY_AXIS_X));
}

static void cmd_y_zero(int argc, char *argv[])
{
    (void)argc; (void)argv;
    cmd_xy_manual_result(XY_AXIS_Y, "MANUAL set zero",
                         XY_SetCurrentPositionAsZero(XY_AXIS_Y));
}

static void cmd_x_clear(int argc, char *argv[])
{
    (void)argc; (void)argv;
    cmd_xy_manual_result(XY_AXIS_X, "clear fault", XY_ClearFault(XY_AXIS_X));
}

static void cmd_y_clear(int argc, char *argv[])
{
    (void)argc; (void)argv;
    cmd_xy_manual_result(XY_AXIS_Y, "clear fault", XY_ClearFault(XY_AXIS_Y));
}

static void cmd_pos_rel(int argc, char *argv[])
{
    (void)argc;
    uint8_t  addr   = (uint8_t)atoi(argv[1]);
    uint8_t  dir    = (uint8_t)atoi(argv[2]);
    uint8_t  acc    = (uint8_t)atoi(argv[3]);
    uint16_t speed  = (uint16_t)atoi(argv[4]);
    uint32_t pulses = (uint32_t)atol(argv[5]);

    printf("REL_MOVE: addr=%d dir=%d acc=%d speed=%d pulses=%lu\r\n",
           addr, dir, acc, speed, (unsigned long)pulses);
    smd_pos_rel_move(addr, dir, acc, speed, pulses);
}

static void cmd_pos_abs(int argc, char *argv[])
{
    (void)argc;
    uint8_t  addr   = (uint8_t)atoi(argv[1]);
    uint8_t  dir    = (uint8_t)atoi(argv[2]);
    uint8_t  acc    = (uint8_t)atoi(argv[3]);
    uint16_t speed  = (uint16_t)atoi(argv[4]);
    uint32_t pulses = (uint32_t)atol(argv[5]);

    printf("ABS_MOVE: addr=%d dir=%d acc=%d speed=%d pulses=%lu\r\n",
           addr, dir, acc, speed, (unsigned long)pulses);
    smd_pos_abs_move(addr, dir, acc, speed, pulses);
}

static void cmd_speed(int argc, char *argv[])
{
    (void)argc;
    uint8_t addr = (uint8_t)atoi(argv[1]);
    uint8_t dir  = (uint8_t)atoi(argv[2]);
    uint8_t acc  = (uint8_t)atoi(argv[3]);
    float   rpm  = (float)atof(argv[4]);

    printf("SPEED: addr=%d dir=%d acc=%d rpm=%.1f\r\n", addr, dir, acc, (double)rpm);
    smd_speed_move(addr, dir, acc, rpm);
}

static void cmd_torque(int argc, char *argv[])
{
    (void)argc;
    uint8_t  addr = (uint8_t)atoi(argv[1]);
    uint8_t  dir  = (uint8_t)atoi(argv[2]);
    uint16_t ma   = (uint16_t)atoi(argv[3]);

    printf("TORQUE: addr=%d dir=%d current=%d mA\r\n", addr, dir, ma);
    smd_torque_move(addr, dir, ma);
}

static void cmd_stop(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    printf("STOP: addr=%d\r\n", addr);
    smd_stop_now(addr);
}

static void cmd_enable(int argc, char *argv[])
{
    (void)argc;
    uint8_t addr = (uint8_t)atoi(argv[1]);
    uint8_t en   = (uint8_t)atoi(argv[2]);

    printf("ENABLE: addr=%d %s\r\n", addr, en ? "Disable" : "Enable");
    smd_motor_enable(addr, en);
}

static void cmd_zero(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    printf("ANGLE_ZERO: addr=%d\r\n", addr);
    smd_send_cmd(addr, FCT_ANGLE_ZERO, NULL, 0);
}

static void cmd_clear(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    printf("CLEAR_STATE: addr=%d\r\n", addr);
    smd_clear_state(addr);
}

/* ====================== QUERY COMMANDS ==================================== */

static void cmd_read_sta(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    smd_read_motor_sta(addr);
}
static void cmd_read_pos(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    smd_read_pos(addr);
}
static void cmd_read_speed(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    smd_read_rotate_speed(addr);
}
static void cmd_read_enable(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    smd_read_enable_sta(addr);
}
static void cmd_read_arrived(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    smd_read_arrived_sta(addr);
}
static void cmd_read_vol(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    smd_read_vol(addr);
}
static void cmd_read_ma(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    smd_read_phase_ma(addr);
}
static void cmd_read_ver(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    smd_read_soft_hard_ver(addr);
}
static void cmd_read_clog(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    smd_read_clog_flag(addr);
}
static void cmd_read_pos_err(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    smd_read_pos_error(addr);
}
static void cmd_read_total(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    smd_read_total_pulse(addr);
}
static void cmd_read_sys(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    smd_read_sys_param(addr);
}

/* ====================== CONFIG COMMANDS =================================== */

static void cmd_set_addr(int argc, char *argv[])
{
    (void)argc;
    uint8_t addr     = (uint8_t)atoi(argv[1]);
    uint8_t new_addr = (uint8_t)atoi(argv[2]);
    printf("SET_ADDR: %d -> %d\r\n", addr, new_addr);
    smd_set_slave_addr(addr, new_addr);
}

static void cmd_set_can_id(int argc, char *argv[])
{
    (void)argc;
    uint8_t  addr   = (uint8_t)atoi(argv[1]);
    uint32_t can_id = (uint32_t)strtoul(argv[2], NULL, 16);
    printf("SET_CAN_ID: addr=%d id=0x%08lX\r\n", addr, (unsigned long)can_id);
    smd_set_can_id(addr, can_id);
}

static void cmd_set_mode(int argc, char *argv[])
{
    (void)argc;
    uint8_t addr = (uint8_t)atoi(argv[1]);
    uint8_t mode = (uint8_t)atoi(argv[2]);
    printf("SET_MODE: addr=%d mode=%d\r\n", addr, mode);
    smd_set_mode(addr, mode);
}

static void cmd_set_ma(int argc, char *argv[])
{
    (void)argc;
    uint8_t addr = (uint8_t)atoi(argv[1]);
    int16_t ma   = (int16_t)atoi(argv[2]);
    printf("SET_MA: addr=%d current=%d mA\r\n", addr, ma);
    smd_set_ma(addr, ma);
}

static void cmd_param_save(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    printf("PARAM_SAVE: addr=%d\r\n", addr);
    smd_param_save(addr);
}

/* ====================== SYSTEM COMMANDS =================================== */

static void cmd_restart(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    printf("RESTART: addr=%d\r\n", addr);
    smd_restart(addr);
}

static void cmd_cal(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    printf("CAL_ENCODER: addr=%d\r\n", addr);
    smd_cal_encoder(addr);
}

static void cmd_factory(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    printf("FACTORY_RESET: addr=%d\r\n", addr);
    smd_reset_factory(addr);
}

/* ====================== PUBLIC API ======================================== */

/**
 * @brief  Initialize shell (called once after USART1 init)
 */
void Shell_Init(void)
{
    CLI_StartRx();
    printf("\r\n\r\n");
    printf("==============================================\r\n");
    printf("  Smart Car — System Board v1.2\r\n");
    printf("  STM32H743IIT6 @ 480MHz (HSE 25MHz)\r\n");
    printf("  USART1 Console Ready [115200 8N1]\r\n");
#if CAN_INTERNAL_LOOPBACK_TEST
    printf("  FDCAN2 Internal Loopback [125 kbps, ID=0x%04X]\r\n", CAN_EXTID_DEFAULT);
#else
    printf("  FDCAN2 Motor Bus Ready [125 kbps, ID=0x%04X]\r\n", CAN_EXTID_DEFAULT);
#endif
    printf("==============================================\r\n");
    printf("Type 'help' for commands.\r\n\r\n");
}

/**
 * @brief  Shell polling function — call in main loop
 */
void Shell_Poll(void)
{
    char *line = CLI_GetLine();

    if (line != NULL && line[0] != '\0')
    {
        /* Echo the command */
        printf("> %s\r\n", line);

        /* Parse and execute */
        parse_and_execute(line);
    }
}

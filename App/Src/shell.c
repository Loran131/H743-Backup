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
#include "vision_calibration.h"
#include "xy_vision_align.h"
#include "motion_interfaces.h"
#include "z_axis_link.h"
#include "z_axis.h"
#include "xz_vision_calibration.h"
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
    "  k1_mode <tag|red>  request K230_1 mode, wait for APPLIED\r\n"
    "  k2_mode <tag|red>  request K230_2 mode, wait for APPLIED\r\n"
    "  grip <open|close>  test both C552 gripper channels\r\n"
    "  c552_req <mask>    set required sensor mask (0..0x1F)\r\n"
    "--------------------------------------------------\r\n"
    " X/Y CONTROL (signed pulses, + = away from zero):\r\n"
    "  x_move <delta> [rpm] [acc]   safeguarded relative move\r\n"
    "  y_move <delta> [rpm] [acc]   safeguarded relative move\r\n"
    "  xy_stop                       immediately stop both axes\r\n"
    "  xy_status                     show coordinates and state\r\n"
    "  x_home / y_home               MANUAL sensorless homing\r\n"
    "  x_zero / y_zero               MANUAL current-position zero\r\n"
    "  x_clear / y_clear             clear control-layer fault\r\n"
    "  p2_start [k230] [xstep] [ystep]  start XY vision calibration\r\n"
    "  p2_status / p2_ref / p2_abort    inspect, capture reference, abort\r\n"
    "  p2_save / p2_load / p2_reset     manage EEPROM calibration\r\n"
    "  p2_set <k230> <refx> <refy> <i00> <i01> <i10> <i11>\r\n"
    "  p3_start [xstep=51200] [zstep=64000]  start K230_2 XZ red calibration\r\n"
    "  p3_status / p3_ref / p3_abort         inspect, capture reference, abort\r\n"
    "  p3_save / p3_load / p3_reset          manage XZ calibration EEPROM\r\n"
    "  align_start / align_status / align_abort  XY visual alignment\r\n"
    "--------------------------------------------------\r\n"
    " Z CONTROL (signed pulses, + = away from zero):\r\n"
    "  z_zero                            set current position to zero\r\n"
    "  z_move <delta_pulses> [speed_hz]  relative move / unreferenced jog\r\n"
    "  z_stop                            emergency stop\r\n"
    "  z_clear                           clear fault; zero is required again\r\n"
    "  z_status                          link and motor state\r\n"
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
static void cmd_k1_mode(int argc, char *argv[]);
static void cmd_k2_mode(int argc, char *argv[]);
static void cmd_grip(int argc, char *argv[]);
static void cmd_c552_required(int argc, char *argv[]);
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
static void cmd_p2_start(int argc, char *argv[]);
static void cmd_p2_status(int argc, char *argv[]);
static void cmd_p2_ref(int argc, char *argv[]);
static void cmd_p2_abort(int argc, char *argv[]);
static void cmd_p2_save(int argc, char *argv[]);
static void cmd_p2_load(int argc, char *argv[]);
static void cmd_p2_reset(int argc, char *argv[]);
static void cmd_p2_set(int argc, char *argv[]);
static void cmd_align_start(int argc, char *argv[]);
static void cmd_align_status(int argc, char *argv[]);
static void cmd_align_abort(int argc, char *argv[]);
static void cmd_p3_start(int argc, char *argv[]);
static void cmd_p3_status(int argc, char *argv[]);
static void cmd_p3_ref(int argc, char *argv[]);
static void cmd_p3_abort(int argc, char *argv[]);
static void cmd_p3_save(int argc, char *argv[]);
static void cmd_p3_load(int argc, char *argv[]);
static void cmd_p3_reset(int argc, char *argv[]);
static void cmd_z_move(int argc, char *argv[]);
static void cmd_z_zero(int argc, char *argv[]);
static void cmd_z_stop(int argc, char *argv[]);
static void cmd_z_clear(int argc, char *argv[]);
static void cmd_z_status(int argc, char *argv[]);

static const ShellCmd_t g_cmd_table[] = {
    {"cans",      "",           1, cmd_can_status},
    {"canprobe",  "[addr]",     1, cmd_can_probe},
    {"C552",      "",           1, cmd_c552},
    {"k1_mode",   "<tag|red>",  2, cmd_k1_mode},
    {"k2_mode",   "<tag|red>",  2, cmd_k2_mode},
    {"grip",      "<open|close>", 2, cmd_grip},
    {"c552_req",  "<mask>",     2, cmd_c552_required},
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
    {"p2_start",   "[k230=1|2] [xstep=51200] [ystep=12800]", 1, cmd_p2_start},
    {"p2_status",  "",           1, cmd_p2_status},
    {"p2_ref",     "",           1, cmd_p2_ref},
    {"p2_abort",   "",           1, cmd_p2_abort},
    {"p2_save",    "",           1, cmd_p2_save},
    {"p2_load",    "",           1, cmd_p2_load},
    {"p2_reset",   "",           1, cmd_p2_reset},
    {"p2_set", "<k230> <refx> <refy> <i00> <i01> <i10> <i11>", 8, cmd_p2_set},
    {"p3_start",   "[xstep=51200] [zstep=64000]", 1, cmd_p3_start},
    {"p3_status",  "",          1, cmd_p3_status},
    {"p3_ref",     "",          1, cmd_p3_ref},
    {"p3_abort",   "",          1, cmd_p3_abort},
    {"p3_save",    "",          1, cmd_p3_save},
    {"p3_load",    "",          1, cmd_p3_load},
    {"p3_reset",   "",          1, cmd_p3_reset},
    {"align_start", "",          1, cmd_align_start},
    {"align_status", "",         1, cmd_align_status},
    {"align_abort", "",          1, cmd_align_abort},
    {"z_zero",     "",          1, cmd_z_zero},
    {"z_move",     "<delta_pulses> [speed_hz=90000]", 2, cmd_z_move},
    {"z_stop",     "",          1, cmd_z_stop},
    {"z_clear",    "",          1, cmd_z_clear},
    {"z_status",   "",          1, cmd_z_status},
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
                           const C552_TofData *sensor,
                           const C552_Health *health)
{
    if ((health->valid_mask & device_bit) == 0U) {
        printf("  %s: INVALID%s (value ignored)\r\n", name,
               ((health->stale_mask & device_bit) != 0U) ? " STALE" : "");
        return;
    }

    printf("  %s: VALID %s filtered=%u mm min3=%u mm sample_seq=%u "
           "sample_age=%u ms%s%s\r\n", name,
           ((health->stale_mask & device_bit) != 0U) ? "STALE" : "FRESH",
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
                            const C552_Health *health)
{
    if ((health->valid_mask & device_bit) == 0U) {
        printf("  %s: INVALID%s (value ignored)\r\n", name,
               ((health->stale_mask & device_bit) != 0U) ? " STALE" : "");
        return;
    }

    printf("  %s: VALID %s center=(%d,%d) rotation=(", name,
           ((health->stale_mask & device_bit) != 0U) ? "STALE" : "FRESH",
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
    C552_CommandStatus command_status;
    uint8_t has_snapshot;
    uint32_t age = 0U;

    (void)argc;
    (void)argv;
    has_snapshot = C552_GetSnapshot(&data, &health);
    C552_GetDiagnostics(&diagnostics);
    C552_GetCommandStatus(&command_status);
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
        printf("  V3.1 seq=%u status=0x%02X tof3_flags=0x%02X age=%lu ms "
               "valid=0x%02X invalid=0x%02X stale=0x%02X\r\n",
               (unsigned int)data.seq, (unsigned int)data.status,
               (unsigned int)data.tof3_flags,
               (unsigned long)age,
               (unsigned int)health.valid_mask,
               (unsigned int)health.sensor_invalid_mask,
               (unsigned int)health.sensor_stale_mask);
        c552_print_tof("TOF1", C552_DEVICE_TOF1, &data.tof1,
                       &health);
        c552_print_tof("TOF2", C552_DEVICE_TOF2, &data.tof2,
                       &health);
        c552_print_k230("K230_1", C552_DEVICE_K230_1, &data.k230_1,
                        &health);
        c552_print_k230("K230_2", C552_DEVICE_K230_2, &data.k230_2,
                        &health);
        c552_print_tof("TOF3", C552_DEVICE_TOF3, &data.tof3, &health);
    }

    printf("  RX bytes=%lu valid=%lu ver_err=%lu len_err=%lu id_err=%lu "
           "crc_err=%lu\r\n",
           (unsigned long)diagnostics.rx_bytes,
           (unsigned long)diagnostics.valid_frames,
           (unsigned long)diagnostics.version_errors,
           (unsigned long)diagnostics.length_errors,
           (unsigned long)diagnostics.id_errors,
           (unsigned long)diagnostics.crc_errors);
    printf("  ACK valid=%lu format_err=%lu unexpected=%lu "
           "TX[cmd=%lu done=%lu start_err=%lu err=%lu timeout=%lu]\r\n",
           (unsigned long)diagnostics.valid_ack_frames,
           (unsigned long)diagnostics.ack_format_errors,
           (unsigned long)diagnostics.unexpected_acks,
           (unsigned long)diagnostics.tx_commands,
           (unsigned long)diagnostics.tx_completed,
           (unsigned long)diagnostics.tx_start_errors,
           (unsigned long)diagnostics.tx_errors,
           (unsigned long)diagnostics.command_timeouts);
    printf("  CMD state=%s id=0x%02X cmd=0x%02X seq=0x%02X request=0x%02X "
           "result=0x%02X(%s) response=0x%02X\r\n",
           C552_CommandStateString(command_status.state),
           (unsigned int)command_status.id,
           (unsigned int)command_status.command,
           (unsigned int)command_status.sequence,
           (unsigned int)command_status.requested_value,
           (unsigned int)command_status.result,
           C552_AckResultString(command_status.result),
           (unsigned int)command_status.response_value);
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

static void cmd_k1_mode(int argc, char *argv[])
{
    C552_K230Mode mode;
    C552_RequestResult result;
    (void)argc;

    if (strcmp(argv[1], "tag") == 0) {
        mode = C552_K230_MODE_APRILTAG;
    } else if (strcmp(argv[1], "red") == 0) {
        mode = C552_K230_MODE_RED_BLOCK;
    } else {
        printf("Usage: k1_mode <tag|red>\r\n");
        return;
    }
    result = C552_SetK230Mode(C552_ID_K230_1, mode, HAL_GetTick());
    printf("[C552] K230_1 mode=%u request=%s; final success requires "
           "CMD state=APPLIED\r\n",
           (unsigned int)mode,
           (result == C552_REQUEST_OK) ? "QUEUED" :
               ((result == C552_REQUEST_BUSY) ? "BUSY" : "INVALID"));
}

static void cmd_k2_mode(int argc, char *argv[])
{
    C552_K230Mode mode;
    C552_RequestResult result;
    (void)argc;

    if ((VisionCalibration_IsActive() != 0U) ||
        (XZCalibration_IsActive() != 0U) ||
        (XY_VisionAlign_IsActive() != 0U)) {
        printf("[C552] K230_2 mode rejected: visual operation active\r\n");
        return;
    }
    if (strcmp(argv[1], "tag") == 0) {
        mode = C552_K230_MODE_APRILTAG;
    } else if (strcmp(argv[1], "red") == 0) {
        mode = C552_K230_MODE_RED_BLOCK;
    } else {
        printf("Usage: k2_mode <tag|red>\r\n");
        return;
    }
    result = C552_SetK230Mode(C552_ID_K230_2, mode, HAL_GetTick());
    printf("[C552] K230_2 mode=%u request=%s; final success requires "
           "CMD state=APPLIED\r\n",
           (unsigned int)mode,
           (result == C552_REQUEST_OK) ? "QUEUED" :
               ((result == C552_REQUEST_BUSY) ? "BUSY" : "INVALID"));
}

static void cmd_grip(int argc, char *argv[])
{
    C552_GripperState state;
    C552_RequestResult result;
    (void)argc;

    if (strcmp(argv[1], "open") == 0) {
        state = C552_GRIPPER_OPEN;
    } else if (strcmp(argv[1], "close") == 0) {
        state = C552_GRIPPER_CLOSED;
    } else {
        printf("Usage: grip <open|close>\r\n");
        return;
    }
    result = C552_SetGripper(C552_GRIPPER_BOTH, state, HAL_GetTick());
    printf("[C552] gripper=%u request=%s; final success requires "
           "CMD state=APPLIED\r\n",
           (unsigned int)state,
           (result == C552_REQUEST_OK) ? "QUEUED" :
               ((result == C552_REQUEST_BUSY) ? "BUSY" : "INVALID"));
}

static void cmd_c552_required(int argc, char *argv[])
{
    unsigned long mask;
    (void)argc;
    mask = strtoul(argv[1], NULL, 0);
    if ((mask > C552_DEVICE_ALL) ||
        (C552_SetRequiredMask((uint8_t)mask) == 0U)) {
        printf("Usage: c552_req <0..0x1F>\r\n");
        return;
    }
    printf("[C552] required mask=0x%02lX\r\n", mask);
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
    XY_Result result;
    if ((VisionCalibration_ManualMotionAllowed() == 0U) ||
        (XZCalibration_ManualMotionAllowed() == 0U) ||
        ((axis == XY_AXIS_Y) && (XZCalibration_IsActive() != 0U)) ||
        (XY_VisionAlign_IsActive() != 0U)) {
        printf("[XY] automatic operation owns X/Y; abort it first\r\n");
        return;
    }
    result = XY_MoveRelative(axis, delta, speed, acceleration);
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
    XY_VisionAlign_Abort();
    XZCalibration_Abort();
    if (VisionCalibration_ManualMotionAllowed() == 0U) {
        VisionCalibration_Abort();
    } else {
        VisionCalibration_Abort();
        xy_stop_all();
    }
    printf("[XY] stop requested for both axes\r\n");
}

static void cmd_xy_status(int argc, char *argv[])
{
    XY_StartupStatus startup;
    (void)argc;
    (void)argv;
    XY_GetStartupStatus(&startup);
    printf("[XY] startup=%s replies=0x%02X elapsed=%lu ms\r\n",
           XY_StartupStateString(startup.state),
           (unsigned int)startup.response_mask,
           (unsigned long)((startup.finish_tick != 0U ? startup.finish_tick :
                            HAL_GetTick()) - startup.start_tick));
    for (uint8_t i = 0U; i < XY_AXIS_COUNT; ++i) {
        XY_AxisStatus status;
        const XY_AxisConfig *config = XY_GetConfig((XY_Axis)i);
        (void)XY_GetStatus((XY_Axis)i, &status);
        uint32_t completion_ms = (status.completion_tick != 0U) ?
                                 status.completion_tick - status.command_tick : 0U;
        printf("[%c] addr=%u state=%s fault=%u(%s) ref=%u pos=%ld target=%ld "
               "rpm=%d arrived=%u replied=%u limits=%ld..%ld age=%lu "
               "last_cmd=0x%02X completion=%s/%lu ms releases=A%lu/S%lu "
               "reply_age=A%lu/S%lu fault_cmd=0x%02X home_stage=%u retries=%u\r\n",
               (i == XY_AXIS_X) ? 'X' : 'Y', config->motor_address,
               XY_StateString(status.state), (unsigned int)status.fault,
               XY_FaultString(status.fault),
               status.position_valid, (long)status.position_pulses,
               (long)status.target_pulses, (int)status.speed_rpm,
               status.arrived, status.response_seen,
               (long)config->soft_min_pulses,
               (long)config->soft_max_pulses,
               (unsigned long)(HAL_GetTick() - status.last_feedback_tick),
               (unsigned int)status.last_command_function,
               XY_CompletionSourceString(status.completion_source),
               (unsigned long)completion_ms,
               (unsigned long)status.arrived_release_count,
               (unsigned long)status.static_release_count,
               (unsigned long)(status.last_arrived_reply_tick != 0U ?
                   HAL_GetTick() - status.last_arrived_reply_tick : 0U),
               (unsigned long)(status.last_static_reply_tick != 0U ?
                   HAL_GetTick() - status.last_static_reply_tick : 0U),
               (unsigned int)status.fault_function,
               (unsigned int)status.fault_home_stage,
               (unsigned int)status.home_retry_count);
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
    if ((VisionCalibration_ManualMotionAllowed() == 0U) ||
        (XZCalibration_IsActive() != 0U) ||
        (XY_VisionAlign_IsActive() != 0U)) {
        printf("[XY] automatic operation owns X/Y; abort it first\r\n");
        return;
    }
    cmd_xy_manual_result(XY_AXIS_X, "MANUAL sensorless home",
                         XY_HomeSensorless(XY_AXIS_X));
}

static void cmd_y_home(int argc, char *argv[])
{
    (void)argc; (void)argv;
    if ((VisionCalibration_ManualMotionAllowed() == 0U) ||
        (XZCalibration_IsActive() != 0U) ||
        (XY_VisionAlign_IsActive() != 0U)) {
        printf("[XY] automatic operation owns X/Y; abort it first\r\n");
        return;
    }
    cmd_xy_manual_result(XY_AXIS_Y, "MANUAL sensorless home",
                         XY_HomeSensorless(XY_AXIS_Y));
}

static void cmd_x_zero(int argc, char *argv[])
{
    (void)argc; (void)argv;
    if ((VisionCalibration_ManualMotionAllowed() == 0U) ||
        (XZCalibration_IsActive() != 0U) ||
        (XY_VisionAlign_IsActive() != 0U)) {
        printf("[XY] automatic operation owns X/Y; abort it first\r\n");
        return;
    }
    cmd_xy_manual_result(XY_AXIS_X, "MANUAL set zero",
                         XY_SetCurrentPositionAsZero(XY_AXIS_X));
}

static void cmd_y_zero(int argc, char *argv[])
{
    (void)argc; (void)argv;
    if ((VisionCalibration_ManualMotionAllowed() == 0U) ||
        (XZCalibration_IsActive() != 0U) ||
        (XY_VisionAlign_IsActive() != 0U)) {
        printf("[XY] automatic operation owns X/Y; abort it first\r\n");
        return;
    }
    cmd_xy_manual_result(XY_AXIS_Y, "MANUAL set zero",
                         XY_SetCurrentPositionAsZero(XY_AXIS_Y));
}

static void cmd_x_clear(int argc, char *argv[])
{
    (void)argc; (void)argv;
    if (XZCalibration_IsActive() != 0U) {
        printf("[X] rejected: P3 calibration active\r\n");
        return;
    }
    cmd_xy_manual_result(XY_AXIS_X, "clear fault", XY_ClearFault(XY_AXIS_X));
}

static void cmd_y_clear(int argc, char *argv[])
{
    (void)argc; (void)argv;
    if (XZCalibration_IsActive() != 0U) {
        printf("[Y] rejected: P3 calibration active\r\n");
        return;
    }
    cmd_xy_manual_result(XY_AXIS_Y, "clear fault", XY_ClearFault(XY_AXIS_Y));
}

static void p2_print_float6(float value)
{
    int32_t scaled = (int32_t)(value * 1000000.0f);
    uint32_t magnitude;
    if (scaled < 0) {
        printf("-");
        magnitude = (uint32_t)(-(int64_t)scaled);
    } else {
        magnitude = (uint32_t)scaled;
    }
    printf("%lu.%06lu", (unsigned long)(magnitude / 1000000U),
           (unsigned long)(magnitude % 1000000U));
}

static void cmd_p2_status(int argc, char *argv[])
{
    VisionCalibrationStatus status;
    (void)argc; (void)argv;
    VisionCalibration_GetStatus(&status);
    printf("[P2] storage=%s generation=%lu runtime_calibrated=%u\r\n",
           VisionCalibration_StorageStateString(status.storage_state),
           (unsigned long)status.storage_generation,
           (unsigned int)status.result.valid);
    printf("[P2] state=%s fault=%s k230=%u samples=%u seq=%u "
           "pos=(%ld,%ld) base=(%ld,%ld) step=(%ld,%ld) pixel=(%d,%d)\r\n",
           VisionCalibration_StateString(status.state),
           VisionCalibration_FaultString(status.fault),
           (unsigned int)((status.k230_id == VISION_CAL_K230_2_ID) ? 2U : 1U),
           (unsigned int)status.sample_count,
           (unsigned int)status.last_sample_seq,
           (long)status.position_pulses[0], (long)status.position_pulses[1],
           (long)status.base_pulses[0], (long)status.base_pulses[1],
           (long)status.step_pulses[0], (long)status.step_pulses[1],
           (int)status.latest_pixel[0], (int)status.latest_pixel[1]);
    printf("[P2] J pixel/pulse=[[ ");
    p2_print_float6(status.result.pixel_per_pulse[0][0]); printf(", ");
    p2_print_float6(status.result.pixel_per_pulse[0][1]); printf(" ], [ ");
    p2_print_float6(status.result.pixel_per_pulse[1][0]); printf(", ");
    p2_print_float6(status.result.pixel_per_pulse[1][1]);
    printf(" ]] valid=%u ref=(%d,%d)\r\n", status.result.valid,
           (int)status.result.reference_pixel[0],
           (int)status.result.reference_pixel[1]);
    printf("[P2] J^-1 pulse/pixel=[[ ");
    p2_print_float6(status.result.pulse_per_pixel[0][0]); printf(", ");
    p2_print_float6(status.result.pulse_per_pixel[0][1]); printf(" ], [ ");
    p2_print_float6(status.result.pulse_per_pixel[1][0]); printf(", ");
    p2_print_float6(status.result.pulse_per_pixel[1][1]); printf(" ]]\r\n");
}

static void cmd_p2_start(int argc, char *argv[])
{
    unsigned long k230 = (argc > 1) ? strtoul(argv[1], NULL, 0) : 1UL;
    long x_step = (argc > 2) ? strtol(argv[2], NULL, 0) : 51200L;
    long y_step = (argc > 3) ? strtol(argv[3], NULL, 0) : 12800L;
    uint8_t id = (k230 == 2UL) ? VISION_CAL_K230_2_ID :
                 ((k230 == 1UL) ? VISION_CAL_K230_1_ID : 0U);
    if ((XZCalibration_IsActive() != 0U) ||
        (XY_VisionAlign_IsActive() != 0U) || (id == 0U) ||
        (VisionCalibration_Start(id, (int32_t)x_step, (int32_t)y_step,
                                 HAL_GetTick()) == 0U)) {
        printf("[P2] start rejected: require valid arguments and both axes "
               "referenced/IDLE after startup homing\r\n");
        return;
    }
    printf("[P2] started K230_%lu: xstep=%ld ystep=%ld pulses\r\n",
           k230, x_step, y_step);
}

static void cmd_p2_ref(int argc, char *argv[])
{
    VisionCalibrationStatus p2;
    XY_AxisStatus x;
    XY_AxisStatus y;
    (void)argc; (void)argv;
    if (VisionCalibration_CaptureReference(HAL_GetTick()) == 0U) {
        VisionCalibration_GetStatus(&p2);
        (void)XY_GetStatus(XY_AXIS_X, &x);
        (void)XY_GetStatus(XY_AXIS_Y, &y);
        printf("[P2] reference rejected: state=%s fault=%s matrix_valid=%u "
               "X=%s Y=%s; require successful calibration state "
               "MANUAL_ALIGN\r\n",
               VisionCalibration_StateString(p2.state),
               VisionCalibration_FaultString(p2.fault),
               (unsigned int)p2.result.valid,
               XY_StateString(x.state), XY_StateString(y.state));
    } else {
        printf("[P2] reference queued; waiting for both axes IDLE\r\n");
    }
}

static void cmd_p2_abort(int argc, char *argv[])
{
    (void)argc; (void)argv;
    VisionCalibration_Abort();
    printf("[P2] aborted\r\n");
}

static void cmd_p2_save(int argc, char *argv[])
{
    (void)argc; (void)argv;
    printf("[P2] EEPROM save: %s\r\n",
           VisionCalibration_Save() ? "OK" : "FAILED");
}

static void cmd_p2_load(int argc, char *argv[])
{
    (void)argc; (void)argv;
    printf("[P2] EEPROM load: %s\r\n",
           VisionCalibration_Load() ? "OK" : "FAILED");
}

static void cmd_p2_reset(int argc, char *argv[])
{
    (void)argc; (void)argv;
    printf("[P2] EEPROM reset: %s\r\n",
           VisionCalibration_ResetStored() ? "OK" : "FAILED");
}

static void cmd_p2_set(int argc, char *argv[])
{
    unsigned long k230 = strtoul(argv[1], NULL, 0);
    long reference_x = strtol(argv[2], NULL, 0);
    long reference_y = strtol(argv[3], NULL, 0);
    float inverse[2][2];
    uint8_t id;
    (void)argc;

    id = (k230 == 1UL) ? VISION_CAL_K230_1_ID :
         ((k230 == 2UL) ? VISION_CAL_K230_2_ID : 0U);
    if ((id == 0U) || (reference_x < INT16_MIN) ||
        (reference_x > INT16_MAX) || (reference_y < INT16_MIN) ||
        (reference_y > INT16_MAX)) {
        printf("[P2] set rejected: invalid K230 or reference pixel\r\n");
        return;
    }
    inverse[0][0] = strtof(argv[4], NULL);
    inverse[0][1] = strtof(argv[5], NULL);
    inverse[1][0] = strtof(argv[6], NULL);
    inverse[1][1] = strtof(argv[7], NULL);
    printf("[P2] set inverse and EEPROM save: %s\r\n",
           VisionCalibration_SetInverse(id, (int16_t)reference_x,
                                        (int16_t)reference_y, inverse) ?
               "OK" : "FAILED");
}

static void cmd_align_start(int argc, char *argv[])
{
    (void)argc; (void)argv;
    if ((VisionCalibration_IsActive() != 0U) ||
        (XZCalibration_IsActive() != 0U)) {
        printf("[ALIGN] rejected: calibration active\r\n");
        return;
    }
    printf("[ALIGN] start: %s, period=%lu ms\r\n",
           XY_VisionAlign_Start(HAL_GetTick()) ? "OK" : "REJECTED",
           (unsigned long)XY_VISION_ALIGN_PERIOD_MS);
}

static void cmd_align_status(int argc, char *argv[])
{
    XY_VisionAlignStatus status;
    (void)argc; (void)argv;
    XY_VisionAlign_GetStatus(&status);
    printf("[ALIGN] state=%s fault=%s seq=%u pixel=(%d,%d) error=(%d,%d) "
           "step=(%ld,%ld) stable=%u corrections=%lu sample_age=%lu ms\r\n",
           XY_VisionAlign_StateString(status.state),
           XY_VisionAlign_FaultString(status.fault),
           (unsigned int)status.last_sample_seq,
           (int)status.pixel[0], (int)status.pixel[1],
           (int)status.error_pixel[0], (int)status.error_pixel[1],
           (long)status.requested_pulses[0],
           (long)status.requested_pulses[1],
           (unsigned int)status.stable_samples,
           (unsigned long)status.corrections,
           (unsigned long)(HAL_GetTick() - status.last_sample_tick));
    printf("[ALIGN] last_move=%s axis=%c pos=(%ld,%ld) target=(%ld,%ld)\r\n",
           XY_ResultString(status.last_move_result),
           (status.failed_axis == XY_AXIS_X) ? 'X' :
               ((status.failed_axis == XY_AXIS_Y) ? 'Y' : '-'),
           (long)status.axis_position[0], (long)status.axis_position[1],
           (long)status.attempted_target[0],
           (long)status.attempted_target[1]);
}

static void cmd_align_abort(int argc, char *argv[])
{
    (void)argc; (void)argv;
    XY_VisionAlign_Abort();
    printf("[ALIGN] aborted\r\n");
}

static void cmd_p3_start(int argc, char *argv[])
{
    const XY_AxisConfig *x_config = XY_GetConfig(XY_AXIS_X);
    XY_AxisStatus x_status;
    ZAxisControlStatus z_status;
    C552_CommandStatus command_status;
    long x_step = (argc > 1) ? strtol(argv[1], NULL, 0) :
                               XZ_CAL_DEFAULT_X_STEP_PULSES;
    long z_step = (argc > 2) ? strtol(argv[2], NULL, 0) :
                               XZ_CAL_DEFAULT_Z_STEP_PULSES;

    if ((x_step <= 0) || (x_step > INT32_MAX) ||
        (z_step <= 0) || (z_step > INT32_MAX)) {
        printf("[P3] start rejected: xstep and zstep must be > 0\r\n");
        return;
    }
    (void)XY_GetStatus(XY_AXIS_X, &x_status);
    ZAxis_GetControlStatus(&z_status);
    C552_GetCommandStatus(&command_status);
    if (XZCalibration_Start((int32_t)x_step, (int32_t)z_step,
                            HAL_GetTick()) == 0U) {
        printf("[P3] start rejected: X state=%s ref=%u pos=%ld "
               "required=%ld..%ld limits=%ld..%ld\r\n",
               XY_StateString(x_status.state),
               (unsigned int)x_status.position_valid,
               (long)x_status.position_pulses,
               (long)(x_config->soft_min_pulses + x_step),
               (long)(x_config->soft_max_pulses - x_step),
               (long)x_config->soft_min_pulses,
               (long)x_config->soft_max_pulses);
        printf("[P3] Z state=%s ref=%u pos=%ld required=%ld..%ld "
               "limits=%ld..%ld; P2=%u ALIGN=%u C552_CMD=%s\r\n",
               ZAxis_StateString(z_status.state),
               (unsigned int)z_status.position_valid,
               (long)z_status.position_pulses,
               (long)(Z_AXIS_SOFT_MIN_PULSES + z_step),
               (long)(Z_AXIS_SOFT_MAX_PULSES - z_step),
               (long)Z_AXIS_SOFT_MIN_PULSES,
               (long)Z_AXIS_SOFT_MAX_PULSES,
               (unsigned int)VisionCalibration_IsActive(),
               (unsigned int)XY_VisionAlign_IsActive(),
               C552_CommandStateString(command_status.state));
        return;
    }
    printf("[P3] K230_2 RED calibration started: xstep=%ld zstep=%ld\r\n",
           x_step, z_step);
}

static void cmd_p3_status(int argc, char *argv[])
{
    XZCalibrationStatus status;
    (void)argc; (void)argv;
    XZCalibration_GetStatus(&status);
    printf("[P3] state=%s fault=%s sample=%u seq=%u pos=(%ld,%ld) "
           "base=(%ld,%ld) step=(%ld,%ld) pixel=(%d,%d)\r\n",
           XZCalibration_StateString(status.state),
           XZCalibration_FaultString(status.fault),
           (unsigned int)status.sample_count,
           (unsigned int)status.last_sample_seq,
           (long)status.position_pulses[0], (long)status.position_pulses[1],
           (long)status.base_pulses[0], (long)status.base_pulses[1],
           (long)status.step_pulses[0], (long)status.step_pulses[1],
           (int)status.latest_pixel[0], (int)status.latest_pixel[1]);
    printf("[P3] valid=%u ref=(%d,%d) storage=%s generation=%lu\r\n",
           (unsigned int)status.result.valid,
           (int)status.result.reference_pixel[0],
           (int)status.result.reference_pixel[1],
           VisionCalibration_StorageStateString(status.storage_state),
           (unsigned long)status.storage_generation);
    printf("[P3] J=[[%.9f, %.9f], [%.9f, %.9f]]\r\n",
           (double)status.result.pixel_per_pulse[0][0],
           (double)status.result.pixel_per_pulse[0][1],
           (double)status.result.pixel_per_pulse[1][0],
           (double)status.result.pixel_per_pulse[1][1]);
    printf("[P3] inv=[[%.6f, %.6f], [%.6f, %.6f]]\r\n",
           (double)status.result.pulse_per_pixel[0][0],
           (double)status.result.pulse_per_pixel[0][1],
           (double)status.result.pulse_per_pixel[1][0],
           (double)status.result.pulse_per_pixel[1][1]);
}

static void cmd_p3_ref(int argc, char *argv[])
{
    (void)argc; (void)argv;
    printf("[P3] capture reference: %s\r\n",
           XZCalibration_CaptureReference(HAL_GetTick()) ?
           "QUEUED" : "REJECTED (require MANUAL_ALIGN)");
}

static void cmd_p3_abort(int argc, char *argv[])
{
    (void)argc; (void)argv;
    XZCalibration_Abort();
    printf("[P3] aborted\r\n");
}

static void cmd_p3_save(int argc, char *argv[])
{
    (void)argc; (void)argv;
    if (XZCalibration_IsActive() != 0U) {
        printf("[P3] EEPROM save: REJECTED (calibration active)\r\n");
        return;
    }
    printf("[P3] EEPROM save: %s\r\n", XZCalibration_Save() ? "OK" : "FAILED");
}

static void cmd_p3_load(int argc, char *argv[])
{
    (void)argc; (void)argv;
    if (XZCalibration_IsActive() != 0U) {
        printf("[P3] EEPROM load: REJECTED (calibration active)\r\n");
        return;
    }
    printf("[P3] EEPROM load: %s\r\n", XZCalibration_Load() ? "OK" : "FAILED");
}

static void cmd_p3_reset(int argc, char *argv[])
{
    (void)argc; (void)argv;
    if (XZCalibration_IsActive() != 0U) {
        printf("[P3] EEPROM reset: REJECTED (calibration active)\r\n");
        return;
    }
    printf("[P3] EEPROM reset: %s\r\n",
           XZCalibration_ResetStored() ? "OK" : "FAILED");
}

static void cmd_z_move(int argc, char *argv[])
{
    long pulses = strtol(argv[1], NULL, 0);
    unsigned long speed = (argc > 2) ? strtoul(argv[2], NULL, 0) :
                                      Z_AXIS_DEFAULT_SPEED_HZ;
    ZAxisControlResult result;

    if (XZCalibration_ManualMotionAllowed() == 0U) {
        printf("[Z] rejected: P3 XZ calibration owns motion\r\n");
        return;
    }

    if ((pulses < INT32_MIN) || (pulses > INT32_MAX) ||
        (speed > UINT32_MAX)) {
        printf("[Z] invalid numeric range\r\n");
        return;
    }
    result = ZAxisControl_MoveRelative((int32_t)pulses, (uint32_t)speed);
    printf("[Z] move pulses=%ld speed=%lu Hz: %s\r\n",
           pulses, speed, ZAxis_ResultString(result));
    if (result == Z_RESULT_OK) {
        ZAxisControlStatus status;
        ZAxis_GetControlStatus(&status);
        if (status.position_valid == 0U) {
            printf("[Z] unreferenced jog: coordinate remains invalid; "
                   "run z_zero at the mechanical zero\r\n");
        }
    }
}

static void cmd_z_zero(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    if (XZCalibration_IsActive() != 0U) {
        printf("[Z] rejected: P3 calibration active\r\n");
        return;
    }
    printf("[Z] set current position as zero: %s\r\n",
           ZAxis_ResultString(ZAxisControl_SetZero()));
}

static void cmd_z_stop(int argc, char *argv[])
{
    ZAxisControlResult result;
    (void)argc;
    (void)argv;
    if (XZCalibration_IsActive() != 0U) {
        XZCalibration_Abort();
        printf("[Z] stop requested; P3 aborted\r\n");
        return;
    }
    result = ZAxisControl_Stop();
    printf("[Z] stop: %s\r\n", ZAxis_ResultString(result));
}

static void cmd_z_clear(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    if (XZCalibration_IsActive() != 0U) {
        printf("[Z] rejected: P3 calibration active\r\n");
        return;
    }
    printf("[Z] clear fault: %s\r\n",
           ZAxis_ResultString(ZAxisControl_ClearFault()));
}

static void cmd_z_status(int argc, char *argv[])
{
    ZAxisStatus status;
    ZAxisControlStatus control;
    (void)argc;
    (void)argv;
    ZAxisLink_GetStatus(&status);
    ZAxis_GetControlStatus(&control);
    printf("[Z] control=%s fault=%s referenced=%u position=%ld "
           "target=%ld limits=[%ld,%ld] default=%lu Hz\r\n",
           ZAxis_StateString(control.state), ZAxis_FaultString(control.fault),
           (unsigned int)control.position_valid,
           (long)control.position_pulses, (long)control.target_pulses,
           (long)Z_AXIS_SOFT_MIN_PULSES, (long)Z_AXIS_SOFT_MAX_PULSES,
           (unsigned long)Z_AXIS_DEFAULT_SPEED_HZ);
    printf("[Z] state=%s rx=%s response=0x%02X status=0x%02X speed=%lu "
           "steps=%lu age=%lu ms\r\n",
           ZAxisLink_StateString(status.state),
           status.rx_ready ? "READY" : "DOWN",
           (unsigned int)status.last_response_command,
           (unsigned int)status.last_status,
           (unsigned long)status.actual_speed_hz,
           (unsigned long)status.completed_steps,
           (unsigned long)(HAL_GetTick() - status.last_response_tick));
    printf("[Z] frames=%lu crc=%lu format=%lu unexpected=%lu uart=%lu "
           "timeouts=%lu\r\n",
           (unsigned long)status.valid_frames,
           (unsigned long)status.crc_errors,
           (unsigned long)status.frame_errors,
           (unsigned long)status.unexpected_frames,
           (unsigned long)status.uart_errors,
           (unsigned long)status.timeouts);
}

static void cmd_pos_rel(int argc, char *argv[])
{
    (void)argc;
    if (XZCalibration_IsActive() != 0U) {
        printf("REL_MOVE rejected: P3 calibration active\r\n");
        return;
    }
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
    if (XZCalibration_IsActive() != 0U) {
        printf("ABS_MOVE rejected: P3 calibration active\r\n");
        return;
    }
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
    if (XZCalibration_IsActive() != 0U) {
        printf("SPEED rejected: P3 calibration active\r\n");
        return;
    }
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
    if (XZCalibration_IsActive() != 0U) {
        printf("TORQUE rejected: P3 calibration active\r\n");
        return;
    }
    uint8_t  addr = (uint8_t)atoi(argv[1]);
    uint8_t  dir  = (uint8_t)atoi(argv[2]);
    uint16_t ma   = (uint16_t)atoi(argv[3]);

    printf("TORQUE: addr=%d dir=%d current=%d mA\r\n", addr, dir, ma);
    smd_torque_move(addr, dir, ma);
}

static void cmd_stop(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    if (XZCalibration_IsActive() != 0U) {
        XZCalibration_Abort();
        printf("STOP: P3 aborted; tracked axis stop requested\r\n");
        return;
    }
    printf("STOP: addr=%d\r\n", addr);
    smd_stop_now(addr);
}

static void cmd_enable(int argc, char *argv[])
{
    (void)argc;
    if (XZCalibration_IsActive() != 0U) {
        printf("ENABLE rejected: P3 calibration active\r\n");
        return;
    }
    uint8_t addr = (uint8_t)atoi(argv[1]);
    uint8_t en   = (uint8_t)atoi(argv[2]);

    printf("ENABLE: addr=%d %s\r\n", addr, en ? "Disable" : "Enable");
    smd_motor_enable(addr, en);
}

static void cmd_zero(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    if (XZCalibration_IsActive() != 0U) {
        printf("ANGLE_ZERO rejected: P3 calibration active\r\n");
        return;
    }
    printf("ANGLE_ZERO: addr=%d\r\n", addr);
    smd_send_cmd(addr, FCT_ANGLE_ZERO, NULL, 0);
}

static void cmd_clear(int argc, char *argv[])
{
    uint8_t addr = get_addr(argc, argv, 1);
    if (XZCalibration_IsActive() != 0U) {
        printf("CLEAR_STATE rejected: P3 calibration active\r\n");
        return;
    }
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

#include "xz_vision_calibration.h"

#include "c552.h"
#include "eeprom.h"
#include "main.h"
#include "xy_motor.h"
#include "z_axis.h"
#include <stddef.h>
#include <string.h>

#define XZ_CAL_SAMPLE_COUNT        5U
#define XZ_CAL_PIXEL_SPREAD_MAX    4
#define XZ_CAL_SETTLE_MS           200U
#define XZ_CAL_SAMPLE_TIMEOUT_MS   5000U
#define XZ_CAL_MODE_TIMEOUT_MS     1000U
#define XZ_CAL_X_SPEED_RPM         300U
#define XZ_CAL_Z_SPEED_HZ          90000U
#define XZ_CAL_ACCELERATION        200U
#define XZ_CAL_EEPROM_ADDRESS      64U
#define XZ_CAL_STORAGE_MAGIC       0x335A5843UL
#define XZ_CAL_STORAGE_VERSION     1U
#define XZ_CAL_STORAGE_LENGTH      56U
#define XZ_CAL_STORAGE_CRC_OFFSET  52U

typedef struct {
    uint8_t count;
    uint16_t last_seq;
    int32_t sum_x;
    int32_t sum_y;
    int16_t min_x;
    int16_t max_x;
    int16_t min_y;
    int16_t max_y;
} XZCollector;

typedef struct {
    int32_t motor[2];
    int16_t pixel[2];
} XZPoint;

XZVisionCalibration g_xz_vision_calibration;
static XZCalibrationStatus g_cal;
static XZCollector g_collector;
static XZPoint g_base;
static XZPoint g_x_pos;
static XZPoint g_x_neg;
static XZPoint g_z_pos;
static XZPoint g_z_neg;
static uint8_t g_action_started;
static uint8_t g_sample_unstable;
static uint16_t g_mode_applied_seq;
static uint32_t g_deadline;

static uint8_t get_red_sample(C552_K230Data *sensor);

static float xz_absf(float value) { return (value < 0.0f) ? -value : value; }

static void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value; data[1] = (uint8_t)(value >> 8U);
}

static void write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value; data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U); data[3] = (uint8_t)(value >> 24U);
}

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
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

static uint8_t float_valid(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return ((bits & 0x7F800000UL) != 0x7F800000UL) ? 1U : 0U;
}

static uint8_t result_sane(const XZVisionCalibration *result)
{
    float det;
    if ((result == NULL) || (result->valid == 0U)) return 0U;
    for (uint8_t r = 0U; r < 2U; ++r) {
        for (uint8_t c = 0U; c < 2U; ++c) {
            if ((float_valid(result->pixel_per_pulse[r][c]) == 0U) ||
                (float_valid(result->pulse_per_pixel[r][c]) == 0U)) return 0U;
        }
    }
    det = result->pixel_per_pulse[0][0] * result->pixel_per_pulse[1][1] -
          result->pixel_per_pulse[0][1] * result->pixel_per_pulse[1][0];
    return (xz_absf(det) >= 1.0e-12f) ? 1U : 0U;
}

static uint8_t state_active(XZCalibrationState state)
{
    return ((state != XZ_CAL_IDLE) && (state != XZ_CAL_COMPLETE) &&
            (state != XZ_CAL_FAULT)) ? 1U : 0U;
}

static void set_state(XZCalibrationState state, uint32_t now)
{
    g_cal.state = state;
    g_cal.state_tick = now;
    g_action_started = 0U;
}

static void set_fault(XZCalibrationFault fault, uint32_t now)
{
    g_cal.fault = fault;
    g_cal.result.valid = 0U;
    set_state(XZ_CAL_FAULT, now);
}

static uint8_t get_positions(int32_t *x, int32_t *z,
                             XY_AxisStatus *xs, ZAxisControlStatus *zs)
{
    if ((XY_GetStatus(XY_AXIS_X, xs) == 0U) || (zs == NULL)) return 0U;
    ZAxis_GetControlStatus(zs);
    if (x != NULL) *x = xs->position_pulses;
    if (z != NULL) *z = zs->position_pulses;
    return 1U;
}

static uint8_t axes_idle(void)
{
    XY_AxisStatus x;
    ZAxisControlStatus z;
    return (get_positions(NULL, NULL, &x, &z) &&
            (x.state == XY_STATE_IDLE) && (x.position_valid != 0U) &&
            (z.state == Z_STATE_IDLE) && (z.position_valid != 0U)) ? 1U : 0U;
}

static uint8_t axes_faulted(void)
{
    XY_AxisStatus x;
    ZAxisControlStatus z;
    if (get_positions(NULL, NULL, &x, &z) == 0U) return 1U;
    return ((x.state == XY_STATE_FAULT) || (z.state == Z_STATE_FAULT)) ? 1U : 0U;
}

static void reset_collector(void)
{
    memset(&g_collector, 0, sizeof(g_collector));
    g_collector.last_seq = g_mode_applied_seq;
    g_cal.sample_count = 0U;
}

static void begin_sampling(XZCalibrationState state, uint32_t now)
{
    C552_K230Data sensor;
    reset_collector();
    if (get_red_sample(&sensor) != 0U) {
        g_collector.last_seq = sensor.sample_seq;
    }
    g_sample_unstable = 0U;
    set_state(state, now);
    g_deadline = now + XZ_CAL_SAMPLE_TIMEOUT_MS;
}

static uint8_t get_red_sample(C552_K230Data *sensor)
{
    C552_Data data;
    C552_Health health;
    if ((C552_GetSnapshot(&data, &health) == 0U) ||
        ((health.ready_mask & C552_DEVICE_K230_2) == 0U)) return 0U;
    *sensor = data.k230_2;
    return 1U;
}

static uint8_t collect_sample(uint32_t now, XZPoint *point)
{
    C552_K230Data sensor;
    XY_AxisStatus x;
    ZAxisControlStatus z;
    if ((uint32_t)(now - g_cal.state_tick) < XZ_CAL_SETTLE_MS) return 0U;
    if ((get_red_sample(&sensor) == 0U) ||
        (sensor.sample_seq == g_collector.last_seq)) return 0U;
    g_collector.last_seq = sensor.sample_seq;
    g_cal.last_sample_seq = sensor.sample_seq;
    g_cal.latest_pixel[0] = sensor.center_x;
    g_cal.latest_pixel[1] = sensor.center_y;
    if (g_collector.count == 0U) {
        g_collector.min_x = g_collector.max_x = sensor.center_x;
        g_collector.min_y = g_collector.max_y = sensor.center_y;
    } else {
        if (sensor.center_x < g_collector.min_x) g_collector.min_x = sensor.center_x;
        if (sensor.center_x > g_collector.max_x) g_collector.max_x = sensor.center_x;
        if (sensor.center_y < g_collector.min_y) g_collector.min_y = sensor.center_y;
        if (sensor.center_y > g_collector.max_y) g_collector.max_y = sensor.center_y;
    }
    g_collector.sum_x += sensor.center_x;
    g_collector.sum_y += sensor.center_y;
    ++g_collector.count;
    g_cal.sample_count = g_collector.count;
    if (g_collector.count < XZ_CAL_SAMPLE_COUNT) return 0U;
    if (((g_collector.max_x - g_collector.min_x) > XZ_CAL_PIXEL_SPREAD_MAX) ||
        ((g_collector.max_y - g_collector.min_y) > XZ_CAL_PIXEL_SPREAD_MAX)) {
        uint16_t rejected_seq = g_collector.last_seq;
        g_sample_unstable = 1U;
        reset_collector();
        g_collector.last_seq = rejected_seq;
        return 0U;
    }
    if (get_positions(&point->motor[0], &point->motor[1], &x, &z) == 0U) return 0U;
    point->pixel[0] = (int16_t)(g_collector.sum_x / XZ_CAL_SAMPLE_COUNT);
    point->pixel[1] = (int16_t)(g_collector.sum_y / XZ_CAL_SAMPLE_COUNT);
    return 1U;
}

static uint8_t start_x_move(int32_t target)
{
    XY_Result result = XY_MoveAbsolute(XY_AXIS_X, target,
                                       XZ_CAL_X_SPEED_RPM,
                                       XZ_CAL_ACCELERATION);
    return (result == XY_RESULT_OK) ? 1U : ((result == XY_RESULT_BUSY) ? 2U : 0U);
}

static uint8_t start_z_move(int32_t target)
{
    ZAxisControlResult result =
        ZAxisControl_MoveAbsolute(target, XZ_CAL_Z_SPEED_HZ);
    return (result == Z_RESULT_OK) ? 1U : ((result == Z_RESULT_BUSY) ? 2U : 0U);
}

static void poll_move(uint8_t z_axis, int32_t target,
                      XZCalibrationState next, uint8_t sample_next,
                      uint32_t now)
{
    if (g_action_started == 0U) {
        uint8_t result = z_axis ? start_z_move(target) : start_x_move(target);
        if (result == 1U) g_action_started = 1U;
        else if (result == 0U) set_fault(XZ_CAL_FAULT_MOVE, now);
    } else if (axes_idle() != 0U) {
        if (sample_next != 0U) begin_sampling(next, now);
        else set_state(next, now);
    }
}

static uint8_t fit_matrix(void)
{
    float dx = (float)(g_x_pos.motor[0] - g_x_neg.motor[0]);
    float dz = (float)(g_z_pos.motor[1] - g_z_neg.motor[1]);
    float j00, j10, j01, j11, det, nx2, nz2;
    if ((xz_absf(dx) < 1.0f) || (xz_absf(dz) < 1.0f)) return 0U;
    j00 = (float)(g_x_pos.pixel[0] - g_x_neg.pixel[0]) / dx;
    j10 = (float)(g_x_pos.pixel[1] - g_x_neg.pixel[1]) / dx;
    j01 = (float)(g_z_pos.pixel[0] - g_z_neg.pixel[0]) / dz;
    j11 = (float)(g_z_pos.pixel[1] - g_z_neg.pixel[1]) / dz;
    det = j00 * j11 - j01 * j10;
    nx2 = j00 * j00 + j10 * j10;
    nz2 = j01 * j01 + j11 * j11;
    if ((nx2 < 1.0e-12f) || (nz2 < 1.0e-12f) ||
        ((det * det) < (0.0025f * nx2 * nz2))) return 0U;
    g_cal.result.pixel_per_pulse[0][0] = j00;
    g_cal.result.pixel_per_pulse[0][1] = j01;
    g_cal.result.pixel_per_pulse[1][0] = j10;
    g_cal.result.pixel_per_pulse[1][1] = j11;
    g_cal.result.pulse_per_pixel[0][0] = j11 / det;
    g_cal.result.pulse_per_pixel[0][1] = -j01 / det;
    g_cal.result.pulse_per_pixel[1][0] = -j10 / det;
    g_cal.result.pulse_per_pixel[1][1] = j00 / det;
    return 1U;
}

void XZCalibration_Init(uint32_t now)
{
    memset(&g_cal, 0, sizeof(g_cal));
    memset(&g_xz_vision_calibration, 0, sizeof(g_xz_vision_calibration));
    g_cal.state = XZ_CAL_IDLE;
    g_cal.state_tick = now;
    g_cal.storage_state = VISION_CAL_STORAGE_NOT_LOADED;
    (void)XZCalibration_Load();
}

uint8_t XZCalibration_Start(int32_t x_step, int32_t z_step, uint32_t now)
{
    const XY_AxisConfig *x_cfg = XY_GetConfig(XY_AXIS_X);
    XY_AxisStatus x;
    ZAxisControlStatus z;
    VisionCalibrationStorageState storage_state = g_cal.storage_state;
    uint32_t generation = g_cal.storage_generation;
    C552_RequestResult request;
    if ((state_active(g_cal.state) != 0U) ||
        (VisionCalibration_IsActive() != 0U) ||
        (x_step <= 0) || (z_step <= 0) ||
        (get_positions(NULL, NULL, &x, &z) == 0U) ||
        (x.state != XY_STATE_IDLE) || (x.position_valid == 0U) ||
        (z.state != Z_STATE_IDLE) || (z.position_valid == 0U) ||
        ((int64_t)x.position_pulses - x_step < x_cfg->soft_min_pulses) ||
        ((int64_t)x.position_pulses + x_step > x_cfg->soft_max_pulses) ||
        ((int64_t)z.position_pulses - z_step < Z_AXIS_SOFT_MIN_PULSES) ||
        ((int64_t)z.position_pulses + z_step > Z_AXIS_SOFT_MAX_PULSES)) return 0U;
    request = C552_SetK230Mode(C552_ID_K230_2, C552_K230_MODE_RED_BLOCK, now);
    if (request != C552_REQUEST_OK) return 0U;
    memset(&g_cal, 0, sizeof(g_cal));
    memset(&g_base, 0, sizeof(g_base));
    memset(&g_x_pos, 0, sizeof(g_x_pos));
    memset(&g_x_neg, 0, sizeof(g_x_neg));
    memset(&g_z_pos, 0, sizeof(g_z_pos));
    memset(&g_z_neg, 0, sizeof(g_z_neg));
    g_cal.storage_state = storage_state;
    g_cal.storage_generation = generation;
    g_cal.step_pulses[0] = x_step;
    g_cal.step_pulses[1] = z_step;
    g_cal.base_pulses[0] = x.position_pulses;
    g_cal.base_pulses[1] = z.position_pulses;
    set_state(XZ_CAL_WAIT_RED_MODE, now);
    g_deadline = now + XZ_CAL_MODE_TIMEOUT_MS;
    return 1U;
}

uint8_t XZCalibration_CaptureReference(uint32_t now)
{
    if (g_cal.state != XZ_CAL_MANUAL_ALIGN) return 0U;
    set_state(XZ_CAL_WAIT_REFERENCE_IDLE, now);
    return 1U;
}

void XZCalibration_Abort(void)
{
    XY_AxisStatus x;
    ZAxisControlStatus z;
    if (state_active(g_cal.state) == 0U) return;
    if ((XY_GetStatus(XY_AXIS_X, &x) != 0U) &&
        ((x.state == XY_STATE_STARTING) || (x.state == XY_STATE_MOVING) ||
         (x.state == XY_STATE_STOPPING))) {
        XY_Stop(XY_AXIS_X);
    }
    ZAxis_GetControlStatus(&z);
    if ((z.state == Z_STATE_STARTING) || (z.state == Z_STATE_MOVING) ||
        (z.state == Z_STATE_STOPPING)) {
        (void)ZAxisControl_Stop();
    }
    g_cal.result.valid = 0U;
    g_cal.fault = XZ_CAL_FAULT_NONE;
    set_state(XZ_CAL_IDLE, HAL_GetTick());
}

void XZCalibration_Poll(uint32_t now)
{
    C552_CommandStatus command;
    C552_K230Data sensor;
    XY_AxisStatus x;
    ZAxisControlStatus z;
    if (state_active(g_cal.state) == 0U) return;
    if (axes_faulted() != 0U) { set_fault(XZ_CAL_FAULT_AXIS, now); return; }
    if (get_positions(&g_cal.position_pulses[0], &g_cal.position_pulses[1],
                      &x, &z) == 0U) { set_fault(XZ_CAL_FAULT_AXIS, now); return; }
    if (((g_cal.state == XZ_CAL_WAIT_RED_MODE) ||
         (g_cal.state == XZ_CAL_SAMPLE_BASE) ||
         (g_cal.state == XZ_CAL_SAMPLE_X_POS) ||
         (g_cal.state == XZ_CAL_SAMPLE_X_NEG) ||
         (g_cal.state == XZ_CAL_SAMPLE_Z_POS) ||
         (g_cal.state == XZ_CAL_SAMPLE_Z_NEG) ||
         (g_cal.state == XZ_CAL_CAPTURE_REFERENCE)) &&
        ((int32_t)(now - g_deadline) >= 0)) {
        set_fault(g_sample_unstable ? XZ_CAL_FAULT_UNSTABLE :
                  ((g_cal.state == XZ_CAL_WAIT_RED_MODE) ? XZ_CAL_FAULT_MODE :
                   XZ_CAL_FAULT_VISION_TIMEOUT), now);
        return;
    }
    switch (g_cal.state) {
    case XZ_CAL_WAIT_RED_MODE:
        C552_GetCommandStatus(&command);
        if ((command.id == C552_ID_K230_2) &&
            (command.command == C552_COMMAND_SET_K230_MODE) &&
            (command.requested_value == C552_K230_MODE_RED_BLOCK) &&
            (command.state == C552_COMMAND_APPLIED)) {
            g_mode_applied_seq = get_red_sample(&sensor) ? sensor.sample_seq : 0U;
            set_state(XZ_CAL_MOVE_BASE_X, now);
        } else if ((command.state == C552_COMMAND_FAILED) ||
                   (command.state == C552_COMMAND_TIMEOUT)) {
            set_fault(XZ_CAL_FAULT_MODE, now);
        }
        break;
    case XZ_CAL_MOVE_BASE_X:
        poll_move(0U, g_cal.base_pulses[0], XZ_CAL_MOVE_BASE_Z, 0U, now); break;
    case XZ_CAL_MOVE_BASE_Z:
        poll_move(1U, g_cal.base_pulses[1], XZ_CAL_SAMPLE_BASE, 1U, now); break;
    case XZ_CAL_SAMPLE_BASE:
        if (collect_sample(now, &g_base)) set_state(XZ_CAL_MOVE_X_POS, now);
        break;
    case XZ_CAL_MOVE_X_POS:
        poll_move(0U, g_cal.base_pulses[0] + g_cal.step_pulses[0],
                  XZ_CAL_SAMPLE_X_POS, 1U, now); break;
    case XZ_CAL_SAMPLE_X_POS:
        if (collect_sample(now, &g_x_pos)) set_state(XZ_CAL_MOVE_X_NEG, now);
        break;
    case XZ_CAL_MOVE_X_NEG:
        poll_move(0U, g_cal.base_pulses[0] - g_cal.step_pulses[0],
                  XZ_CAL_SAMPLE_X_NEG, 1U, now); break;
    case XZ_CAL_SAMPLE_X_NEG:
        if (collect_sample(now, &g_x_neg)) set_state(XZ_CAL_RETURN_X, now);
        break;
    case XZ_CAL_RETURN_X:
        poll_move(0U, g_cal.base_pulses[0], XZ_CAL_MOVE_Z_POS, 0U, now); break;
    case XZ_CAL_MOVE_Z_POS:
        poll_move(1U, g_cal.base_pulses[1] + g_cal.step_pulses[1],
                  XZ_CAL_SAMPLE_Z_POS, 1U, now); break;
    case XZ_CAL_SAMPLE_Z_POS:
        if (collect_sample(now, &g_z_pos)) set_state(XZ_CAL_MOVE_Z_NEG, now);
        break;
    case XZ_CAL_MOVE_Z_NEG:
        poll_move(1U, g_cal.base_pulses[1] - g_cal.step_pulses[1],
                  XZ_CAL_SAMPLE_Z_NEG, 1U, now); break;
    case XZ_CAL_SAMPLE_Z_NEG:
        if (collect_sample(now, &g_z_neg)) set_state(XZ_CAL_RETURN_Z, now);
        break;
    case XZ_CAL_RETURN_Z:
        poll_move(1U, g_cal.base_pulses[1], XZ_CAL_FIT, 0U, now); break;
    case XZ_CAL_FIT:
        if (fit_matrix() == 0U) set_fault(XZ_CAL_FAULT_SINGULAR_MATRIX, now);
        else set_state(XZ_CAL_MANUAL_ALIGN, now);
        break;
    case XZ_CAL_WAIT_REFERENCE_IDLE:
        if (axes_idle()) begin_sampling(XZ_CAL_CAPTURE_REFERENCE, now);
        break;
    case XZ_CAL_CAPTURE_REFERENCE:
        if (collect_sample(now, &g_base)) {
            g_cal.result.reference_pixel[0] = g_base.pixel[0];
            g_cal.result.reference_pixel[1] = g_base.pixel[1];
            g_cal.result.valid = 1U;
            g_xz_vision_calibration = g_cal.result;
            (void)XZCalibration_Save();
            set_state(XZ_CAL_COMPLETE, now);
        }
        break;
    default: break;
    }
}

uint8_t XZCalibration_Save(void)
{
    uint8_t record[XZ_CAL_STORAGE_LENGTH];
    uint16_t offset = 20U;
    uint32_t generation = g_cal.storage_generation + 1U;
    if (result_sane(&g_xz_vision_calibration) == 0U) return 0U;
    memset(record, 0, sizeof(record));
    write_u16(&record[4], XZ_CAL_STORAGE_VERSION);
    write_u16(&record[6], XZ_CAL_STORAGE_LENGTH);
    write_u32(&record[8], generation);
    record[12] = C552_ID_K230_2;
    record[13] = 0U; record[14] = 2U;
    write_u16(&record[16], (uint16_t)g_xz_vision_calibration.reference_pixel[0]);
    write_u16(&record[18], (uint16_t)g_xz_vision_calibration.reference_pixel[1]);
    for (uint8_t matrix = 0U; matrix < 2U; ++matrix) {
        float (*values)[2] = matrix ? g_xz_vision_calibration.pulse_per_pixel :
                                     g_xz_vision_calibration.pixel_per_pulse;
        for (uint8_t r = 0U; r < 2U; ++r) for (uint8_t c = 0U; c < 2U; ++c) {
            uint32_t bits; memcpy(&bits, &values[r][c], sizeof(bits));
            write_u32(&record[offset], bits); offset += 4U;
        }
    }
    write_u32(&record[XZ_CAL_STORAGE_CRC_OFFSET],
              storage_crc32(&record[4], XZ_CAL_STORAGE_CRC_OFFSET - 4U));
    if ((EEPROM_Write(XZ_CAL_EEPROM_ADDRESS, record, 4U) != HAL_OK) ||
        (EEPROM_Write(XZ_CAL_EEPROM_ADDRESS + 4U, &record[4],
                      XZ_CAL_STORAGE_LENGTH - 4U) != HAL_OK)) {
        g_cal.storage_state = VISION_CAL_STORAGE_IO_ERROR; return 0U;
    }
    write_u32(record, XZ_CAL_STORAGE_MAGIC);
    if (EEPROM_Write(XZ_CAL_EEPROM_ADDRESS, record, 4U) != HAL_OK) {
        g_cal.storage_state = VISION_CAL_STORAGE_IO_ERROR; return 0U;
    }
    g_cal.storage_generation = generation;
    g_cal.storage_state = VISION_CAL_STORAGE_VALID;
    return 1U;
}

uint8_t XZCalibration_Load(void)
{
    uint8_t record[XZ_CAL_STORAGE_LENGTH];
    XZVisionCalibration loaded;
    uint16_t offset = 20U;
    if (EEPROM_Read(XZ_CAL_EEPROM_ADDRESS, record, sizeof(record)) != HAL_OK) {
        g_cal.storage_state = VISION_CAL_STORAGE_IO_ERROR; return 0U;
    }
    if (read_u32(record) != XZ_CAL_STORAGE_MAGIC) {
        g_cal.storage_state = VISION_CAL_STORAGE_EMPTY; return 0U;
    }
    if ((read_u16(&record[4]) != XZ_CAL_STORAGE_VERSION) ||
        (read_u16(&record[6]) != XZ_CAL_STORAGE_LENGTH) ||
        (read_u32(&record[XZ_CAL_STORAGE_CRC_OFFSET]) !=
         storage_crc32(&record[4], XZ_CAL_STORAGE_CRC_OFFSET - 4U))) {
        g_cal.storage_state = VISION_CAL_STORAGE_INVALID; return 0U;
    }
    memset(&loaded, 0, sizeof(loaded));
    loaded.reference_pixel[0] = (int16_t)read_u16(&record[16]);
    loaded.reference_pixel[1] = (int16_t)read_u16(&record[18]);
    for (uint8_t matrix = 0U; matrix < 2U; ++matrix) {
        float (*values)[2] = matrix ? loaded.pulse_per_pixel : loaded.pixel_per_pulse;
        for (uint8_t r = 0U; r < 2U; ++r) for (uint8_t c = 0U; c < 2U; ++c) {
            uint32_t bits = read_u32(&record[offset]);
            memcpy(&values[r][c], &bits, sizeof(bits)); offset += 4U;
        }
    }
    loaded.valid = 1U;
    if (result_sane(&loaded) == 0U) {
        g_cal.storage_state = VISION_CAL_STORAGE_INVALID; return 0U;
    }
    g_xz_vision_calibration = loaded;
    g_cal.result = loaded;
    g_cal.storage_generation = read_u32(&record[8]);
    g_cal.storage_state = VISION_CAL_STORAGE_VALID;
    return 1U;
}

uint8_t XZCalibration_ResetStored(void)
{
    uint8_t zero[4] = {0U, 0U, 0U, 0U};
    if (EEPROM_Write(XZ_CAL_EEPROM_ADDRESS, zero, sizeof(zero)) != HAL_OK) {
        g_cal.storage_state = VISION_CAL_STORAGE_IO_ERROR; return 0U;
    }
    memset(&g_xz_vision_calibration, 0, sizeof(g_xz_vision_calibration));
    memset(&g_cal.result, 0, sizeof(g_cal.result));
    g_cal.storage_generation = 0U;
    g_cal.storage_state = VISION_CAL_STORAGE_EMPTY;
    return 1U;
}

void XZCalibration_GetStatus(XZCalibrationStatus *status)
{
    if (status != NULL) *status = g_cal;
}

uint8_t XZCalibration_ManualMotionAllowed(void)
{
    return ((g_cal.state == XZ_CAL_IDLE) ||
            (g_cal.state == XZ_CAL_MANUAL_ALIGN) ||
            (g_cal.state == XZ_CAL_COMPLETE) ||
            (g_cal.state == XZ_CAL_FAULT)) ? 1U : 0U;
}

uint8_t XZCalibration_IsActive(void) { return state_active(g_cal.state); }

const char *XZCalibration_StateString(XZCalibrationState state)
{
    static const char *const names[] = {
        "IDLE", "WAIT_RED_MODE", "MOVE_BASE_X", "MOVE_BASE_Z", "SAMPLE_BASE",
        "MOVE_X_POS", "SAMPLE_X_POS", "MOVE_X_NEG", "SAMPLE_X_NEG", "RETURN_X",
        "MOVE_Z_POS", "SAMPLE_Z_POS", "MOVE_Z_NEG", "SAMPLE_Z_NEG", "RETURN_Z",
        "FIT", "MANUAL_ALIGN", "WAIT_REFERENCE_IDLE", "CAPTURE_REFERENCE",
        "COMPLETE", "FAULT"
    };
    return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ? names[state] : "UNKNOWN";
}

const char *XZCalibration_FaultString(XZCalibrationFault fault)
{
    static const char *const names[] = {
        "NONE", "BUSY", "ARGUMENT", "MODE", "AXIS", "MOVE",
        "VISION_TIMEOUT", "UNSTABLE", "SINGULAR_MATRIX"
    };
    return ((uint32_t)fault < (sizeof(names) / sizeof(names[0]))) ? names[fault] : "UNKNOWN";
}

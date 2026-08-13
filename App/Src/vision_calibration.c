#include "vision_calibration.h"
#include "c552.h"
#include "eeprom.h"
#include "motion_interfaces.h"
#include <stddef.h>
#include <string.h>

#define VISION_CAL_SAMPLE_COUNT       5U
#define VISION_CAL_PIXEL_SPREAD_MAX   4
#define VISION_CAL_SETTLE_MS          200U
#define VISION_CAL_SAMPLE_TIMEOUT_MS  5000U
#define VISION_CAL_X_SPEED_RPM        300U
#define VISION_CAL_Y_SPEED_RPM        5U
#define VISION_CAL_ACCELERATION       200U
#define VISION_CAL_EEPROM_ADDRESS     0U
#define VISION_CAL_STORAGE_MAGIC      0x324C4143UL
#define VISION_CAL_STORAGE_VERSION    1U
#define VISION_CAL_STORAGE_LENGTH     56U
#define VISION_CAL_STORAGE_CRC_OFFSET 52U

typedef struct {
    uint8_t count;
    uint16_t last_seq;
    int32_t sum_x;
    int32_t sum_y;
    int16_t min_x;
    int16_t max_x;
    int16_t min_y;
    int16_t max_y;
} VisionSampleCollector;

typedef struct {
    int32_t motor[2];
    int16_t pixel[2];
} VisionCalibrationPoint;

static VisionCalibrationStatus g_cal;
static VisionSampleCollector g_collector;
static VisionCalibrationPoint g_base;
static VisionCalibrationPoint g_x_pos;
static VisionCalibrationPoint g_x_neg;
static VisionCalibrationPoint g_y_pos;
static VisionCalibrationPoint g_y_neg;
static uint8_t g_action_started;
static uint8_t g_sample_unstable_seen;
static uint32_t g_sample_deadline_tick;

static float vision_absf(float value);

static void vision_write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void vision_write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static uint16_t vision_read_u16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t vision_read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static uint32_t vision_storage_crc32(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < length; ++i) {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

static uint8_t vision_float_valid(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return ((bits & 0x7F800000UL) != 0x7F800000UL) ? 1U : 0U;
}

static uint8_t vision_result_sane(const XY_VisionCalibration *calibration)
{
    float determinant;
    float identity00;
    float identity01;
    float identity10;
    float identity11;
    uint8_t row;
    uint8_t column;

    if ((calibration->calibrated == 0U) ||
        ((calibration->k230_id != VISION_CAL_K230_1_ID) &&
         (calibration->k230_id != VISION_CAL_K230_2_ID)) ||
        (calibration->axes[0] != XY_AXIS_X) ||
        (calibration->axes[1] != XY_AXIS_Y)) return 0U;
    for (row = 0U; row < 2U; ++row) {
        for (column = 0U; column < 2U; ++column) {
            if ((vision_float_valid(calibration->pixel_per_pulse[row][column]) == 0U) ||
                (vision_float_valid(calibration->pulse_per_pixel[row][column]) == 0U)) {
                return 0U;
            }
        }
    }
    determinant =
        calibration->pixel_per_pulse[0][0] *
        calibration->pixel_per_pulse[1][1] -
        calibration->pixel_per_pulse[0][1] *
        calibration->pixel_per_pulse[1][0];
    if (vision_absf(determinant) < 1.0e-12f) return 0U;
    identity00 = calibration->pixel_per_pulse[0][0] *
                 calibration->pulse_per_pixel[0][0] +
                 calibration->pixel_per_pulse[0][1] *
                 calibration->pulse_per_pixel[1][0];
    identity01 = calibration->pixel_per_pulse[0][0] *
                 calibration->pulse_per_pixel[0][1] +
                 calibration->pixel_per_pulse[0][1] *
                 calibration->pulse_per_pixel[1][1];
    identity10 = calibration->pixel_per_pulse[1][0] *
                 calibration->pulse_per_pixel[0][0] +
                 calibration->pixel_per_pulse[1][1] *
                 calibration->pulse_per_pixel[1][0];
    identity11 = calibration->pixel_per_pulse[1][0] *
                 calibration->pulse_per_pixel[0][1] +
                 calibration->pixel_per_pulse[1][1] *
                 calibration->pulse_per_pixel[1][1];
    if ((vision_absf(identity00 - 1.0f) > 0.02f) ||
        (vision_absf(identity01) > 0.02f) ||
        (vision_absf(identity10) > 0.02f) ||
        (vision_absf(identity11 - 1.0f) > 0.02f)) return 0U;
    return 1U;
}

static void vision_publish_result(const VisionCalibrationResult *result)
{
    g_xy_vision_calibration.k230_id = result->k230_id;
    g_xy_vision_calibration.axes[0] = result->axes[0];
    g_xy_vision_calibration.axes[1] = result->axes[1];
    memcpy(g_xy_vision_calibration.reference_pixel, result->reference_pixel,
           sizeof(g_xy_vision_calibration.reference_pixel));
    memcpy(g_xy_vision_calibration.pixel_per_pulse, result->pixel_per_pulse,
           sizeof(g_xy_vision_calibration.pixel_per_pulse));
    memcpy(g_xy_vision_calibration.pulse_per_pixel, result->pulse_per_pixel,
           sizeof(g_xy_vision_calibration.pulse_per_pixel));
    g_xy_vision_calibration.calibrated = 1U;
}

static void vision_copy_runtime_to_result(void)
{
    g_cal.result.k230_id = g_xy_vision_calibration.k230_id;
    g_cal.result.axes[0] = g_xy_vision_calibration.axes[0];
    g_cal.result.axes[1] = g_xy_vision_calibration.axes[1];
    memcpy(g_cal.result.reference_pixel,
           g_xy_vision_calibration.reference_pixel,
           sizeof(g_cal.result.reference_pixel));
    memcpy(g_cal.result.pixel_per_pulse,
           g_xy_vision_calibration.pixel_per_pulse,
           sizeof(g_cal.result.pixel_per_pulse));
    memcpy(g_cal.result.pulse_per_pixel,
           g_xy_vision_calibration.pulse_per_pixel,
           sizeof(g_cal.result.pulse_per_pixel));
    g_cal.result.valid = g_xy_vision_calibration.calibrated;
}

static void vision_finish_move(VisionCalibrationState next, uint32_t now);

static float vision_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t vision_state_active(VisionCalibrationState state)
{
    return ((state != VISION_CAL_IDLE) &&
            (state != VISION_CAL_COMPLETE) &&
            (state != VISION_CAL_FAULT)) ? 1U : 0U;
}

static void vision_set_state(VisionCalibrationState state, uint32_t now)
{
    g_cal.state = state;
    g_cal.state_tick = now;
    g_action_started = 0U;
}

static void vision_set_fault(VisionCalibrationFault fault, uint32_t now)
{
    g_cal.fault = fault;
    g_cal.result.valid = 0U;
    vision_set_state(VISION_CAL_FAULT, now);
}

static uint8_t vision_get_axes(XY_AxisStatus *x, XY_AxisStatus *y)
{
    return (XY_GetStatus(XY_AXIS_X, x) && XY_GetStatus(XY_AXIS_Y, y)) ?
           1U : 0U;
}

static void vision_reset_collector(void)
{
    memset(&g_collector, 0, sizeof(g_collector));
    g_cal.sample_count = 0U;
}

static void vision_begin_sampling(VisionCalibrationState state, uint32_t now)
{
    vision_reset_collector();
    g_sample_unstable_seen = 0U;
    vision_set_state(state, now);
    g_sample_deadline_tick = now + VISION_CAL_SAMPLE_TIMEOUT_MS;
}

static uint8_t vision_get_k230(C552_K230Data *sensor, uint16_t *seq)
{
    C552_Data data;
    C552_Health health;
    uint8_t mask;

    if (C552_GetSnapshot(&data, &health) == 0U) return 0U;
    if (g_cal.k230_id == VISION_CAL_K230_1_ID) {
        mask = C552_DEVICE_K230_1;
        *sensor = data.k230_1;
    } else if (g_cal.k230_id == VISION_CAL_K230_2_ID) {
        mask = C552_DEVICE_K230_2;
        *sensor = data.k230_2;
    } else {
        return 0U;
    }
    if ((health.ready_mask & mask) == 0U) return 0U;
    *seq = sensor->sample_seq;
    return 1U;
}

static uint8_t vision_collect_sample(uint32_t now, VisionCalibrationPoint *point)
{
    C552_K230Data sensor;
    XY_AxisStatus x;
    XY_AxisStatus y;
    uint16_t seq;

    if ((uint32_t)(now - g_cal.state_tick) < VISION_CAL_SETTLE_MS) return 0U;
    if (vision_get_k230(&sensor, &seq) == 0U) return 0U;
    if ((g_collector.count != 0U) && (seq == g_collector.last_seq)) return 0U;
    g_collector.last_seq = seq;
    g_cal.last_sample_seq = seq;
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
    if (g_collector.count < VISION_CAL_SAMPLE_COUNT) return 0U;

    if (((g_collector.max_x - g_collector.min_x) > VISION_CAL_PIXEL_SPREAD_MAX) ||
        ((g_collector.max_y - g_collector.min_y) > VISION_CAL_PIXEL_SPREAD_MAX)) {
        g_sample_unstable_seen = 1U;
        vision_reset_collector();
        return 0U;
    }
    if (vision_get_axes(&x, &y) == 0U) return 0U;
    point->motor[0] = x.position_pulses;
    point->motor[1] = y.position_pulses;
    point->pixel[0] = (int16_t)(g_collector.sum_x / VISION_CAL_SAMPLE_COUNT);
    point->pixel[1] = (int16_t)(g_collector.sum_y / VISION_CAL_SAMPLE_COUNT);
    return 1U;
}

static uint8_t vision_axis_idle(XY_Axis axis)
{
    XY_AxisStatus status;
    return (XY_GetStatus(axis, &status) &&
            (status.state == XY_STATE_IDLE)) ? 1U : 0U;
}

static uint8_t vision_axis_fault(void)
{
    XY_AxisStatus x;
    XY_AxisStatus y;
    if (vision_get_axes(&x, &y) == 0U) return 1U;
    return ((x.state == XY_STATE_FAULT) || (y.state == XY_STATE_FAULT)) ? 1U : 0U;
}

static XY_Result vision_start_move(XY_Axis axis, int32_t target)
{
    uint16_t speed = (axis == XY_AXIS_X) ? VISION_CAL_X_SPEED_RPM :
                     VISION_CAL_Y_SPEED_RPM;
    return XY_MoveAbsolute(axis, target, speed, VISION_CAL_ACCELERATION);
}

static void vision_poll_move(XY_Axis axis, int32_t target,
                             VisionCalibrationState next, uint8_t sample_next,
                             uint32_t now)
{
    XY_Result result;
    if (g_action_started == 0U) {
        result = vision_start_move(axis, target);
        if (result == XY_RESULT_OK) {
            g_action_started = 1U;
        } else if (result != XY_RESULT_BUSY) {
            vision_set_fault(VISION_CAL_FAULT_MOVE, now);
        }
    } else if (vision_axis_idle(axis) != 0U) {
        if (sample_next != 0U) vision_finish_move(next, now);
        else vision_set_state(next, now);
    }
}

static void vision_finish_move(VisionCalibrationState next, uint32_t now)
{
    vision_begin_sampling(next, now);
}

static uint8_t vision_fit_matrix(void)
{
    float dx = (float)(g_x_pos.motor[0] - g_x_neg.motor[0]);
    float dy = (float)(g_y_pos.motor[1] - g_y_neg.motor[1]);
    float j00;
    float j10;
    float j01;
    float j11;
    float det;
    float nx2;
    float ny2;

    if ((vision_absf(dx) < 1.0f) || (vision_absf(dy) < 1.0f)) return 0U;
    j00 = (float)(g_x_pos.pixel[0] - g_x_neg.pixel[0]) / dx;
    j10 = (float)(g_x_pos.pixel[1] - g_x_neg.pixel[1]) / dx;
    j01 = (float)(g_y_pos.pixel[0] - g_y_neg.pixel[0]) / dy;
    j11 = (float)(g_y_pos.pixel[1] - g_y_neg.pixel[1]) / dy;
    det = (j00 * j11) - (j01 * j10);
    nx2 = (j00 * j00) + (j10 * j10);
    ny2 = (j01 * j01) + (j11 * j11);
    if ((nx2 < 1.0e-12f) || (ny2 < 1.0e-12f) ||
        ((det * det) < (0.0025f * nx2 * ny2))) return 0U;

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

void VisionCalibration_Init(uint32_t now)
{
    memset(&g_cal, 0, sizeof(g_cal));
    g_cal.state = VISION_CAL_IDLE;
    g_cal.state_tick = now;
    g_cal.result.axes[0] = XY_AXIS_X;
    g_cal.result.axes[1] = XY_AXIS_Y;
    g_cal.storage_state = VISION_CAL_STORAGE_NOT_LOADED;
    (void)VisionCalibration_Load();
}

uint8_t VisionCalibration_Save(void)
{
    uint8_t record[VISION_CAL_STORAGE_LENGTH];
    uint32_t generation;
    uint8_t row;
    uint8_t column;
    uint16_t offset = 20U;

    if (vision_result_sane(&g_xy_vision_calibration) == 0U) return 0U;
    memset(record, 0, sizeof(record));
    generation = g_cal.storage_generation + 1U;
    vision_write_u16(&record[4], VISION_CAL_STORAGE_VERSION);
    vision_write_u16(&record[6], VISION_CAL_STORAGE_LENGTH);
    vision_write_u32(&record[8], generation);
    record[12] = g_xy_vision_calibration.k230_id;
    record[13] = (uint8_t)g_xy_vision_calibration.axes[0];
    record[14] = (uint8_t)g_xy_vision_calibration.axes[1];
    vision_write_u16(&record[16],
                     (uint16_t)g_xy_vision_calibration.reference_pixel[0]);
    vision_write_u16(&record[18],
                     (uint16_t)g_xy_vision_calibration.reference_pixel[1]);
    for (row = 0U; row < 2U; ++row) {
        for (column = 0U; column < 2U; ++column) {
            uint32_t bits;
            memcpy(&bits,
                   &g_xy_vision_calibration.pixel_per_pulse[row][column],
                   sizeof(bits));
            vision_write_u32(&record[offset], bits);
            offset += 4U;
        }
    }
    for (row = 0U; row < 2U; ++row) {
        for (column = 0U; column < 2U; ++column) {
            uint32_t bits;
            memcpy(&bits,
                   &g_xy_vision_calibration.pulse_per_pixel[row][column],
                   sizeof(bits));
            vision_write_u32(&record[offset], bits);
            offset += 4U;
        }
    }
    vision_write_u32(&record[VISION_CAL_STORAGE_CRC_OFFSET],
                     vision_storage_crc32(&record[4],
                                          VISION_CAL_STORAGE_CRC_OFFSET - 4U));

    /* Commit the magic last so an interrupted page write cannot look valid. */
    if ((EEPROM_Write(VISION_CAL_EEPROM_ADDRESS, record, 4U) != HAL_OK) ||
        (EEPROM_Write(VISION_CAL_EEPROM_ADDRESS + 4U, &record[4],
                      VISION_CAL_STORAGE_LENGTH - 4U) != HAL_OK)) {
        g_cal.storage_state = VISION_CAL_STORAGE_IO_ERROR;
        return 0U;
    }
    vision_write_u32(record, VISION_CAL_STORAGE_MAGIC);
    if (EEPROM_Write(VISION_CAL_EEPROM_ADDRESS, record, 4U) != HAL_OK) {
        g_cal.storage_state = VISION_CAL_STORAGE_IO_ERROR;
        return 0U;
    }
    g_cal.storage_generation = generation;
    g_cal.storage_state = VISION_CAL_STORAGE_VALID;
    return 1U;
}

uint8_t VisionCalibration_Load(void)
{
    uint8_t record[VISION_CAL_STORAGE_LENGTH];
    XY_VisionCalibration loaded;
    uint8_t row;
    uint8_t column;
    uint16_t offset = 20U;

    if (EEPROM_Read(VISION_CAL_EEPROM_ADDRESS, record, sizeof(record)) != HAL_OK) {
        g_cal.storage_state = VISION_CAL_STORAGE_IO_ERROR;
        return 0U;
    }
    if (vision_read_u32(record) != VISION_CAL_STORAGE_MAGIC) {
        g_cal.storage_state = VISION_CAL_STORAGE_EMPTY;
        return 0U;
    }
    if ((vision_read_u16(&record[4]) != VISION_CAL_STORAGE_VERSION) ||
        (vision_read_u16(&record[6]) != VISION_CAL_STORAGE_LENGTH) ||
        (vision_read_u32(&record[VISION_CAL_STORAGE_CRC_OFFSET]) !=
         vision_storage_crc32(&record[4],
                              VISION_CAL_STORAGE_CRC_OFFSET - 4U))) {
        g_cal.storage_state = VISION_CAL_STORAGE_INVALID;
        return 0U;
    }

    memset(&loaded, 0, sizeof(loaded));
    loaded.k230_id = record[12];
    loaded.axes[0] = (XY_Axis)record[13];
    loaded.axes[1] = (XY_Axis)record[14];
    loaded.reference_pixel[0] = (int16_t)vision_read_u16(&record[16]);
    loaded.reference_pixel[1] = (int16_t)vision_read_u16(&record[18]);
    for (row = 0U; row < 2U; ++row) {
        for (column = 0U; column < 2U; ++column) {
            uint32_t bits = vision_read_u32(&record[offset]);
            memcpy(&loaded.pixel_per_pulse[row][column], &bits, sizeof(bits));
            offset += 4U;
        }
    }
    for (row = 0U; row < 2U; ++row) {
        for (column = 0U; column < 2U; ++column) {
            uint32_t bits = vision_read_u32(&record[offset]);
            memcpy(&loaded.pulse_per_pixel[row][column], &bits, sizeof(bits));
            offset += 4U;
        }
    }
    loaded.calibrated = 1U;
    if (vision_result_sane(&loaded) == 0U) {
        g_cal.storage_state = VISION_CAL_STORAGE_INVALID;
        return 0U;
    }
    g_xy_vision_calibration = loaded;
    g_cal.storage_generation = vision_read_u32(&record[8]);
    g_cal.storage_state = VISION_CAL_STORAGE_VALID;
    vision_copy_runtime_to_result();
    return 1U;
}

uint8_t VisionCalibration_ResetStored(void)
{
    uint8_t invalid_magic[4] = {0U, 0U, 0U, 0U};

    if (EEPROM_Write(VISION_CAL_EEPROM_ADDRESS, invalid_magic,
                     sizeof(invalid_magic)) != HAL_OK) {
        g_cal.storage_state = VISION_CAL_STORAGE_IO_ERROR;
        return 0U;
    }
    memset(&g_xy_vision_calibration, 0, sizeof(g_xy_vision_calibration));
    g_xy_vision_calibration.axes[0] = XY_AXIS_X;
    g_xy_vision_calibration.axes[1] = XY_AXIS_Y;
    memset(&g_cal.result, 0, sizeof(g_cal.result));
    g_cal.result.axes[0] = XY_AXIS_X;
    g_cal.result.axes[1] = XY_AXIS_Y;
    g_cal.storage_generation = 0U;
    g_cal.storage_state = VISION_CAL_STORAGE_EMPTY;
    return 1U;
}

uint8_t VisionCalibration_SetInverse(uint8_t k230_id,
                                     int16_t reference_x,
                                     int16_t reference_y,
                                     const float pulse_per_pixel[2][2])
{
    XY_VisionCalibration calibration;
    float determinant;

    if (pulse_per_pixel == NULL) return 0U;
    determinant = pulse_per_pixel[0][0] * pulse_per_pixel[1][1] -
                  pulse_per_pixel[0][1] * pulse_per_pixel[1][0];
    if ((vision_float_valid(determinant) == 0U) ||
        (vision_absf(determinant) < 1.0e-6f)) return 0U;

    memset(&calibration, 0, sizeof(calibration));
    calibration.k230_id = k230_id;
    calibration.axes[0] = XY_AXIS_X;
    calibration.axes[1] = XY_AXIS_Y;
    calibration.reference_pixel[0] = reference_x;
    calibration.reference_pixel[1] = reference_y;
    memcpy(calibration.pulse_per_pixel, pulse_per_pixel,
           sizeof(calibration.pulse_per_pixel));
    calibration.pixel_per_pulse[0][0] = pulse_per_pixel[1][1] / determinant;
    calibration.pixel_per_pulse[0][1] = -pulse_per_pixel[0][1] / determinant;
    calibration.pixel_per_pulse[1][0] = -pulse_per_pixel[1][0] / determinant;
    calibration.pixel_per_pulse[1][1] = pulse_per_pixel[0][0] / determinant;
    calibration.calibrated = 1U;
    if (vision_result_sane(&calibration) == 0U) return 0U;

    g_xy_vision_calibration = calibration;
    vision_copy_runtime_to_result();
    return VisionCalibration_Save();
}

uint8_t VisionCalibration_Start(uint8_t k230_id, int32_t x_step_pulses,
                                int32_t y_step_pulses, uint32_t now)
{
    const XY_AxisConfig *x_cfg = XY_GetConfig(XY_AXIS_X);
    const XY_AxisConfig *y_cfg = XY_GetConfig(XY_AXIS_Y);
    XY_AxisStatus x_status;
    XY_AxisStatus y_status;
    int64_t x_base;
    int64_t y_base;
    VisionCalibrationStorageState storage_state;
    uint32_t storage_generation;

    if (vision_state_active(g_cal.state) != 0U) return 0U;
    if (((k230_id != VISION_CAL_K230_1_ID) &&
         (k230_id != VISION_CAL_K230_2_ID)) ||
        (x_step_pulses <= 0) || (y_step_pulses <= 0)) return 0U;
    if ((vision_get_axes(&x_status, &y_status) == 0U) ||
        (x_status.state != XY_STATE_IDLE) ||
        (y_status.state != XY_STATE_IDLE) ||
        (x_status.position_valid == 0U) ||
        (y_status.position_valid == 0U)) return 0U;
    x_base = (int64_t)x_cfg->soft_min_pulses + (2LL * x_step_pulses);
    y_base = (int64_t)y_cfg->soft_min_pulses + (2LL * y_step_pulses);
    if (((x_base + x_step_pulses) > x_cfg->soft_max_pulses) ||
        ((y_base + y_step_pulses) > y_cfg->soft_max_pulses)) return 0U;

    storage_state = g_cal.storage_state;
    storage_generation = g_cal.storage_generation;
    memset(&g_cal, 0, sizeof(g_cal));
    g_cal.storage_state = storage_state;
    g_cal.storage_generation = storage_generation;
    memset(&g_base, 0, sizeof(g_base));
    memset(&g_x_pos, 0, sizeof(g_x_pos));
    memset(&g_x_neg, 0, sizeof(g_x_neg));
    memset(&g_y_pos, 0, sizeof(g_y_pos));
    memset(&g_y_neg, 0, sizeof(g_y_neg));
    g_cal.k230_id = k230_id;
    g_cal.step_pulses[0] = x_step_pulses;
    g_cal.step_pulses[1] = y_step_pulses;
    g_cal.base_pulses[0] = (int32_t)x_base;
    g_cal.base_pulses[1] = (int32_t)y_base;
    g_cal.result.k230_id = k230_id;
    g_cal.result.axes[0] = XY_AXIS_X;
    g_cal.result.axes[1] = XY_AXIS_Y;
    vision_reset_collector();
    vision_set_state(VISION_CAL_MOVE_BASE_X, now);
    return 1U;
}

uint8_t VisionCalibration_CaptureReference(uint32_t now)
{
    if (g_cal.state != VISION_CAL_MANUAL_ALIGN) return 0U;
    vision_set_state(VISION_CAL_WAIT_REFERENCE_IDLE, now);
    return 1U;
}

void VisionCalibration_Abort(void)
{
    if (vision_state_active(g_cal.state) == 0U) return;
    if ((g_cal.state != VISION_CAL_MANUAL_ALIGN) &&
        (g_cal.state != VISION_CAL_CAPTURE_REFERENCE)) {
        xy_stop_all();
    }
    g_cal.fault = VISION_CAL_FAULT_NONE;
    g_cal.result.valid = 0U;
    vision_set_state(VISION_CAL_IDLE, HAL_GetTick());
}

void VisionCalibration_Poll(uint32_t now)
{
    XY_AxisStatus x;
    XY_AxisStatus y;

    if (vision_state_active(g_cal.state) == 0U) return;
    if (vision_axis_fault() != 0U) {
        vision_set_fault(VISION_CAL_FAULT_AXIS, now);
        return;
    }
    if (vision_get_axes(&x, &y) != 0U) {
        g_cal.position_pulses[0] = x.position_pulses;
        g_cal.position_pulses[1] = y.position_pulses;
    }
    if (((g_cal.state == VISION_CAL_SAMPLE_BASE) ||
         (g_cal.state == VISION_CAL_SAMPLE_X_POS) ||
         (g_cal.state == VISION_CAL_SAMPLE_X_NEG) ||
         (g_cal.state == VISION_CAL_SAMPLE_Y_POS) ||
         (g_cal.state == VISION_CAL_SAMPLE_Y_NEG) ||
         (g_cal.state == VISION_CAL_CAPTURE_REFERENCE)) &&
        ((int32_t)(now - g_sample_deadline_tick) >= 0)) {
        vision_set_fault((g_sample_unstable_seen != 0U) ?
                         VISION_CAL_FAULT_UNSTABLE :
                         VISION_CAL_FAULT_VISION_TIMEOUT, now);
        return;
    }

    switch (g_cal.state) {
    case VISION_CAL_MOVE_BASE_X:
        vision_poll_move(XY_AXIS_X, g_cal.base_pulses[0],
                         VISION_CAL_MOVE_BASE_Y, 0U, now);
        break;
    case VISION_CAL_MOVE_BASE_Y:
        vision_poll_move(XY_AXIS_Y, g_cal.base_pulses[1],
                         VISION_CAL_SAMPLE_BASE, 1U, now);
        break;
    case VISION_CAL_SAMPLE_BASE:
        if (vision_collect_sample(now, &g_base)) vision_set_state(VISION_CAL_MOVE_X_POS, now);
        break;
    case VISION_CAL_MOVE_X_POS:
        vision_poll_move(XY_AXIS_X,
                         g_cal.base_pulses[0] + g_cal.step_pulses[0],
                         VISION_CAL_SAMPLE_X_POS, 1U, now);
        break;
    case VISION_CAL_SAMPLE_X_POS:
        if (vision_collect_sample(now, &g_x_pos)) vision_set_state(VISION_CAL_MOVE_X_NEG, now);
        break;
    case VISION_CAL_MOVE_X_NEG:
        vision_poll_move(XY_AXIS_X,
                         g_cal.base_pulses[0] - g_cal.step_pulses[0],
                         VISION_CAL_SAMPLE_X_NEG, 1U, now);
        break;
    case VISION_CAL_SAMPLE_X_NEG:
        if (vision_collect_sample(now, &g_x_neg)) vision_set_state(VISION_CAL_RETURN_X, now);
        break;
    case VISION_CAL_RETURN_X:
        vision_poll_move(XY_AXIS_X, g_cal.base_pulses[0],
                         VISION_CAL_MOVE_Y_POS, 0U, now);
        break;
    case VISION_CAL_MOVE_Y_POS:
        vision_poll_move(XY_AXIS_Y,
                         g_cal.base_pulses[1] + g_cal.step_pulses[1],
                         VISION_CAL_SAMPLE_Y_POS, 1U, now);
        break;
    case VISION_CAL_SAMPLE_Y_POS:
        if (vision_collect_sample(now, &g_y_pos)) vision_set_state(VISION_CAL_MOVE_Y_NEG, now);
        break;
    case VISION_CAL_MOVE_Y_NEG:
        vision_poll_move(XY_AXIS_Y,
                         g_cal.base_pulses[1] - g_cal.step_pulses[1],
                         VISION_CAL_SAMPLE_Y_NEG, 1U, now);
        break;
    case VISION_CAL_SAMPLE_Y_NEG:
        if (vision_collect_sample(now, &g_y_neg)) vision_set_state(VISION_CAL_RETURN_Y, now);
        break;
    case VISION_CAL_RETURN_Y:
        vision_poll_move(XY_AXIS_Y, g_cal.base_pulses[1],
                         VISION_CAL_FIT, 0U, now);
        break;
    case VISION_CAL_FIT:
        if (vision_fit_matrix() == 0U) {
            vision_set_fault(VISION_CAL_FAULT_SINGULAR_MATRIX, now);
        } else {
            vision_set_state(VISION_CAL_MANUAL_ALIGN, now);
        }
        break;
    case VISION_CAL_WAIT_REFERENCE_IDLE:
        if ((vision_axis_idle(XY_AXIS_X) != 0U) &&
            (vision_axis_idle(XY_AXIS_Y) != 0U)) {
            vision_begin_sampling(VISION_CAL_CAPTURE_REFERENCE, now);
        }
        break;
    case VISION_CAL_CAPTURE_REFERENCE:
        if (vision_collect_sample(now, &g_base)) {
            g_cal.result.reference_pixel[0] = g_base.pixel[0];
            g_cal.result.reference_pixel[1] = g_base.pixel[1];
            g_cal.result.valid = 1U;
            vision_publish_result(&g_cal.result);
            (void)VisionCalibration_Save();
            vision_set_state(VISION_CAL_COMPLETE, now);
        }
        break;
    default:
        break;
    }
}

void VisionCalibration_GetStatus(VisionCalibrationStatus *status)
{
    if (status != NULL) *status = g_cal;
}

uint8_t VisionCalibration_ManualMotionAllowed(void)
{
    return ((g_cal.state == VISION_CAL_IDLE) ||
            (g_cal.state == VISION_CAL_MANUAL_ALIGN) ||
            (g_cal.state == VISION_CAL_COMPLETE) ||
            (g_cal.state == VISION_CAL_FAULT)) ? 1U : 0U;
}

uint8_t VisionCalibration_IsActive(void)
{
    return vision_state_active(g_cal.state);
}

const char *VisionCalibration_StateString(VisionCalibrationState state)
{
    static const char *const names[] = {
        "IDLE", "MOVE_BASE_X", "MOVE_BASE_Y",
        "SAMPLE_BASE", "MOVE_X_POS", "SAMPLE_X_POS", "MOVE_X_NEG",
        "SAMPLE_X_NEG", "RETURN_X", "MOVE_Y_POS", "SAMPLE_Y_POS",
        "MOVE_Y_NEG", "SAMPLE_Y_NEG", "RETURN_Y", "FIT",
        "MANUAL_ALIGN", "WAIT_REFERENCE_IDLE", "CAPTURE_REFERENCE",
        "COMPLETE", "FAULT"
    };
    return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
           names[state] : "UNKNOWN";
}

const char *VisionCalibration_FaultString(VisionCalibrationFault fault)
{
    static const char *const names[] = {
        "NONE", "BUSY", "INVALID_ARGUMENT", "AXIS", "MOVE",
        "VISION_TIMEOUT", "UNSTABLE", "SINGULAR_MATRIX"
    };
    return ((uint32_t)fault < (sizeof(names) / sizeof(names[0]))) ?
           names[fault] : "UNKNOWN";
}

const char *VisionCalibration_StorageStateString(
    VisionCalibrationStorageState state)
{
    static const char *const names[] = {
        "NOT_LOADED", "VALID", "EMPTY", "IO_ERROR", "INVALID"
    };
    return ((uint32_t)state < (sizeof(names) / sizeof(names[0]))) ?
           names[state] : "UNKNOWN";
}

#ifndef VISION_CALIBRATION_H
#define VISION_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "xy_motor.h"
#include <stdint.h>

#define VISION_CAL_K230_1_ID       0x11U
#define VISION_CAL_K230_2_ID       0x12U

typedef enum {
    VISION_CAL_IDLE = 0,
    VISION_CAL_MOVE_BASE_X,
    VISION_CAL_MOVE_BASE_Y,
    VISION_CAL_SAMPLE_BASE,
    VISION_CAL_MOVE_X_POS,
    VISION_CAL_SAMPLE_X_POS,
    VISION_CAL_MOVE_X_NEG,
    VISION_CAL_SAMPLE_X_NEG,
    VISION_CAL_RETURN_X,
    VISION_CAL_MOVE_Y_POS,
    VISION_CAL_SAMPLE_Y_POS,
    VISION_CAL_MOVE_Y_NEG,
    VISION_CAL_SAMPLE_Y_NEG,
    VISION_CAL_RETURN_Y,
    VISION_CAL_FIT,
    VISION_CAL_MANUAL_ALIGN,
    VISION_CAL_WAIT_REFERENCE_IDLE,
    VISION_CAL_CAPTURE_REFERENCE,
    VISION_CAL_COMPLETE,
    VISION_CAL_FAULT
} VisionCalibrationState;

typedef enum {
    VISION_CAL_FAULT_NONE = 0,
    VISION_CAL_FAULT_BUSY,
    VISION_CAL_FAULT_INVALID_ARGUMENT,
    VISION_CAL_FAULT_AXIS,
    VISION_CAL_FAULT_MOVE,
    VISION_CAL_FAULT_VISION_TIMEOUT,
    VISION_CAL_FAULT_UNSTABLE,
    VISION_CAL_FAULT_SINGULAR_MATRIX
} VisionCalibrationFault;

typedef enum {
    VISION_CAL_STORAGE_NOT_LOADED = 0,
    VISION_CAL_STORAGE_VALID,
    VISION_CAL_STORAGE_EMPTY,
    VISION_CAL_STORAGE_IO_ERROR,
    VISION_CAL_STORAGE_INVALID
} VisionCalibrationStorageState;

typedef struct {
    uint8_t valid;
    uint8_t k230_id;
    XY_Axis axes[2];
    int16_t reference_pixel[2];
    float pixel_per_pulse[2][2];
    float pulse_per_pixel[2][2];
} VisionCalibrationResult;

typedef struct {
    VisionCalibrationState state;
    VisionCalibrationFault fault;
    uint8_t k230_id;
    uint8_t sample_count;
    uint16_t last_sample_seq;
    int32_t step_pulses[2];
    int32_t base_pulses[2];
    int32_t position_pulses[2];
    int16_t latest_pixel[2];
    uint32_t state_tick;
    VisionCalibrationStorageState storage_state;
    uint32_t storage_generation;
    VisionCalibrationResult result;
} VisionCalibrationStatus;

void VisionCalibration_Init(uint32_t now);
void VisionCalibration_Poll(uint32_t now);
uint8_t VisionCalibration_Start(uint8_t k230_id, int32_t x_step_pulses,
                                int32_t y_step_pulses, uint32_t now);
uint8_t VisionCalibration_CaptureReference(uint32_t now);
uint8_t VisionCalibration_Save(void);
uint8_t VisionCalibration_Load(void);
uint8_t VisionCalibration_ResetStored(void);
uint8_t VisionCalibration_SetInverse(uint8_t k230_id,
                                     int16_t reference_x,
                                     int16_t reference_y,
                                     const float pulse_per_pixel[2][2]);
void VisionCalibration_Abort(void);
void VisionCalibration_GetStatus(VisionCalibrationStatus *status);
uint8_t VisionCalibration_ManualMotionAllowed(void);
const char *VisionCalibration_StateString(VisionCalibrationState state);
const char *VisionCalibration_FaultString(VisionCalibrationFault fault);
const char *VisionCalibration_StorageStateString(
    VisionCalibrationStorageState state);

#ifdef __cplusplus
}
#endif

#endif

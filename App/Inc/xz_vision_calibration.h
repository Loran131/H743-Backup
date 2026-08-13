#ifndef XZ_VISION_CALIBRATION_H
#define XZ_VISION_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vision_calibration.h"
#include <stdint.h>

#define XZ_CAL_DEFAULT_X_STEP_PULSES 51200
#define XZ_CAL_DEFAULT_Z_STEP_PULSES 64000

typedef enum {
    XZ_CAL_IDLE = 0,
    XZ_CAL_WAIT_RED_MODE,
    XZ_CAL_MOVE_BASE_X,
    XZ_CAL_MOVE_BASE_Z,
    XZ_CAL_SAMPLE_BASE,
    XZ_CAL_MOVE_X_POS,
    XZ_CAL_SAMPLE_X_POS,
    XZ_CAL_MOVE_X_NEG,
    XZ_CAL_SAMPLE_X_NEG,
    XZ_CAL_RETURN_X,
    XZ_CAL_MOVE_Z_POS,
    XZ_CAL_SAMPLE_Z_POS,
    XZ_CAL_MOVE_Z_NEG,
    XZ_CAL_SAMPLE_Z_NEG,
    XZ_CAL_RETURN_Z,
    XZ_CAL_FIT,
    XZ_CAL_MANUAL_ALIGN,
    XZ_CAL_WAIT_REFERENCE_IDLE,
    XZ_CAL_CAPTURE_REFERENCE,
    XZ_CAL_COMPLETE,
    XZ_CAL_FAULT
} XZCalibrationState;

typedef enum {
    XZ_CAL_FAULT_NONE = 0,
    XZ_CAL_FAULT_BUSY,
    XZ_CAL_FAULT_ARGUMENT,
    XZ_CAL_FAULT_MODE,
    XZ_CAL_FAULT_AXIS,
    XZ_CAL_FAULT_MOVE,
    XZ_CAL_FAULT_VISION_TIMEOUT,
    XZ_CAL_FAULT_UNSTABLE,
    XZ_CAL_FAULT_SINGULAR_MATRIX
} XZCalibrationFault;

typedef struct {
    uint8_t valid;
    int16_t reference_pixel[2];
    float pixel_per_pulse[2][2];
    float pulse_per_pixel[2][2];
} XZVisionCalibration;

typedef struct {
    XZCalibrationState state;
    XZCalibrationFault fault;
    uint8_t sample_count;
    uint16_t last_sample_seq;
    int32_t step_pulses[2];
    int32_t base_pulses[2];
    int32_t position_pulses[2];
    int16_t latest_pixel[2];
    uint32_t state_tick;
    VisionCalibrationStorageState storage_state;
    uint32_t storage_generation;
    XZVisionCalibration result;
} XZCalibrationStatus;

extern XZVisionCalibration g_xz_vision_calibration;

void XZCalibration_Init(uint32_t now);
void XZCalibration_Poll(uint32_t now);
uint8_t XZCalibration_Start(int32_t x_step_pulses,
                            int32_t z_step_pulses, uint32_t now);
uint8_t XZCalibration_CaptureReference(uint32_t now);
void XZCalibration_Abort(void);
uint8_t XZCalibration_Save(void);
uint8_t XZCalibration_Load(void);
uint8_t XZCalibration_ResetStored(void);
void XZCalibration_GetStatus(XZCalibrationStatus *status);
uint8_t XZCalibration_ManualMotionAllowed(void);
uint8_t XZCalibration_IsActive(void);
const char *XZCalibration_StateString(XZCalibrationState state);
const char *XZCalibration_FaultString(XZCalibrationFault fault);

#ifdef __cplusplus
}
#endif

#endif

#ifndef __AIRMOUSE_H__
#define __AIRMOUSE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * Axis mapping configuration:
 * Choose source from: gx, gy, gz
 * Signs: +1 or -1
 * Default: dx = gz, dy = gx
 */
#define X_SOURCE        gz
#define Y_SOURCE        gx
#define X_SIGN          (1)
#define Y_SIGN          (1)

/* Mouse sensitivity multiplier */
#define SENSITIVITY     0.5f

/* Gyro dead zone in degrees per second */
#define DEADZONE_DPS    1.5f

/* Low-pass filter alpha factor (0.0 to 1.0) */
#define LPF_ALPHA       0.3f

/* Button debounce time in ms */
#define BUTTON_DEBOUNCE_MS  20

/* Global debug struct for GDB inspection */
typedef struct {
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;
    float bias_x;
    float bias_y;
    float bias_z;
    int8_t dx;
    int8_t dy;
    uint8_t buttons;
    uint8_t enabled;
} airmouse_debug_t;

extern volatile airmouse_debug_t g_airmouse_debug;

/* Function prototypes */
int airmouse_init(void);
void airmouse_step(void);

#ifdef __cplusplus
}
#endif

#endif /* __AIRMOUSE_H__ */

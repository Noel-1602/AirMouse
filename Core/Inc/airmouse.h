#ifndef __AIRMOUSE_H__
#define __AIRMOUSE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Feature switches: 1 = on, 0 = off */
#define TILT_CORRECTION 1
#define POINTER_ACCEL   1
#define PRECISION_MODE  1
#define IDLE_MODE       1

/*
 * Axis mapping configuration:
 * Choose source from: gx, gy, gz
 * Signs: +1 or -1
 * Default: dx = gz, dy = gx
 */
#define X_SOURCE gz
#define Y_SOURCE gx
#define X_SIGN (-1)
#define Y_SIGN (1)

/* Mouse sensitivity multiplier */
#define SENSITIVITY 0.5f

/* Gyro dead zone in degrees per second */
#define DEADZONE_DPS 1.5f

/* Low-pass filter alpha factor (0.0 to 1.0) */
#define LPF_ALPHA 0.3f

/* Button debounce time in ms */
#define BUTTON_DEBOUNCE_MS 20

/* Auto-detect mode at startup: 1 = auto-detect USB vs Wireless, 0 = use OUTPUT_MODE_WIRELESS */
#define AUTO_DETECT_MODE 1

/* Output mode: 1 = HC-05 Wireless UART, 0 = USB HID (used if AUTO_DETECT_MODE is 0) */
#define OUTPUT_MODE_WIRELESS 1

/* Wireless packet transmission period in ms */
#define PACKET_PERIOD_MS 20

/* Feature 1: Tilt Correction (accelerometer roll compensation) */
#define ROLL_GYRO_SOURCE   gy     /* Gyro axis pointing along the hand */
#define ROLL_ACC_A         ax     /* Accelerometer axis A perpendicular to roll gyro */
#define ROLL_ACC_B         az     /* Accelerometer axis B perpendicular to roll gyro */
#define ROLL_SIGN          (+1)   /* Sign multiplier to match gyro roll rate to acc roll */
#define TILT_ALPHA         0.98f  /* Complementary filter gyro weight factor (0.0 to 1.0) */
#define TILT_DT            0.01f  /* Complementary filter time step in seconds (10 ms) */
#define TILT_ACC_MIN_G     0.8f   /* Min total acceleration (g) to trust accelerometer */
#define TILT_ACC_MAX_G     1.2f   /* Max total acceleration (g) to trust accelerometer */

/* Feature 2: Pointer Acceleration */
#define ACCEL_GAIN         1.5f   /* Maximum additional acceleration gain multiplier */
#define ACCEL_REF_DPS      100.0f /* Reference speed for acceleration gain curve (deg/s) */
#define ACCEL_CAP_DPS      300.0f /* Speed cap for acceleration curve (deg/s) */

/* Feature 3: Precision Mode */
#define PRECISION_FACTOR   0.35f  /* Sensitivity multiplier while precision button PB15 is held */

/* Feature 4: Idle Mode (battery saving) */
#define IDLE_IN_USB_MODE   0      /* Allow idle mode in USB HID mode (1 = yes, 0 = wireless only) */
#define IDLE_TIMEOUT_MS    30000  /* Inactivity timeout before entering idle mode (ms) */
#define IDLE_POLL_MS       50     /* Sensor poll period during idle mode (ms) */
#define IDLE_GYRO_DPS      3.0f   /* Max gyro magnitude for stationary detection (deg/s) */
#define IDLE_WAKE_DPS      10.0f  /* Gyro magnitude threshold to wake from idle (deg/s) */
#define IDLE_HEARTBEAT_MS  500    /* Keepalive packet period during idle in wireless mode (ms) */

/* Output mode enum */
typedef enum output_mode_t {
  MODE_USB_HID = 0,
  MODE_WIRELESS = 1
} output_mode_t;

extern output_mode_t g_output_mode;

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
  output_mode_t g_output_mode;
  float roll_deg;
  float accel_gain;
  uint8_t precision_on;
  uint8_t idle;
} airmouse_debug_t;

extern volatile airmouse_debug_t g_airmouse_debug;

/* Function prototypes */
int airmouse_init(void);
void airmouse_step(void);

#ifdef __cplusplus
}
#endif

#endif /* __AIRMOUSE_H__ */

#include "airmouse.h"
#include "hc05.h"
#include "i2c.h"
#include "usb_device.h"
#include "usbd_hid.h"
#include <math.h>

#define MPU6050_ADDR_7BIT        0x68
#define MPU6050_I2C_ADDR         (MPU6050_ADDR_7BIT << 1)

#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

/* Macro concatenation helpers for axis names */
#define _CONCAT2(a, b) a##b
#define EVAL_CONCAT(a, b) _CONCAT2(a, b)

/* Global debug struct for GDB inspection */
volatile airmouse_debug_t g_airmouse_debug = {0};

/* Runtime output mode */
output_mode_t g_output_mode = OUTPUT_MODE_WIRELESS ? MODE_WIRELESS : MODE_USB_HID;

/* USB Device handle */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* Button debounce state structure */
typedef struct {
    uint8_t raw_state;        /* 1 = pressed (active-low pin is LOW), 0 = released */
    uint8_t debounced_state;  /* 1 = pressed, 0 = released */
    uint8_t prev_state;       /* For edge detection */
    uint32_t last_change_tick;
} button_debounce_t;

static button_debounce_t btn_left = {0};
static button_debounce_t btn_right = {0};
static button_debounce_t btn_recenter = {0};
static button_debounce_t btn_precision = {0};

/* Filter and accumulator states */
static float filt_gx = 0.0f;
static float filt_gy = 0.0f;
static float filt_gz = 0.0f;
static float rem_x = 0.0f;
static float rem_y = 0.0f;
static uint32_t last_step_tick = 0;

/* Feature 1: Tilt correction roll estimate */
static float s_roll_deg = 0.0f;

/* Feature 4: Idle mode state machine */
static uint8_t s_idle = 0;
static uint32_t last_activity_tick = 0;
static uint32_t last_mode_switch_tick = 0;
static uint32_t last_idle_packet_tick = 0;

/* Non-blocking LED feedback state machine (PC13 active-low) */
#define BLINK_HALF_PERIOD_MS 100

static uint8_t s_blink_active = 0;
static uint8_t s_blink_phase = 0;
static uint8_t s_blink_total_phases = 0;
static uint32_t s_blink_phase_start = 0;
static uint8_t s_usb_seen_configured = 0;

/* Helper: Active-low LED control on PC13 */
static inline void set_led(uint8_t on) {
    if (on) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); /* LED ON */
    } else {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);   /* LED OFF */
    }
}

/* Helper: Start non-blocking LED feedback for mode switch */
static void start_mode_switch_led_feedback(output_mode_t mode) {
    s_blink_active = 1;
    s_blink_phase = 0;
    /* 3 short blinks for USB HID (6 phases), 2 short blinks for Wireless (4 phases) */
    s_blink_total_phases = (mode == MODE_USB_HID) ? 6 : 4;
    s_blink_phase_start = HAL_GetTick();
    set_led(1);
}

/* Helper: Update non-blocking LED state or maintain normal LED behaviour */
static void update_led(uint32_t now) {
    if (s_blink_active) {
        if ((now - s_blink_phase_start) >= BLINK_HALF_PERIOD_MS) {
            s_blink_phase++;
            s_blink_phase_start = now;
            if (s_blink_phase >= s_blink_total_phases) {
                s_blink_active = 0;
#if IDLE_MODE
                set_led(s_idle ? 0 : g_airmouse_debug.enabled);
#else
                set_led(g_airmouse_debug.enabled);
#endif
            } else {
                /* Even phase = ON, Odd phase = OFF */
                set_led((s_blink_phase % 2) == 0);
            }
        }
    } else {
#if IDLE_MODE
        set_led(s_idle ? 0 : g_airmouse_debug.enabled);
#else
        set_led(g_airmouse_debug.enabled);
#endif
    }
}

/* Helper: Continuous deadband filter */
static inline float apply_deadzone(float val, float dz) {
    if (val > dz) {
        return val - dz;
    } else if (val < -dz) {
        return val + dz;
    }
    return 0.0f;
}

/* Helper: Update button debounce state */
static void update_button(button_debounce_t *btn, uint8_t is_pressed, uint32_t now) {
    if (is_pressed != btn->raw_state) {
        btn->raw_state = is_pressed;
        btn->last_change_tick = now;
    }
    if ((now - btn->last_change_tick) >= BUTTON_DEBOUNCE_MS) {
        btn->debounced_state = btn->raw_state;
    }
}

/*
 * Calibrate Gyro and Initialise Roll:
 * Averages ~500 gyro samples while still and stores as bias.
 * Initialises roll from the accelerometer.
 * Fast-blinks the LED during calibration.
 */
static void airmouse_calibrate(void) {
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;
#if TILT_CORRECTION
    int32_t sum_ax = 0;
    int32_t sum_ay = 0;
    int32_t sum_az = 0;
#endif
    uint32_t valid_samples = 0;
    uint8_t buf[14];

    for (int i = 0; i < 500; i++) {
        /* Blink fast: toggle every 25 samples (~50 ms) */
        if ((i % 25) == 0) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }

        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
            s_usb_seen_configured = 1;
        }

        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H,
                             I2C_MEMADD_SIZE_8BIT, buf, 14, 20) == HAL_OK) {
#if TILT_CORRECTION
            int16_t raw_ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
            int16_t raw_ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
            int16_t raw_az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
            sum_ax += raw_ax;
            sum_ay += raw_ay;
            sum_az += raw_az;
#endif
            int16_t gx = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
            int16_t gy = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
            int16_t gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);
            sum_x += gx;
            sum_y += gy;
            sum_z += gz;
            valid_samples++;
        }
        HAL_Delay(2);
    }

    if (valid_samples > 0) {
        g_airmouse_debug.bias_x = (float)sum_x / (float)valid_samples;
        g_airmouse_debug.bias_y = (float)sum_y / (float)valid_samples;
        g_airmouse_debug.bias_z = (float)sum_z / (float)valid_samples;

#if TILT_CORRECTION
        float ax = (float)sum_ax / (float)valid_samples / 16384.0f;
        float ay = (float)sum_ay / (float)valid_samples / 16384.0f;
        float az = (float)sum_az / (float)valid_samples / 16384.0f;
        (void)ay;
        s_roll_deg = atan2f((float)(ROLL_ACC_A), (float)(ROLL_ACC_B)) * (180.0f / 3.14159265358979323846f);
        g_airmouse_debug.roll_deg = s_roll_deg;
#else
        s_roll_deg = 0.0f;
        g_airmouse_debug.roll_deg = 0.0f;
#endif
    }

    /* Reset filter states and accumulators */
    filt_gx = 0.0f;
    filt_gy = 0.0f;
    filt_gz = 0.0f;
    rem_x = 0.0f;
    rem_y = 0.0f;
}

int airmouse_init(void) {
    uint8_t who_am_i = 0;
    uint8_t reg_val = 0;
    HAL_StatusTypeDef status;

    /* Configure PB15 as input with internal pull-up (Precision button) */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Verify WHO_AM_I (0x75) - accept 0x68, 0x70, or 0x72 */
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, MPU6050_REG_WHO_AM_I,
                              I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100);
    if (status != HAL_OK) {
        return -1;
    }
    if (who_am_i != 0x68 && who_am_i != 0x70 && who_am_i != 0x72) {
        return -2;
    }

    /* Write 0x00 to PWR_MGMT_1 (0x6B), wait 100 ms */
    reg_val = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1,
                               I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100);
    if (status != HAL_OK) {
        return -3;
    }
    HAL_Delay(100);

    /* GYRO_CONFIG (0x1B) = 0x08 (±500 deg/s, 65.5 LSB per deg/s) */
    reg_val = 0x08;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, MPU6050_REG_GYRO_CONFIG,
                               I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100);
    if (status != HAL_OK) {
        return -4;
    }

    /* CONFIG (0x1A) = 0x03 (DLPF) */
    reg_val = 0x03;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, MPU6050_REG_CONFIG,
                               I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100);
    if (status != HAL_OK) {
        return -5;
    }

    /* Initial gyro calibration and roll estimation */
    airmouse_calibrate();

    /* Safety: cursor disabled after boot until recentre pressed once */
    g_airmouse_debug.enabled = 0;
    set_led(0); /* LED off = movement disabled */

#if AUTO_DETECT_MODE
    /*
     * Boot: wait up to 1500 ms after startup.
     * Gyro calibration ran during this window without blocking USB enumeration.
     * If USB is CONFIGURED at any point in the 1500 ms window, start in MODE_USB_HID.
     * Otherwise start in MODE_WIRELESS.
     */
    while (HAL_GetTick() < 1500) {
        if (s_usb_seen_configured || (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)) {
            s_usb_seen_configured = 1;
            break;
        }
    }
    if (s_usb_seen_configured || (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)) {
        g_output_mode = MODE_USB_HID;
    } else {
        g_output_mode = MODE_WIRELESS;
    }
#else
    g_output_mode = (OUTPUT_MODE_WIRELESS != 0) ? MODE_WIRELESS : MODE_USB_HID;
#endif

    g_airmouse_debug.g_output_mode = g_output_mode;

    /* Initialize button states */
    uint32_t now = HAL_GetTick();
    last_step_tick = now;
    last_activity_tick = now;
    last_mode_switch_tick = now;
    last_idle_packet_tick = now;
    s_idle = 0;

    uint8_t left_raw = (HAL_GPIO_ReadPin(BTN_LEFT_GPIO_Port, BTN_LEFT_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    uint8_t right_raw = (HAL_GPIO_ReadPin(BTN_RIGHT_GPIO_Port, BTN_RIGHT_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    uint8_t recenter_raw = (HAL_GPIO_ReadPin(BTN_RECENTER_GPIO_Port, BTN_RECENTER_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    uint8_t precision_raw = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) ? 1 : 0;

    btn_left.raw_state = left_raw;
    btn_left.debounced_state = left_raw;
    btn_left.prev_state = left_raw;
    btn_left.last_change_tick = now;

    btn_right.raw_state = right_raw;
    btn_right.debounced_state = right_raw;
    btn_right.prev_state = right_raw;
    btn_right.last_change_tick = now;

    btn_recenter.raw_state = recenter_raw;
    btn_recenter.debounced_state = recenter_raw;
    btn_recenter.prev_state = recenter_raw;
    btn_recenter.last_change_tick = now;

    btn_precision.raw_state = precision_raw;
    btn_precision.debounced_state = precision_raw;
    btn_precision.prev_state = precision_raw;
    btn_precision.last_change_tick = now;

    g_airmouse_debug.accel_gain = 1.0f;
    g_airmouse_debug.precision_on = 0;
    g_airmouse_debug.idle = 0;

    /* Always call hc05_init() at startup (USART2 must be ready in both modes) */
    hc05_init();

    return 0;
}

void airmouse_step(void) {
    uint32_t now = HAL_GetTick();

#if IDLE_MODE
    uint32_t step_interval = s_idle ? IDLE_POLL_MS : 10;
#else
    uint32_t step_interval = 10;
#endif

    if ((now - last_step_tick) < step_interval) {
#if IDLE_MODE
        if (s_idle) {
            __WFI();
        }
#endif
        return;
    }
    last_step_tick = now;

    /* 1. Read and debounce buttons (active-low with internal pull-up) */
    uint8_t left_raw = (HAL_GPIO_ReadPin(BTN_LEFT_GPIO_Port, BTN_LEFT_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    uint8_t right_raw = (HAL_GPIO_ReadPin(BTN_RIGHT_GPIO_Port, BTN_RIGHT_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    uint8_t recenter_raw = (HAL_GPIO_ReadPin(BTN_RECENTER_GPIO_Port, BTN_RECENTER_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    uint8_t precision_raw = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) ? 1 : 0;

    update_button(&btn_left, left_raw, now);
    update_button(&btn_right, right_raw, now);
    update_button(&btn_recenter, recenter_raw, now);
    update_button(&btn_precision, precision_raw, now);

#if IDLE_MODE
    uint8_t any_button_down = (left_raw || right_raw || recenter_raw || precision_raw);
    uint8_t any_button_debounced = (btn_left.debounced_state || btn_right.debounced_state ||
                                   btn_recenter.debounced_state || btn_precision.debounced_state);

    /* Wake immediately on any button press (including PB15) */
    if (s_idle && (any_button_down || any_button_debounced)) {
        s_idle = 0;
        g_airmouse_debug.idle = 0;
        filt_gx = 0.0f;
        filt_gy = 0.0f;
        filt_gz = 0.0f;
        rem_x = 0.0f;
        rem_y = 0.0f;
        last_activity_tick = now;
        set_led(g_airmouse_debug.enabled);
    }
#endif

    static uint32_t recenter_press_start_tick = 0;

    /* Detect press and release edges of debounced B14 */
    uint8_t recenter_press_edge = (btn_recenter.debounced_state && !btn_recenter.prev_state);
    uint8_t recenter_release_edge = (!btn_recenter.debounced_state && btn_recenter.prev_state);
    btn_recenter.prev_state = btn_recenter.debounced_state;

    if (recenter_press_edge) {
        recenter_press_start_tick = now;
    }

    if (recenter_release_edge) {
        uint32_t hold_duration = now - recenter_press_start_tick;

        if (hold_duration < 800) {
            /* Short press (< 800 ms): existing behaviour (recentre, recalibrate bias, toggle enabled) */
            s_blink_active = 0; /* cancel any active mode blink */
            airmouse_calibrate();

            /* Toggle movement enabled state */
            g_airmouse_debug.enabled = !g_airmouse_debug.enabled;
            set_led(g_airmouse_debug.enabled);

            /* Resync buttons and timer after calibration */
            now = HAL_GetTick();
            last_step_tick = now;
            last_activity_tick = now;
            s_idle = 0;
            g_airmouse_debug.idle = 0;

            recenter_raw = (HAL_GPIO_ReadPin(BTN_RECENTER_GPIO_Port, BTN_RECENTER_Pin) == GPIO_PIN_RESET) ? 1 : 0;
            btn_recenter.raw_state = recenter_raw;
            btn_recenter.debounced_state = recenter_raw;
            btn_recenter.prev_state = recenter_raw;
            btn_recenter.last_change_tick = now;
            return;
        } else if (hold_duration >= 1500) {
            /* Long press (>= 1500 ms): toggle g_output_mode, do NOT recentre or recalibrate */
            if (g_output_mode == MODE_USB_HID) {
                g_output_mode = MODE_WIRELESS;
            } else {
                g_output_mode = MODE_USB_HID;
            }
            g_airmouse_debug.g_output_mode = g_output_mode;

            /* Force cursor movement to disabled (enabled = 0) and zero dx/dy so it cannot jump */
            g_airmouse_debug.enabled = 0;
            rem_x = 0.0f;
            rem_y = 0.0f;
            filt_gx = 0.0f;
            filt_gy = 0.0f;
            filt_gz = 0.0f;
            g_airmouse_debug.dx = 0;
            g_airmouse_debug.dy = 0;

            last_mode_switch_tick = now;
            last_activity_tick = now;
            s_idle = 0;
            g_airmouse_debug.idle = 0;

            /* Non-blocking LED feedback for the new mode: 3 short blinks = USB, 2 short blinks = wireless */
            start_mode_switch_led_feedback(g_output_mode);

            /* Resync timer and return without jumping */
            now = HAL_GetTick();
            last_step_tick = now;
            update_led(now);
            return;
        } else {
            /* Held 800 to 1500 ms: ignore */
        }
    }

    /* 2. Burst-read 14 bytes from MPU6050 (0x3B) */
    uint8_t buf[14];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H,
                                                I2C_MEMADD_SIZE_8BIT, buf, 14, 10);
#if IDLE_MODE
    float gyro_mag = 0.0f;
#endif

    if (status == HAL_OK) {
#if TILT_CORRECTION
        int16_t raw_ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
        int16_t raw_ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
        int16_t raw_az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
#endif

        int16_t raw_gx = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
        int16_t raw_gy = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
        int16_t raw_gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

        g_airmouse_debug.gx_raw = raw_gx;
        g_airmouse_debug.gy_raw = raw_gy;
        g_airmouse_debug.gz_raw = raw_gz;

        /* Step 2: Subtract bias, convert to deg/s (65.5 LSB per deg/s) */
        float gx_dps = ((float)raw_gx - g_airmouse_debug.bias_x) / 65.5f;
        float gy_dps = ((float)raw_gy - g_airmouse_debug.bias_y) / 65.5f;
        float gz_dps = ((float)raw_gz - g_airmouse_debug.bias_z) / 65.5f;

#if IDLE_MODE
        gyro_mag = sqrtf(gx_dps * gx_dps + gy_dps * gy_dps + gz_dps * gz_dps);

        /* Wake from idle if gyro magnitude exceeds wake threshold */
        if (s_idle && (gyro_mag >= IDLE_WAKE_DPS)) {
            s_idle = 0;
            g_airmouse_debug.idle = 0;
            filt_gx = 0.0f;
            filt_gy = 0.0f;
            filt_gz = 0.0f;
            rem_x = 0.0f;
            rem_y = 0.0f;
            last_activity_tick = now;
            set_led(g_airmouse_debug.enabled);
        }
#endif

#if TILT_CORRECTION
        /* Accelerometer roll estimation */
        float ax = (float)raw_ax / 16384.0f;
        float ay = (float)raw_ay / 16384.0f;
        float az = (float)raw_az / 16384.0f;
        (void)ay;
        float a_total = sqrtf(ax * ax + ay * ay + az * az);
        float roll_gyro_dps = (float)(ROLL_SIGN) * (EVAL_CONCAT(ROLL_GYRO_SOURCE, _dps));

        if (a_total >= TILT_ACC_MIN_G && a_total <= TILT_ACC_MAX_G) {
            float acc_roll = atan2f((float)(ROLL_ACC_A), (float)(ROLL_ACC_B)) * (180.0f / 3.14159265358979323846f);
            s_roll_deg = TILT_ALPHA * (s_roll_deg + roll_gyro_dps * TILT_DT) + (1.0f - TILT_ALPHA) * acc_roll;
        } else {
            /* Trust gyro only when experiencing acceleration or shaking */
            s_roll_deg = s_roll_deg + roll_gyro_dps * TILT_DT;
        }
        g_airmouse_debug.roll_deg = s_roll_deg;

        /* Rotate the two pointing gyro axes by -roll before deadzone and LPF */
        float rad = -s_roll_deg * (3.14159265358979323846f / 180.0f);
        float cos_r = cosf(rad);
        float sin_r = sinf(rad);

        float in_x = EVAL_CONCAT(X_SOURCE, _dps);
        float in_y = EVAL_CONCAT(Y_SOURCE, _dps);
        float rot_x = in_x * cos_r - in_y * sin_r;
        float rot_y = in_x * sin_r + in_y * cos_r;
        EVAL_CONCAT(X_SOURCE, _dps) = rot_x;
        EVAL_CONCAT(Y_SOURCE, _dps) = rot_y;
#else
        g_airmouse_debug.roll_deg = 0.0f;
#endif

        /* Step 3: Dead zone about 1.5 deg/s per axis */
        gx_dps = apply_deadzone(gx_dps, DEADZONE_DPS);
        gy_dps = apply_deadzone(gy_dps, DEADZONE_DPS);
        gz_dps = apply_deadzone(gz_dps, DEADZONE_DPS);

        /* Step 4: Low-pass filter, alpha = 0.3 */
        filt_gx = (LPF_ALPHA * gx_dps) + ((1.0f - LPF_ALPHA) * filt_gx);
        filt_gy = (LPF_ALPHA * gy_dps) + ((1.0f - LPF_ALPHA) * filt_gy);
        filt_gz = (LPF_ALPHA * gz_dps) + ((1.0f - LPF_ALPHA) * filt_gz);
    }

#if IDLE_MODE
    /* If currently in idle mode, suppress motion and send keepalive packets */
    if (s_idle) {
        set_led(0);
        g_airmouse_debug.dx = 0;
        g_airmouse_debug.dy = 0;
        g_airmouse_debug.accel_gain = 1.0f;
#if PRECISION_MODE
        g_airmouse_debug.precision_on = btn_precision.debounced_state;
#else
        g_airmouse_debug.precision_on = 0;
#endif

        uint8_t idle_buttons = 0;
        if (btn_left.debounced_state) {
            idle_buttons |= 0x01;
        }
        if (btn_right.debounced_state) {
            idle_buttons |= 0x02;
        }
        g_airmouse_debug.buttons = idle_buttons;

        if (g_output_mode == MODE_WIRELESS) {
            if ((now - last_idle_packet_tick) >= IDLE_HEARTBEAT_MS) {
                last_idle_packet_tick = now;
                hc05_send_mouse(0, 0, idle_buttons);
            }
        }
        return;
    }
#endif

    /* Step 5 & 6: Axis mapping & sensitivity scaling */
    float gx = filt_gx;
    float gy = filt_gy;
    float gz = filt_gz;
    (void)gx;
    (void)gy;
    (void)gz;

    int8_t out_dx = 0;
    int8_t out_dy = 0;

#if PRECISION_MODE
    uint8_t precision_active = btn_precision.debounced_state;
    g_airmouse_debug.precision_on = precision_active;
#else
    g_airmouse_debug.precision_on = 0;
#endif

    float gain = 1.0f;

    if (g_airmouse_debug.enabled) {
        float raw_x = (float)(X_SIGN) * (X_SOURCE);
        float raw_y = (float)(Y_SIGN) * (Y_SOURCE);

        float sens = SENSITIVITY;
#if PRECISION_MODE
        if (precision_active) {
            sens *= PRECISION_FACTOR;
        }
#endif

#if POINTER_ACCEL
#if PRECISION_MODE
        if (!precision_active) {
#endif
            float speed = sqrtf(raw_x * raw_x + raw_y * raw_y);
            float capped_speed = (speed > ACCEL_CAP_DPS) ? ACCEL_CAP_DPS : speed;
            gain = 1.0f + (ACCEL_GAIN * capped_speed / ACCEL_REF_DPS);
#if PRECISION_MODE
        }
#endif
#endif

        g_airmouse_debug.accel_gain = gain;

        float mouse_x = raw_x * gain * sens;
        float mouse_y = raw_y * gain * sens;

        float total_x = mouse_x + rem_x;
        float total_y = mouse_y + rem_y;

        int32_t step_dx = (int32_t)total_x;
        int32_t step_dy = (int32_t)total_y;

        rem_x = total_x - (float)step_dx;
        rem_y = total_y - (float)step_dy;

        /* Clamp to [-127, 127] */
        if (step_dx > 127) step_dx = 127;
        if (step_dx < -127) step_dx = -127;
        if (step_dy > 127) step_dy = 127;
        if (step_dy < -127) step_dy = -127;

        out_dx = (int8_t)step_dx;
        out_dy = (int8_t)step_dy;
    } else {
        rem_x = 0.0f;
        rem_y = 0.0f;
        out_dx = 0;
        out_dy = 0;
        g_airmouse_debug.accel_gain = 1.0f;
    }

    g_airmouse_debug.dx = out_dx;
    g_airmouse_debug.dy = out_dy;
    g_airmouse_debug.g_output_mode = g_output_mode;

    /* Left and right buttons (active even if movement disabled) */
    uint8_t buttons = 0;
    if (btn_left.debounced_state) {
        buttons |= 0x01; /* Left button */
    }
    if (btn_right.debounced_state) {
        buttons |= 0x02; /* Right button */
    }
    g_airmouse_debug.buttons = buttons;

    if (g_output_mode == MODE_USB_HID) {
        /* Send 4-byte report {buttons, dx, dy, 0} only if USB is CONFIGURED */
        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
            static uint8_t report[4];
            report[0] = buttons;
            report[1] = (uint8_t)out_dx;
            report[2] = (uint8_t)out_dy;
            report[3] = 0;
            USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
        }
    } else {
        /* Wireless (HC-05 UART) mode: Send 5-byte packet every PACKET_PERIOD_MS (20 ms) */
        static uint32_t last_packet_tick = 0;
        if ((now - last_packet_tick) >= PACKET_PERIOD_MS) {
            last_packet_tick = now;
            hc05_send_mouse(out_dx, out_dy, buttons);
        }
    }

    /* Update non-blocking LED feedback or normal state */
    update_led(now);

#if IDLE_MODE
    /* Idle mode entry detection */
    int idle_allowed = (g_output_mode == MODE_WIRELESS) || IDLE_IN_USB_MODE;
    if (any_button_down || any_button_debounced || out_dx != 0 || out_dy != 0 || gyro_mag >= IDLE_GYRO_DPS) {
        last_activity_tick = now;
    } else if (idle_allowed && ((now - last_mode_switch_tick) >= 2000)) {
        if ((now - last_activity_tick) >= IDLE_TIMEOUT_MS) {
            s_idle = 1;
            g_airmouse_debug.idle = 1;
            set_led(0);
            rem_x = 0.0f;
            rem_y = 0.0f;
            filt_gx = 0.0f;
            filt_gy = 0.0f;
            filt_gz = 0.0f;
            last_idle_packet_tick = now;
        }
    }
#else
    g_airmouse_debug.idle = 0;
#endif
}

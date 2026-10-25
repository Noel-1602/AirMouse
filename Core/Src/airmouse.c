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

/* Global debug struct for GDB inspection */
volatile airmouse_debug_t g_airmouse_debug = {0};

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

/* Filter and accumulator states */
static float filt_gx = 0.0f;
static float filt_gy = 0.0f;
static float filt_gz = 0.0f;
static float rem_x = 0.0f;
static float rem_y = 0.0f;
static uint32_t last_step_tick = 0;

/* Helper: Active-low LED control on PC13 */
static inline void set_led(uint8_t on) {
    if (on) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); /* LED ON */
    } else {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);   /* LED OFF */
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
 * Calibrate Gyro:
 * Averages ~500 gyro samples while still and stores as bias.
 * Fast-blinks the LED during calibration.
 */
static void airmouse_calibrate(void) {
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;
    uint32_t valid_samples = 0;
    uint8_t buf[14];

    for (int i = 0; i < 500; i++) {
        /* Blink fast: toggle every 25 samples (~50 ms) */
        if ((i % 25) == 0) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }

        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H,
                             I2C_MEMADD_SIZE_8BIT, buf, 14, 20) == HAL_OK) {
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

    /* Initial gyro calibration (averages 500 samples, fast LED blink) */
    airmouse_calibrate();

    /* Safety: cursor disabled after boot until recentre pressed once */
    g_airmouse_debug.enabled = 0;
    set_led(0); /* LED off = movement disabled */

    /* Initialize button states */
    uint32_t now = HAL_GetTick();
    last_step_tick = now;

    uint8_t left_raw = (HAL_GPIO_ReadPin(BTN_LEFT_GPIO_Port, BTN_LEFT_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    uint8_t right_raw = (HAL_GPIO_ReadPin(BTN_RIGHT_GPIO_Port, BTN_RIGHT_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    uint8_t recenter_raw = (HAL_GPIO_ReadPin(BTN_RECENTER_GPIO_Port, BTN_RECENTER_Pin) == GPIO_PIN_RESET) ? 1 : 0;

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

#if OUTPUT_MODE_WIRELESS
    hc05_init();
#endif

    return 0;
}

void airmouse_step(void) {
    uint32_t now = HAL_GetTick();
    if ((now - last_step_tick) < 10) {
        return;
    }
    last_step_tick = now;

    /* 1. Read and debounce buttons (active-low with internal pull-up) */
    uint8_t left_raw = (HAL_GPIO_ReadPin(BTN_LEFT_GPIO_Port, BTN_LEFT_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    uint8_t right_raw = (HAL_GPIO_ReadPin(BTN_RIGHT_GPIO_Port, BTN_RIGHT_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    uint8_t recenter_raw = (HAL_GPIO_ReadPin(BTN_RECENTER_GPIO_Port, BTN_RECENTER_Pin) == GPIO_PIN_RESET) ? 1 : 0;

    update_button(&btn_left, left_raw, now);
    update_button(&btn_right, right_raw, now);
    update_button(&btn_recenter, recenter_raw, now);

    /* Detect rising edge of debounced pressed state (recenter press) */
    uint8_t recenter_press_edge = (btn_recenter.debounced_state && !btn_recenter.prev_state);
    btn_recenter.prev_state = btn_recenter.debounced_state;

    if (recenter_press_edge) {
        /* Recalibrate gyro and fast-blink LED */
        airmouse_calibrate();

        /* Toggle movement enabled state */
        g_airmouse_debug.enabled = !g_airmouse_debug.enabled;
        set_led(g_airmouse_debug.enabled);

        /* Resync buttons and timer after calibration */
        now = HAL_GetTick();
        last_step_tick = now;

        recenter_raw = (HAL_GPIO_ReadPin(BTN_RECENTER_GPIO_Port, BTN_RECENTER_Pin) == GPIO_PIN_RESET) ? 1 : 0;
        btn_recenter.raw_state = recenter_raw;
        btn_recenter.debounced_state = recenter_raw;
        btn_recenter.prev_state = recenter_raw;
        btn_recenter.last_change_tick = now;
        return;
    }

    /* 2. Burst-read 14 bytes from MPU6050 (0x3B) */
    uint8_t buf[14];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H,
                                                I2C_MEMADD_SIZE_8BIT, buf, 14, 10);
    if (status == HAL_OK) {
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

        /* Step 3: Dead zone about 1.5 deg/s per axis */
        gx_dps = apply_deadzone(gx_dps, DEADZONE_DPS);
        gy_dps = apply_deadzone(gy_dps, DEADZONE_DPS);
        gz_dps = apply_deadzone(gz_dps, DEADZONE_DPS);

        /* Step 4: Low-pass filter, alpha = 0.3 */
        filt_gx = (LPF_ALPHA * gx_dps) + ((1.0f - LPF_ALPHA) * filt_gx);
        filt_gy = (LPF_ALPHA * gy_dps) + ((1.0f - LPF_ALPHA) * filt_gy);
        filt_gz = (LPF_ALPHA * gz_dps) + ((1.0f - LPF_ALPHA) * filt_gz);
    }

    /* Step 5 & 6: Axis mapping & sensitivity scaling */
    float gx = filt_gx;
    float gy = filt_gy;
    float gz = filt_gz;
    (void)gx;
    (void)gy;
    (void)gz;

    int8_t out_dx = 0;
    int8_t out_dy = 0;

    if (g_airmouse_debug.enabled) {
        float mouse_x = (float)(X_SIGN) * (X_SOURCE) * SENSITIVITY;
        float mouse_y = (float)(Y_SIGN) * (Y_SOURCE) * SENSITIVITY;

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
    }

    g_airmouse_debug.dx = out_dx;
    g_airmouse_debug.dy = out_dy;

    /* Left and right buttons (active even if movement disabled) */
    uint8_t buttons = 0;
    if (btn_left.debounced_state) {
        buttons |= 0x01; /* Left button */
    }
    if (btn_right.debounced_state) {
        buttons |= 0x02; /* Right button */
    }
    g_airmouse_debug.buttons = buttons;

#if OUTPUT_MODE_WIRELESS
    /* Wireless (HC-05 UART) mode: Send 5-byte packet every PACKET_PERIOD_MS (20 ms) */
    static uint32_t last_packet_tick = 0;
    if ((now - last_packet_tick) >= PACKET_PERIOD_MS) {
        last_packet_tick = now;
        hc05_send_mouse(out_dx, out_dy, buttons);
    }
#else
    /* Send 4-byte report {buttons, dx, dy, 0} only if configured */
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        static uint8_t report[4];
        report[0] = buttons;
        report[1] = (uint8_t)out_dx;
        report[2] = (uint8_t)out_dy;
        report[3] = 0;
        USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
    }
#endif
}

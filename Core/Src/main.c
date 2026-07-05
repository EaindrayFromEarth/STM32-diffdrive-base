/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "motor_control.h"
#include "usbd_cdc_if.h"
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* DWT-based debounce for Motor2 EXTI (PA7)
 * STM32F407 runs at 168 MHz → 168 cycles per microsecond
 * Change 168 if you ever change your system clock */
// #define DWT_US_TO_CYCLES(us)  ((us) * 168u)

/* Minimum microseconds between two valid encoder edges.
 * At max motor speed 178 RPM with ~1237 CPR (both edges):
 *   f_max = (178/60) × 1237 = 3670 edges/sec
 *   t_min = 1/3670 = 272 us
 * We use 200us so we never accidentally reject real pulses
 * but still catch bounce (which is typically <100us) */
// #define ENC2_DEBOUNCE_US      200u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t last_rpm_time = 0;
uint32_t last_pid_time = 0;
uint16_t adc_value; // ADC variable

MPU6050_t imu1;
uint32_t last_imu_time = 0;

extern volatile uint8_t reset_request;
extern volatile uint32_t reset_token;
extern volatile uint8_t dbg_stream;

extern volatile int32_t enc_zero_m1;
extern volatile int32_t enc_zero_m2;
extern volatile uint8_t enc_request;

volatile int g_mpu_init_status = -1; // 0 = HAL_OK
volatile int g_mpu_calib_status = -1;

const int PWM_MIN_M1 = 28;
const int PWM_MIN_M2 = 28;

const uint32_t KICK_MS = 100;
const int KICK_EXTRA_M1 = 0;
const int KICK_EXTRA_M2 = 0; // try 8..15

const float ACC_LIMIT_M1 = 25.0f; // rad/s^2
const float ACC_LIMIT_M2 = 25.0f; // rad/s^2 (more gentle for motor2)

const float COUNTS_PER_REV_M1 = 1240.00f;
const float COUNTS_PER_REV_M2 = 1239.67f;

// ---- wheel speed trims (start values) ----
const float VEL_SCALE_M1_FWD = 1.00f; // was 0.84f
const float VEL_SCALE_M1_REV = 1.00f; // was 1.00f

const float VEL_SCALE_M2_FWD = 1.045f;
const float VEL_SCALE_M2_REV = 1.00f;

Motor motor1 = {
    .encoder_tim = &htim2,
    .pwm_tim = &htim4,
    .in1_port = GPIOD,
    .in1_pin = GPIO_PIN_13,
    .in2_port = GPIOD,
    .in2_pin = GPIO_PIN_14,
    .encoder_count = 0,
    .rpm = 0.0f,
    .target_velocity = 0.0f,
    .error_integral = 0.0f,
    .prev_error = 0.0f,
    .kp = 0.30f,
    .ki = 0.02f,
    .kd = 0.00f,
    .enc_sign = -1};

Motor motor2 = {
    .encoder_tim = &htim3,
    .pwm_tim = &htim9,
    .in1_port = GPIOE,
    .in1_pin = GPIO_PIN_0,
    .in2_port = GPIOE,
    .in2_pin = GPIO_PIN_1,
    .encoder_count = 0,
    .rpm = 0.0f,
    .target_velocity = 0.0f,
    .error_integral = 0.0f,
    .prev_error = 0.0f,
    .kp = 0.30f,
    .ki = 0.02f,
    .kd = 0.00f,
    .enc_sign = +1};

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void set_motor_speed(Motor *motor, int speed_percent, uint8_t direction, uint8_t brake);
void read_encoder(Motor *motor);
void calculate_rpm(Motor *motor);
void pid_control(Motor *motor);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_USB_DEVICE_Init();
    MX_TIM9_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();
    /* USER CODE BEGIN 2 */
    /* Wake + configure the MPU6050. Keep the robot STILL for ~1 second after reset
       (gyro bias is measured here). Results are printed from the main loop. */
    g_mpu_init_status = (int)MPU6050_Init(&imu1);
    if (g_mpu_init_status == 0)
    {                                                                // HAL_OK
        g_mpu_calib_status = (int)MPU6050_CalibrateGyro(&imu1, 200); // ~1 s, robot still
    }

    /* DWT cycle counter (leave as-is) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    /* USER CODE END 2 */
    /* Enable DWT cycle counter for microsecond timing in EXTI ISR.
     * Must be done before the while loop starts.
     * DEMCR: enable trace, CTRL: start counter */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    // Start peripherals for both motors
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    htim2.Instance->CNT = 0;

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    htim3.Instance->CNT = 0;

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // Motor1 PWM
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); // Motor2 PWM

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_value, 1);

    uint8_t test_phase = 0;
    uint8_t usb_connected = 0;

    uint32_t motor_test_start = HAL_GetTick();

    // Blink LED to indicate code is running
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

    // Send test message
    CDC_Transmit_FS((uint8_t *)"USB CDC Working!\r\n", 18);
    HAL_Delay(1000);

    while (1)
    {
        /* ---- MPU6050 DIAGNOSTIC: runs when you type 'i' in the console ---- */
        if (i2c_scan_request)
        {
            i2c_scan_request = 0;
            CDC_Printf("\r\n--- MPU6050 DIAG ---\r\n");
            for (uint8_t a = 0x68; a <= 0x69; a++)
            {
                HAL_StatusTypeDef s = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 5, 100);
                CDC_Printf("scan 0x%02X -> %s (HAL=%d)\r\n", a, (s == HAL_OK) ? "FOUND" : "no", (int)s);
            }
            uint8_t who = 0xAA;
            HAL_StatusTypeDef r = HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x75,
                                                   I2C_MEMADD_SIZE_8BIT, &who, 1, 100);
            CDC_Printf("WHO_AM_I read=%s (HAL=%d) val=0x%02X (expect 0x68)\r\n",
                       (r == HAL_OK) ? "OK" : "FAIL", (int)r, who);
            CDC_Printf("MPU_Init=%d Calib=%d | live gz=%.4f yaw=%.4f\r\n",
                       g_mpu_init_status, g_mpu_calib_status, imu1.gz_rad_s, imu1.yaw_rad);
        }
        /* ---- HANDLE RESET REQUEST (from 'z' command) ---- */
        if (reset_request)
        {
            reset_request = 0;

            // Stop motors immediately
            motor1.target_velocity = 0.0f;
            motor2.target_velocity = 0.0f;
            set_motor_speed(&motor1, 0, 0, 0);
            set_motor_speed(&motor2, 0, 0, 0);

            // Reset software counts
            motor1.encoder_count = 0;
            motor2.encoder_count = 0;

            // Reset hardware encoder counters
            __HAL_TIM_SET_COUNTER(motor1.encoder_tim, 0);
            if (motor2.encoder_tim)
            {
                __HAL_TIM_SET_COUNTER(motor2.encoder_tim, 0);
            }

            // Reset PID state
            motor1.error_integral = 0.0f;
            motor1.prev_error = 0.0f;
            motor2.error_integral = 0.0f;
            motor2.prev_error = 0.0f;

            // Reset RPM
            motor1.rpm = 0.0f;
            motor2.rpm = 0.0f;

            // Reset "ENC zero offsets"
            enc_zero_m1 = 0;
            enc_zero_m2 = 0;

            // Reset timing so we don't get huge dt spikes
            last_rpm_time = HAL_GetTick();
            last_pid_time = HAL_GetTick();

            // Tell calculate_rpm/pid_control to clear their internal statics
            reset_token++;

            //  print ENC immediately (so you see "ENC 0 0" after 'z') (optional)
            enc_request = 1;
        }
        if (enc_request)
        {
            enc_request = 0;
            send_encoder_values();
        }

        uint32_t now = HAL_GetTick();
        if (now - last_imu_time >= 10)
        { // 100 Hz
            float dt;

            if (last_imu_time == 0)
            {
                dt = 0.01f;
            }
            else
            {
                dt = (now - last_imu_time) / 1000.0f;
            }

            if (MPU6050_Update(&imu1, dt) != HAL_OK)
            {
            }

            last_imu_time = now;
        }
        static uint8_t sync_stop_active = 0;
        static uint32_t sync_stop_t0 = 0;

        uint8_t cmd_zero_all =
            (fabsf(motor1.target_velocity) < 1e-3f) &&
            (fabsf(motor2.target_velocity) < 1e-3f);
        // ---- detect STOP -> MOVE transition ----
        static uint8_t was_cmd_zero_all = 1;

        if (!cmd_zero_all && was_cmd_zero_all)
        {

            // Reset timing so first dt isn't weird (even if calculate_rpm uses dt later)
            last_rpm_time = now;
            last_pid_time = now;

            reset_token++;

            motor1.error_integral = 0.0f;
            motor1.prev_error = 0.0f;
            motor2.error_integral = 0.0f;
            motor2.prev_error = 0.0f;

            motor1.rpm = 0.0f;
            motor2.rpm = 0.0f;
        }

        was_cmd_zero_all = cmd_zero_all;

        if (cmd_zero_all)
        {
            if (!sync_stop_active)
            {
                sync_stop_active = 1;
                sync_stop_t0 = now;
            }

            if ((now - sync_stop_t0) < 200)
            {
                set_motor_speed(&motor1, 25, 0, 1);
                set_motor_speed(&motor2, 35, 0, 1);
            }
            else
            {
                set_motor_speed(&motor1, 0, 0, 0);
                set_motor_speed(&motor2, 0, 0, 0);
            }

            read_encoder(&motor1);
            read_encoder(&motor2);

            HAL_Delay(10);
            continue;
        }
        else
        {
            sync_stop_active = 0;
        }

        if (now - last_rpm_time >= 10)
        {
            read_encoder(&motor1);
            read_encoder(&motor2);
            calculate_rpm(&motor1);
            calculate_rpm(&motor2);
            static uint32_t last_dbg = 0;
            static uint32_t last_hits = 0;

            if (dbg_stream && (now - last_dbg >= 200))
            {
                last_dbg = now;
                CDC_Printf("gz=%.4f yaw=%.4f\r\n", imu1.gz_rad_s, imu1.yaw_rad);
            }

            last_rpm_time = now;
        }

        if (now - last_pid_time >= 20)
        {
            pid_control(&motor1);
            pid_control(&motor2);
            last_pid_time = now;
        }

        static uint32_t last_print = 0;

        HAL_Delay(10);

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

void set_motor_speed(Motor *motor, int speed_percent, uint8_t direction, uint8_t brake)
{
    if (speed_percent < 0)
        speed_percent = 0;
    if (speed_percent > 100)
        speed_percent = 100;
    motor->last_pwm = speed_percent;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(motor->pwm_tim);
    uint32_t pwm_val = (speed_percent * arr) / 100;
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, TIM_CHANNEL_1, pwm_val);
    // If PWM is zero and we're not braking, coast (IN1=0, IN2=0)
    if (!brake && speed_percent == 0)
    {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
        return;
    }

    if (brake)
    {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);

        return;
    }

    GPIO_PinState in1 = direction ? GPIO_PIN_SET : GPIO_PIN_RESET;
    GPIO_PinState in2 = direction ? GPIO_PIN_RESET : GPIO_PIN_SET;

    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, in1);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, in2);

    if (motor == &motor2 && speed_percent > 0)
    {
    }
}

void read_encoder(Motor *motor)
{
    int16_t raw = (int16_t)(motor->encoder_tim->Instance->CNT);
    motor->encoder_tim->Instance->CNT = 0;

    int32_t delta = (int32_t)motor->enc_sign * (int32_t)raw;
    motor->last_delta_counts = delta;
    motor->encoder_count += delta;
}

void calculate_rpm(Motor *motor)
{
    extern volatile uint32_t reset_token;

    static uint32_t last_time[2] = {0, 0};
    static int32_t last_count[2] = {0, 0};
    static float rpm_filt[2] = {0.0f, 0.0f};

    static uint32_t seen_token[2] = {0, 0};

    uint8_t idx = (motor == &motor2) ? 1 : 0;
    uint32_t now = HAL_GetTick();

    /* ---- CLEAR INTERNAL STATE AFTER 'z' ---- */
    if (seen_token[idx] != reset_token)
    {
        seen_token[idx] = reset_token;
        last_time[idx] = 0;
        last_count[idx] = motor->encoder_count;
        rpm_filt[idx] = 0.0f;
        motor->rpm = 0.0f;
        return;
    }

    uint32_t period_ms = 10;
    if (now - last_time[idx] < period_ms)
        return;

    if (last_time[idx] == 0)
    {
        last_time[idx] = now;
        last_count[idx] = motor->encoder_count;
        motor->rpm = 0.0f;
        return;
    }
    uint32_t dt_ms = now - last_time[idx];
    if (dt_ms == 0)
        return;

    int32_t diff = motor->encoder_count - last_count[idx];
    int32_t adiff = (diff < 0) ? -diff : diff;

    float dt = dt_ms / 1000.0f;
    float cpr = (motor == &motor2) ? COUNTS_PER_REV_M2 : COUNTS_PER_REV_M1;

    int32_t min_counts = (motor == &motor2) ? 2 : 2; // or 1, but SAME for both

    float rpm_meas;
    if (adiff >= min_counts)
    {
        rpm_meas = (diff * 60.0f) / (cpr * dt);
    }
    else
    {
        if (fabsf(motor->target_velocity) < 0.2f)
        {
            rpm_meas = 0.0f;
        }
        else
        {
            float decay = 0.90f;
            rpm_meas = rpm_filt[idx] * decay;
        }
    }
    /* ---- CLAMP HERE ---- */
    if (rpm_meas > 220.0f)
        rpm_meas = 220.0f;
    if (rpm_meas < -220.0f)
        rpm_meas = -220.0f;

    float alpha = 0.60f;

    rpm_filt[idx] = alpha * rpm_filt[idx] + (1.0f - alpha) * rpm_meas;

    motor->rpm = rpm_filt[idx];

    last_count[idx] = motor->encoder_count;
    last_time[idx] = now;
}

void pid_control(Motor *motor)
{
    extern volatile uint32_t reset_token;

    static uint32_t last_t[2] = {0, 0};
    static float ramp_target[2] = {0.0f, 0.0f};
    static uint32_t seen_token[2] = {0, 0};

    static uint32_t kick_t0[2] = {0, 0};
    static float last_cmd[2] = {0.0f, 0.0f};
    static int32_t kick_start_enc[2] = {0, 0};
    static uint8_t moving[2] = {0, 0};
    static uint8_t stop_state[2] = {0, 0}; // 0=RUN, 1=STOPPING, 2=STOPPED
    static uint8_t stop_good[2] = {0, 0};

    static uint32_t last_dbg[2] = {0, 0};
    static uint32_t dbg_until[2] = {0, 0};
    static int last_pwm[2] = {0, 0};
    static uint8_t printed_stopping[2] = {0, 0};
    static uint8_t printed_stopped[2] = {0, 0};

    uint8_t idx = (motor == &motor2) ? 1 : 0;
    uint32_t now = HAL_GetTick();

    const int32_t KICK_COUNTS_M1 = 6;
    const int32_t KICK_COUNTS_M2 = 12;
    const uint32_t KICK_MIN_MS = 30;
    const uint32_t KICK_WIN_MS = (motor == &motor2) ? 300 : 150;

    if (seen_token[idx] != reset_token)
    {
        seen_token[idx] = reset_token;

        last_t[idx] = 0;
        ramp_target[idx] = 0.0f;

        kick_t0[idx] = 0;
        last_cmd[idx] = 0.0f;
        kick_start_enc[idx] = motor->encoder_count;
        moving[idx] = 0;

        stop_state[idx] = 0;
        stop_good[idx] = 0;

        last_dbg[idx] = 0;
        dbg_until[idx] = 0;

        motor->error_integral = 0.0f;
        motor->prev_error = 0.0f;

        last_pwm[idx] = 0;
        printed_stopping[idx] = 0;
        printed_stopped[idx] = 0;
    }

    uint8_t cmd_zero = (fabsf(motor->target_velocity) < 1e-3f);
    if (cmd_zero)
    {
        if (stop_state[idx] == 0)
        {
            stop_state[idx] = 1;
            printed_stopping[idx] = 0;
            printed_stopped[idx] = 0;
            if (dbg_stream && !printed_stopping[idx])
            {
                printed_stopping[idx] = 1;
                CDC_Printf("M%u -> STOPPING\r\n", (unsigned)(idx + 1));
            }
            stop_good[idx] = 0;
            dbg_until[idx] = now + 1500;
            last_dbg[idx] = 0;

            motor->error_integral = 0.0f;
            motor->prev_error = 0.0f;

            kick_t0[idx] = 0;
            moving[idx] = 0;
            last_pwm[idx] = 0;
        }
    }
    else
    {
        stop_state[idx] = 0;
        stop_good[idx] = 0;
        printed_stopping[idx] = 0;
        printed_stopped[idx] = 0;
    }

    if (stop_state[idx] == 2 && cmd_zero)
    {
        set_motor_speed(motor, 0, 0, 0);
        last_pwm[idx] = 0;

        if (dbg_stream && printed_stopped[idx] == 0)
        {
            printed_stopped[idx] = 1;
            CDC_Printf("M%u still STOPPED (holding)\r\n", (unsigned)(idx + 1));
        }
        return;
    }

    // ---- detect start (0 -> nonzero command) ----
    float cmd_now = motor->target_velocity;
    float cmd_prev = last_cmd[idx];

    uint8_t was_zero = (fabsf(cmd_prev) < 1e-3f);
    uint8_t now_nonzero = (fabsf(cmd_now) > 1e-3f);

    // SIGN CHANGE detection (e.g., -5 -> +5 or +5 -> -5)
    uint8_t sign_change =
        (cmd_prev > 1e-3f && cmd_now < -1e-3f) ||
        (cmd_prev < -1e-3f && cmd_now > 1e-3f);

    if ((was_zero && now_nonzero) || sign_change)
    {
        kick_t0[idx] = now;
        kick_start_enc[idx] = motor->encoder_count;
        moving[idx] = 0;

        dbg_until[idx] = now + 800;

        motor->error_integral = 0.0f;
        motor->prev_error = 0.0f;

        ramp_target[idx] = 0.0f;
        last_t[idx] = 0;
        last_pwm[idx] = 0;
    }

    last_cmd[idx] = cmd_now;

    float dt = (last_t[idx] == 0) ? 0.02f : (now - last_t[idx]) / 1000.0f;
    if (dt <= 0.0f)
        dt = 0.02f;
    last_t[idx] = now;

    float accel_limit = (motor == &motor2) ? ACC_LIMIT_M2 : ACC_LIMIT_M1;
    float max_step = accel_limit * dt;

    float boosted_cmd = motor->target_velocity;
    {
        float t_other = (motor == &motor2) ? motor1.target_velocity
                                           : motor2.target_velocity;
        uint8_t both_active =
            (fabsf(boosted_cmd) > 0.05f) && (fabsf(t_other) > 0.05f);
        if (both_active)
        {
            if ((boosted_cmd * t_other) < 0.0f)
            {
                boosted_cmd *= 5.0f;
            }
            else
            {
                boosted_cmd *= 3.0f;
            }
            if (boosted_cmd > 18.0f)
                boosted_cmd = 18.0f;
            if (boosted_cmd < -18.0f)
                boosted_cmd = -18.0f;
        }
    }
    float cmd = (stop_state[idx] == 1) ? 0.0f : boosted_cmd;
    float delta = cmd - ramp_target[idx];

    if (delta > max_step)
        delta = max_step;
    if (delta < -max_step)
        delta = -max_step;

    ramp_target[idx] += delta;
    float target = ramp_target[idx];

    float current_velocity = motor->rpm * (2.0f * M_PI) / 60.0f;
    if (stop_state[idx] == 1)
    {
        const float V_EPS = 0.15f; // rad/s
        const float T_EPS = 0.15f; // rad/s

        if (fabsf(ramp_target[idx]) < T_EPS && fabsf(current_velocity) < V_EPS)
        {
            if (++stop_good[idx] >= 3)
            {
                set_motor_speed(motor, 0, 0, 0);

                motor->error_integral = 0.0f;
                motor->prev_error = 0.0f;
                ramp_target[idx] = 0.0f;
                last_t[idx] = 0;

                moving[idx] = 0;
                kick_t0[idx] = 0;

                stop_state[idx] = 2; // STOPPED
                stop_good[idx] = 0;
                if (dbg_stream && !printed_stopped[idx])
                {
                    printed_stopped[idx] = 1;
                    CDC_Printf("M%u -> STOPPED (PWM cut)\r\n", (unsigned)(idx + 1));
                }
                return;
            }
        }
        else
        {
            stop_good[idx] = 0;
        }
    }

    float error = target - current_velocity;

    float p_term = motor->kp * error;

    if (stop_state[idx] == 0)
    {
        motor->error_integral += error * dt;
    }

    float i_term = motor->ki * motor->error_integral;

    float d_term = motor->kd * (error - motor->prev_error) / dt;
    motor->prev_error = error;

    float output = p_term + i_term + d_term;

    uint8_t dir = (target < 0.0f);
    uint8_t brake = 0;
    float u = output;

    const float BRAKE_DB = 0.10f; // start at 0.10, tune 0.05..0.20

    if (target >= 0.0f)
    {
        if (u < -BRAKE_DB)
        {
            brake = 1;
            u = -u;
        }
        else if (u < 0.0f)
        {
            u = 0.0f;
        }
    }
    else
    {
        if (u > BRAKE_DB)
        {
            brake = 1;
        }
        u = fabsf(u);
        if (!brake && u < BRAKE_DB)
            u = 0.0f;
    }

    int speed_percent = (int)(u * 100.0f);
    if (speed_percent > 100)
        speed_percent = 100;
    if (stop_state[idx] == 1)
    {
        brake = 1;
        dir = 0;

        const int STOP_PWM_MAX = 50;
        if (speed_percent > STOP_PWM_MAX)
            speed_percent = STOP_PWM_MAX;
    }

    if (!brake)
    {
        float tmag = fabsf(target);

        if (speed_percent > 0 && tmag > 0.1f)
        {

            if (motor == &motor2)
            {
                if (speed_percent < PWM_MIN_M2)
                    speed_percent = PWM_MIN_M2;
            }
            else
            {
                if (!moving[idx])
                {
                    if (speed_percent < PWM_MIN_M1)
                        speed_percent = PWM_MIN_M1;
                }
            }
        }
    }

    if (kick_t0[idx] != 0)
    {
        int32_t moved = motor->encoder_count - kick_start_enc[idx];
        if (moved < 0)
            moved = -moved;

        int32_t kick_counts = (motor == &motor2) ? KICK_COUNTS_M2 : KICK_COUNTS_M1;

        if ((now - kick_t0[idx]) >= KICK_MIN_MS && moved >= kick_counts)
        {
            moving[idx] = 1;
            kick_t0[idx] = 0;
        }
    }

    if (!brake)
    {
        if (speed_percent > 0 && !moving[idx] &&
            kick_t0[idx] != 0 && (now - kick_t0[idx]) < KICK_MS)
        {
            int base = (motor == &motor2) ? PWM_MIN_M2 : PWM_MIN_M1;
            int extra = (motor == &motor2) ? KICK_EXTRA_M2 : KICK_EXTRA_M1;
            int min_kick = base + extra;

            if (speed_percent < min_kick)
                speed_percent = min_kick;
            if (speed_percent > 100)
                speed_percent = 100;
        }
        else if (kick_t0[idx] != 0 && (now - kick_t0[idx]) >= KICK_WIN_MS)
        {

            int32_t moved = motor->encoder_count - kick_start_enc[idx];
            if (moved < 0)
                moved = -moved;

            if (moved >= 1 || fabsf(motor->rpm) > 1.0f)
            {
                moving[idx] = 1;
            }
            else
            {
                moving[idx] = 0;
            }

            kick_t0[idx] = 0;
        }
    }

    int step = (motor == &motor2) ? 4 : 8;
    int sp = speed_percent;

    if (sp > last_pwm[idx] + step)
        sp = last_pwm[idx] + step;
    if (sp < last_pwm[idx] - step)
        sp = last_pwm[idx] - step;

    last_pwm[idx] = sp;
    speed_percent = sp;

    if (dbg_until[idx] != 0 && (int32_t)(now - dbg_until[idx]) >= 0)
    {
        dbg_until[idx] = 0;
    }
    if (dbg_stream && (now - last_dbg[idx]) >= 100)
    {
        last_dbg[idx] = now;

        float cur = motor->rpm * (2.0f * M_PI) / 60.0f;
        uint32_t kick_age = (kick_t0[idx] == 0) ? 9999u : (now - kick_t0[idx]);

        CDC_Printf("M%u cmd=%.2f ramp=%.2f rpm=%.1f cur=%.2f pwm=%d dir=%u brk=%u mov=%u kickms=%lu stop=%u\r\n",
                   (unsigned)(idx + 1),
                   motor->target_velocity,
                   target,
                   motor->rpm,
                   cur,
                   speed_percent,
                   (unsigned)dir,
                   (unsigned)brake,
                   (unsigned)moving[idx],
                   (unsigned long)kick_age,
                   (unsigned)stop_state[idx]);
    }

    set_motor_speed(motor, speed_percent, dir, brake);
}

static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi
                                    : x;
}

void set_motor1_velocity(float v)
{
    v = clampf(v, -18.0f, 18.0f);
    float s = (v >= 0.0f) ? VEL_SCALE_M1_FWD : VEL_SCALE_M1_REV;
    motor1.target_velocity = s * v;
}

void set_motor2_velocity(float v)
{
    v = clampf(v, -18.0f, 18.0f);
    float s = (v >= 0.0f) ? VEL_SCALE_M2_FWD : VEL_SCALE_M2_REV;
    motor2.target_velocity = s * v;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

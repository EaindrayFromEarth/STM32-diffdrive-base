#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "tim.h"
#include "gpio.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef* encoder_tim;
    TIM_HandleTypeDef* pwm_tim;
    GPIO_TypeDef*      in1_port;
    uint32_t           in1_pin;
    GPIO_TypeDef*      in2_port;
    uint32_t           in2_pin;

    int32_t encoder_count;
    float   rpm;
    float   target_velocity;
    float   error_integral;
    float   prev_error;

    float   kp, ki, kd;   // PID gains for this motor
    int8_t  enc_sign;     // +1 or -1 for encoder direction
    int32_t last_delta_counts;
    int last_pwm;
} Motor;


// These are defined in main.c
extern Motor motor1;
extern Motor motor2;

// Function prototypes (already present, but make sure they match):
void set_motor_speed(Motor* motor, int speed_percent, uint8_t direction, uint8_t brake);
void read_encoder(Motor* motor);
void calculate_rpm(Motor* motor);
void pid_control(Motor* motor);
void set_motor1_velocity(float velocity);
void set_motor2_velocity(float velocity);

#endif // MOTOR_CONTROL_H

#ifndef MOTOR_H
#define MOTOR_H

#include "tim.h"
#include <stdint.h>

// Motor decay mode definitions
#define FAST_DECAY_MODE 1        // Fast decay mode for quicker motor stopping
#define SLOW_DECAY_MODE 0        // Slow decay mode for smoother motor stopping
#define PWM_MAX_SET_NUM 99       // Maximum PWM duty cycle value (0-99)
#define PWM_MIN_SET_NUM 0        // Minimum PWM duty cycle value (0-99)

// Motor state enumeration
typedef enum{
    MOTOR_DIR_FORWARD,    // Motor rotates in forward direction
    MOTOR_DIR_REVERSE,    // Motor rotates in reverse direction
    MOTOR_DIR_STOPPING,   // Motor is in the process of stopping (for smooth transitions)
    MOTOR_DIR_STOPPED,    // Motor is completely stopped
}Motor_Sate;

// Function prototypes for motor control
void motor_init(TIM_HandleTypeDef *htim_in1,uint32_t in1,TIM_HandleTypeDef *htim_in2,uint32_t in2);  // Initialize motor with timer handles and channels
void motor_set_state(uint8_t speed,Motor_Sate state,TIM_HandleTypeDef *htim,uint32_t in1,TIM_HandleTypeDef *htim_in2,uint32_t in2);  // Set motor speed and direction
void motor_stop(TIM_HandleTypeDef *htim_in1,uint32_t in1,TIM_HandleTypeDef *htim_in2,uint32_t in2,uint8_t state);  // Stop motor with specified decay mode

#endif /* MOTOR_H */
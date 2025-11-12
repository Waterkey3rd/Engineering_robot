#include "motor.h"
#include <stdint.h>

/**
 * @brief Initialize a motor with PWM channels
 * 
 * Starts PWM generation on both input channels and sets them to maximum value
 * (which effectively stops the motor since both inputs are high)
 * 
 * @param htim_in1 Timer handle for motor input 1
 * @param in1 Timer channel for motor input 1
 * @param htim_in2 Timer handle for motor input 2
 * @param in2 Timer channel for motor input 2
 */
void motor_init(TIM_HandleTypeDef *htim_in1,uint32_t in1,TIM_HandleTypeDef *htim_in2,uint32_t in2)
{
    HAL_TIM_PWM_Start(htim_in1,in1);                    // Start PWM on input 1
    __HAL_TIM_SET_COMPARE(htim_in1, in1, PWM_MAX_SET_NUM);  // Set input 1 to maximum (stops motor)
    HAL_TIM_PWM_Start(htim_in2,in2);                    // Start PWM on input 2
    __HAL_TIM_SET_COMPARE(htim_in2, in2, PWM_MAX_SET_NUM);  // Set input 2 to maximum (stops motor)
}

/**
 * @brief Set the state (speed and direction) of a motor
 * 
 * Configures the motor's speed and direction based on the specified state
 * 
 * @param speed Speed value (0-99) for the motor
 * @param state Direction/state for the motor
 * @param htim_in1 Timer handle for motor input 1
 * @param in1 Timer channel for motor input 1
 * @param htim_in2 Timer handle for motor input 2
 * @param in2 Timer channel for motor input 2
 */
void motor_set_state(uint8_t speed,Motor_Sate state,TIM_HandleTypeDef *htim_in1,uint32_t in1,TIM_HandleTypeDef *htim_in2,uint32_t in2)
{
    switch (state) {
        case MOTOR_DIR_FORWARD:  // Set motor to rotate forward
            __HAL_TIM_SET_COMPARE(htim_in1, in1, speed);  // Set input 1 to specified speed
            __HAL_TIM_SET_COMPARE(htim_in2, in2, 0);      // Set input 2 to minimum (off)
            break;
        case MOTOR_DIR_REVERSE:  // Set motor to rotate in reverse
            __HAL_TIM_SET_COMPARE(htim_in1, in1, 0);      // Set input 1 to minimum (off)
            __HAL_TIM_SET_COMPARE(htim_in2, in2, speed);  // Set input 2 to specified speed
            break;
        case MOTOR_DIR_STOPPED:  // Stop motor with fast decay (quick stop)
            motor_stop(htim_in1,in1,htim_in2,in2,FAST_DECAY_MODE);
            break;
        case MOTOR_DIR_STOPPING:  // Stop motor with slow decay (smooth stop)
            motor_stop(htim_in1,in1,htim_in2,in2,SLOW_DECAY_MODE);
            break;
    }
}

/**
 * @brief Stop a motor using specified decay mode
 * 
 * Stops the motor using either fast decay (braking) or slow decay (coasting)
 * 
 * @param htim_in1 Timer handle for motor input 1
 * @param in1 Timer channel for motor input 1
 * @param htim_in2 Timer handle for motor input 2
 * @param in2 Timer channel for motor input 2
 * @param state Decay mode (FAST_DECAY_MODE for quick stop, SLOW_DECAY_MODE for smooth stop)
 */
void motor_stop(TIM_HandleTypeDef *htim_in1,uint32_t in1,TIM_HandleTypeDef *htim_in2,uint32_t in2,uint8_t state)
{
    if(state==FAST_DECAY_MODE)  // Fast decay mode - both inputs set to high for braking
    {
        __HAL_TIM_SET_COMPARE(htim_in1 , in1, PWM_MAX_SET_NUM);  // Set both inputs high (brakes motor)
        __HAL_TIM_SET_COMPARE(htim_in2 , in2, PWM_MAX_SET_NUM);
    }
    else if(state==SLOW_DECAY_MODE)  // Slow decay mode - both inputs set to low for coasting
    {
        __HAL_TIM_SET_COMPARE(htim_in1 , in1, 0);  // Set both inputs low (allows coasting)
        __HAL_TIM_SET_COMPARE(htim_in2 , in2, 0);
    }
}
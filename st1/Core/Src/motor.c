#include "motor.h"
#include <stdint.h>

void motor_init(TIM_HandleTypeDef *htim_in1,uint32_t in1,TIM_HandleTypeDef *htim_in2,uint32_t in2)
{
    HAL_TIM_PWM_Start(htim_in1,in1);
    __HAL_TIM_SET_COMPARE(htim_in1, in1, PWM_MAX_SET_NUM);
    HAL_TIM_PWM_Start(htim_in2,in2);
    __HAL_TIM_SET_COMPARE(htim_in2, in2, PWM_MAX_SET_NUM);
}

void motor_set_state(uint8_t speed,Motor_Sate state,TIM_HandleTypeDef *htim_in1,uint32_t in1,TIM_HandleTypeDef *htim_in2,uint32_t in2)
{
    switch (state) {
        case MOTOR_DIR_FORWARD:
            __HAL_TIM_SET_COMPARE(htim_in1, in1, speed);
            __HAL_TIM_SET_COMPARE(htim_in2, in2, 0);            
            break;
        case MOTOR_DIR_REVERSE:
            __HAL_TIM_SET_COMPARE(htim_in1, in1, 0);
            __HAL_TIM_SET_COMPARE(htim_in2, in2, speed);
            break;
        case MOTOR_DIR_STOPPED:
            motor_stop(htim_in1,in1,htim_in2,in2,FAST_DECAY_MODE);
            break;
        case MOTOR_DIR_STOPPING:
            motor_stop(htim_in1,in1,htim_in2,in2,SLOW_DECAY_MODE);
            break;
    }
}

void motor_stop(TIM_HandleTypeDef *htim_in1,uint32_t in1,TIM_HandleTypeDef *htim_in2,uint32_t in2,uint8_t state)
{
    if(state==FAST_DECAY_MODE)
    {
        __HAL_TIM_SET_COMPARE(htim_in1 , in1, PWM_MAX_SET_NUM);
        __HAL_TIM_SET_COMPARE(htim_in2 , in2, PWM_MAX_SET_NUM);
    }
    else if(state==SLOW_DECAY_MODE)
    {
        __HAL_TIM_SET_COMPARE(htim_in1 , in1, 0);
        __HAL_TIM_SET_COMPARE(htim_in2 , in2, 0);
    }
}
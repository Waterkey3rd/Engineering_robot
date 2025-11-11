#ifndef MOTOR_H
#define MOTOR_H

#include "tim.h"
#include <stdint.h>

#define FAST_DECAY_MODE 1
#define SLOW_DECAY_MODE 0
#define PWM_MAX_SET_NUM 99
#define PWM_MIN_SET_NUM 0

typedef enum{
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_REVERSE,
    MOTOR_DIR_STOPPING,
    MOTOR_DIR_STOPPED,
}Motor_Sate;

void motor_init(TIM_HandleTypeDef *htim_in1,uint32_t in1,TIM_HandleTypeDef *htim_in2,uint32_t in2);//init
void motor_set_state(uint8_t speed,Motor_Sate state,TIM_HandleTypeDef *htim,uint32_t in1,TIM_HandleTypeDef *htim_in2,uint32_t in2);
void motor_stop(TIM_HandleTypeDef *htim_in1,uint32_t in1,TIM_HandleTypeDef *htim_in2,uint32_t in2,uint8_t state);

#endif /* MOTOR_H */
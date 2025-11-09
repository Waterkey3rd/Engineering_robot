#ifndef MOTOR_H
#define MOTOR_H

#include "tim.h"
#include <stdint.h>

typedef enum{
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_REVERSE,
    MOTOR_DIR_STOP
}Motor_Sate;

void motor_init(TIM_HandleTypeDef *htim,uint32_t Channel);//init
void motor_set_state(uint8_t speed,Motor_Sate state,TIM_HandleTypeDef *htim,uint32_t Channel);
void motor_stop(TIM_HandleTypeDef *htim,uint32_t Channel);

#endif /* MOTOR_H */
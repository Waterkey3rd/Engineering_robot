#ifndef CHASSIS_H
#define CHASSIS_H

#include "motor.h"
#include "tim.h"
#include <stdint.h>

#define motor1_d 1//motor1_direction
#define motor2_d 1//motor2_direction
#define motor3_d 1//motor3_direction
#define motor4_d 1//motor4_direction
//start motor_in_channel
#define motor1_in1_tim htim4
#define motor1_in1_channel TIM_CHANNEL_1
#define motor2_in1_tim htim4 
#define motor2_in1_channel TIM_CHANNEL_2
#define motor3_in1_tim htim4 
#define motor3_in1_channel TIM_CHANNEL_3
#define motor4_in1_tim htim4 
#define motor4_in1_channel TIM_CHANNEL_4
#define motor1_in2_tim htim3
#define motor1_in2_channel TIM_CHANNEL_1
#define motor2_in2_tim htim3 
#define motor2_in2_channel TIM_CHANNEL_2
#define motor3_in2_tim htim3 
#define motor3_in2_channel TIM_CHANNEL_3
#define motor4_in2_tim htim3 
#define motor4_in2_channel TIM_CHANNEL_4
//end motor_in_channel

enum ChassisMotionState{//ChassisMotionState
    CHASSIS_STATE_STOPPED,
    CHASSIS_STATE_FORWARD,
    CHASSIS_STATE_LEFT,
    CHASSIS_STATE_RIGHT,
    CHASSIS_STATE_ROTATE_CW,
    CHASSIS_STATE_ROTATE_CCW
};

extern uint8_t chassis_translational_velocity;//chassis_translational_velocity,but use pwm_duty_cycle to inform
extern uint8_t chassis_angular_velocity;//chassis_angular_velocity,but use pwm_duty_cycle to inform
extern enum ChassisMotionState current_chassis_state;//chassis_state,inform the chassis_state_machine 

void chassis_init();//init,start all motors'pwm,set all motor's pwm_duty_cycle 0
void chassis_motion();//arrcording to the variable "current_chassis_state" ,to set the chassis'movement.

#endif /* CHASSIS_H */
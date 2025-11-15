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
#define MOTOR1_IN1_HTIM htim4
#define MOTOR1_IN1_CHANNEL TIM_CHANNEL_1
#define MOTOR2_IN1_HTIM htim4
#define MOTOR2_IN1_CHANNEL TIM_CHANNEL_2
#define MOTOR3_IN1_HTIM htim4
#define MOTOR3_IN1_CHANNEL TIM_CHANNEL_3
#define MOTOR4_IN1_HTIM htim4
#define MOTOR4_IN1_CHANNEL TIM_CHANNEL_4
#define MOTOR1_IN2_HTIM htim3
#define MOTOR1_IN2_CHANNEL TIM_CHANNEL_1
#define MOTOR2_IN2_HTIM htim3
#define MOTOR2_IN2_CHANNEL TIM_CHANNEL_2
#define MOTOR3_IN2_HTIM htim3
#define MOTOR3_IN2_CHANNEL TIM_CHANNEL_3
#define MOTOR4_IN2_HTIM htim3
#define MOTOR4_IN2_CHANNEL TIM_CHANNEL_4
//end motor_in_channel
#define MAX_SPEED 99
#define MIN_SPEED 0
#define STOPPING_COUNT_NUM 2
#define ROTATE_MODE_ON (uint8_t)1
#define ROTATE_MODE_OFF (uint8_t)0

typedef enum {//ChassisMotionState
    CHASSIS_STATE_STOPPED,
    CHASSIS_STATE_STOPPING,
    CHASSIS_STATE_FORWARD,
    CHASSIS_STATE_BACK,
    CHASSIS_STATE_LEFT,
    CHASSIS_STATE_RIGHT,
    CHASSIS_STATE_ROTATE_CW,
    CHASSIS_STATE_ROTATE_CCW
}ChassisMotionState;

extern uint8_t chassis_velocity;
// extern uint8_t chassis_translational_velocity;//chassis_translational_velocity,but use pwm_duty_cycle to inform
// extern uint8_t chassis_angular_velocity;//chassis_angular_velocity,but use pwm_duty_cycle to inform
extern ChassisMotionState chassis_current_state;//chassis_state,inform the chassis_state_machine
extern uint8_t chassis_a; 
extern uint8_t chassis_stopping_count;
extern uint8_t chassis_target_v;

void chassis_init();//init,start all motors'pwm,set all motor's pwm_duty_cycle 0
void chassis_motion();//arrcording to the variable "current_chassis_state" ,to set the chassis'movement.
void chassis_stopped();
void chassis_stopping();
void chassis_fowardmove(uint16_t v);
void chassis_backmove(uint16_t v);
void chassis_leftmove(uint16_t v);
void chassis_rightmove(uint16_t v);
void chassis_rotate_cw(uint16_t v);
void chassis_rotate_ccw(uint16_t v);
void chassis_change_state(ChassisMotionState targetstate,uint8_t target_v);

#endif /* CHASSIS_H */
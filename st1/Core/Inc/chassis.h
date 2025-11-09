#ifndef CHASSIS_H
#define CHASSIS_H

#include "motor.h"
#include <stdint.h>

#define motor1_d 1//motor1_direction
#define motor2_d 1//motor2_direction
#define motor3_d 1//motor3_direction
#define motor4_d 1//motor4_direction

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


#endif /* CHASSIS_H */
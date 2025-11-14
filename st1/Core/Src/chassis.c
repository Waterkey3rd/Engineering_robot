#include "chassis.h"
#include "motor.h"
#include <stdint.h>

uint8_t chassis_velocity;
// extern uint8_t chassis_translational_velocity;//chassis_translational_velocity,but use pwm_duty_cycle to inform
// extern uint8_t chassis_angular_velocity;//chassis_angular_velocity,but use pwm_duty_cycle to inform
ChassisMotionState chassis_current_state;//chassis_state,inform the chassis_state_machine
uint8_t chassis_a; 
uint8_t chassis_stopping_count;
uint8_t chassis_target_v;

void chassis_init()
{
    // chassis_translational_velocity=0;
    // chassis_angular_velocity=0;
    chassis_velocity=0;
    chassis_a=0;
    chassis_stopping_count=0;
    chassis_target_v=0;
    chassis_current_state=CHASSIS_STATE_STOPPED; 
    motor_init(&MOTOR1_IN1_HTIM,MOTOR1_IN1_CHANNEL,&MOTOR1_IN2_HTIM,MOTOR1_IN2_CHANNEL);
    motor_init(&MOTOR2_IN1_HTIM,MOTOR2_IN1_CHANNEL,&MOTOR2_IN2_HTIM,MOTOR2_IN2_CHANNEL);
    motor_init(&MOTOR3_IN1_HTIM,MOTOR3_IN1_CHANNEL,&MOTOR3_IN2_HTIM,MOTOR3_IN2_CHANNEL);
    motor_init(&MOTOR4_IN1_HTIM,MOTOR4_IN1_CHANNEL,&MOTOR4_IN2_HTIM,MOTOR4_IN2_CHANNEL);      
}

void chassis_change_state(ChassisMotionState targetstate,uint8_t target_v)
{
    if(targetstate!=chassis_current_state)
    {
        if(chassis_current_state!=CHASSIS_STATE_STOPPING&&chassis_current_state!=CHASSIS_STATE_STOPPED)
        {
            chassis_stopping_count=STOPPING_COUNT_NUM;
            chassis_current_state=CHASSIS_STATE_STOPPING;
            chassis_velocity=0;
            chassis_a=0;
        }
        else if(chassis_current_state==CHASSIS_STATE_STOPPING)
        {
            if(chassis_stopping_count>0)
            {
                chassis_stopping_count--;
            }
            else
            {
                chassis_current_state=targetstate;
                chassis_target_v=target_v;
                chassis_a=1;
            }
        }
        else
        {
            chassis_current_state=targetstate;
            chassis_target_v=target_v;
            chassis_velocity=0;
            chassis_a=1;            
        }
    }
    else 
    {
        if(chassis_a+chassis_velocity<=chassis_target_v&&chassis_current_state!=CHASSIS_STATE_STOPPED)
            chassis_velocity=chassis_velocity+chassis_a;    
    }
}

void chassis_motion()
{
    switch (chassis_current_state) {
        case CHASSIS_STATE_STOPPING:
            chassis_stopping();
            break;
        case CHASSIS_STATE_STOPPED:
            chassis_stopped();
            break;
        case CHASSIS_STATE_BACK:
            chassis_backmove(chassis_velocity);
            break;
        case CHASSIS_STATE_FORWARD:
            chassis_fowardmove(chassis_velocity);
            break;
        case CHASSIS_STATE_LEFT:
            chassis_leftmove(chassis_velocity);
            break;
        case CHASSIS_STATE_RIGHT:
            chassis_rightmove(chassis_velocity);
            break;
        case CHASSIS_STATE_ROTATE_CW:
            chassis_rotate_cw(chassis_velocity);
            break;
        case CHASSIS_STATE_ROTATE_CCW:
            chassis_rotate_ccw(chassis_velocity);
            break;
    }
}

void chassis_stopped()
{
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPED, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPED, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPED, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPED, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);
}
void chassis_stopping()
{
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPING, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPING, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPING, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPING, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);
}

void chassis_fowardmove(uint16_t v)
{
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);
}

void chassis_backmove(uint16_t v)
{
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);     
}
 
void chassis_leftmove(uint16_t v)
{
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);
}
 
void chassis_rightmove(uint16_t v)
{
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);
}

void chassis_rotate_cw(uint16_t v)
{
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);
}

void chassis_rotate_ccw(uint16_t v)
{
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);
}

#include "chassis.h"
#include "motor.h"
#include <stdint.h>

// Global variables for chassis state management
uint8_t chassis_velocity;                    // Current velocity of the chassis (0-99)
// extern uint8_t chassis_translational_velocity;//chassis_translational_velocity,but use pwm_duty_cycle to inform
// extern uint8_t chassis_angular_velocity;//chassis_angular_velocity,but use pwm_duty_cycle to inform
ChassisMotionState chassis_current_state;    // Current state of the chassis movement
uint8_t chassis_a;                           // Acceleration value for smooth transitions
uint8_t chassis_stopping_count;              // Counter for stopping state transitions
uint8_t chassis_target_v;                    // Target velocity for the chassis

/**
 * @brief Initialize the chassis system
 * 
 * Sets all chassis parameters to zero, stops all motors, and initializes motor PWM channels
 */
void chassis_init()
{
    // chassis_translational_velocity=0;
    // chassis_angular_velocity=0;
    chassis_velocity=0;                     // Start with zero velocity
    chassis_a=0;                            // Start with zero acceleration
    chassis_stopping_count=0;               // Reset stopping counter
    chassis_target_v=0;                     // Set target velocity to zero
    chassis_current_state=CHASSIS_STATE_STOPPED; // Set initial state to stopped
    
    // Initialize all four motors with their respective timer handles and channels
    motor_init(&MOTOR1_IN1_HTIM,MOTOR1_IN1_CHANNEL,&MOTOR1_IN2_HTIM,MOTOR1_IN2_CHANNEL);
    motor_init(&MOTOR2_IN1_HTIM,MOTOR2_IN1_CHANNEL,&MOTOR2_IN2_HTIM,MOTOR2_IN2_CHANNEL);
    motor_init(&MOTOR3_IN1_HTIM,MOTOR3_IN1_CHANNEL,&MOTOR3_IN2_HTIM,MOTOR3_IN2_CHANNEL);
    motor_init(&MOTOR4_IN1_HTIM,MOTOR4_IN1_CHANNEL,&MOTOR4_IN2_HTIM,MOTOR4_IN2_CHANNEL);
}

/**
 * @brief Change the chassis state with a target velocity
 * 
 * Handles state transitions with smooth stopping, ensuring the chassis stops briefly 
 * before changing to a new movement state
 * 
 * @param targetstate Target movement state to change to
 * @param target_v Target velocity for the new state
 */
void chassis_change_state(ChassisMotionState targetstate,uint8_t target_v)
{
    if(targetstate!=chassis_current_state) // Check if state is changing
    {
        if(chassis_current_state!=CHASSIS_STATE_STOPPING) // If not already stopping, start stopping
        {
            chassis_stopping_count=STOPPING_COUNT_NUM; // Set stopping count
            chassis_current_state=CHASSIS_STATE_STOPPING; // Change to stopping state
            chassis_velocity=0; // Stop current movement
            chassis_a=0; // Reset acceleration
        }
        else if(chassis_current_state==CHASSIS_STATE_STOPPING) // If already in stopping state
        {
            if(chassis_stopping_count>0) // Decrement the stopping counter
            {
                chassis_stopping_count--;
            }
            else // When counter reaches zero, transition to target state
            {
                chassis_current_state=targetstate; // Change to the target state
                chassis_a=1; // Set acceleration to start accelerating
            }
        }
    }
    else // If staying in same state, gradually increase velocity up to target
    {
        if(chassis_a+chassis_velocity<=chassis_target_v&&chassis_current_state!=CHASSIS_STATE_STOPPED)
            chassis_velocity+=chassis_a; // Increment velocity by acceleration
    }
}

/**
 * @brief Execute movement based on current chassis state
 * 
 * Calls the appropriate movement function based on the current chassis state
 */
void chassis_motion()
{
    switch (chassis_current_state) {
        case CHASSIS_STATE_STOPPING: // Execute stopping motion
            chassis_stopping();
            break;
        case CHASSIS_STATE_STOPPED: // Execute stopped motion (all motors off)
            chassis_stopped();
            break;
        case CHASSIS_STATE_BACK: // Move backwards
            chassis_backmove(chassis_velocity);
            break;
        case CHASSIS_STATE_FORWARD: // Move forwards
            chassis_fowardmove(chassis_velocity);
            break;
        case CHASSIS_STATE_LEFT: // Move left (strafe)
            chassis_leftmove(chassis_velocity);
            break;
        case CHASSIS_STATE_RIGHT: // Move right (strafe)
            chassis_rightmove(chassis_velocity);
            break;
        case CHASSIS_STATE_ROTATE_CW: // Rotate clockwise
            chassis_rotate_cw(chassis_velocity);
            break;
        case CHASSIS_STATE_ROTATE_CCW: // Rotate counter-clockwise
            chassis_rotate_ccw(chassis_velocity);
            break;
    }
}

/**
 * @brief Stop all motors (immediate stop)
 * 
 * Sets all motors to STOPPED state with minimum PWM value
 */
void chassis_stopped()
{
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPED, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPED, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPED, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPED, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);
}

/**
 * @brief Bring all motors to gradual stop
 * 
 * Sets all motors to STOPPING state for smooth transition
 */
void chassis_stopping()
{
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPING, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPING, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPING, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(PWM_MIN_SET_NUM,MOTOR_DIR_STOPPING, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);
}

/**
 * @brief Move chassis forward
 * 
 * Sets motors to move the chassis in forward direction at specified velocity
 * Motor1 and Motor4 rotate forward, Motor2 and Motor3 rotate in reverse
 * 
 * @param v Velocity at which to move (0-99)
 */
void chassis_fowardmove(uint16_t v)
{
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);
}

/**
 * @brief Move chassis backward
 * 
 * Sets motors to move the chassis in backward direction at specified velocity
 * Motor1 and Motor4 rotate in reverse, Motor2 and Motor3 rotate forward
 * 
 * @param v Velocity at which to move (0-99)
 */
void chassis_backmove(uint16_t v)
{
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);
}

/**
 * @brief Move chassis left (strafe)
 * 
 * Sets motors to move the chassis left at specified velocity
 * Motor1 and Motor2 rotate in reverse, Motor3 and Motor4 rotate forward
 * 
 * @param v Velocity at which to move (0-99)
 */
void chassis_leftmove(uint16_t v)
{
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);
}

/**
 * @brief Move chassis right (strafe)
 * 
 * Sets motors to move the chassis right at specified velocity
 * Motor1 and Motor2 rotate forward, Motor3 and Motor4 rotate in reverse
 * 
 * @param v Velocity at which to move (0-99)
 */
void chassis_rightmove(uint16_t v)
{
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);
}

/**
 * @brief Rotate chassis clockwise
 * 
 * Sets all motors to rotate in the same direction to achieve clockwise rotation
 * 
 * @param v Velocity at which to rotate (0-99)
 */
void chassis_rotate_cw(uint16_t v)
{
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_FORWARD, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);
}

/**
 * @brief Rotate chassis counter-clockwise
 * 
 * Sets all motors to rotate in the reverse direction to achieve counter-clockwise rotation
 * 
 * @param v Velocity at which to rotate (0-99)
 */
void chassis_rotate_ccw(uint16_t v)
{
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR1_IN1_HTIM, MOTOR1_IN1_CHANNEL, &MOTOR1_IN2_HTIM, MOTOR1_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR2_IN1_HTIM, MOTOR2_IN1_CHANNEL, &MOTOR2_IN2_HTIM, MOTOR2_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR3_IN1_HTIM, MOTOR3_IN1_CHANNEL, &MOTOR3_IN2_HTIM, MOTOR3_IN2_CHANNEL);
    motor_set_state(v,MOTOR_DIR_REVERSE, &MOTOR4_IN1_HTIM, MOTOR4_IN1_CHANNEL, &MOTOR4_IN2_HTIM, MOTOR4_IN2_CHANNEL);
}
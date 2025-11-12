#ifndef CHASSIS_H
#define CHASSIS_H

#include "motor.h"
#include "tim.h"
#include <stdint.h>

// Motor direction definitions (1 = forward, -1 = reverse)
#define motor1_d 1//motor1_direction
#define motor2_d 1//motor2_direction
#define motor3_d 1//motor3_direction
#define motor4_d 1//motor4_direction

// Motor input channel definitions - these connect to timer channels for PWM control
#define MOTOR1_IN1_HTIM htim4              // Handle for timer used for motor1 input 1
#define MOTOR1_IN1_CHANNEL TIM_CHANNEL_1   // Timer channel for motor1 input 1
#define MOTOR2_IN1_HTIM htim4              // Handle for timer used for motor2 input 1
#define MOTOR2_IN1_CHANNEL TIM_CHANNEL_2   // Timer channel for motor2 input 1
#define MOTOR3_IN1_HTIM htim4              // Handle for timer used for motor3 input 1
#define MOTOR3_IN1_CHANNEL TIM_CHANNEL_3   // Timer channel for motor3 input 1
#define MOTOR4_IN1_HTIM htim4              // Handle for timer used for motor4 input 1
#define MOTOR4_IN1_CHANNEL TIM_CHANNEL_4   // Timer channel for motor4 input 1
#define MOTOR1_IN2_HTIM htim3              // Handle for timer used for motor1 input 2
#define MOTOR1_IN2_CHANNEL TIM_CHANNEL_1   // Timer channel for motor1 input 2
#define MOTOR2_IN2_HTIM htim3              // Handle for timer used for motor2 input 2
#define MOTOR2_IN2_CHANNEL TIM_CHANNEL_2   // Timer channel for motor2 input 2
#define MOTOR3_IN2_HTIM htim3              // Handle for timer used for motor3 input 2
#define MOTOR3_IN2_CHANNEL TIM_CHANNEL_3   // Timer channel for motor3 input 2
#define MOTOR4_IN2_HTIM htim3              // Handle for timer used for motor4 input 2
#define MOTOR4_IN2_CHANNEL TIM_CHANNEL_4   // Timer channel for motor4 input 2

// Speed and state definitions
#define MAX_SPEED 99                       // Maximum speed value for motors (PWM duty cycle)
#define MIN_SPEED 0                        // Minimum speed value for motors (PWM duty cycle)
#define STOPPING_COUNT_NUM 4               // Number of cycles to wait when transitioning between states
#define ROTATE_MODE_ON 1                   // Flag value for enabling rotate mode
#define ROTATE_MODE_OFF 0                  // Flag value for disabling rotate mode

// Enumeration for chassis motion states
typedef enum {
    CHASSIS_STATE_STOPPED,     // Chassis is completely stopped
    CHASSIS_STATE_STOPPING,    // Chassis is in the process of stopping (transition state)
    CHASSIS_STATE_FORWARD,     // Chassis is moving forward
    CHASSIS_STATE_BACK,        // Chassis is moving backward
    CHASSIS_STATE_LEFT,        // Chassis is moving left (strafing)
    CHASSIS_STATE_RIGHT,       // Chassis is moving right (strafing)
    CHASSIS_STATE_ROTATE_CW,   // Chassis is rotating clockwise
    CHASSIS_STATE_ROTATE_CCW   // Chassis is rotating counter-clockwise
}ChassisMotionState;

// Global variables for chassis state management
extern uint8_t chassis_velocity;              // Current velocity of the chassis (0-99)
extern ChassisMotionState chassis_current_state; // Current state of the chassis movement
extern uint8_t chassis_a;                     // Acceleration value for smooth transitions
extern uint8_t chassis_stopping_count;        // Counter for stopping state transitions
extern uint8_t chassis_target_v;              // Target velocity for the chassis

// Function prototypes for chassis control
void chassis_init();                          // Initialize chassis - start PWM for all motors and set to 0
void chassis_motion();                        // Execute movement based on current chassis state
void chassis_stopped();                       // Set all motors to stopped state
void chassis_stopping();                      // Set all motors to stopping state (for smooth transitions)
void chassis_fowardmove(uint16_t v);          // Move chassis forward at specified velocity
void chassis_backmove(uint16_t v);            // Move chassis backward at specified velocity
void chassis_leftmove(uint16_t v);            // Move chassis left (strafe) at specified velocity
void chassis_rightmove(uint16_t v);           // Move chassis right (strafe) at specified velocity
void chassis_rotate_cw(uint16_t v);           // Rotate chassis clockwise at specified velocity
void chassis_rotate_ccw(uint16_t v);          // Rotate chassis counter-clockwise at specified velocity
void chassis_change_state(ChassisMotionState targetstate,uint8_t target_v); // Change chassis state with target velocity

#endif /* CHASSIS_H */
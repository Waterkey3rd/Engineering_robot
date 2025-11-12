#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "usart.h"
#include "dma.h"
#include <stdint.h>
#include <sys/types.h>

// Enumeration for UART communication states
typedef enum{
    COMM_BLOCKED,     // Communication channel is blocked/not active
    COMM_SENDING,     // Currently sending data
    COMM_RECEIVING,   // Currently receiving data
    COMM_COMPLETE     // Communication operation is complete
}Uart_State;

// Protocol packet structure for communication with remote controller
// All fields are bit-packed to fit in 8 bytes exactly
typedef struct {
    uint8_t build_start_button :1 ;  // 1bit - Start construction flag
    uint8_t catch_button : 1;        // 1bit - Catch mechanism control (collect/tighten)
    uint8_t arm_unflod_button : 1;   // 1bit - Arm unfold/fold control
    uint8_t arm_storage_button : 1;  // 1bit - Switch arm to storage state
    uint8_t rotate_mode : 1;         // 1bit - Flag to enable rotation mode
    uint8_t reserved : 3;            // 3bit - Padding bits (unused)
    signed char chassis_x :8;        // 8bit - Chassis X-axis movement (-128 to 127)
    signed char chassis_y :8;        // 8bit - Chassis Y-axis movement (-128 to 127)
    signed char PTZ_x :8;            // 8bit - Pan-Tilt-Zoom X-axis control (-128 to 127)
    signed char PTZ_y :8;            // 8bit - Pan-Tilt-Zoom Y-axis control (-128 to 127)
} __attribute__((packed)) ProtocolPacket;

// UART handle definitions
#define HC05_HUART huart3              // Handle for HC-05 Bluetooth module UART
#define MUC_HUART  huart2              // Handle for microcontroller UART
#define DELAY_TIME 20                  // Delay time in milliseconds between operations
#define PACKET_LENGTH 8                // Fixed length of communication packets in bytes
#define HC05_MESSAGE_SIZE PACKET_LENGTH*sizeof(uint8_t)  // Size of HC-05 message in bytes
#define MUC_MESSAGE_SIZE PACKET_LENGTH*sizeof(uint8_t)   // Size of MUC message in bytes

// Global variables for communication system
extern signed char uart_receivemessage[PACKET_LENGTH];  // Buffer for received UART messages
extern char uart_transmitmessage[PACKET_LENGTH];        // Buffer for transmitted UART messages
extern ProtocolPacket hc05_packet;                      // Parsed protocol packet from HC-05
extern Uart_State hc05_state;                          // State of HC-05 communication channel
extern Uart_State muc_state;                           // State of microcontroller communication channel

// Function prototypes for communication system
void communication_init(void);                               // Initialize communication system
void communication_start(void);                              // Start communication (begin receiving)
void parse_hc05_protocol_data(signed char* message,uint16_t message_l);  // Parse HC-05 protocol data
void send_mcn_data(signed char* message,uint16_t message_l); // Send data through microcontroller UART
uint8_t read_message(signed char* message,uint16_t message_l); // Read a message from buffer
void protocolpacket_init();                                  // Initialize protocol packet structure
void comm_change_chassis();                                  // Process communication data to control chassis

#endif /* COMMUNICATION_H */
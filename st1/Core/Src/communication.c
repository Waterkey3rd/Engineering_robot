#include "communication.h"
#include "chassis.h"
#include "stm32f1xx_hal_uart.h"
#include "stdlib.h"
#include <stdint.h>
#include <sys/_intsup.h>

// External UART and DMA handles for communication peripherals
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
uint8_t checksum;  // Variable to store calculated checksum for data validation

// Global communication buffers and state variables
signed char uart_receivemessage[PACKET_LENGTH];  // Buffer for received messages
char uart_transmitmessage[PACKET_LENGTH];        // Buffer for transmitted messages
ProtocolPacket hc05_packet;                      // Parsed protocol packet from HC-05
Uart_State hc05_state;                          // State of HC-05 communication channel
Uart_State muc_state;                           // State of microcontroller communication channel

/**
 * @brief Initialize the communication system
 * 
 * Sets up communication states and initializes the protocol packet structure
 */
void communication_init()
{
    hc05_state=COMM_BLOCKED;  // Initially block HC-05 communication
    muc_state=COMM_BLOCKED;   // Initially block microcontroller communication
    protocolpacket_init();    // Initialize protocol packet with default values
}

/**
 * @brief Initialize the protocol packet structure with default values
 * 
 * Sets all fields in the HC-05 protocol packet to zero/neutral values
 */
void protocolpacket_init()
{
  hc05_packet.build_start_button=0;      // Start construction flag: OFF
  hc05_packet.catch_button=0;            // Catch mechanism: OFF
  hc05_packet.arm_unflod_button=0;       // Arm unfold/fold: OFF
  hc05_packet.arm_storage_button=0;      // Arm storage state: OFF
  hc05_packet.rotate_mode=0;             // Rotation mode: OFF
  hc05_packet.reserved=0;                // Reserved bits: 0
  hc05_packet.chassis_x=0;               // Chassis X-axis: neutral
  hc05_packet.chassis_y=0;               // Chassis Y-axis: neutral
  hc05_packet.PTZ_x=0;                   // PTZ X-axis: neutral
  hc05_packet.PTZ_y=0;                   // PTZ Y-axis: neutral
}

/**
 * @brief Start the communication system
 * 
 * Begins receiving data via DMA from the HC-05 Bluetooth module
 */
void communication_start()
{
  hc05_state=COMM_RECEIVING;  // Set HC-05 state to receiving
  HAL_UART_Receive_DMA(&HC05_HUART, (uint8_t*)uart_receivemessage, HC05_MESSAGE_SIZE);  // Start DMA receive
}

/**
 * @brief Parse and process HC-05 protocol data
 * 
 * When communication is complete, reads message from buffer, validates it,
 * and transmits it to the microcontroller if valid
 * 
 * @param message Pointer to the message buffer
 * @param message_l Length of the message
 */
void parse_hc05_protocol_data(signed char* message,uint16_t message_l)
{
  if(hc05_state==COMM_COMPLETE)  // Check if HC-05 has completed receiving
  {
    if(read_message(message, message_l))  // Validate received message
    {
      if(muc_state==COMM_BLOCKED)  // Only transmit if microcontroller is ready
      {
        HAL_UART_Transmit_DMA(&MUC_HUART, (uint8_t*)message, MUC_MESSAGE_SIZE);  // Send message via DMA
        muc_state=COMM_SENDING;  // Update microcontroller state to sending
      }
      hc05_state=COMM_BLOCKED;  // Reset HC-05 state to blocked
    }
  }
}

/**
 * @brief Process communication data to control chassis movement
 * 
 * Interprets the received protocol packet and sets chassis movement accordingly
 * Supports both regular movement mode and rotation mode
 */
void comm_change_chassis()
{
  if(hc05_packet.rotate_mode==ROTATE_MODE_ON)  // If rotation mode is enabled
    {
      if(hc05_packet.chassis_x==0)  // If no X-axis movement requested
      {
        // chassis_target_v=0;
        // chassis_current_state=CHASSIS_STATE_STOPPED;
        chassis_change_state(CHASSIS_STATE_STOPPED, 0);  // Stop the chassis
      }
      else if(hc05_packet.chassis_x>0)  // If positive X-axis movement (rotate clockwise)
      {
        // chassis_target_v=MAX_SPEED<hc05_packet.chassis_x?MAX_SPEED:hc05_packet.chassis_x;
        // chassis_current_state=CHASSIS_STATE_ROTATE_CW;
        chassis_change_state(CHASSIS_STATE_ROTATE_CW, MAX_SPEED<hc05_packet.chassis_x?MAX_SPEED:hc05_packet.chassis_x);
      }
      else  // If negative X-axis movement (rotate counter-clockwise)
      {
        // chassis_target_v=MAX_SPEED<(-hc05_packet.chassis_x)?MAX_SPEED:(-hc05_packet.chassis_x);
        // chassis_current_state=CHASSIS_STATE_ROTATE_CCW;
        chassis_change_state(CHASSIS_STATE_ROTATE_CCW, MAX_SPEED<(-hc05_packet.chassis_x)?MAX_SPEED:(-hc05_packet.chassis_x));
      }
    }
  else  // Regular movement mode (not rotation mode)
    {
      if(hc05_packet.chassis_x==0&&hc05_packet.chassis_y==0)  // If no movement requested
      {
        // chassis_target_v=0;
        // chassis_current_state=CHASSIS_STATE_STOPPED;
        chassis_change_state(CHASSIS_STATE_STOPPED, 0);  // Stop the chassis
      }
      else  // Movement in some direction
      {
        if(abs(hc05_packet.chassis_x)>=abs(hc05_packet.chassis_y))  // X-axis movement dominates
        {
          if(hc05_packet.chassis_x>0)  // Positive X-axis: forward/backward
          {
            // chassis_target_v=MAX_SPEED<hc05_packet.chassis_x?MAX_SPEED:hc05_packet.chassis_x;
            // chassis_current_state=CHASSIS_STATE_FORWARD;
            chassis_change_state(CHASSIS_STATE_FORWARD, MAX_SPEED<hc05_packet.chassis_x?MAX_SPEED:hc05_packet.chassis_x);
          }
          else  // Negative X-axis: forward/backward
          {
            // chassis_target_v=MAX_SPEED<(-hc05_packet.chassis_x)?MAX_SPEED:(-hc05_packet.chassis_x);
            // chassis_current_state=CHASSIS_STATE_BACK;
            chassis_change_state(CHASSIS_STATE_BACK, MAX_SPEED<(-hc05_packet.chassis_x)?MAX_SPEED:(-hc05_packet.chassis_x));
          }
        }
        else  // Y-axis movement dominates
        {
          if(hc05_packet.chassis_y>0)  // Positive Y-axis: right/left strafe
          {
            // chassis_target_v=MAX_SPEED<hc05_packet.chassis_y?MAX_SPEED:hc05_packet.chassis_y;
            // chassis_current_state=CHASSIS_STATE_RIGHT;
            chassis_change_state(CHASSIS_STATE_RIGHT, MAX_SPEED<hc05_packet.chassis_y?MAX_SPEED:hc05_packet.chassis_y);
          }
          else  // Negative Y-axis: right/left strafe
          {
            // chassis_target_v=MAX_SPEED<(-hc05_packet.chassis_y)?MAX_SPEED:(-hc05_packet.chassis_y);
            // chassis_current_state=CHASSIS_STATE_LEFT;
            chassis_change_state(CHASSIS_STATE_LEFT, MAX_SPEED<(-hc05_packet.chassis_y)?MAX_SPEED:(-hc05_packet.chassis_y));
          }
        }
      }

    }
}

/**
 * @brief Send data through microcontroller UART and prepare for next reception
 * 
 * When transmission is complete, starts receiving data from HC-05 again
 * 
 * @param message Pointer to the message buffer
 * @param message_l Length of the message
 */
void send_mcn_data(signed char* message,uint16_t message_l)//其实发送完继续接受
{
  if(muc_state==COMM_COMPLETE)  // Check if microcontroller transmission is complete
  {
    if(hc05_state==COMM_BLOCKED)  // Only start receiving if HC-05 is ready
    {
      HAL_UART_Receive_DMA(&HC05_HUART, (uint8_t*)uart_receivemessage, HC05_MESSAGE_SIZE);  // Start DMA receive
      hc05_state=COMM_RECEIVING;  // Update HC-05 state to receiving
    }
    muc_state=COMM_BLOCKED;  // Reset microcontroller state to blocked
  }
}

/**
 * @brief Read and validate a message from the buffer
 * 
 * Checks message framing (0xA5 start and 0x5A end), calculates and verifies checksum,
 * then parses the message into the protocol packet structure
 * 
 * @param message Pointer to the message buffer
 * @param message_l Length of the message
 * @return uint8_t 1 if message is valid, 0 otherwise
 */
uint8_t read_message(signed char* message,uint16_t message_l)
{
  if((uint8_t)message[0]==0xA5&&(uint8_t)message[message_l-1]==0x5A)  // Check message framing
  {
    checksum = 0;
    for(uint8_t i=1;i<message_l-2;i++)  // Calculate checksum from data bytes
      checksum+=message[i];
    if(checksum==message[message_l-2])//协议校验无误  // Verify checksum
    {
      // Parse bit-packed fields from message bytes
      hc05_packet.build_start_button=(message[1]>>0) & 0x01;  // Extract build start button
      hc05_packet.catch_button=(message[1]>>1) & 0x01;        // Extract catch button
      hc05_packet.arm_unflod_button=(message[1]>>2) & 0x01;   // Extract arm unfold button
      hc05_packet.arm_storage_button=(message[1]>>3) & 0x01;  // Extract arm storage button
      hc05_packet.rotate_mode=(message[1]>>4) & 0x01;         // Extract rotate mode flag
      hc05_packet.reserved=(message[1]>>5) & 0x07;            // Extract reserved bits
      // Parse signed char fields
      hc05_packet.chassis_x=message[2];  // X-axis movement value
      hc05_packet.chassis_y=message[3];  // Y-axis movement value
      hc05_packet.PTZ_x=message[4];      // PTZ X-axis value
      hc05_packet.PTZ_y=message[5];      // PTZ Y-axis value
      return 1;  // Return success
    }
  }
  return 0;  // Return failure if message is invalid
}

/**
 * @brief UART receive complete callback for HC-05
 * 
 * Called when DMA reception from HC-05 is complete, updates communication state
 * 
 * @param huart Pointer to the UART handle that triggered the callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart==&HC05_HUART)  // Check if callback is for HC-05 UART
  {
    if(hc05_state==COMM_RECEIVING){  // Only update state if currently receiving
      hc05_state=COMM_COMPLETE;  // Set HC-05 state to complete
    }
    // HAL_UART_Transmit_DMA(&huart2, recivedata, Size);
  }
}

/**
 * @brief UART transmit complete callback for microcontroller
 * 
 * Called when DMA transmission to microcontroller is complete, updates communication state
 * 
 * @param huart Pointer to the UART handle that triggered the callback
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart==&MUC_HUART)  // Check if callback is for microcontroller UART
  {
    if(muc_state==COMM_SENDING)  // Only update state if currently sending
    {
      muc_state=COMM_COMPLETE;  // Set microcontroller state to complete
    }
  }
}

// ButtonProtocol parse_protocol(uint8_t data) {
//     ButtonProtocol proto;
//     proto.build_start_button = (data >> 0) & 0x01;
//     proto.catch_button = (data >> 1) & 0x01;
//     proto.arm_unflod_button = (data >> 2) & 0x01;
//     proto.arm_storage_button = (data >> 3) & 0x01;
//     proto.rotate_mode = (data >> 4) & 0x01;
//     proto.reserved = (data >> 5) & 0x07;
//     return proto;
// }

// uint8_t pack_protocol(ButtonProtocol proto) {
//     uint8_t data = 0;
//     data |= (proto.build_start_button & 0x01) << 0;
//     data |= (proto.catch_button & 0x01) << 1;
//     data |= (proto.arm_unflod_button & 0x01) << 2;
//     data |= (proto.arm_storage_button & 0x01) << 3;
//     data |= (proto.rotate_mode & 0x01) << 4;
//     data |= (proto.reserved & 0x07) << 5;
//     return data;
// }
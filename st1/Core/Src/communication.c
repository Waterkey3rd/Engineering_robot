#include "communication.h"
#include "chassis.h"
#include "stm32f1xx_hal_uart.h"
#include "stdlib.h"
#include <stdint.h>
#include <sys/_intsup.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
uint8_t checksum;

signed char uart_receivemessage[PACKET_LENGTH];
char uart_transmitmessage[PACKET_LENGTH];
ProtocolPacket hc05_packet;
Uart_State hc05_state;
Uart_State muc_state;

void communication_init()
{
    hc05_state=COMM_BLOCKED;
    muc_state=COMM_BLOCKED;
    protocolpacket_init();
}

void protocolpacket_init()
{
  hc05_packet.build_start_button=0;
  hc05_packet.catch_button=0;
  hc05_packet.arm_unflod_button=0;
  hc05_packet.arm_storage_button=0;
  hc05_packet.rotate_mode=0;
  hc05_packet.reserved=0;
  hc05_packet.chassis_x=0;
  hc05_packet.chassis_y=0;
  hc05_packet.PTZ_x=0;
  hc05_packet.PTZ_y=0;
}

void communication_start()
{
  hc05_state=COMM_RECEIVING;
  HAL_UART_Receive_DMA(&HC05_HUART, (uint8_t*)uart_receivemessage, HC05_MESSAGE_SIZE);
}

void parse_hc05_protocol_data(signed char* message,uint16_t message_l)
{
  if(hc05_state==COMM_COMPLETE)
  {
    if(read_message(message, message_l))
    {
      if(muc_state==COMM_BLOCKED)
      {
        HAL_UART_Transmit_DMA(&MUC_HUART, (uint8_t*)message, MUC_MESSAGE_SIZE);
        muc_state=COMM_SENDING;
      }
      hc05_state=COMM_BLOCKED;
    }
  }
}

void comm_change_chassis()
{
  if(hc05_packet.rotate_mode==ROTATE_MODE_ON)
    {
      if(hc05_packet.chassis_x==0)
      {
        // chassis_target_v=0;
        // chassis_current_state=CHASSIS_STATE_STOPPED;
        chassis_change_state(CHASSIS_STATE_STOPPED, 0);
      }
      else if(hc05_packet.chassis_x>0)
      {
        // chassis_target_v=MAX_SPEED<hc05_packet.chassis_x?MAX_SPEED:hc05_packet.chassis_x;
        // chassis_current_state=CHASSIS_STATE_ROTATE_CW;
        chassis_change_state(CHASSIS_STATE_ROTATE_CW, MAX_SPEED<hc05_packet.chassis_x?MAX_SPEED:hc05_packet.chassis_x);
      }
      else 
      {
        // chassis_target_v=MAX_SPEED<(-hc05_packet.chassis_x)?MAX_SPEED:(-hc05_packet.chassis_x);
        // chassis_current_state=CHASSIS_STATE_ROTATE_CCW;  
        chassis_change_state(CHASSIS_STATE_ROTATE_CCW, MAX_SPEED<(-hc05_packet.chassis_x)?MAX_SPEED:(-hc05_packet.chassis_x));     
      }
    }
  else
    {
      if(hc05_packet.chassis_x==0&&hc05_packet.chassis_y==0)
      {
        // chassis_target_v=0;
        // chassis_current_state=CHASSIS_STATE_STOPPED;
        chassis_change_state(CHASSIS_STATE_STOPPED, 0);
      }
      else 
      {
        if(abs(hc05_packet.chassis_x)>=abs(hc05_packet.chassis_y))
        {
          if(hc05_packet.chassis_x>0)
          {
            // chassis_target_v=MAX_SPEED<hc05_packet.chassis_x?MAX_SPEED:hc05_packet.chassis_x;
            // chassis_current_state=CHASSIS_STATE_FORWARD; 
            chassis_change_state(CHASSIS_STATE_FORWARD, MAX_SPEED<hc05_packet.chassis_x?MAX_SPEED:hc05_packet.chassis_x);
          }
          else 
          {
            // chassis_target_v=MAX_SPEED<(-hc05_packet.chassis_x)?MAX_SPEED:(-hc05_packet.chassis_x);
            // chassis_current_state=CHASSIS_STATE_BACK; 
            chassis_change_state(CHASSIS_STATE_BACK, MAX_SPEED<(-hc05_packet.chassis_x)?MAX_SPEED:(-hc05_packet.chassis_x));
          }
        }
        else 
        {
          if(hc05_packet.chassis_y>0)
          {
            // chassis_target_v=MAX_SPEED<hc05_packet.chassis_y?MAX_SPEED:hc05_packet.chassis_y;
            // chassis_current_state=CHASSIS_STATE_RIGHT; 
            chassis_change_state(CHASSIS_STATE_RIGHT, MAX_SPEED<hc05_packet.chassis_y?MAX_SPEED:hc05_packet.chassis_y);
          }
          else 
          {
            // chassis_target_v=MAX_SPEED<(-hc05_packet.chassis_y)?MAX_SPEED:(-hc05_packet.chassis_y);
            // chassis_current_state=CHASSIS_STATE_LEFT; 
            chassis_change_state(CHASSIS_STATE_LEFT, MAX_SPEED<(-hc05_packet.chassis_y)?MAX_SPEED:(-hc05_packet.chassis_y));
          }          
        }        
      }

    }
}

void send_mcn_data(signed char* message,uint16_t message_l)//其实是发送完继续接受
{
  if(muc_state==COMM_COMPLETE)
  {
    if(hc05_state==COMM_BLOCKED)
    {
      HAL_UART_Receive_DMA(&HC05_HUART, (uint8_t*)uart_receivemessage, HC05_MESSAGE_SIZE);
      hc05_state=COMM_RECEIVING;
    }
    muc_state=COMM_BLOCKED;
  }
}

uint8_t read_message(signed char* message,uint16_t message_l)
{
  if((uint8_t)message[0]==0xA5&&(uint8_t)message[message_l-1]==0x5A)
  {
    checksum = 0;
    for(uint8_t i=1;i<message_l-2;i++)
      checksum+=message[i];
    if(checksum==message[message_l-2])//协议校验无误
    {
      hc05_packet.build_start_button=(message[1]>>0) & 0x01;
      hc05_packet.catch_button=(message[1]>>1) & 0x01;
      hc05_packet.arm_unflod_button=(message[1]>>2) & 0x01;
      hc05_packet.arm_storage_button=(message[1]>>3) & 0x01;
      hc05_packet.rotate_mode=(message[1]>>4) & 0x01;
      hc05_packet.reserved=(message[1]>>5) & 0x07;
      hc05_packet.chassis_x=message[2];
      hc05_packet.chassis_y=message[3];
      hc05_packet.PTZ_x=message[4];
      hc05_packet.PTZ_y=message[5];
      return 1;
    }
  }
  return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart==&HC05_HUART)
  {
    if(hc05_state==COMM_RECEIVING){
      hc05_state=COMM_COMPLETE;
    }
    // HAL_UART_Transmit_DMA(&huart2, recivedata, Size); 
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart==&MUC_HUART)
  {
    if(muc_state==COMM_SENDING)
    {
      muc_state=COMM_COMPLETE;
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
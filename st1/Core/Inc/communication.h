#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "usart.h"
#include "dma.h"
#include <stdint.h>
#include <sys/types.h>

typedef enum{
    COMM_BLOCKED,
    COMM_SENDING,
    COMM_RECEIVING,
    COMM_COMPLETE
}Uart_State;

typedef struct {
    uint8_t build_start_button :1 ;  // 1bit - 构造是否开始
    uint8_t catch_button : 1;        // 1bit - 抓取装置收集收紧
    uint8_t arm_unflod_button : 1;   // 1bit - 机械臂的展开与收拢
    uint8_t arm_storage_button : 1;  // 1bit - 机械臂切换到存储状态
    uint8_t rotate_mode : 1;         // 1bit - 标识是否进入旋转模式
    uint8_t reserved : 3;            // 3bit - 填充位
    signed char chassis_x :8;            //   
    signed char chassis_y :8;            //
    signed char PTZ_x :8;                //
    signed char PTZ_y :8;                //
} __attribute__((packed)) ProtocolPacket;

#define HC05_HUART huart3
#define MUC_HUART  huart2
#define DELAY_TIME 20
#define PACKET_LENGTH 8
#define HC05_MESSAGE_SIZE PACKET_LENGTH*sizeof(uint8_t)
#define MUC_MESSAGE_SIZE PACKET_LENGTH*sizeof(uint8_t)

extern signed char uart_receivemessage[PACKET_LENGTH];
extern char uart_transmitmessage[PACKET_LENGTH];
extern ProtocolPacket hc05_packet;
extern Uart_State hc05_state;
extern Uart_State muc_state;

void communication_init(void);//init
void communication_start(void);
void parse_hc05_protocol_data(signed char* message,uint16_t message_l);
void send_mcn_data(signed char* message,uint16_t message_l);
uint8_t read_message(signed char* message,uint16_t message_l);
void protocolpacket_init();
void comm_change_chassis();

#endif /* COMMUNICATION_H */
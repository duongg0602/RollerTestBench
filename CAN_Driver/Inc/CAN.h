/*
 * CAN.h
 *
 *  Created on: 11 thg 8, 2026
 *      Author: ASUS_PC
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_
#include "main.h"
#include "global.h"

#define CAN_RX_BUFFER_SIZE 32
#define CAN_TX_BUFFER_SIZE 32

typedef struct
{
    uint32_t ID;
    uint8_t DLC;
    uint8_t Data[8];
} CAN_Frame;

typedef struct
{
    volatile uint16_t cnt;
    CAN_Frame frame;
} CAN_SharedData;

typedef struct {
	CAN_SharedData slot[CAN_RX_BUFFER_SIZE];
	volatile uint32_t head;
	volatile uint32_t tail;
}CAN_RX_Buffer;

typedef struct {
	CAN_SharedData slot[CAN_RX_BUFFER_SIZE];
	volatile uint32_t head;
	volatile uint32_t tail;
}CAN_TX_Buffer;

extern CAN_RX_Buffer CAN_RX_BUFFER;
extern CAN_TX_Buffer CAN_TX_BUFFER;
extern CAN_SharedData RX_BUFFER;
extern CAN_SharedData TX_BUFFER;
extern uint8_t CAN_rx_data[64];
extern CAN_RxHeaderTypeDef CAN_RX;
extern CAN_TxHeaderTypeDef CAN_TX;
extern CAN_HandleTypeDef hcan;

extern uint8_t TX_Data[64];
extern uint32_t RX_FIFO;
extern uint32_t TX_MailBox;
extern uint16_t Rx_count;
extern uint16_t Tx_count;
extern volatile uint8_t can_tx_state;

void CAN_Config(uint32_t* ID, uint8_t count);
uint8_t CAN_ReadRxMessage();
uint8_t CAN_Send(uint32_t ID, uint8_t len, uint8_t* data);
void CAN_Transmit();
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);


#endif /* INC_CAN_H_ */

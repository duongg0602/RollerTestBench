/*
 * CAN.c
 *
 *  Created on: 11 thg 8, 2026
 *      Author: ASUS_PC
 */
#include "CAN.h"

CAN_TX_Buffer CAN_TX_BUFFER;
CAN_RX_Buffer CAN_RX_BUFFER;
CAN_SharedData RX_BUFFER;
CAN_SharedData TX_BUFFER;
CAN_RxHeaderTypeDef CAN_RX;
CAN_TxHeaderTypeDef CAN_TX;
uint8_t CAN_rx_data[64];
uint8_t TX_Data[64];
CAN_HandleTypeDef hcan;
uint32_t RX_FIFO;
uint32_t TX_MailBox;
uint16_t Rx_count = 0;
uint16_t Tx_count = 0;

volatile uint16_t Tx_error = 0;
volatile uint32_t CAN_tx_ver = 0;

/* CAS lock: 0 = unlocked, 1 = locked */
static volatile uint32_t CAN_tx_lock = 0;
static volatile uint8_t CAN_tx_inflight = 0;
static volatile uint8_t CAN_tx_pending = 0;

static uint32_t wd_ver = 0;
static uint16_t wd_cnt = 0;

void CAN_Config(uint32_t* ID, uint8_t count){
	CAN_FilterTypeDef canfilter;
	canfilter.FilterActivation = CAN_FILTER_ENABLE;
	canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilter.FilterFIFOAssignment = CAN_RX_FIFO0;
	if(ID == NULL || count == 0){
		canfilter.FilterBank = 0;
		canfilter.FilterIdHigh = 0x0000;
		canfilter.FilterIdLow = 0x0000;
		canfilter.FilterMaskIdHigh = 0x0000;
		canfilter.FilterMaskIdLow = 0x0000;
		HAL_CAN_ConfigFilter(&hcan, &canfilter);
	}
	else{
		uint8_t bank;
		if(count > 14){
			count = 14;
		}
		for(bank = 0; bank < count; bank++){
			uint32_t id = ID[bank] & 0x7FF;
			canfilter.FilterBank = bank;
			canfilter.FilterIdHigh = (uint16_t)id << 5;
			canfilter.FilterIdLow = 0x0000;
			canfilter.FilterMaskIdHigh = 0x7FF << 5;
			canfilter.FilterMaskIdLow = 0x0006;
			HAL_CAN_ConfigFilter(&hcan, &canfilter);
		}
	}


	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupt when receive 1 byte
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);
}

uint8_t CAN_ReadRxMessage(){
	uint16_t pre_cnt = 0;
	uint16_t post_cnt = 0;

	uint32_t tail = CAN_RX_BUFFER.tail;

	if(tail == CAN_RX_BUFFER.head){
		return 0;
	}

	CAN_SharedData *slot = &CAN_RX_BUFFER.slot[tail % CAN_RX_BUFFER_SIZE];

    do
    {
        pre_cnt = slot->cnt;

        RX_BUFFER.frame = slot->frame;

        post_cnt = slot->cnt;

        if (pre_cnt == post_cnt)
        {
            CAN_RX_BUFFER.tail = tail + 1;
            return 1;
        }
    } while(pre_cnt != post_cnt);

    return 0;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcanx){
	if(HAL_CAN_GetRxMessage(hcanx, RX_FIFO, &CAN_RX, CAN_rx_data) == HAL_OK){
		Rx_count++;
	    uint32_t head = CAN_RX_BUFFER.head;
	    uint32_t tail = CAN_RX_BUFFER.tail;

	    if((head - tail) >= CAN_RX_BUFFER_SIZE)
	    {
	        return;
	    }

	    CAN_SharedData *slot = &CAN_RX_BUFFER.slot[head % CAN_RX_BUFFER_SIZE];

	    slot->frame.ID = CAN_RX.StdId;
	    slot->frame.DLC = CAN_RX.DLC;

		for(uint8_t i = 0; i < CAN_RX.DLC; i++){
			slot->frame.Data[i] = CAN_rx_data[i];
		}

		slot->cnt++;

		CAN_RX_BUFFER.head = head + 1;
	}
}

uint8_t CAN_Send(uint32_t ID, uint8_t len, uint8_t* data){

	uint32_t head = CAN_TX_BUFFER.head;
	uint32_t tail = CAN_TX_BUFFER.tail;

	CAN_SharedData *slot = &CAN_TX_BUFFER.slot[head % CAN_TX_BUFFER_SIZE];
	if(head - tail >= CAN_TX_BUFFER_SIZE){
		return 0;
	}
	slot->cnt++;
	slot->frame.ID = ID;
	slot->frame.DLC = len;
	for(uint8_t i = 0; i < len; i++){
		slot->frame.Data[i] = data[i];
	}

	CAN_TX_BUFFER.head = head + 1;
	return 1;
}

static inline uint8_t tx_ver_cas(uint32_t expected, uint32_t desired)
{
    do
    {
        if (__LDREXW(&CAN_tx_ver) != expected)
        {
            __CLREX();
            return 0;
        }
    } while (__STREXW(desired, &CAN_tx_ver) != 0);

    __DMB();
    return 1;
}

static void CAN_TxOwnerSend(uint32_t v){

	uint32_t head = CAN_TX_BUFFER.head;
	uint32_t tail = CAN_TX_BUFFER.tail;

	if(head == tail){
		tx_ver_cas(v, v + 1);
		return;
	}

	CAN_SharedData *slot = &CAN_TX_BUFFER.slot[tail % CAN_TX_BUFFER_SIZE];
	CAN_TX_BUFFER.tail = tail + 1;

	CAN_TX.StdId = slot->frame.ID;
	CAN_TX.DLC = slot->frame.DLC;
	CAN_TX.ExtId = 0; //standard CAN
	CAN_TX.IDE = CAN_ID_STD;
	CAN_TX.RTR = CAN_RTR_DATA;
	CAN_TX.TransmitGlobalTime = DISABLE;

	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0){
		return;
	}

	if(HAL_CAN_AddTxMessage(&hcan, &CAN_TX, slot->frame.Data, &TX_MailBox) == HAL_OK){
		(void)tx_ver_cas(v, v + 2);
	}
    else
    {
    	Tx_error++;
        (void)tx_ver_cas(v, v + 1);
    }
}

static void CAN_OnTxEvent(void)
{
    uint32_t v = CAN_tx_ver;

    if ((v & 1) == 0)
    {
        return;
    }
    CAN_TxOwnerSend(v);
}

void CAN_Transmit(void)
{
    uint32_t v = CAN_tx_ver;

    if ((v & 1) == 0)
    {
        wd_cnt = 0;
        if (tx_ver_cas(v, v + 1U))
        {
            CAN_TxOwnerSend(v + 1U);
        }
        return;
    }
    if (v != wd_ver)
    {
        wd_ver = v;
        wd_cnt = 0;
    }
    else if (++wd_cnt >= CAN_TX_TIMEOUT_TICKS)
    {
        wd_cnt = 0;

        HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 3)
        {
            if (tx_ver_cas(v, v + 1U))
            {
                return;
            }
        }
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcanx){
	Tx_count++;
	CAN_OnTxEvent();
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
	Tx_count++;
	CAN_OnTxEvent();
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
	Tx_count++;
	CAN_OnTxEvent();
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcanx) { (void)hcanx; Tx_error++; CAN_OnTxEvent(); }
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcanx) { (void)hcanx; Tx_error++; CAN_OnTxEvent(); }
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcanx) { (void)hcanx; Tx_error++; CAN_OnTxEvent(); }

/*
 * CAN.c
 *
 *  Created on: 11 thg 8, 2026
 *      Author: ASUS_PC
 */
#include "CAN.h"

GUI_Motor_Driver_Control_ID040 Message_ID040;
GUI_Generator_Driver_Control_ID080 Message_ID080;
ABS_Control_state_GUI_ID070 Message_ID070;
GainOffsetMotor_GUI_ID0c0 Message_ID0c0;
GainOffsetGenerator_GUI_ID100 Message_ID100;
GainOffsetBrake_GUI_ID200 Message_ID200;
SetPulseWheel_GUI_ID180 Message_ID180;
SetPulseRoller_GUI_ID1c0 Message_ID1C0;
Motor_Driver_Control_ID401 Message_ID401;
MotorControl_ID411 Message_ID411;
BrakePress_ID421 Message_ID421;
ABS_Control_ID431 Message_ID431;
Generator_Driver_Control_ID402 Message_ID402;
GeneratorControl_ID412 Message_ID412;
MotorTorque_ID403 Message_ID403;
GeneratorTorque_ID404 Message_ID404;
BrakeTorque_ID405 Message_ID405;
WheelSpeed_ID406 Message_ID406;
RollerSpeed_ID407 Message_ID407;

GUI_Motor_Driver_Control_ID040 TX_ID040;
GUI_Generator_Driver_Control_ID080 TX_ID080;
ABS_Control_state_GUI_ID070 TX_ID070;
GainOffsetMotor_GUI_ID0c0 TX_ID0c0;
GainOffsetGenerator_GUI_ID100 TX_ID100;
GainOffsetBrake_GUI_ID200 TX_ID200;
SetPulseWheel_GUI_ID180 TX_ID180;
SetPulseRoller_GUI_ID1c0 TX_ID1C0;
Motor_Driver_Control_ID401 TX_ID401;
MotorControl_ID411 TX_ID411;
BrakePress_ID421 TX_ID421;
ABS_Control_ID431 TX_ID431;
Generator_Driver_Control_ID402 TX_ID402;
GeneratorControl_ID412 TX_ID412;
MotorTorque_ID403 TX_ID403;
GeneratorTorque_ID404 TX_ID404;
BrakeTorque_ID405 TX_ID405;
WheelSpeed_ID406 TX_ID406;
RollerSpeed_ID407 TX_ID407;

CAN_RxHeaderTypeDef CAN_RX;
CAN_TxHeaderTypeDef CAN_TX;
uint8_t TX_Data[64];
CAN_HandleTypeDef hcan;
uint8_t CAN_rx_data[64] = {0};
uint32_t RX_FIFO;
uint32_t TX_MailBox;
uint16_t Rx_count = 0;
uint16_t Tx_count = 0;

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


void CAN_Transmit(uint32_t id, uint32_t len, uint8_t *data){
	CAN_TX.StdId = id;
	CAN_TX.DLC = len;
	CAN_TX.ExtId = 0; //standard CAN
	CAN_TX.IDE = CAN_ID_STD;
	CAN_TX.RTR = CAN_RTR_DATA;
	CAN_TX.TransmitGlobalTime = DISABLE;

	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);

	if(HAL_CAN_AddTxMessage(&hcan, &CAN_TX, data, &TX_MailBox) != HAL_OK){
		Error_Handler();
	}
}

void CAN_Send_ID040(void){
	uint8_t buf[6];
	buf[0] = TX_ID040.mode;
	buf[1] = (uint8_t)(TX_ID040.motor_set_speed & 0xFF);
	buf[2] = (uint8_t)(TX_ID040.motor_set_speed >> 8);
	buf[3] = (uint8_t)(TX_ID040.motor_set_torque & 0xFF);
	buf[4] = (uint8_t)(TX_ID040.motor_set_torque >> 8);
	buf[5] = TX_ID040.motor_driver_power;
	CAN_Transmit(0x040, sizeof(buf), buf);
}

void CAN_Send_ID070(void){
	uint8_t buf[1];
	buf[0] = TX_ID070.ABS_Control_state;
	CAN_Transmit(0x070, sizeof(buf), buf);
}

void CAN_Send_ID080(void){
	uint8_t buf[6];
	buf[0] = TX_ID080.mode;
	buf[1] = (uint8_t)(TX_ID080.generator_set_speed & 0xFF);
	buf[2] = (uint8_t)(TX_ID080.generator_set_speed >> 8);
	buf[3] = (uint8_t)(TX_ID080.generator_set_torque & 0xFF);
	buf[4] = (uint8_t)(TX_ID080.generator_set_torque >> 8);
	buf[5] = TX_ID080.generator_driver_power;
	CAN_Transmit(0x080, sizeof(buf), buf);
}

void CAN_Send_ID0c0(void){
	uint8_t buf[4];
	buf[0] = (uint8_t)(TX_ID0c0.Motor_Torque_gain & 0xFF);
	buf[1] = (uint8_t)(TX_ID0c0.Motor_Torque_gain >> 8);
	buf[2] = (uint8_t)(TX_ID0c0.Motor_Torque_offset & 0xFF);
	buf[3] = (uint8_t)(TX_ID0c0.Motor_Torque_offset >> 8);
	CAN_Transmit(0x0C0, sizeof(buf), buf);
}

void CAN_Send_ID100(void){
	uint8_t buf[4];
	buf[0] = (uint8_t)(TX_ID100.Generator_Torque_gain & 0xFF);
	buf[1] = (uint8_t)(TX_ID100.Generator_Torque_gain >> 8);
	buf[2] = (uint8_t)(TX_ID100.generator_Torque_offset & 0xFF);
	buf[3] = (uint8_t)(TX_ID100.generator_Torque_offset >> 8);
	CAN_Transmit(0x100, sizeof(buf), buf);
}

void CAN_Send_ID200(void){
	uint8_t buf[4];
	buf[0] = (uint8_t)(TX_ID200.Brake_Torque_gain & 0xFF);
	buf[1] = (uint8_t)(TX_ID200.Brake_Torque_gain >> 8);
	buf[2] = (uint8_t)(TX_ID200.Brake_Torque_offset & 0xFF);
	buf[3] = (uint8_t)(TX_ID200.Brake_Torque_offset >> 8);
	CAN_Transmit(0x200, sizeof(buf), buf);
}

void CAN_Send_ID180(void){
	uint8_t buf[1];
	buf[0] = TX_ID180.WheelPulsePerRevolution;
	CAN_Transmit(0x180, sizeof(buf), buf);
}

void CAN_Send_ID1C0(void){
	uint8_t buf[1];
	buf[0] = TX_ID1C0.RollerPulsePerRevolution;
	CAN_Transmit(0x1C0, sizeof(buf), buf);
}

void CAN_Send_ID401(void){
	uint8_t buf[7];
	buf[0] = (uint8_t)(TX_ID401.Motor_Driver_Volt & 0xFF);
	buf[1] = (uint8_t)(TX_ID401.Motor_Driver_Volt >> 8);
	buf[2] = (uint8_t)(TX_ID401.Motor_Driver_Current & 0xFF);
	buf[3] = (uint8_t)(TX_ID401.Motor_Driver_Current >> 8);
	buf[4] = (uint8_t)(TX_ID401.Motor_Driver_Control_Signal_Volt & 0xFF);
	buf[5] = (uint8_t)(TX_ID401.Motor_Driver_Control_Signal_Volt >> 8);
	buf[6] = TX_ID401.Motor_Driver_Control_Mode;
	CAN_Transmit(0x401, sizeof(buf), buf);
}

void CAN_Send_ID411(void){
	uint8_t buf[7];
	buf[0] = (uint8_t)(TX_ID411.Motor_volt & 0xFF);
	buf[1] = (uint8_t)(TX_ID411.Motor_volt >> 8);
	buf[2] = (uint8_t)(TX_ID411.Motor_Current & 0xFF);
	buf[3] = (uint8_t)(TX_ID411.Motor_Current >> 8);
	buf[4] = TX_ID411.Motor_Contactor;
	buf[5] = (uint8_t)(TX_ID411.Error_code & 0xFF);
	buf[6] = (uint8_t)(TX_ID411.Error_code >> 8);
	CAN_Transmit(0x411, sizeof(buf), buf);
}

void CAN_Send_ID421(void){
	uint8_t buf[4];
	buf[0] = (uint8_t)(TX_ID421.PriBrakePress & 0xFF);
	buf[1] = (uint8_t)(TX_ID421.PriBrakePress >> 8);
	buf[2] = (uint8_t)(TX_ID421.SecBrakePress & 0xFF);
	buf[3] = (uint8_t)(TX_ID421.SecBrakePress >> 8);
	CAN_Transmit(0x421, sizeof(buf), buf);
}

void CAN_Send_ID431(void){
	uint8_t buf[4];
	buf[0] = TX_ID431.ABS_Control_state;
	buf[1] = TX_ID431.ABS_Pump_state;
	buf[2] = TX_ID431.ABS_Increase_Valve;
	buf[3] = TX_ID431.ABS_Decrease_valve;
	CAN_Transmit(0x431, sizeof(buf), buf);
}

void CAN_Send_ID402(void){
	uint8_t buf[7];
	buf[0] = (uint8_t)(TX_ID402.Generator_Driver_Volt & 0xFF);
	buf[1] = (uint8_t)(TX_ID402.Generator_Driver_Volt >> 8);
	buf[2] = (uint8_t)(TX_ID402.Generator_Driver_Current & 0xFF);
	buf[3] = (uint8_t)(TX_ID402.Generator_Driver_Current >> 8);
	buf[4] = (uint8_t)(TX_ID402.Generator_Driver_Control_Signal_Volt & 0xFF);
	buf[5] = (uint8_t)(TX_ID402.Generator_Driver_Control_Signal_Volt >> 8);
	buf[6] = TX_ID402.Generator_Driver_Control_Mode;
	CAN_Transmit(0x402, sizeof(buf), buf);
}

void CAN_Send_ID412(void){
	uint8_t buf[7];
	buf[0] = (uint8_t)(TX_ID412.Generator_volt & 0xFF);
	buf[1] = (uint8_t)(TX_ID412.Generator_volt >> 8);
	buf[2] = (uint8_t)(TX_ID412.Generator_Current & 0xFF);
	buf[3] = (uint8_t)(TX_ID412.Generator_Current >> 8);
	buf[4] = TX_ID412.Generator_Contactor;
	buf[5] = (uint8_t)(TX_ID412.Error_code & 0xFF);
	buf[6] = (uint8_t)(TX_ID412.Error_code >> 8);
	CAN_Transmit(0x412, sizeof(buf), buf);
}

void CAN_Send_ID403(void){
	uint8_t buf[8];
	buf[0] = (uint8_t)(TX_ID403.Motor_Torque & 0xFF);
	buf[1] = (uint8_t)(TX_ID403.Motor_Torque >> 8);
	buf[2] = (uint8_t)(TX_ID403.Motor_Torque_Gain & 0xFF);
	buf[3] = (uint8_t)(TX_ID403.Motor_Torque_Gain >> 8);
	buf[4] = (uint8_t)(TX_ID403.Motor_Torque_Offset & 0xFF);
	buf[5] = (uint8_t)(TX_ID403.Motor_Torque_Offset >> 8);
	buf[6] = (uint8_t)(TX_ID403.Error_Code & 0xFF);
	buf[7] = (uint8_t)(TX_ID403.Error_Code >> 8);
	CAN_Transmit(0x403, sizeof(buf), buf);
}

void CAN_Send_ID404(void){
	uint8_t buf[8];
	buf[0] = (uint8_t)(TX_ID404.Generator_Torque & 0xFF);
	buf[1] = (uint8_t)(TX_ID404.Generator_Torque >> 8);
	buf[2] = (uint8_t)(TX_ID404.Generator_Torque_Gain & 0xFF);
	buf[3] = (uint8_t)(TX_ID404.Generator_Torque_Gain >> 8);
	buf[4] = (uint8_t)(TX_ID404.Generator_Torque_Offset & 0xFF);
	buf[5] = (uint8_t)(TX_ID404.Generator_Torque_Offset >> 8);
	buf[6] = (uint8_t)(TX_ID404.Error_Code & 0xFF);
	buf[7] = (uint8_t)(TX_ID404.Error_Code >> 8);
	CAN_Transmit(0x404, sizeof(buf), buf);
}

void CAN_Send_ID405(void){
	uint8_t buf[8];
	buf[0] = (uint8_t)(TX_ID405.Brake_Torque & 0xFF);
	buf[1] = (uint8_t)(TX_ID405.Brake_Torque >> 8);
	buf[2] = (uint8_t)(TX_ID405.Brake_Torque_Gain & 0xFF);
	buf[3] = (uint8_t)(TX_ID405.Brake_Torque_Gain >> 8);
	buf[4] = (uint8_t)(TX_ID405.Brake_Torque_Offset & 0xFF);
	buf[5] = (uint8_t)(TX_ID405.Brake_Torque_Offset >> 8);
	buf[6] = (uint8_t)(TX_ID405.Error_Code & 0xFF);
	buf[7] = (uint8_t)(TX_ID405.Error_Code >> 8);
	CAN_Transmit(0x405, sizeof(buf), buf);
}

void CAN_Send_ID406(void){
	uint8_t buf[6];
	buf[0] = (uint8_t)(TX_ID406.WheelSpeed & 0xFF);
	buf[1] = (uint8_t)(TX_ID406.WheelSpeed >> 8);
	buf[2] = (uint8_t)(TX_ID406.PulsePerRevolution & 0xFF);
	buf[3] = (uint8_t)(TX_ID406.PulsePerRevolution >> 8);
	buf[4] = (uint8_t)(TX_ID406.Error_code & 0xFF);
	buf[5] = (uint8_t)(TX_ID406.Error_code >> 8);
	CAN_Transmit(0x406, sizeof(buf), buf);
}

void CAN_Send_ID407(void){
	uint8_t buf[6];
	buf[0] = (uint8_t)(TX_ID407.RollerSpeed & 0xFF);
	buf[1] = (uint8_t)(TX_ID407.RollerSpeed >> 8);
	buf[2] = (uint8_t)(TX_ID407.PulsePerRevolution & 0xFF);
	buf[3] = (uint8_t)(TX_ID407.PulsePerRevolution >> 8);
	buf[4] = (uint8_t)(TX_ID407.Error_code & 0xFF);
	buf[5] = (uint8_t)(TX_ID407.Error_code >> 8);
	CAN_Transmit(0x407, sizeof(buf), buf);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcanx){
	if(HAL_CAN_GetRxMessage(hcanx, RX_FIFO, &CAN_RX, CAN_rx_data) == HAL_OK){
		Rx_count++;
		switch(CAN_RX.StdId){
			case 0x040:
				Message_ID040.mode = CAN_rx_data[0];
				Message_ID040.motor_set_speed = CAN_rx_data[1] | CAN_rx_data[2] << 8;
				Message_ID040.motor_set_torque = CAN_rx_data[3] | CAN_rx_data[4] << 8;
				Message_ID040.motor_driver_power = CAN_rx_data[5];
				break;
			case 0x070:
				Message_ID070.ABS_Control_state = CAN_rx_data[0];
				break;
			case 0x080:
				Message_ID080.mode = CAN_rx_data[0];
				Message_ID080.generator_set_speed = CAN_rx_data[1] | CAN_rx_data[2] << 8;
				Message_ID080.generator_set_torque = CAN_rx_data[3] | CAN_rx_data[4] << 8;
				Message_ID080.generator_driver_power = CAN_rx_data[5];
				break;
			case 0x0C0:
				Message_ID0c0.Motor_Torque_gain = CAN_rx_data[0] | CAN_rx_data[1] << 8;
				Message_ID0c0.Motor_Torque_offset = CAN_rx_data[2] | CAN_rx_data[3] << 8;
				break;
			case 0x100:
				Message_ID100.Generator_Torque_gain = CAN_rx_data[0] | CAN_rx_data[1] << 8;
				Message_ID100.generator_Torque_offset = CAN_rx_data[2] | CAN_rx_data[3] << 8;
				break;
			case 0x200:
				Message_ID200.Brake_Torque_gain = CAN_rx_data[0] | CAN_rx_data[1] << 8;
				Message_ID200.Brake_Torque_offset = CAN_rx_data[2] | CAN_rx_data[3] << 8;
				break;
			case 0x180:
				Message_ID180.WheelPulsePerRevolution = CAN_rx_data[0];
				break;
			case 0x1C0:
				Message_ID1C0.RollerPulsePerRevolution = CAN_rx_data[0];
				break;
			case 0x401:
				Message_ID401.Motor_Driver_Volt = CAN_rx_data[0] | CAN_rx_data[1] << 8;
				Message_ID401.Motor_Driver_Current = CAN_rx_data[2] | CAN_rx_data[3] << 8;
				Message_ID401.Motor_Driver_Control_Signal_Volt = CAN_rx_data[4] | CAN_rx_data[5] << 8;
				Message_ID401.Motor_Driver_Control_Mode = CAN_rx_data[6];
				break;
			case 0x411:
				Message_ID411.Motor_volt = CAN_rx_data[0] | CAN_rx_data[1] << 8;
				Message_ID411.Motor_Current = CAN_rx_data[2] | CAN_rx_data[3] << 8;
				Message_ID411.Motor_Contactor = CAN_rx_data[4];
				Message_ID411.Error_code = CAN_rx_data[5] | CAN_rx_data[6] << 8;
				break;
			case 0x421:
				Message_ID421.PriBrakePress = CAN_rx_data[0] | CAN_rx_data[1] << 8;
				Message_ID421.SecBrakePress = CAN_rx_data[2] | CAN_rx_data[3] << 8;
				break;
			case 0x431:
				Message_ID431.ABS_Control_state = CAN_rx_data[0];
				Message_ID431.ABS_Pump_state = CAN_rx_data[1];
				Message_ID431.ABS_Increase_Valve = CAN_rx_data[2];
				Message_ID431.ABS_Decrease_valve = CAN_rx_data[3];
				break;
			case 0x402:
				Message_ID402.Generator_Driver_Volt = CAN_rx_data[0] | CAN_rx_data[1] << 8;
				Message_ID402.Generator_Driver_Current = CAN_rx_data[2] | CAN_rx_data[3] << 8;
				Message_ID402.Generator_Driver_Control_Signal_Volt = CAN_rx_data[4] | CAN_rx_data[5] << 8;
				Message_ID402.Generator_Driver_Control_Mode = CAN_rx_data[6];
				break;
			case 0x412:
				Message_ID412.Generator_volt = CAN_rx_data[0] | CAN_rx_data[1] << 8;
				Message_ID412.Generator_Current = CAN_rx_data[2] | CAN_rx_data[3] << 8;
				Message_ID412.Generator_Contactor = CAN_rx_data[4];
				Message_ID412.Error_code = CAN_rx_data[5] | CAN_rx_data[6] << 8;
				break;
			case 0x403:
				Message_ID403.Motor_Torque = CAN_rx_data[0] | CAN_rx_data[1] << 8;
				Message_ID403.Motor_Torque_Gain = CAN_rx_data[2] | CAN_rx_data[3] << 8;
				Message_ID403.Motor_Torque_Offset = CAN_rx_data[4] | CAN_rx_data[5] << 8;
				Message_ID403.Error_Code = CAN_rx_data[6] | CAN_rx_data[7] << 8;
				break;
			case 0x404:
				Message_ID404.Generator_Torque = CAN_rx_data[0] | CAN_rx_data[1] << 8;
				Message_ID404.Generator_Torque_Gain = CAN_rx_data[2] | CAN_rx_data[3] << 8;
				Message_ID404.Generator_Torque_Offset = CAN_rx_data[4] | CAN_rx_data[5] << 8;
				Message_ID404.Error_Code = CAN_rx_data[6] | CAN_rx_data[7] << 8;
				break;
			case 0x405:
				Message_ID405.Brake_Torque = CAN_rx_data[0] | CAN_rx_data[1] << 8;
				Message_ID405.Brake_Torque_Gain = CAN_rx_data[2] | CAN_rx_data[3] << 8;
				Message_ID405.Brake_Torque_Offset = CAN_rx_data[4] | CAN_rx_data[5] << 8;
				Message_ID405.Error_Code = CAN_rx_data[6] | CAN_rx_data[7] << 8;
				break;
			case 0x406:
				Message_ID406.WheelSpeed = CAN_rx_data[0] | CAN_rx_data[1] << 8;
				Message_ID406.PulsePerRevolution = CAN_rx_data[2] | CAN_rx_data[3] << 8;
				Message_ID406.Error_code = CAN_rx_data[4] | CAN_rx_data[5] << 8;
				break;
			case 0x407:
				Message_ID407.RollerSpeed = CAN_rx_data[0] | CAN_rx_data[1] << 8;
				Message_ID407.PulsePerRevolution = CAN_rx_data[2] | CAN_rx_data[3] << 8;
				Message_ID407.Error_code = CAN_rx_data[4] | CAN_rx_data[5] << 8;
				break;
			default:
				break;
		}
	}

}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcanx){
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	__HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_RQCP0);
//	switch(can_tx_state){
//	case 1:
//		CAN_Send_ID412();
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		can_tx_state = 0;
//		break;
//	default:
//		can_tx_state = 0;
//		break;
//	}
//	switch(can_tx_state){
//	case 1:
//		CAN_Send_ID411();
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		can_tx_state = 2;
//		break;
//	case 2:
//		CAN_Send_ID421();
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		can_tx_state = 3;
//		break;
//	case 3:
//		CAN_Send_ID431();
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		can_tx_state = 0;
//		break;
//	default:
//		can_tx_state = 0;
//		break;
//	}
	Tx_count++;
}



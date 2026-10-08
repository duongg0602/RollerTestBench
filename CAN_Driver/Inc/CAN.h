/*
 * CAN.h
 *
 *  Created on: 11 thg 8, 2026
 *      Author: ASUS_PC
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_
#include "main.h"

typedef struct{
	uint8_t mode;
	uint16_t motor_set_speed;
	uint16_t motor_set_torque;
	uint8_t motor_driver_power;
}GUI_Motor_Driver_Control_ID040;

typedef struct{
	uint8_t mode;
	uint16_t generator_set_speed;
	uint16_t generator_set_torque;
	uint8_t generator_driver_power;
}GUI_Generator_Driver_Control_ID080;

typedef struct{
	uint8_t ABS_Control_state;
}ABS_Control_state_GUI_ID070;

typedef struct{
	uint16_t Motor_Torque_gain;
	uint16_t Motor_Torque_offset;
}GainOffsetMotor_GUI_ID0c0;

typedef struct{
	uint16_t Generator_Torque_gain;
	uint16_t generator_Torque_offset;
}GainOffsetGenerator_GUI_ID100;

typedef struct{
	uint16_t Brake_Torque_gain;
	uint16_t Brake_Torque_offset;
}GainOffsetBrake_GUI_ID200;

typedef struct{
	uint8_t WheelPulsePerRevolution;
}SetPulseWheel_GUI_ID180;

typedef struct{
	uint8_t RollerPulsePerRevolution;
}SetPulseRoller_GUI_ID1c0;

typedef struct{
	uint16_t Motor_Driver_Volt;
	uint16_t Motor_Driver_Current;
	uint16_t Motor_Driver_Control_Signal_Volt;
	uint8_t Motor_Driver_Control_Mode;
}Motor_Driver_Control_ID401;

typedef struct{
	uint16_t Motor_volt;
	uint16_t Motor_Current;
	uint8_t Motor_Contactor;
	uint16_t Error_code;
}MotorControl_ID411;

typedef struct{
	uint16_t PriBrakePress;
	uint16_t SecBrakePress;
}BrakePress_ID421;

typedef struct{
	uint8_t ABS_Control_state;
	uint8_t ABS_Pump_state;
	uint8_t ABS_Increase_Valve;
	uint8_t ABS_Decrease_valve;
}ABS_Control_ID431;

typedef struct{
	uint16_t Generator_Driver_Volt;
	uint16_t Generator_Driver_Current;
	uint16_t Generator_Driver_Control_Signal_Volt;
	uint8_t Generator_Driver_Control_Mode;
}Generator_Driver_Control_ID402;

typedef struct{
	uint16_t Generator_volt;
	uint16_t Generator_Current;
	uint8_t Generator_Contactor;
	uint16_t Error_code;
}GeneratorControl_ID412;

typedef struct{
	uint16_t Motor_Torque;
	uint16_t Motor_Torque_Gain;
	uint16_t Motor_Torque_Offset;
	uint16_t Error_Code;
}MotorTorque_ID403;

typedef struct{
	uint16_t Generator_Torque;
	uint16_t Generator_Torque_Gain;
	uint16_t Generator_Torque_Offset;
	uint16_t Error_Code;
}GeneratorTorque_ID404;

typedef struct{
	uint16_t Brake_Torque;
	uint16_t Brake_Torque_Gain;
	uint16_t Brake_Torque_Offset;
	uint16_t Error_Code;
}BrakeTorque_ID405;

typedef struct{
	uint16_t WheelSpeed;
	uint16_t PulsePerRevolution;
	uint16_t Error_code;
}WheelSpeed_ID406;

typedef struct{
	uint16_t RollerSpeed;
	uint16_t PulsePerRevolution;
	uint16_t Error_code;
}RollerSpeed_ID407;

extern GUI_Motor_Driver_Control_ID040 Message_ID040;
extern GUI_Generator_Driver_Control_ID080 Message_ID080;
extern ABS_Control_state_GUI_ID070 Message_ID070;
extern GainOffsetMotor_GUI_ID0c0 Message_ID0c0;
extern GainOffsetGenerator_GUI_ID100 Message_ID100;
extern GainOffsetBrake_GUI_ID200 Message_ID200;
extern SetPulseWheel_GUI_ID180 Message_ID180;
extern SetPulseRoller_GUI_ID1c0 Message_ID1C0;
extern Motor_Driver_Control_ID401 Message_ID401;
extern MotorControl_ID411 Message_ID411;
extern BrakePress_ID421 Message_ID421;
extern ABS_Control_ID431 Message_ID431;
extern Generator_Driver_Control_ID402 Message_ID402;
extern GeneratorControl_ID412 Message_ID412;
extern MotorTorque_ID403 Message_ID403;
extern GeneratorTorque_ID404 Message_ID404;
extern BrakeTorque_ID405 Message_ID405;
extern WheelSpeed_ID406 Message_ID406;
extern RollerSpeed_ID407 Message_ID407;

extern GUI_Motor_Driver_Control_ID040 TX_ID040;
extern GUI_Generator_Driver_Control_ID080 TX_ID080;
extern ABS_Control_state_GUI_ID070 TX_ID070;
extern GainOffsetMotor_GUI_ID0c0 TX_ID0c0;
extern GainOffsetGenerator_GUI_ID100 TX_ID100;
extern GainOffsetBrake_GUI_ID200 TX_ID200;
extern SetPulseWheel_GUI_ID180 TX_ID180;
extern SetPulseRoller_GUI_ID1c0 TX_ID1C0;
extern Motor_Driver_Control_ID401 TX_ID401;
extern MotorControl_ID411 TX_ID411;
extern BrakePress_ID421 TX_ID421;
extern ABS_Control_ID431 TX_ID431;
extern Generator_Driver_Control_ID402 TX_ID402;
extern GeneratorControl_ID412 TX_ID412;
extern MotorTorque_ID403 TX_ID403;
extern GeneratorTorque_ID404 TX_ID404;
extern BrakeTorque_ID405 TX_ID405;
extern WheelSpeed_ID406 TX_ID406;
extern RollerSpeed_ID407 TX_ID407;

extern uint8_t CAN_rx_data[64];
extern CAN_RxHeaderTypeDef CAN_RX;
extern CAN_TxHeaderTypeDef CAN_TX;
extern CAN_HandleTypeDef hcan;

extern uint8_t tx_data[64];
extern uint32_t RX_FIFO;
extern uint32_t TX_MailBox;
extern uint16_t Rx_count;
extern uint16_t Tx_count;
extern volatile uint8_t can_tx_state;

void CAN_Config(uint32_t* ID, uint8_t count);
void CAN_Transmit(uint32_t id, uint32_t len, uint8_t *data);
void CAN_Send_ID040(void);
void CAN_Send_ID070(void);
void CAN_Send_ID080(void);
void CAN_Send_ID0c0(void);
void CAN_Send_ID100(void);
void CAN_Send_ID200(void);
void CAN_Send_ID180(void);
void CAN_Send_ID1C0(void);
void CAN_Send_ID401(void);
void CAN_Send_ID411(void);
void CAN_Send_ID421(void);
void CAN_Send_ID431(void);
void CAN_Send_ID402(void);
void CAN_Send_ID412(void);
void CAN_Send_ID403(void);
void CAN_Send_ID404(void);
void CAN_Send_ID405(void);
void CAN_Send_ID406(void);
void CAN_Send_ID407(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);


#endif /* INC_CAN_H_ */

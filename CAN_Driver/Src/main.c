/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN.h"
#include "uart_framework.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

CAN_RxHeaderTypeDef CAN_RX;
CAN_TxHeaderTypeDef CAN_TX;

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

uint8_t rx_buffer[64];
volatile uint32_t cnt = 0;
volatile uint8_t can_tx_state = 0;
uint32_t IDList[] = {0x080, 0x404, 0x407};

#pragma pack(push, 1)
typedef struct{
	uint16_t Motor_Driver_Volt;
	uint16_t Motor_Driver_Current;
	uint16_t Motor_Driver_Control_Signal_Volt;
	uint8_t Motor_Driver_Control_Mode;
	uint16_t Motor_volt;
	uint16_t Motor_Current;
	uint8_t Motor_Contactor;
	uint16_t Error_code_ID411;
	uint16_t PriBrakePress;
	uint16_t SecBrakePress;
	uint8_t ABS_Control_state;
	uint8_t ABS_Pump_state;
	uint8_t ABS_Increase_Valve;
	uint8_t ABS_Decrease_valve;
	uint16_t Generator_Driver_Volt;
	uint16_t Generator_Driver_Current;
	uint16_t Generator_Driver_Control_Signal_Volt;
	uint8_t Generator_Driver_Control_Mode;
	uint16_t Generator_volt;
	uint16_t Generator_Current;
	uint8_t Generator_Contactor;
	uint16_t Error_code_ID412;
	uint16_t Motor_Torque;
	uint16_t Motor_Torque_Gain;
	uint16_t Motor_Torque_Offset;
	uint16_t Error_Code_ID403;
	uint16_t Generator_Torque;
	uint16_t Generator_Torque_Gain;
	uint16_t Generator_Torque_Offset;
	uint16_t Error_Code_ID404;
	uint16_t Brake_Torque;
	uint16_t Brake_Torque_Gain;
	uint16_t Brake_Torque_Offset;
	uint16_t Error_Code_ID405;
	uint16_t WheelSpeed;
	uint16_t PulsePerRevolution_Wheel;
	uint16_t Error_code_ID406;
	uint16_t RollerSpeed;
	uint16_t PulsePerRevolution_Roller;
	uint16_t Error_code_ID407;
}payload_t;
#pragma pack(pop)

payload_t CAN_UART_Data;
UART_Output_t UART_RX_DATA;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	cnt++;
//	if(cnt >= 100){
//		TX_ID401.Motor_Driver_Control_Mode++;
//		TX_ID401.Motor_Driver_Volt++;
//		TX_ID401.Motor_Driver_Current++;
//		TX_ID401.Motor_Driver_Control_Signal_Volt++;
//		TX_ID411.Error_code++;
//		TX_ID411.Motor_Contactor++;
//		TX_ID411.Motor_Current++;
//		TX_ID411.Motor_volt++;
//		TX_ID421.PriBrakePress++;
//		TX_ID421.SecBrakePress++;
//		TX_ID431.ABS_Control_state++;
//		TX_ID431.ABS_Decrease_valve++;
//		TX_ID431.ABS_Increase_Valve++;
//		TX_ID431.ABS_Pump_state++;
//		CAN_Send_ID401();
//		TX_ID402.Generator_Driver_Control_Mode++;
//		TX_ID402.Generator_Driver_Volt++;
//		TX_ID402.Generator_Driver_Current++;
//		TX_ID402.Generator_Driver_Control_Signal_Volt++;
//		TX_ID412.Error_code++;
//		TX_ID412.Generator_Contactor++;
//		TX_ID412.Generator_Current++;
//		TX_ID412.Generator_volt++;
//		CAN_Send_ID402();
//		can_tx_state = 1;
//		cnt = 0;
//	}
//	if(cnt >= 40){
//		TX_ID403.Error_Code++;
//		TX_ID403.Motor_Torque++;
//		TX_ID403.Motor_Torque_Gain++;
//		TX_ID403.Motor_Torque_Offset++;
//		TX_ID404.Error_Code++;
//		TX_ID404.Generator_Torque++;
//		TX_ID404.Generator_Torque_Gain++;
//		TX_ID404.Generator_Torque_Offset++;
//		TX_ID405.Error_Code++;
//		TX_ID405.Brake_Torque++;
//		TX_ID405.Brake_Torque_Gain++;
//		TX_ID405.Brake_Torque_Offset++;
//		TX_ID406.Error_code++;
//		TX_ID406.WheelSpeed++;
//		TX_ID406.PulsePerRevolution++;
//		TX_ID407.Error_code++;
//		TX_ID407.RollerSpeed++;
//		TX_ID407.PulsePerRevolution++;
//		CAN_Send_ID403();
//		CAN_Send_ID404();
//		CAN_Send_ID405();
//		CAN_Send_ID406();
//		CAN_Send_ID407();
//		cnt = 0;
//	}
	if(cnt >= 100){
		CAN_UART_Data.Motor_Driver_Volt = Message_ID401.Motor_Driver_Volt;
		CAN_UART_Data.Motor_Driver_Current = Message_ID401.Motor_Driver_Current;
		CAN_UART_Data.Motor_Driver_Control_Signal_Volt = Message_ID401.Motor_Driver_Control_Signal_Volt;
		CAN_UART_Data.Motor_Driver_Control_Mode = Message_ID401.Motor_Driver_Control_Mode;
		CAN_UART_Data.Motor_volt = Message_ID411.Motor_volt;
		CAN_UART_Data.Motor_Current = Message_ID411.Motor_Current;
		CAN_UART_Data.Motor_Contactor = Message_ID411.Motor_Contactor;
		CAN_UART_Data.Error_code_ID411 = Message_ID411.Error_code;
		CAN_UART_Data.PriBrakePress = Message_ID421.PriBrakePress;
		CAN_UART_Data.SecBrakePress = Message_ID421.SecBrakePress;
		CAN_UART_Data.ABS_Control_state = Message_ID431.ABS_Control_state;
		CAN_UART_Data.ABS_Decrease_valve = Message_ID431.ABS_Decrease_valve;
		CAN_UART_Data.ABS_Increase_Valve = Message_ID431.ABS_Increase_Valve;
		CAN_UART_Data.ABS_Pump_state = Message_ID431.ABS_Pump_state;
		CAN_UART_Data.Generator_Driver_Volt = Message_ID402.Generator_Driver_Volt;
		CAN_UART_Data.Generator_Driver_Current = Message_ID402.Generator_Driver_Current;
		CAN_UART_Data.Generator_Driver_Control_Signal_Volt = Message_ID402.Generator_Driver_Control_Signal_Volt;
		CAN_UART_Data.Generator_Driver_Control_Mode = Message_ID402.Generator_Driver_Control_Mode;
		CAN_UART_Data.Generator_volt = Message_ID412.Generator_volt;
		CAN_UART_Data.Generator_Current = Message_ID412.Generator_Current;
		CAN_UART_Data.Generator_Contactor = Message_ID412.Generator_Contactor;
		CAN_UART_Data.Error_code_ID412 = Message_ID412.Error_code;
		CAN_UART_Data.Motor_Torque = Message_ID403.Motor_Torque;
		CAN_UART_Data.Motor_Torque_Gain = Message_ID403.Motor_Torque_Gain;
		CAN_UART_Data.Motor_Torque_Offset = Message_ID403.Motor_Torque_Offset;
		CAN_UART_Data.Error_Code_ID403 = Message_ID403.Error_Code;
		CAN_UART_Data.Generator_Torque = Message_ID404.Generator_Torque;
		CAN_UART_Data.Generator_Torque_Gain = Message_ID404.Generator_Torque_Gain;
		CAN_UART_Data.Generator_Torque_Offset = Message_ID404.Generator_Torque_Offset;
		CAN_UART_Data.Error_Code_ID404 = Message_ID404.Error_Code;
		CAN_UART_Data.Brake_Torque = Message_ID405.Brake_Torque;
		CAN_UART_Data.Brake_Torque_Gain = Message_ID405.Brake_Torque_Gain;
		CAN_UART_Data.Brake_Torque_Offset = Message_ID405.Brake_Torque_Offset;
		CAN_UART_Data.Error_Code_ID405 = Message_ID405.Error_Code;
		CAN_UART_Data.WheelSpeed = Message_ID406.WheelSpeed;
		CAN_UART_Data.PulsePerRevolution_Wheel = Message_ID406.PulsePerRevolution;
		CAN_UART_Data.Error_code_ID406 = Message_ID406.Error_code;
		CAN_UART_Data.RollerSpeed = Message_ID407.RollerSpeed;
		CAN_UART_Data.PulsePerRevolution_Roller = Message_ID407.PulsePerRevolution;
		CAN_UART_Data.Error_code_ID407 = Message_ID407.Error_code;
		uint8_t* data = (uint8_t*)&CAN_UART_Data;
		if(UART1_SendExpectedData(data, sizeof(payload_t))){
			UART1_Send();
		}
		cnt = 0;
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  CAN_Config(NULL, 0);
  HAL_TIM_Base_Start_IT(&htim3);
  UART1_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  UART1_Parse();
	  if(UART1_GetReceivedData(&UART_RX_DATA)){
		  TX_ID040.mode = UART_RX_DATA.payload[0];
		  TX_ID040.motor_set_speed = UART_RX_DATA.payload[1] | UART_RX_DATA.payload[2] << 8;
		  TX_ID040.motor_set_torque = UART_RX_DATA.payload[3] | UART_RX_DATA.payload[4] << 8;
		  TX_ID040.motor_driver_power = UART_RX_DATA.payload[5];
		  CAN_Send_ID040();
		  TX_ID070.ABS_Control_state = UART_RX_DATA.payload[6];
		  CAN_Send_ID070();
		  TX_ID080.mode = UART_RX_DATA.payload[7];
		  TX_ID080.generator_set_speed = UART_RX_DATA.payload[8] | UART_RX_DATA.payload[9] << 8;
		  TX_ID080.generator_set_torque = UART_RX_DATA.payload[10] | UART_RX_DATA.payload[11] << 8;
		  TX_ID080.generator_driver_power = UART_RX_DATA.payload[12];
		  CAN_Send_ID080();
		  TX_ID0c0.Motor_Torque_gain = UART_RX_DATA.payload[13] | UART_RX_DATA.payload[14] << 8;
		  TX_ID0c0.Motor_Torque_offset = UART_RX_DATA.payload[15] | UART_RX_DATA.payload[16] << 8;
		  CAN_Send_ID0c0();
		  TX_ID100.Generator_Torque_gain = UART_RX_DATA.payload[17] | UART_RX_DATA.payload[18] << 8;
		  TX_ID100.generator_Torque_offset = UART_RX_DATA.payload[19] | UART_RX_DATA.payload[20] << 8;
		  CAN_Send_ID100();
		  TX_ID200.Brake_Torque_gain = UART_RX_DATA.payload[21] | UART_RX_DATA.payload[22] << 8;
		  TX_ID200.Brake_Torque_offset = UART_RX_DATA.payload[23] | UART_RX_DATA.payload[24] << 8;
		  CAN_Send_ID200();
		  TX_ID180.WheelPulsePerRevolution = UART_RX_DATA.payload[25] | UART_RX_DATA.payload[26] << 8;
		  CAN_Send_ID180();
		  TX_ID1C0.RollerPulsePerRevolution = UART_RX_DATA.payload[27] | UART_RX_DATA.payload[28] << 8;
		  CAN_Send_ID1C0();
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

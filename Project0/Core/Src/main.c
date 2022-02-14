/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stdio.h"
#include <stdlib.h>
#include "string.h"
#include "math.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

uint64_t _micros = 0;
float EncoderVel = 0;
uint64_t Timestamp_Encoder = 0;

typedef struct _UartStructure
{
	UART_HandleTypeDef *huart;
	uint16_t TxLen, RxLen;
	uint8_t *TxBuffer;
	uint16_t TxTail, TxHead;
	uint8_t *RxBuffer;
	uint16_t RxTail; //RXHeadUseDMA

} UARTStucrture;

UARTStucrture UART2 = { 0 };

typedef struct _MotorDriveStructure
{
	uint8_t DIR;
	uint32_t PWMOut;
}MotorDriveStructure;

MotorDriveStructure DCMotorStruc = {0};

typedef enum
{
	PP_STARTandMode,
	PP_Frame2_Data_0,
	PP_Frame2_Data_1,
	PP_Frame3_Data_0,
	PP_Frame3_Data_1,
	PP_CheckSum,
} PPSTATE;

PPSTATE Munmunbot_Protocol_State = PP_STARTandMode;

typedef enum
{
	STATE_Disconnected,    /// Set_station_position
	STATE_Idle,
	STATE_PrepareDATA,
	STATE_Calculation,
	STATE_Link_Moving,
	STATE_End_Effector_Working,
	STATE_SetHome
} Robot_STATE;

Robot_STATE Munmunbot_State = STATE_Disconnected;

typedef enum
{
	SetHomeState_0,
	SetHomeState_1,
	SetHomeState_2
} SetHome_STATE;

SetHome_STATE SethomeMode = SetHomeState_0;

typedef struct _TrajectoryGenerationStructure
{
	float BlendTimeLSPB;
	float BlendTimeTriangular;
	float LinearTimeLSPB;

	float Theta_min_for_LSPB;
	float AngularVelocityMax_Setting;
	float AngularAccerationMax_Setting;

	float AngularVelocity;
	float AngularAcceration;
	float AngularDisplacementDesire;

	float Theta_Stamp_0;
	float Theta_Stamp_1;
	float Theta_Stamp_2;

	double Equation_Realtime_Sec;

	uint64_t Equation_Timestamp;
	uint64_t Loop_Timestamp;

	uint32_t Loop_Freq;
	uint64_t Loop_Period;

	float Desire_Theta;
	float Start_Theta;
	float Delta_Theta;
	float Abs_Delta_Theta;

	uint32_t Mode;
	uint32_t Submode;

} TrajectoryGenerationStructure;

typedef struct _ConverterUnitSystemStructure
{
	uint32_t PPR;
	uint32_t PPRxQEI;
	uint32_t RPMp;

} ConverterUnitSystemStructure;

typedef struct _PIDStructure
{
	float Kp;
	float Ki;
	float Kd;
	float ControllerOutput;
	float OutputDesire;
	float OutputFeedback;
	float Integral_Value;
	float NowError;
	float PreviousError;
	double SamplingTime;
} PIDStructure;


///Station Setting
uint16_t StationPos[10] = {10,20,30,40,50,60,70,80,90,100};

uint8_t Angularpos_InputArray[256] = {0};
uint16_t Angularpos_InputNumber = 0;

typedef enum
{
	LMM_Not_Set,
	LMM_Set_Pos_Directly,
	LMM_Set_Goal_1_Station,
	LMM_Set_Goal_n_Station
} LinkMovingMode;

LinkMovingMode MovingLinkMode = LMM_Not_Set;

uint8_t Current_Station = 0;
uint8_t NumberOfStationToGo = 0;
uint8_t NumberOfStationPTR = 0;
uint32_t Plant_input = 0;
uint8_t Moving_Link_Task_Flag = 0;

PIDStructure PositionPIDController = {0};
PIDStructure VelocityPIDController  = {0};

TrajectoryGenerationStructure TrjStruc = {0};
ConverterUnitSystemStructure CUSSStruc = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
uint64_t micros();
void EncoderVelocityAndPosition_Update();

void ConverterUnitSystemStructureInit(ConverterUnitSystemStructure *CUSSvar);
void TrajectoryGenerationStructureInit(TrajectoryGenerationStructure *TGSvar, ConverterUnitSystemStructure *CUSSvar);

void TrajectoryGenerationPrepareDATA();
void TrajectoryGenerationCalculation();
void TrajectoryGenerationProcess();

void VelocityControllerInit(PIDStructure *VCvar,TrajectoryGenerationStructure *TGSvar);
void DisplacementControllerInit(PIDStructure *VCvar,TrajectoryGenerationStructure *TGSvar);
void PIDController2in1();

void UARTInit(UARTStucrture *uart);
void UARTResetStart(UARTStucrture *uart);
uint32_t UARTGetRxHead(UARTStucrture *uart);
int16_t UARTReadChar(UARTStucrture *uart);
void UARTTxDumpBuffer(UARTStucrture *uart);
void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len);
void Munmunbot_Protocol(int16_t dataIn,UARTStucrture *uart);
void Encoder_SetHome_Position();

void ACK1Return(UARTStucrture *uart);
void ACK2Return(UARTStucrture *uart);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	// start PWM
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  ///Init Data
  ConverterUnitSystemStructureInit(&CUSSStruc);
  TrajectoryGenerationStructureInit(&TrjStruc, &CUSSStruc);

  VelocityControllerInit(&VelocityPIDController, &TrjStruc);
  DisplacementControllerInit(&PositionPIDController, &TrjStruc);

  ///UART init
  UART2.huart = &huart2;
  UART2.RxLen = 255;
  UART2.TxLen = 255;
  UARTInit(&UART2);
  UARTResetStart(&UART2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  int16_t inputChar = UARTReadChar(&UART2);
	  if (inputChar != -1)
	  {
		  Munmunbot_Protocol(inputChar, &UART2);

	  }

	  switch (Munmunbot_State)
	  {
	  	  case STATE_Disconnected:
	  		  break;
	  	  case STATE_Idle:
		  	  break;

	  	  case STATE_PrepareDATA:
	  		  TrajectoryGenerationPrepareDATA();
		  	  break;

	  	  case STATE_Calculation:
	  		  TrajectoryGenerationCalculation();
	  		  Munmunbot_State = STATE_Link_Moving;
	  		  break;

	   	  case STATE_Link_Moving:
	   		  if (micros()-TrjStruc.Loop_Timestamp >=  TrjStruc.Loop_Period)
	   		  {
	   			  // GEN Trajectory
	   			  TrajectoryGenerationProcess();
	   			  EncoderVelocityAndPosition_Update();
	   			  PIDController2in1();
	   			  Plant_input = PositionPIDController.ControllerOutput;
	   			  DCMotorStruc.PWMOut = abs(Plant_input);
	   			  if (DCMotorStruc.PWMOut > 10000)
	   			  {
	   				 DCMotorStruc.PWMOut = 10000;
	   			  }
	   			  if (Plant_input >= 0)
	   			  {
	   				  DCMotorStruc.DIR = 1;
	   			  }
	   			  else if (Plant_input < 0)
	   			  {
	   				  DCMotorStruc.DIR = 0;
	   			  }
	   			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, DCMotorStruc.DIR);
	   			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, DCMotorStruc.PWMOut);
	   			  TrjStruc.Loop_Timestamp = micros();

	   			  if ((PositionPIDController.OutputFeedback <= TrjStruc.Desire_Theta + 1) &&
	   					  (PositionPIDController.OutputFeedback >= TrjStruc.Desire_Theta - 1) &&
						  (Moving_Link_Task_Flag == 1))
	   			  {
	   				  if(MovingLinkMode == LMM_Set_Pos_Directly)
	   				  {
	   					Munmunbot_State = STATE_Idle;
	   					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	   					ACK2Return(&UART2);
	   				  }
	   				  else if ((MovingLinkMode == LMM_Set_Goal_1_Station) || (MovingLinkMode == LMM_Set_Goal_n_Station))
	   				  {
	   					Munmunbot_State = STATE_End_Effector_Working;
	   					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	   				  }
	   				 TrjStruc.Start_Theta = PositionPIDController.OutputFeedback;  //set new start theta
	   				 Moving_Link_Task_Flag = 0;
	   			  }

	   		  }

	  		  break;
	  	  case STATE_End_Effector_Working:
	  		  ///I2C implement



	  		  break;

	  	  case STATE_SetHome:

	  		  switch (SethomeMode)
	  		  {
				case SetHomeState_0:
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);
					SethomeMode = SetHomeState_1;

					break;
				case SetHomeState_1:
					if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 1)
					{
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
						SethomeMode = SetHomeState_2;
					}
					break;
				case SetHomeState_2:
					if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 0)
					{
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
						Encoder_SetHome_Position();
						SethomeMode = SetHomeState_0;
						Munmunbot_State = STATE_Idle;
						ACK1Return(&UART2);
					}
				    break;
			  }

	  }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8191;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 512000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Motor_DIR_GPIO_Port, Motor_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LimitSwitch_Signal_Pin */
  GPIO_InitStruct.Pin = LimitSwitch_Signal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LimitSwitch_Signal_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor_DIR_Pin */
  GPIO_InitStruct.Pin = Motor_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Motor_DIR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

#define  HTIM_ENCODER htim1
#define  MAX_SUBPOSITION_OVERFLOW 4096
#define  MAX_ENCODER_PERIOD 8192

void EncoderVelocityAndPosition_Update()
{
	//Save Last state
	static uint32_t EncoderLastPosition = 0;
	static uint64_t EncoderLastTimestamp = 0;
	static uint32_t Velocity_Output = 0;
	//read data
	uint32_t EncoderNowPosition = HTIM_ENCODER.Instance->CNT;

	uint64_t EncoderNowTimestamp = micros();

	int32_t EncoderPositionDiff;
	uint64_t EncoderTimeDiff;

	EncoderTimeDiff = EncoderNowTimestamp - EncoderLastTimestamp;
	EncoderPositionDiff = EncoderNowPosition - EncoderLastPosition;

	//compensate overflow and underflow
	if (EncoderPositionDiff >= MAX_SUBPOSITION_OVERFLOW)
	{
		EncoderPositionDiff -= MAX_ENCODER_PERIOD;
	}
	else if (-EncoderPositionDiff >= MAX_SUBPOSITION_OVERFLOW)
	{
		EncoderPositionDiff += MAX_ENCODER_PERIOD;
	}

	//Update Position and time

	EncoderLastPosition = EncoderNowPosition;
	EncoderLastTimestamp = EncoderNowTimestamp;

	//Calculate velocity and Encoder Pos
	PositionPIDController.OutputFeedback = EncoderNowPosition;

	Velocity_Output = (EncoderPositionDiff * 1000000) / (float) EncoderTimeDiff;  /// Pulse per second

	// LPF
	VelocityPIDController.OutputFeedback = (Velocity_Output + (VelocityPIDController.OutputFeedback*249))/250.0;
}

void Encoder_SetHome_Position()
{
	HTIM_ENCODER.Instance->CNT = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		_micros += 4294967295;
	}
}

uint64_t micros()
{
	return _micros + htim2.Instance->CNT;
}

void ConverterUnitSystemStructureInit(ConverterUnitSystemStructure *CUSSvar)
{
	CUSSvar->PPR = 2048;
	CUSSvar->PPRxQEI = CUSSvar->PPR * 4;
	CUSSvar->RPMp = 10;
}

void TrajectoryGenerationStructureInit(TrajectoryGenerationStructure *TGSvar , ConverterUnitSystemStructure *CUSSvar)
{
	TGSvar->AngularAccerationMax_Setting = (0.25*(CUSSvar->PPRxQEI))/3.141;
	TGSvar->AngularVelocityMax_Setting = ((CUSSvar->PPRxQEI)*10)/(60.0);  //pps
	TGSvar->Start_Theta = 0;
	TGSvar->Mode = 0;
	TGSvar->Submode = 0;
	TGSvar->Loop_Freq = 10000;
	TGSvar->Loop_Period = 1000000/(TGSvar->Loop_Freq);
	TGSvar->BlendTimeLSPB = TGSvar->AngularVelocityMax_Setting/(TGSvar->AngularAccerationMax_Setting);
	TGSvar->Theta_min_for_LSPB = TGSvar->AngularVelocityMax_Setting*TGSvar->BlendTimeLSPB;
}

void VelocityControllerInit(PIDStructure *VCvar,TrajectoryGenerationStructure *TGSvar)
{
	VCvar->Kp = 5;
	VCvar->Ki = 0.2;
	VCvar->Kd = 0.1;
	VCvar->Integral_Value = 0;
	VCvar->SamplingTime = (TGSvar->Loop_Period)/1000000.0;
}

void DisplacementControllerInit(PIDStructure *VCvar,TrajectoryGenerationStructure *TGSvar)
{
	VCvar->Kp = 5;
	VCvar->Ki = 0.2;
	VCvar->Kd = 0;
	VCvar->Integral_Value = 0;
	VCvar->SamplingTime = (TGSvar->Loop_Period)/1000000.0;
}

void TrajectoryGenerationVelocityMaxSetting(TrajectoryGenerationStructure *TGSvar , ConverterUnitSystemStructure *CUSSvar)
{
	TGSvar->AngularVelocityMax_Setting = ((CUSSvar->PPRxQEI)*(CUSSvar->RPMp))/(60.0);   ///RPM to pps
	TGSvar->BlendTimeLSPB = TGSvar->AngularVelocityMax_Setting/(TGSvar->AngularAccerationMax_Setting);
	TGSvar->Theta_min_for_LSPB = TGSvar->AngularVelocityMax_Setting*TGSvar->BlendTimeLSPB;
}

void TrajectoryGenerationPrepareDATA()
{
	if (MovingLinkMode == LMM_Set_Pos_Directly)
	  {
		  TrjStruc.Desire_Theta = (Angularpos_InputNumber*CUSSStruc.PPRxQEI/(10000.0*2.0*3.141));
		  if (TrjStruc.Desire_Theta >= CUSSStruc.PPRxQEI)
		  {
			 TrjStruc.Desire_Theta -= CUSSStruc.PPRxQEI;
		  }
		  TrjStruc.Delta_Theta = TrjStruc.Desire_Theta - TrjStruc.Start_Theta; //// No implement
		  Munmunbot_State = STATE_Calculation;
	  }

	else if (MovingLinkMode == LMM_Set_Goal_1_Station || MovingLinkMode == LMM_Set_Goal_n_Station )
	  {
		  if (NumberOfStationToGo == 0)
			{
				Munmunbot_State = STATE_Idle;
				NumberOfStationPTR = 0;
				NumberOfStationToGo = 0;
				MovingLinkMode = LMM_Not_Set;
				ACK2Return(&UART2);
			}
		  else
		  {
			Current_Station = Angularpos_InputArray[NumberOfStationPTR];
			if (Current_Station > 10)
			{
				NumberOfStationPTR += 1;
				NumberOfStationToGo -= 1;
			}
			else
			{
				TrjStruc.Desire_Theta = (StationPos[Current_Station-1]*CUSSStruc.PPRxQEI/(360.0));
				if (TrjStruc.Desire_Theta >= CUSSStruc.PPRxQEI)
				{
					TrjStruc.Desire_Theta -= CUSSStruc.PPRxQEI;
				}
				TrjStruc.Delta_Theta = TrjStruc.Desire_Theta - TrjStruc.Start_Theta; //// No implement
				Munmunbot_State = STATE_Calculation;

				NumberOfStationPTR += 1;
				NumberOfStationToGo -= 1;
			}

		  }
	  }
	  else
	  {
		MovingLinkMode = LMM_Not_Set;
		Munmunbot_State = STATE_Idle;
	  }
}

void TrajectoryGenerationCalculation()
{
	if (TrjStruc.Delta_Theta < 0)
	  {
		 TrjStruc.AngularAcceration = TrjStruc.AngularAccerationMax_Setting * -1;
		 TrjStruc.AngularVelocity = TrjStruc.AngularVelocityMax_Setting *-1;
		 TrjStruc.Abs_Delta_Theta = TrjStruc.Delta_Theta * -1;
	  }
	  else if (TrjStruc.Delta_Theta > 0)
	  {
		 TrjStruc.AngularAcceration = TrjStruc.AngularAccerationMax_Setting;
		 TrjStruc.AngularVelocity = TrjStruc.AngularVelocityMax_Setting;
		 TrjStruc.Abs_Delta_Theta = TrjStruc.Delta_Theta;
	  }
	  if (TrjStruc.Abs_Delta_Theta < TrjStruc.Theta_min_for_LSPB)   ///Triangular mode0
	  {
		 TrjStruc.BlendTimeTriangular = sqrt(TrjStruc.Abs_Delta_Theta/TrjStruc.AngularAccerationMax_Setting);
		 TrjStruc.Theta_Stamp_0 = TrjStruc.Start_Theta;
		 TrjStruc.Theta_Stamp_1 = ((TrjStruc.AngularAcceration*(TrjStruc.BlendTimeTriangular*TrjStruc.BlendTimeTriangular))/2.0) + TrjStruc.Theta_Stamp_0;
		 TrjStruc.Mode = 0;
		 TrjStruc.Submode = 0;
	  }

	  else if (TrjStruc.Abs_Delta_Theta >= TrjStruc.Theta_min_for_LSPB)  ///LSPB mode1
	  {
		  TrjStruc.LinearTimeLSPB = (TrjStruc.Abs_Delta_Theta-TrjStruc.Theta_min_for_LSPB)/TrjStruc.AngularVelocityMax_Setting;
		  TrjStruc.Theta_Stamp_0 = TrjStruc.Start_Theta;
		  TrjStruc.Theta_Stamp_1 = ((TrjStruc.AngularAcceration*(TrjStruc.BlendTimeLSPB*TrjStruc.BlendTimeLSPB))/2.0) + TrjStruc.Theta_Stamp_0;
		  TrjStruc.Theta_Stamp_2 = (TrjStruc.AngularVelocity*TrjStruc.LinearTimeLSPB) + TrjStruc.Theta_Stamp_1;
		  TrjStruc.Mode = 1;
		  TrjStruc.Submode = 0;
	  }
	 TrjStruc.Equation_Timestamp = micros();
	 TrjStruc.Loop_Timestamp = micros();
}

void TrajectoryGenerationProcess()
{

	TrjStruc.Equation_Realtime_Sec = (micros()-TrjStruc.Equation_Timestamp)/1000000.0;

	 switch (TrjStruc.Mode)
	  {
		  case 0: ///Triangular
			  if (TrjStruc.Submode == 0)
			  {
				  TrjStruc.AngularDisplacementDesire =
						  ((TrjStruc.AngularAcceration*0.5)*(TrjStruc.Equation_Realtime_Sec*TrjStruc.Equation_Realtime_Sec))
						  +TrjStruc.Theta_Stamp_0;

				  if (micros()-TrjStruc.Equation_Timestamp >= TrjStruc.BlendTimeTriangular*1000000)
				  {
					  TrjStruc.Equation_Timestamp = micros();
					  TrjStruc.Submode = 1;
				  }
			  }
			  else if (TrjStruc.Submode == 1)
			  {
				  TrjStruc.AngularDisplacementDesire =
						  ((TrjStruc.AngularAcceration*-0.5)*(TrjStruc.Equation_Realtime_Sec*TrjStruc.Equation_Realtime_Sec))
						  + (TrjStruc.AngularAcceration*TrjStruc.BlendTimeTriangular*(TrjStruc.Equation_Realtime_Sec))
						  + TrjStruc.Theta_Stamp_1;
				  if (micros()-TrjStruc.Equation_Timestamp >= TrjStruc.BlendTimeTriangular*1000000)
				  {
					  TrjStruc.Equation_Timestamp = micros();
					  TrjStruc.Submode = 0;
					  TrjStruc.Mode = 2; ///Final Value Mode
				  }
			  }
			  break;

		  case 1: ///LSPB
			  if (TrjStruc.Submode == 0)
			  {
				  TrjStruc.AngularDisplacementDesire =
							((TrjStruc.AngularAcceration*0.5)*(TrjStruc.Equation_Realtime_Sec*TrjStruc.Equation_Realtime_Sec))
							+TrjStruc.Theta_Stamp_0;

				  if (micros()-TrjStruc.Equation_Timestamp >= TrjStruc.BlendTimeLSPB*1000000)
				  {
					  TrjStruc.Equation_Timestamp = micros();
					  TrjStruc.Submode = 1;
				  }
			  }
			  else if (TrjStruc.Submode == 1)
			  {
				  TrjStruc.AngularDisplacementDesire =
						  (TrjStruc.AngularVelocity*(TrjStruc.Equation_Realtime_Sec))
						  +TrjStruc.Theta_Stamp_1;
				  if (micros()-TrjStruc.Equation_Timestamp >= TrjStruc.LinearTimeLSPB*1000000)
				  {
					  TrjStruc.Equation_Timestamp = micros();
					  TrjStruc.Submode = 2;
				  }
			  }
			  else if (TrjStruc.Submode == 2)
			  {
				  TrjStruc.AngularDisplacementDesire =
						  ((TrjStruc.AngularAcceration*-0.5)*(TrjStruc.Equation_Realtime_Sec*TrjStruc.Equation_Realtime_Sec))
						  + (TrjStruc.AngularVelocity*(TrjStruc.Equation_Realtime_Sec))
						  + TrjStruc.Theta_Stamp_2;
				  if (micros()-TrjStruc.Equation_Timestamp >= TrjStruc.BlendTimeLSPB*1000000)
				  {
					  TrjStruc.Equation_Timestamp = micros();
					  TrjStruc.Submode = 0;
					  TrjStruc.Mode = 2; ///Final Value Mode
				  }
			  }
			  break;
		  case 2:
			  Moving_Link_Task_Flag = 1;
			  TrjStruc.AngularDisplacementDesire = TrjStruc.Desire_Theta;
			  break;
		  }
}

void PIDController2in1()
{
	PositionPIDController.OutputDesire = TrjStruc.AngularDisplacementDesire;
    PositionPIDController.NowError = PositionPIDController.OutputFeedback-PositionPIDController.OutputDesire;
    PositionPIDController.Integral_Value += PositionPIDController.NowError*PositionPIDController.SamplingTime;
    PositionPIDController.ControllerOutput = (PositionPIDController.Kp*PositionPIDController.NowError)
					  +(PositionPIDController.Ki * PositionPIDController.Integral_Value)
					  +(PositionPIDController.Kd * (PositionPIDController.NowError-PositionPIDController.PreviousError)/PositionPIDController.SamplingTime);
    PositionPIDController.PreviousError = PositionPIDController.NowError;

//    VelocityPIDController.OutputDesire = PositionPIDController.ControllerOutput;
//    VelocityPIDController.NowError = VelocityPIDController.OutputFeedback-VelocityPIDController.OutputDesire;
//    VelocityPIDController.Integral_Value += VelocityPIDController.NowError*VelocityPIDController.SamplingTime;
//    VelocityPIDController.ControllerOutput = (VelocityPIDController.Kp*VelocityPIDController.NowError)
//					  +(VelocityPIDController.Ki * VelocityPIDController.Integral_Value)
//					  +(VelocityPIDController.Kd * (VelocityPIDController.NowError-VelocityPIDController.PreviousError)/VelocityPIDController.SamplingTime);
//    VelocityPIDController.PreviousError = VelocityPIDController.NowError;

}



///UART ZONE
void UARTInit(UARTStucrture *uart)
{
	//dynamic memory allocate
	uart->RxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.RxLen);
	uart->TxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.TxLen);
	uart->RxTail = 0;
	uart->TxTail = 0;
	uart->TxHead = 0;
}

void UARTResetStart(UARTStucrture *uart)
{
	HAL_UART_Receive_DMA(uart->huart, uart->RxBuffer, uart->RxLen);
}

uint32_t UARTGetRxHead(UARTStucrture *uart)
{
	return uart->RxLen - __HAL_DMA_GET_COUNTER(uart->huart->hdmarx);
}

int16_t UARTReadChar(UARTStucrture *uart)
{
	int16_t Result = -1; // -1 Mean no new data

	//check Buffer Position
	if (uart->RxTail != UARTGetRxHead(uart))
	{
		//get data from buffer
		Result = uart->RxBuffer[uart->RxTail];
		uart->RxTail = (uart->RxTail + 1) % uart->RxLen;

	}
	return Result;
}

void UARTTxDumpBuffer(UARTStucrture *uart)
{
	static uint8_t MultiProcessBlocker = 0;

	if (uart->huart->gState == HAL_UART_STATE_READY && !MultiProcessBlocker)
	{
		MultiProcessBlocker = 1;

		if (uart->TxHead != uart->TxTail)
		{
			//find len of data in buffer (Circular buffer but do in one way)
			uint16_t sentingLen =
					uart->TxHead > uart->TxTail ?
							uart->TxHead - uart->TxTail :
							uart->TxLen - uart->TxTail;

			//sent data via DMA
			HAL_UART_Transmit_DMA(uart->huart, &(uart->TxBuffer[uart->TxTail]),
					sentingLen);
			//move tail to new position
			uart->TxTail = (uart->TxTail + sentingLen) % uart->TxLen;

		}
		MultiProcessBlocker = 0;
	}
}

void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len)
{
	//check data len is more than buffur?
	uint16_t lenAddBuffer = (len <= uart->TxLen) ? len : uart->TxLen;
	// find number of data before end of ring buffer
	uint16_t numberOfdataCanCopy =
			lenAddBuffer <= uart->TxLen - uart->TxHead ?
					lenAddBuffer : uart->TxLen - uart->TxHead;
	//copy data to the buffer
	memcpy(&(uart->TxBuffer[uart->TxHead]), pData, numberOfdataCanCopy);

	//Move Head to new position

	uart->TxHead = (uart->TxHead + lenAddBuffer) % uart->TxLen;
	//Check that we copy all data That We can?
	if (lenAddBuffer != numberOfdataCanCopy)
	{
		memcpy(uart->TxBuffer, &(pData[numberOfdataCanCopy]),
				lenAddBuffer - numberOfdataCanCopy);
	}
	UARTTxDumpBuffer(uart);

}

void ACK1Return(UARTStucrture *uart)
{
	{
	uint8_t temp[] = {0x58, 0b01110101};
	UARTTxWrite(uart, temp, 2);
	}
}

void ACK2Return(UARTStucrture *uart)
{
	{
	uint8_t temp[] = {70, 110};
	UARTTxWrite(uart, temp, 2);
	}
}

void DEBUG_UART(UARTStucrture *uart)
{
	{
	uint8_t temp[] = {0xff, 0xff};
	UARTTxWrite(uart, temp, 2);
	}
}

void Munmunbot_Protocol(int16_t dataIn,UARTStucrture *uart)
{
	//static PPSTATE Munmunbot_Protocol_State = PP_STARTandMode;
	static uint16_t n_station = 0;
	static uint16_t n_station_mem= 0;
	static uint8_t ProtocolMode = 0;
	static uint8_t parameter[256] = {0};
	static uint8_t parameter_ptr = 0;
	static uint16_t Data_HAck = 0;
	static uint32_t CheckSum = 0;
	static uint16_t DataForReturn = 0;

	switch (Munmunbot_Protocol_State)
	{
		case PP_STARTandMode:
			if (((dataIn>>4) & 0b1111) == 0b1001)
			{
				CheckSum = dataIn;
				ProtocolMode = dataIn & 0b1111;

				if (ProtocolMode == 7)
				{
					Munmunbot_Protocol_State = PP_Frame3_Data_0; ///Frame3
				}
				else if (ProtocolMode == 1 || ProtocolMode == 4 || ProtocolMode == 5 ||ProtocolMode == 6)
				{
					Munmunbot_Protocol_State = PP_Frame2_Data_0; //Frame2
				}
				else if (ProtocolMode == 2 || ProtocolMode == 3 || ProtocolMode == 8 || ProtocolMode == 9 || ProtocolMode == 10 ||
						ProtocolMode == 11 ||ProtocolMode == 12 ||ProtocolMode == 13 || ProtocolMode == 14 )
				{
					Munmunbot_Protocol_State = PP_CheckSum;   /// Frame1
				}
			}
			break;
		case PP_Frame2_Data_0:
			 CheckSum += dataIn;
			 Data_HAck = ((dataIn&0b11111111)<<8)&0b1111111100000000;
			 parameter[0] = dataIn&0b1111;
			 parameter[1] = (dataIn>>4)&0b1111;
			 Munmunbot_Protocol_State = PP_Frame2_Data_1;

			 break;
		case PP_Frame2_Data_1:
			 CheckSum += dataIn;
			 Data_HAck = (dataIn&0b11111111) | Data_HAck;
			 parameter[2] = dataIn&0b1111;
			 parameter[3] = (dataIn>>4)&0b1111;
			 Munmunbot_Protocol_State = PP_CheckSum;
			 break;

		case PP_Frame3_Data_0:
		     CheckSum += dataIn;
		     n_station = dataIn;
		     n_station_mem = n_station;
		     Munmunbot_Protocol_State = PP_Frame3_Data_1;

		   break;

		case PP_Frame3_Data_1:
				CheckSum += dataIn;
				if (n_station > 2)
				{
					parameter[parameter_ptr] = dataIn&0b1111;
					parameter_ptr += 1;
					parameter[parameter_ptr] = (dataIn>>4)&0b1111;
					parameter_ptr += 1;
					n_station -= 2;
				}
				else if (n_station == 1)
				{
					parameter[parameter_ptr] = dataIn&0b1111;
					n_station -= 1;
				}
				if  (n_station == 0)
				{
					Munmunbot_Protocol_State = PP_CheckSum;
				}
				break;

			case PP_CheckSum:
			{
				CheckSum = (~CheckSum) & 0xff;
				if (CheckSum == dataIn)
				{

					switch (ProtocolMode)
					{
					case 1: ///Test Command ##Complete##
						{
						uint8_t temp[] =
						{0b10010001,
						((parameter[1] & 0xff) << 4)  | (parameter[0]& 0xff),
						((parameter[3] & 0xff) << 4)  | (parameter[2]& 0xff),
						0b0 , 0x58 ,0x75 };
						temp[3] = (~(temp[0]+temp[1]+temp[2]))& 0xff;
						UARTTxWrite(uart, temp, 6);
						}
						break;
					case 2: //Connect MCU ##Complete##
						if (Munmunbot_State == STATE_Disconnected)
						{
							Munmunbot_State = STATE_Idle;
						}
						ACK1Return(uart);
						break;
					case 3: //Disconnect MCU ##Complete##
						if (Munmunbot_State == STATE_Idle)
						{
							Munmunbot_State = STATE_Disconnected;
						}
						ACK1Return(uart);
						break;
					case 4: //Set Angular Velocity ##Complete##
						if (Munmunbot_State == STATE_Idle)
						{
							CUSSStruc.RPMp = Data_HAck;
							TrajectoryGenerationVelocityMaxSetting(&TrjStruc , &CUSSStruc);
						}
						ACK1Return(uart);
						break;
					case 5:   //Set Angular pos ##Complete##
						if (Munmunbot_State == STATE_Idle)
						{
							Angularpos_InputNumber = Data_HAck;
							MovingLinkMode = LMM_Set_Pos_Directly;
						}
						ACK1Return(uart);
						break;
					case 6:  /// Set 1 Station ##Complete##
						if (Munmunbot_State == STATE_Idle)
						{
//							Angularpos_InputArray[0] = parameter[0];
							Angularpos_InputArray[0] = parameter[2];    //150 0x00 0x0A Checksum
							MovingLinkMode = LMM_Set_Goal_1_Station;
							NumberOfStationToGo = 1;
						}
						ACK1Return(uart);
						break;
					case 7:  /// Set n Station ##Complete##
						if (Munmunbot_State == STATE_Idle)
						{
							MovingLinkMode = LMM_Set_Goal_n_Station;
							for (int i = 0; i < n_station_mem; i++)
							{
								Angularpos_InputArray[i] = parameter[i];
							}
							NumberOfStationToGo = n_station_mem;
							ACK1Return(uart);
						}
						else
						{
							ACK1Return(uart);
						}
						break;
					case 8:  /// Go go ##Complete##  ///But must implement return ACK after it's done
						if (Munmunbot_State == STATE_Idle)
						{
							Munmunbot_State = STATE_PrepareDATA;
							ACK1Return(uart);
						}
						else
						{
							{
								uint8_t temp[] =
								{0x58 ,  0x75 ,
										70,  110};  ///ACK1 + ACK2
								UARTTxWrite(uart, temp, 4);
							}
						}
						break;

					case 9:  /// Return Current Station   ##Complete##

						{
							uint8_t temp[] =
							{0x58 ,  0x75 ,
									153,  0b0,  0b0, 0b0}; ///ACK1 + Mode 9
							uint8_t Shift = 2;
							DataForReturn = Current_Station&(0xff);
							temp[1+Shift] = (DataForReturn>>8)&(0xff);
							temp[2+Shift] = (DataForReturn)&(0xff);
							temp[3+Shift] = ~(temp[0+Shift]+temp[1+Shift]+temp[2+Shift]);
							UARTTxWrite(uart, temp, 4+Shift);
						}

						break;

					case 10: /// Return Angular Position  ##Complete##
						{
							uint8_t temp[] =
							{0x58 , 0x75 ,154, 0b0,  0b0, 0b0};
							uint8_t Shift = 2;
							DataForReturn = (PositionPIDController.OutputFeedback*2*3.141*10000)/(CUSSStruc.PPRxQEI);  ///pulse to (radian*10000)
							temp[1+Shift] = (DataForReturn>>8)&(0xff);
							temp[2+Shift] = (DataForReturn)&(0xff);
							temp[3+Shift] = ~(temp[0+Shift]+temp[1+Shift]+temp[2+Shift]);
							UARTTxWrite(uart, temp, 4+Shift);
						}

						break;

					case 11: /// Return Angular Velocity Max  ##Complete##
							{
								uint8_t temp[] =
								{0x58 , 0x75 ,155, 0b0,  0b0, 0b0};
								uint8_t Shift = 2;
								DataForReturn = (TrjStruc.AngularVelocityMax_Setting*60)/(CUSSStruc.PPRxQEI);  ///pps to RPM
								temp[1+Shift] = (DataForReturn>>8)&(0xff);
								temp[2+Shift] = (DataForReturn)&(0xff);
								temp[3+Shift] = ~(temp[0+Shift]+temp[1+Shift]+temp[2+Shift]);
								UARTTxWrite(uart, temp, 4+Shift);
							}
						break;

					case 12:
						if (Munmunbot_State == STATE_Idle)
						{

						}
						ACK1Return(uart);
						break;
					case 13:
						if (Munmunbot_State == STATE_Idle)
						{

						}
						ACK1Return(uart);
						break;

					case 14: /// Sethome  ##Complete##
						if (Munmunbot_State == STATE_Idle)
						{
							Munmunbot_State = STATE_SetHome;
							SethomeMode = SetHomeState_0;
						}
						ACK1Return(uart);
						break;
				    }
			   }
			n_station = 0;
			ProtocolMode = 0;
			parameter_ptr = 0;
			Data_HAck = 0;
			CheckSum = 0;
			Munmunbot_Protocol_State = PP_STARTandMode;
			break;
			}
	}
}


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

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

/* USER CODE BEGIN PV */

uint64_t _micros = 0;
float EncoderVel = 0;
uint64_t Timestamp_Encoder = 0;





typedef enum
{
	STATE_Setting,    /// Set_station_position
	STATE_Idle,
	STATE_Calculation,
	STATE_Link_Moving,
	STATE_End_Effector_Working
} Robot_STATE;

Robot_STATE Munmunbot_State = STATE_Setting;

typedef struct _TrajectoryGenerationStructure
{
	uint32_t BlendTimeLSPB;
	uint32_t BlendTimeTriangular;
	uint32_t LinearTimeLSPB;

	uint32_t Theta_min_for_LSPB;
	uint32_t AngularVelocityMax_Setting;
	uint32_t AngularAccerationMax_Setting;

	uint32_t AngularVelocity;
	uint32_t AngularAcceration;
	uint32_t AngularDisplacementDesire;

	uint32_t Theta_Stamp_0;
	uint32_t Theta_Stamp_1;
	uint32_t Theta_Stamp_2;

	uint64_t Equation_Timestamp;
	uint64_t Loop_Timestamp;

	uint32_t Loop_Freq;
	uint32_t Loop_Period;

	uint32_t Desire_Theta;
	uint32_t Start_Theta;
	uint32_t Delta_Theta;
	uint32_t Abs_Delta_Theta;

	uint32_t Mode;
	uint32_t Submode;

} TrajectoryGenerationStructure;

typedef struct _ConverterUnitSystemStructure
{
	uint32_t PPR;
	uint32_t PPRxQEI;
	uint32_t RPMp;

} ConverterUnitSystemStructure;

TrajectoryGenerationStructure TrjStruc = {0};
ConverterUnitSystemStructure CUSSStruc = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

uint64_t micros();
float EncoderVelocity_Update();

void ConverterUnitSystemStructureInit(ConverterUnitSystemStructure *CUSSvar);
void TrajectoryGenerationStructureInit(TrajectoryGenerationStructure *TGSvar, ConverterUnitSystemStructure *CUSSvar);


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



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  switch (Munmunbot_State)
	  {
	  	  case STATE_Setting:
	  		  break;
	  	  case STATE_Idle:

	  		/// Get Vmax input
//	  		  if (Vmax_trigger == 1)
//			  {
//				  ///Set AngularVelomax
//				  TrjStruc.AngularVelocityMax_Setting = VelocityMax;
//				  TrjStruc.BlendTimeLSPB = TrjStruc.AngularVelocityMax_Setting/(TrjStruc.AngularAccerationMax_Setting);
//				  ///find Minimum LSPB Angular Distance
//				  TrjStruc.Theta_min_for_LSPB = TrjStruc.AngularVelocityMax_Setting*TrjStruc.BlendTimeLSPB;
//			  }

	  		 /// Get the input
//	  		  if (input[0] != 0)
//	  		  {
//	  			  TrjStruc.Start_Theta = htim1.Instance->CNT;
//	  			  TrjStruc.Desire_Theta = input[0];
//	  			  /// เลื�?อ�? Array
//	  			  TrjStruc.Delta_Theta = TrjStruc.Desire_Theta - TrjStruc.Start_Theta; //// No implement
//	  			  Munmunbot_State = STATE_Calculation;
//	  		  }
//		  	  break;

	  	  case STATE_Calculation:
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
	  		  if (TrjStruc.Abs_Delta_Theta < TrjStruc.Theta_min_for_LSPB)
	  		  {
	  			 TrjStruc.BlendTimeTriangular = sqrt(TrjStruc.Abs_Delta_Theta/TrjStruc.AngularAccerationMax_Setting);
	  			 TrjStruc.Theta_Stamp_0 = TrjStruc.Start_Theta;
				 TrjStruc.Theta_Stamp_1 = ((TrjStruc.AngularAcceration*(TrjStruc.BlendTimeTriangular)^2)/2.0) + TrjStruc.Theta_Stamp_0;
				 TrjStruc.Mode = 1;
				 TrjStruc.Submode = 0;
	  		  }

	  		  else if (TrjStruc.Abs_Delta_Theta >= TrjStruc.Theta_min_for_LSPB)
	  		  {
	  			  TrjStruc.LinearTimeLSPB = (TrjStruc.Abs_Delta_Theta-TrjStruc.Theta_min_for_LSPB)/TrjStruc.AngularVelocityMax_Setting;
	  			  TrjStruc.Theta_Stamp_0 = TrjStruc.Start_Theta;
	  			  TrjStruc.Theta_Stamp_1 = ((TrjStruc.AngularAcceration*(TrjStruc.Theta_min_for_LSPB)^2)/2.0) + TrjStruc.Theta_Stamp_0;
	  			  TrjStruc.Theta_Stamp_2 = (TrjStruc.AngularVelocity*TrjStruc.LinearTimeLSPB) + TrjStruc.Theta_Stamp_1;
	  			  TrjStruc.Mode = 0;
	  			  TrjStruc.Submode = 0;
	  		  }
	  		 Munmunbot_State = STATE_Link_Moving;
	  		 TrjStruc.Equation_Timestamp = micros();
	  		 TrjStruc.Loop_Timestamp = micros();
	  		  break;

	   	  case STATE_Link_Moving:
	   		  if (micros()-TrjStruc.Loop_Timestamp >=  TrjStruc.Loop_Period)
	   		  {
	   			  switch (TrjStruc.Mode)
	   			  {
					  case 0: ///LSPB
						  if (TrjStruc.Submode == 0)
						  {
							  TrjStruc.AngularDisplacementDesire =
									  ((TrjStruc.AngularAcceration*0.5)*((micros()-TrjStruc.Equation_Timestamp)^2))+TrjStruc.Theta_Stamp_0;
							  if (micros()-TrjStruc.Equation_Timestamp >= TrjStruc.BlendTimeTriangular)
							  {
								  TrjStruc.Equation_Timestamp = micros();
								  TrjStruc.Mode = 2;
							  }
						  }
						  else if (TrjStruc.Submode == 1)
						  {
							  TrjStruc.AngularDisplacementDesire =
									  ((TrjStruc.AngularAcceration*-0.5)*((micros()-TrjStruc.Equation_Timestamp)^2))
									  + (TrjStruc.AngularAcceration*TrjStruc.BlendTimeTriangular*(micros()-TrjStruc.Equation_Timestamp))
									  + TrjStruc.Theta_Stamp_1;
							  if (micros()-TrjStruc.Equation_Timestamp >= TrjStruc.BlendTimeTriangular)
							  {
								  TrjStruc.Equation_Timestamp = micros();
								  TrjStruc.Mode = 2; ///unkownmode
							  }
						  }
						  break;

					  case 1: ///Triangular
						  if (TrjStruc.Submode == 0)
						  {
							  TrjStruc.AngularDisplacementDesire =
										((TrjStruc.AngularAcceration*0.5)*((micros()-TrjStruc.Equation_Timestamp)^2))+TrjStruc.Theta_Stamp_0;
							  if (micros()-TrjStruc.Equation_Timestamp >= TrjStruc.BlendTimeLSPB)
							  {
								  TrjStruc.Equation_Timestamp = micros();
								  TrjStruc.Mode = 2;
							  }
						  }
						  else if (TrjStruc.Submode == 1)
						  {
							  TrjStruc.AngularDisplacementDesire =
									  (TrjStruc.AngularVelocity*(micros()-TrjStruc.Equation_Timestamp))+TrjStruc.Theta_Stamp_1;
							  if (micros()-TrjStruc.Equation_Timestamp >= TrjStruc.LinearTimeLSPB)
							  {
								  TrjStruc.Equation_Timestamp = micros();
								  TrjStruc.Mode = 2;
							  }
						  }
						  else if (TrjStruc.Submode == 2)
						  {
							  TrjStruc.AngularDisplacementDesire =
									  ((TrjStruc.AngularAcceration*-0.5)*((micros()-TrjStruc.Equation_Timestamp)^2))
									  + (TrjStruc.AngularVelocity*(micros()-TrjStruc.Equation_Timestamp))
									  + TrjStruc.Theta_Stamp_2;
							  if (micros()-TrjStruc.Equation_Timestamp >= TrjStruc.BlendTimeLSPB)
							  {
								  TrjStruc.Equation_Timestamp = micros();
								  TrjStruc.Mode = 2; ///unkownmode
							  }
						  }
						  break;
					  case 2:
						  TrjStruc.AngularDisplacementDesire = TrjStruc.Desire_Theta;
						  break;
	   			  }
	   		  }




	   		  /// PID Implement


	  		  break;
	  	  case STATE_End_Effector_Working:
	  		  break;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
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

}

/* USER CODE BEGIN 4 */

#define  HTIM_ENCODER htim1
#define  MAX_SUBPOSITION_OVERFLOW 4096
#define  MAX_ENCODER_PERIOD 8192

float EncoderVelocity_Update()
{
	//Save Last state
	static uint32_t EncoderLastPosition = 0;
	static uint64_t EncoderLastTimestamp = 0;

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

	//Calculate velocity
	//EncoderTimeDiff is in uS
	return (EncoderPositionDiff * 1000000) / (float) EncoderTimeDiff;

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
	CUSSvar->RPMp = 255;
}

void TrajectoryGenerationStructureInit(TrajectoryGenerationStructure *TGSvar , ConverterUnitSystemStructure *CUSSvar)
{
	TGSvar->AngularAccerationMax_Setting = (0.25*(CUSSvar->PPRxQEI))/3.141;
	TGSvar->AngularVelocityMax_Setting = ((CUSSvar->PPRxQEI)*(CUSSvar->RPMp)*10)/(255.0*60.0);
	TGSvar->Start_Theta = 0;
	TGSvar->Mode = 0;
	TGSvar->Submode = 0;
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

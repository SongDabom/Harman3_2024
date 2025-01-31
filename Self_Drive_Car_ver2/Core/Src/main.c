/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "C:\Users\SAMSUNG\STM32CubeIDE\Common\myLib.h"
#include "delay.h"
#include "bitmap.h"
#include "oled.h"
#include "spi.h"

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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char buf1[100], dum1;
int idx1 = 0;
char dum[20];
int num1 = 0;
int num2 = 0;
int num3 = 0;
int num4 = 0;
int num5 = 0;

int Front_Dist = 15;
int Side_Dist = 15;
int Digonal_Dist = 13;
int Max_Speed_Dist = 20;

int MaxSpeed = 50000;
int speed = 0;
int cnt = 0;

unsigned int handle_flag = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if(huart == &huart1)  // BT input
   {

		if(strncmp(buf1,"A0",2) == 0)      //string , if you input "FAST" on phone (3 = 3 character)
		{
			handle_flag = 1;
			htim1.Instance->CCR2 = 0;
			htim3.Instance->CCR1 = 0;

		}
		if(strncmp(buf1,"P0",2) == 0)      //string , if you input "FAST" on phone (3 = 3 character)
		{
			handle_flag = 0;
		}
		if(handle_flag == 1)
		{
			if(strncmp(buf1,"F0",2) == 0)      //string , if you input "FAST" on phone (3 = 3 character)
					{
					  // forward
					  HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, 0);
					  HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 1);

					  HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, 1);
					  HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, 0);
					  htim1.Instance->CCR2 = speed;
					  htim3.Instance->CCR1 = speed;

					}
					if(strncmp(buf1,"B0",2) == 0)      //string , if you input "FAST" on phone (3 = 3 character)
					{
					  HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, 1);
					  HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 0);

					  HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, 0);
					  HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, 1);

					  htim1.Instance->CCR2 = speed;
					  htim3.Instance->CCR1 = speed;

					}
					if(strncmp(buf1,"L0",2) == 0)      //string , if you input "FAST" on phone (3 = 3 character)
					{
						  htim1.Instance->CCR2 = 0;
						  htim3.Instance->CCR1 = speed;


					}
					if(strncmp(buf1,"R0",2) == 0)      //string , if you input "FAST" on phone (3 = 3 character)
					{

						  htim1.Instance->CCR2 = speed;
						  htim3.Instance->CCR1 = 0;
					}
		}

		if(strncmp(buf1,"T0",2) == 0)
		{
		   if(speed == MaxSpeed) speed = MaxSpeed;
		   else speed += 1000;
		   SSD1306_Clear();
		   SSD1306_GotoXY(65,30);
		   sprintf(dum, "%d", speed);
		   SSD1306_Puts(dum, &Font_11x18, 1);
		   SSD1306_UpdateScreen();
		}
		if(strncmp(buf1,"X0",2) == 0)
		{
		   if(speed == 0) speed = 0;
		   else speed -= 1000;
		   //SSD1306_Clear();
		   SSD1306_Clear();

		   SSD1306_GotoXY(65,30);
		   sprintf(dum, "%d", speed);
		   SSD1306_Puts(dum, &Font_11x18, 1);
		   SSD1306_UpdateScreen();
}

      buf1[idx1++] = dum1;
      HAL_UART_Receive_IT(&huart1, &dum1, 1);

   }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(idx1)
	   {
		  buf1[idx1] = 0;
		  printf("UART1>%s\r\n",buf1);
		  idx1 = 0;
	   }

}

void SelfDrive_Mode()
{
	num1 = usDist1() * 100;
	num2 = usDist2() * 100;
	num3 = usDist3() * 100;
	num4 = usDist4() * 100;
	num5 = usDist5() * 100;

	while(num1 <= 0) num1 = usDist1() * 100;
	while(num2 <= 0) num2 = usDist2() * 100;
	while(num3 <= 0) num3 = usDist3() * 100;
	while(num4 <= 0) num4 = usDist4() * 100;
	while(num5 <= 0) num5 = usDist5() * 100;
	//printf("num1 : %d  num2 : %d  num3 : %d  num4 : %d  num5 : %d\r\n", num1, num2, num3, num4, num5);
	//printf("%d	%d\r\n", abs(num4-num5), abs(num2-num3));

	if(num1 >= Front_Dist)
	{

		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, 0);
		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 1);
		//LEFT FRONT
		HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, 1);
		HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, 0);

		if(num2 > Digonal_Dist && num3 > Digonal_Dist)
		{
			htim1.Instance->CCR2 = speed;
			htim3.Instance->CCR1 = speed;
		}
		else if(num2 <= Digonal_Dist && num3 > Digonal_Dist)
		{
			htim1.Instance->CCR2 = speed;
			htim3.Instance->CCR1 = 0;
		}
		else if(num3 <= Digonal_Dist && num2 > Digonal_Dist)
		{
			htim1.Instance->CCR2 = 0;
			htim3.Instance->CCR1 = speed;
		}
	}
	else if(num1 < Front_Dist)
	{
		cnt++;

		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, 1);
		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 0);

		HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, 0);
		HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, 1);

		//LEFT FRONT

		htim1.Instance->CCR2 = speed;
		htim3.Instance->CCR1 = speed;
		HAL_Delay(100);

		if(cnt == 10);
		{
			cnt = 0;
			if(num4 > num5)
			{
				htim1.Instance->CCR2 = speed;
				htim3.Instance->CCR1 = 0;
			}
			else if(num5 > num4)
			{
				htim1.Instance->CCR2 = 0;
				htim3.Instance->CCR1 = speed;
			}
			HAL_Delay(1500);
		}
	}


}
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  ProgramStart();

  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, 0);
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, 1);

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  HAL_UART_RxCpltCallback(&huart1);
  HAL_TIM_PeriodElapsedCallback(&htim2);

  SSD1306_Init();
  SSD1306_Clear();

  SSD1306_GotoXY(65,30);
  sprintf(dum, "%d", speed);
  SSD1306_Puts(dum, &Font_11x18, 1);
  SSD1306_UpdateScreen();

  //Left Motor Forward
  HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, 0);
  HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 1);
  //Right Motor Forward
  HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, 0);
  HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_RxCpltCallback(&huart1);
	  HAL_TIM_PeriodElapsedCallback(&htim2);

	  if(handle_flag == 0)
	  {
		  SelfDrive_Mode();
		  SSD1306_GotoXY(5,10);
		  SSD1306_Puts("AUTO: ", &Font_11x18, 1);
		  SSD1306_UpdateScreen();
	  }
	  else
	  {
		  SSD1306_GotoXY(1,10);
		  SSD1306_Puts("MANUAL: ", &Font_11x18, 1);
		  SSD1306_UpdateScreen();
		  HAL_UART_RxCpltCallback(&huart1);
		  HAL_TIM_PeriodElapsedCallback(&htim2);
	  }
		/*
		if((abs(num2 - num3) < Digonal_Dist) || (abs(num4 - num5) < Side_Dist))
		{
			// Motor Speed
			htim1.Instance->CCR2 = speed;
			htim3.Instance->CCR1 = speed;
		}
		else if(num2 < num3)
		{

			htim1.Instance->CCR2 = TurnSpeed;
			htim3.Instance->CCR1 = 0;

		}
		else if(num3 < num2)
		{
			htim1.Instance->CCR2 = 0;
			htim3.Instance->CCR1 = TurnSpeed;

		}

		else if(num4 < num5)
		{
			htim1.Instance->CCR2 = TurnSpeed;
			htim3.Instance->CCR1 = 0;
		}
		else if(num5 < num4)
		{
			htim1.Instance->CCR2 = 0;
			htim3.Instance->CCR1 = TurnSpeed;
		}

*/

		/*
		else if(num1 < 13)
		{
			//Left Motor Reverse
			HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, 1);
			HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, 0);
			//Right Motor Reverse
			HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, 1);
			HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, 0);

			if(num4 < num5)
			{
				htim1.Instance->CCR2 = 0;
				htim3.Instance->CCR1 = TurnSpeed;
			}
			else if(num5 < num4)
			{
				htim1.Instance->CCR2 = TurnSpeed;
				htim3.Instance->CCR1 = 0;
			}
		}*/



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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 84-1;
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
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, A0_Pin|TRIG5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|D7_Pin|D2_Pin|TRIG4_Pin
                          |RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TRIG3_Pin|D6_Pin|TRIG1_Pin|TRIG2_Pin
                          |D3_Pin|D5_Pin|D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A0_Pin TRIG5_Pin */
  GPIO_InitStruct.Pin = A0_Pin|TRIG5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin D7_Pin D2_Pin TRIG4_Pin
                           RST_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|D7_Pin|D2_Pin|TRIG4_Pin
                          |RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ECHO1_Pin ECHO5_Pin */
  GPIO_InitStruct.Pin = ECHO1_Pin|ECHO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ECHO3_Pin ECHO4_Pin ECHO2_Pin */
  GPIO_InitStruct.Pin = ECHO3_Pin|ECHO4_Pin|ECHO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG3_Pin D6_Pin TRIG1_Pin TRIG2_Pin
                           D3_Pin D5_Pin D4_Pin */
  GPIO_InitStruct.Pin = TRIG3_Pin|D6_Pin|TRIG1_Pin|TRIG2_Pin
                          |D3_Pin|D5_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

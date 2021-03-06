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
#include "xprintf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define motor_pr 1440.0 //motor P/R
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
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float pwm[4] = { 0 };
void Count2rpm() {
	pwm[0] = (((float) TIM1->CNT - 30000.0) * 600.0) / motor_pr;
	pwm[1] = (((float) TIM2->CNT - 30000.0) * 600.0) / motor_pr;
	pwm[2] = (((float) TIM3->CNT - 30000.0) * 600.0) / motor_pr;
	pwm[3] = (((float) TIM4->CNT - 30000.0) * 600.0) / motor_pr;
	TIM1->CNT = 30000;
	TIM2->CNT = 30000;
	TIM3->CNT = 30000;
	TIM4->CNT = 30000;
}

void uart_putc(uint8_t c) {
	char buf[1];
	buf[0] = c;
	HAL_UART_Transmit(&huart2, (uint8_t*) buf, sizeof(buf), 0xFFFF);
}
void uart_puts(char *str) {
	while (*str) {
		uart_putc(*str++);
	}
}

TIM_Encoder_InitTypeDef sConfig1;
TIM_Encoder_InitTypeDef sConfig2;
TIM_Encoder_InitTypeDef sConfig3;
TIM_Encoder_InitTypeDef sConfig4;
TIM_MasterConfigTypeDef sMasterConfig1;
TIM_MasterConfigTypeDef sMasterConfig2;
TIM_MasterConfigTypeDef sMasterConfig3;
TIM_MasterConfigTypeDef sMasterConfig4;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	xdev_out(uart_putc);

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
	MX_TIM3_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_TIM10_Init();
	MX_USART6_UART_Init();
	/* USER CODE BEGIN 2 */
	uint8_t buf[5];
	uint16_t tx_rpm[4] = { 500, 500, 500, 500 };
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig1) != HAL_OK) {
		Error_Handler();
	}
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	TIM1->CNT = 30000;

	if (HAL_TIM_Encoder_Init(&htim2, &sConfig2) != HAL_OK) {
		Error_Handler();
	}
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	TIM2->CNT = 30000;

	if (HAL_TIM_Encoder_Init(&htim3, &sConfig3) != HAL_OK) {
		Error_Handler();
	}
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	TIM3->CNT = 30000;

	if (HAL_TIM_Encoder_Init(&htim4, &sConfig4) != HAL_OK) {
		Error_Handler();
	}
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	TIM4->CNT = 30000;
	HAL_TIM_Base_Start_IT(&htim10);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		for (int i = 0; i < 4; i++) {
			tx_rpm[i] = pwm[i] + 500;
		}
		buf[0] = (tx_rpm[0] & 0xFF); //0??????8???
		buf[1] = (((tx_rpm[0] >> 8) + (tx_rpm[1] << 2)) & 0xFF); //0??????2??????1??????6???
		buf[2] = (((tx_rpm[1] >> 6) + (tx_rpm[2] << 4)) & 0xFF); //1??????4??????2??????4???
		buf[3] = (((tx_rpm[2] >> 4) + (tx_rpm[3] << 6)) & 0xFF); //2??????6??????3??????2???
		buf[4] = ((tx_rpm[3] >> 2) & 0xFF);
		HAL_UART_Transmit(&huart6, buf, sizeof(buf), 0xFFFF);
		HAL_Delay(100);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
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
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	//TIM_Encoder_InitTypeDef sConfig = {0};
	//TIM_MasterConfigTypeDef sMasterConfig = {0};
	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 59999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig1.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig1.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig1.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig1.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig1.IC1Filter = 0;
	sConfig1.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig1.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig1.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig1.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig1.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig1.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig1)
			!= HAL_OK) {
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
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	//TIM_Encoder_InitTypeDef sConfig = {0};
	//TIM_MasterConfigTypeDef sMasterConfig = {0};
	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 59999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig2.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig2.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig2.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig2.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig2.IC1Filter = 0;
	sConfig2.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig2.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig2.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig2.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig2.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig2.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig2)
			!= HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 59999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig3.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig3.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig3.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig3.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig3.IC1Filter = 0;
	sConfig3.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig3.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig3.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig3.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig3.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig3.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 59999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig4.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig4.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig4.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig4.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig4.IC1Filter = 0;
	sConfig4.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig4.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig4.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig4.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig4.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig4.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 999;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 8400;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

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

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

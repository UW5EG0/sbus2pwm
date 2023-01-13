/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "fonts.h"
#include "image.h"
#include "math.h"
#include "string.h"
#include "ssd1306.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	int16_t value;
	int16_t trim;

//	uint8_t channel_bits;
}
channel_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//SBUS
#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

#define BUF_RX_LENGTH 25
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;
DMA_HandleTypeDef hdma_tim2_ch3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
channel_t channels[18];
uint16_t failsafe_status;
HAL_StatusTypeDef HAL_status;
uint8_t buf_in[BUF_RX_LENGTH];
char str[20] = {0,};
int speed = 0;
int rotary = 0;
long left , right = 0;
unsigned int packetCounter, oldPacketCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void parse_Motion()
{
	// As a distance from center stick point
	// wrong data protection
	if (failsafe_status != SBUS_SIGNAL_OK || packetCounter == 0) {
			speed = 0;
			rotary = 0;

		} else {

			speed = ((speed * 5) + (channels[0].value - channels[0].trim)*3)/8;
			if (abs(speed) < 32) {
				speed = 0;
			} else {
				speed = (speed > 0)?(speed - 32):(speed + 32);
			}
			rotary = ((rotary * 5) + (channels[1].value - channels[1].trim)*3)/8;
			if (abs(rotary) < 32) {
				rotary = 0;
						} else {
							rotary = (rotary > 0)?(rotary - 32):(rotary + 32);
						}
		}
    //mixes to gears

	left = (speed*2+rotary)*10/8;
	right = (speed*2-rotary)*10/8;
	// make config for direction
	HAL_GPIO_WritePin(EN_LEFT_GPIO_Port, EN_LEFT_Pin, !(abs(left) > 50 || abs(right) > 50) && failsafe_status == SBUS_SIGNAL_OK);
    HAL_GPIO_WritePin(EN_RIGHT_GPIO_Port, EN_RIGHT_Pin, !(abs(left) > 50 || abs(right) > 50) && failsafe_status == SBUS_SIGNAL_OK);
	HAL_GPIO_WritePin(DIR_LEFT_GPIO_Port, DIR_LEFT_Pin, left > 0);
	HAL_GPIO_WritePin(DIR_RIGHT_GPIO_Port, DIR_RIGHT_Pin, right > 0);

	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);

	TIM2->CCR4 = (abs(left) > 950)?(950):(abs(left));
	TIM2->CCR3 = (abs(right) > 950)?(950):(abs(right));
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, abs(right) > 1000)? (1000):(abs(right));


}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	if (buf_in[0] == 0x0F && buf_in[24] == 0x00) {
		channels[0].value = (((buf_in[2] & 0b00000111) << 8) | (buf_in[1])) & 0x07FF;
		channels[0].value+= (channels[0].value < 992)?(128):(0);
			//	((buf_in[1] & 0x00FF) + (buf_in[2] * 0x0100)) % 2048;
		channels[1].value = (buf_in[2] >> 3 | (buf_in[3] << 5)) & 0x07FF;
		channels[2].value = (buf_in[3] >> 6 | (buf_in[4] << 2) | buf_in[5] << 10) & 0x07FF;
		channels[3].value = (buf_in[5] >> 1 | (buf_in[6] << 7)) & 0x07FF;
		channels[4].value = (buf_in[6] >> 4 | (buf_in[7] << 4)) & 0x07FF;
		channels[5].value = (buf_in[7] >> 7 | (buf_in[8] << 1) | buf_in[9] << 9) & 0x07FF;
		channels[6].value = (buf_in[9] >> 2 | (buf_in[10] << 6)) & 0x07FF;
		channels[7].value = (buf_in[10] >> 5 | (buf_in[11] << 3)) & 0x07FF;
		channels[8].value = (buf_in[12] << 0 | (buf_in[13] << 8)) & 0x07FF;
		channels[9].value = (buf_in[13] >> 3 | (buf_in[14] << 5)) & 0x07FF;
		channels[10].value = (buf_in[14] >> 6 | (buf_in[15] << 2) | buf_in[16] << 10) & 0x07FF;
		channels[11].value = (buf_in[16] >> 1 | (buf_in[17] << 7)) & 0x07FF;
		channels[12].value = (buf_in[17] >> 4 | (buf_in[18] << 4)) & 0x07FF;
		channels[13].value = (buf_in[18] >> 7 | (buf_in[19] << 1) | buf_in[20] << 9) & 0x07FF;
		channels[14].value = (buf_in[20] >> 2 | (buf_in[21] << 6)) & 0x07FF;
		channels[15].value = (buf_in[21] >> 5 | (buf_in[22] << 3)) & 0x07FF;

		if (buf_in[23] & (1 << 0)) {
			channels[16].value = 1;
		} else {
			channels[16].value = 0;
		}

		if (buf_in[23] & (1 << 1)) {
			channels[17].value = 1;
		} else {
			channels[17].value = 0;
		}
		// Failsafe
		failsafe_status = SBUS_SIGNAL_OK;
		if (buf_in[23] & (1 << 2)) {
			failsafe_status = SBUS_SIGNAL_LOST;
		}

		if (buf_in[23] & (1 << 3)) {
			failsafe_status = SBUS_SIGNAL_FAILSAFE;
		}

		//	SBUS_footer=buf_in[24];

		// autotrim procedure

		packetCounter+= packetCounter<10;
	}
	HAL_status = HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)buf_in, 25);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
			channels[0].value = 992;
			channels[1].value = 992;
			channels[0].trim = 992;
			channels[1].trim = 992;

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500); // for stable init screen

	ssd1306_Init();
  	ssd1306_SetColor(White);
  	ssd1306_UpdateScreen();
  	ssd1306_DisplayOn();
  	HAL_GPIO_WritePin(RX_INV_GPIO_Port, RX_INV_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)buf_in, 25);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ssd1306_Clear();
	  ssd1306_DrawVerticalLine(64, 0, 3);
	  if (channels[0].value > channels[0].trim) {
	  ssd1306_DrawHorizontalLine(64, 1, (channels[0].value - channels[0].trim)/16);
	  } else {
	  ssd1306_DrawHorizontalLine(64-(channels[0].trim-channels[0].value)/16, 1, (channels[0].trim - channels[0].value)/16);
	  }
	  ssd1306_DrawVerticalLine(64, 5, 3);
	  if (channels[1].value > channels[1].trim) {
		  ssd1306_DrawHorizontalLine(64, 6, (channels[1].value - channels[1].trim)/16);
		  } else {
		  ssd1306_DrawHorizontalLine(64-(channels[1].trim - channels[1].value)/16, 6, (channels[1].trim - channels[1].value)/16);
		  }
		 ssd1306_DrawVerticalLine(64, 9, 3);

  	  ssd1306_DrawHorizontalLine((left >0)?(64):(64 + left/16), 10, (abs(left))/16);
  	  ssd1306_DrawVerticalLine(64, 14, 3);
  	  ssd1306_DrawHorizontalLine((right >0)?(64):(64 + right/16), 15, (abs(right))/16);

      parse_Motion();

      ssd1306_SetCursor(0, 17);
            switch (HAL_status) {
            case HAL_OK: ssd1306_WriteChar('+', Font_7x8); break;
            case HAL_BUSY: ssd1306_WriteChar('~', Font_7x8); break;
            case HAL_ERROR: ssd1306_WriteChar('-', Font_7x8); break;
            case HAL_TIMEOUT: ssd1306_WriteChar('T', Font_7x8); break;
            }
      ssd1306_SetCursor(9, 17);
      sprintf(str,"%02x%02x %2d %4d %4d",buf_in[1],buf_in[2], packetCounter,  channels[0].value,  channels[1].value);
      ssd1306_WriteString(str, Font_7x8);
      ssd1306_UpdateScreen();
	  HAL_Delay(50);
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  packetCounter -= packetCounter>0;


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 100000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_EVEN;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_RIGHT_Pin|DIR_RIGHT_Pin|DIR_LEFT_Pin|EN_LEFT_Pin
                          |RX_INV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_RIGHT_Pin DIR_RIGHT_Pin DIR_LEFT_Pin EN_LEFT_Pin
                           RX_INV_Pin */
  GPIO_InitStruct.Pin = EN_RIGHT_Pin|DIR_RIGHT_Pin|DIR_LEFT_Pin|EN_LEFT_Pin
                          |RX_INV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_A_Pin KEY_B_Pin KEY_C_Pin */
  GPIO_InitStruct.Pin = KEY_A_Pin|KEY_B_Pin|KEY_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TX_INV_Pin */
  GPIO_InitStruct.Pin = TX_INV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TX_INV_GPIO_Port, &GPIO_InitStruct);

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

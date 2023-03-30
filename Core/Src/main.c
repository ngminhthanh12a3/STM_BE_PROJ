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
#include "FRAME_PARSE.h"
#include "max30102.h"
#include "uart_print.h"
#include "spo2_algorithm.h"
#include  "dev_act_hdl.h"
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
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t receive_data;
FrameParse_t FrameParse;
MAX30105_t particleSensor;
USART_H_t USART_handler;
// global definitions.
extern unsigned char RxBuf[RXBUFSIZE];// receive buffer
extern int RxHead; // circular buffer index
extern int RxTail; // circular buffer index

// MAX30102
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
// Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
// To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100];  // infrared LED sensor data
uint16_t redBuffer[100]; // red LED sensor data
#else
uint32_t irBuffer[100];  // infrared LED sensor data
uint32_t redBuffer[100]; // red LED sensor data
#endif
const int32_t ledsBufferLength = 100;  // data length
uint8_t ledBufIndex = 0;
//int32_t MAX30102_O_BUF[4];
int32_t spo2 = 0;          // SPO2 value
int8_t validSPO2 = 0;      // indicator to show if the SPO2 calculation is valid
int32_t heartRate = 0;     // heart rate value
int8_t validHeartRate = 0; // indicator to show if the heart rate calculation is valid

//
uint8_t startToCalculate = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// huart 4
	//	if(*huart == &huart4)
	//	{
	uint8_t rx = huart->Instance->DR;

	// put new char into buffer from UART
	RxBuf[RxHead] = rx;

	if ((++RxHead) > (RXBUFSIZE-1))
	{
		RxHead = 0;
	}

	//
	HAL_UART_Receive_IT(&huart4, &receive_data, 1);
	//		uint8_t __char = _getchar();
	//	    HAL_UART_Transmit(&huart1, &rx, 1, 1000);
	//	    if(rx == 0xE2u)
	//	    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);

	//	}

}

void Handle_UART_FP()
{
	if(FrameParse.FP_MOD != DECT_COML)
	{
		uint8_t __char = _getchar();
		ParseFrameHandler(&FrameParse, __char);
		//		HAL_UART_Transmit(&huart1, &__char, 1, 100);
	}
	else
	{
		HandlerDeviceAction(FrameParse._CMD,
				FrameParse.buffer.len,
				FrameParse.buffer.data);
		FP_Init(&FrameParse);
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
	FP_Init(&FrameParse);
	MAX30105_Init(&particleSensor, &hi2c2);
	USART_Init(&USART_handler, &huart1);
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_UART4_Init();
	MX_USART1_UART_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */
	// Initialize sensor
	if(!MAX30105_begin(&particleSensor, MAX30105_ADDRESS))
	{
		//MAX30105 was not found. Please check wiring/power.
		while(1);
	}

	uint8_t ledBrightness = 60; // Options: 0=Off to 255=50mA
	uint8_t sampleAverage = 4;  // Options: 1, 2, 4, 8, 16, 32
	uint8_t ledMode = 2;        // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
	uint8_t sampleRate = 100;   // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
	int pulseWidth = 411;    // Options: 69, 118, 215, 411
	int adcRange = 4096;     // Options: 2048, 4096, 8192, 16384

	MAX30105_setup(&particleSensor, ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure sensor with these settings

	//
	HAL_UART_Receive_IT(&huart4, &receive_data, 1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if(RxTail != RxHead || FrameParse.FP_MOD == DECT_COML)
			Handle_UART_FP();
		else

			if(startToCalculate)
			{
				uint8_t pData[DEV_0_DT_LEN];
				//		bufferLength = 100; // buffer length of 100 stores 4 seconds of samples running at 25sps
				// read the first 100 samples, and determine the signal range
				for (uint8_t i = 0 ; i < (ledsBufferLength - 1); i++)
				{
					while (!MAX30105_available(&particleSensor)) // do we have new data?
						MAX30105_check(&particleSensor);		// Check the sensor for new data

					redBuffer[i] = MAX30105_getRed(&particleSensor);
					irBuffer[i] = MAX30105_getIR(&particleSensor);
					MAX30105_nextSample(&particleSensor); // We're finished with this sample so move to next sample
					USART_Transmit(&USART_handler, "\r\nred = %d, i = %d", redBuffer[i], i);
					USART_Transmit(&USART_handler, ", ir = %d", irBuffer[i]);

					//
					*((uint16_t*)pData + 0) = redBuffer[i];
					*((uint16_t*)pData + 1) = irBuffer[i];
					*((uint32_t*)pData + 1) = spo2;
					*((uint32_t*)pData + 2) = heartRate;
					pData[DEV_0_DT_LEN - 1] = i; // sychronize
					//				HandlerDeviceAction(0x00, DEV_0_DT_LEN, pData);
					HandleDeviceSendFrame(0x00, DEV_0_DT_LEN, pData);
				}

				// in 100th
				while (!MAX30105_available(&particleSensor)) // do we have new data?
					MAX30105_check(&particleSensor);		// Check the sensor for new data

				redBuffer[ledsBufferLength - 1] = MAX30105_getRed(&particleSensor);
				irBuffer[ledsBufferLength - 1] = MAX30105_getIR(&particleSensor);
				MAX30105_nextSample(&particleSensor); // We're finished with this sample so move to next sample
				//calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
				maxim_heart_rate_and_oxygen_saturation(irBuffer, ledsBufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

				USART_Transmit(&USART_handler, "\r\nred = %d, i = %d", redBuffer[ledsBufferLength - 1], ledsBufferLength - 1);
				USART_Transmit(&USART_handler, ", ir = %d", irBuffer[ledsBufferLength - 1]);
				USART_Transmit(&USART_handler, ", HR = %d", heartRate);
				USART_Transmit(&USART_handler, ", HRvalid = %d", validHeartRate);
				USART_Transmit(&USART_handler, ", SPO2 = %d", spo2);
				USART_Transmit(&USART_handler, ", SPO2Valid = %d", validSPO2);

				//
				*((uint16_t*)pData + 0) = redBuffer[ledsBufferLength - 1];
				*((uint16_t*)pData + 1) = irBuffer[ledsBufferLength - 1];
				*((uint32_t*)pData + 1) = spo2;
				*((uint32_t*)pData + 2) = heartRate;
				pData[DEV_0_DT_LEN - 1] = ledsBufferLength - 1; // sychronize
				//			HandlerDeviceAction(0x00, DEV_0_DT_LEN, pData);
				HandleDeviceSendFrame(0x00, DEV_0_DT_LEN, pData);



				//			startToCalculate = 0;
			}

		//		// Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
		//		while (startToCalculate)
		//		{
		//			uint8_t pData[DEV_0_DT_LEN];
		//			// dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
		//			for (uint8_t i = 25; i < 100; i++)
		//			{
		//				redBuffer[i - 25] = redBuffer[i];
		//				irBuffer[i - 25] = irBuffer[i];
		//
		//				//
		//				*((uint16_t*)pData + 0) = redBuffer[i - 25];
		//				*((uint16_t*)pData + 1) = irBuffer[i - 25];
		//				*((uint32_t*)pData + 1) = spo2;
		//				*((uint32_t*)pData + 2) = heartRate;
		//				HandlerDeviceAction(0x00, DEV_0_DT_LEN, pData);
		//			}
		//
		////
		////			// take 25 sets of samples before calculating the heart rate.
		//			for (uint8_t i = 75; i < 100; i++)
		//			{
		//				while (!MAX30105_available(&particleSensor)) // do we have new data?
		//					MAX30105_check(&particleSensor);		// Check the sensor for new data
		//
		////				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,
		////						!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)
		////				); // Blink onboard LED with every data read
		//
		//				redBuffer[i] = MAX30105_getRed(&particleSensor);
		//				irBuffer[i] = MAX30105_getIR(&particleSensor);
		//				MAX30105_nextSample(&particleSensor); // We're finished with this sample so move to next sample
		//
		////				UART_sendByte(&USART_handler, redBuffer[i]);
		//				USART_Transmit(&USART_handler, "\r\nred = %d", redBuffer[i]);
		//				USART_Transmit(&USART_handler, ", ir = %d", irBuffer[i]);
		//
		//				USART_Transmit(&USART_handler, ", HR = %d", heartRate);
		//				USART_Transmit(&USART_handler, ", HRvalid = %d", validHeartRate);
		//				USART_Transmit(&USART_handler, ", SPO2 = %d", spo2);
		//				USART_Transmit(&USART_handler, ", SPO2Valid = %d", validSPO2);
		//
		//				//
		////				uint8_t pData[DEV_0_DT_LEN];
		//				*((uint16_t*)pData + 0) = redBuffer[i];
		//				*((uint16_t*)pData + 1) = irBuffer[i];
		//				*((uint32_t*)pData + 1) = spo2;
		//				*((uint32_t*)pData + 2) = heartRate;
		//				HandlerDeviceAction(0x00, DEV_0_DT_LEN, pData);
		//			}
		//
		////			// After gathering 25 new samples recalculate HR and SP02
		//			maxim_heart_rate_and_oxygen_saturation(irBuffer, ledsBufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
		//			//
		////			uint8_t pData[DEV_0_DT_LEN];
		//			*((uint16_t*)pData + 0) = redBuffer[ledsBufferLength - 1];
		//			*((uint16_t*)pData + 1) = irBuffer[ledsBufferLength - 1];
		//			*((uint32_t*)pData + 1) = spo2;
		//			*((uint32_t*)pData + 2) = heartRate;
		//			HandlerDeviceAction(0x00, DEV_0_DT_LEN, pData);
		//			//
		//			startToCalculate = 0;
		//		}

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
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin : PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
     ex: printf("Wrong parameters value: fil 	e %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

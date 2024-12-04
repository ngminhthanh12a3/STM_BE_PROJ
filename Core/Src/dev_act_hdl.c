#include  "dev_act_hdl.h"
#include "max30102.h"
#include "spo2_algorithm.h"
#include "uart_print.h"

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern MAX30105_t particleSensor;
extern USART_H_t USART_handler;

// MAX30102
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
// Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
// To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
extern uint16_t irBuffer[100];  // infrared LED sensor data
extern uint16_t redBuffer[100]; // red LED sensor data
#else
extern uint32_t irBuffer[100];  // infrared LED sensor data
extern uint32_t redBuffer[100]; // red LED sensor data
#endif
extern uint8_t ledBufIndex;
extern const int32_t ledsBufferLength;  // data length
extern int32_t spo2;          // SPO2 value
extern int8_t validSPO2;      // indicator to show if the SPO2 calculation is valid
extern int32_t heartRate;     // heart rate value
extern int8_t validHeartRate; // indicator to show if the heart rate calculation is valid

//
extern uint8_t startToCalculate;
void HandlerDeviceAction(uint8_t CMD, uint8_t buffer_len, uint8_t *buffer)
{
	uint8_t devID = CMD & 0x7Fu;
	uint8_t mode = CMD >> 7;
	if(mode == DEV_W_MOD)
	{
		switch (devID) {
		case DEV_0_ID:
			startToCalculate = *buffer;
//			HAL_UART_Transmit(&huart1, &startToCalculate, 1, 100);
			USART_Transmit(&USART_handler, "\r\nReceive start: %d", *buffer);
			break;
		case DEV_1_ID:
			HAL_GPIO_WritePin(DEV_1_PORT, DEV_1_PIN, buffer[0]);

			break;
		default:
			break;
		}

	}
	else
	{
		//
		uint8_t pData[FP_F_MAX_LEN], pLen = 0;
		uint8_t pIndex = 0;
		// | H1 | H2 |
		pData[pIndex++] = FP_H1; pData[pIndex++] = FP_H2;
		pData[pIndex++] = devID; // | CMD |
		//
		switch (devID) {
		case DEV_0_ID:
		{
//			pLen = FP_F_LEN(DEV_0_DT_LEN);
//			pData[pIndex++] = DEV_0_DT_LEN; // | DATA LEN |
//
//			//
//			uint8_t DATA[DEV_0_DT_LEN];
//
//			//
//			while (!MAX30105_available(&particleSensor)) // do we have new data?
//				MAX30105_check(&particleSensor);		// Check the sensor for new data
//			//
//			redBuffer[ledBufIndex] = MAX30105_getRed(&particleSensor);
//			irBuffer[ledBufIndex] = MAX30105_getIR(&particleSensor);
//			*((uint32_t*)DATA + 0) = redBuffer[ledBufIndex];
//			*((uint32_t*)DATA + 1) = irBuffer[ledBufIndex];
//			MAX30105_nextSample(&particleSensor);
//
//			//
//			if(++ledBufIndex >= ledsBufferLength)
//			{
//				//
//				ledBufIndex = 0;
//			}
//
//			//
//			*((uint32_t*)DATA + 2) = spo2;
//			*((uint32_t*)DATA + 3) = heartRate;
//
//			for(uint8_t i = 0;i < DEV_0_DT_LEN;i++)
//			{
////				(DATA & (0xFF << i)) >> i; // | DATA |
//				pData[pIndex++] = DATA[i];
//			}
//
//			//
//			pData[pIndex++] = FP_T1; pData[pIndex++] = FP_T2; // | T1 | T2 |
//
//			// | CRC |
//			uint16_t *pCRC = (uint16_t*)(pData + pIndex);
//			*pCRC = Compute_CRC16(DATA, DEV_0_DT_LEN);
//			startToCalculate = *buffer;

			break;
		}
		case DEV_1_ID:
		{

			break;
		}
		default:
			break;
		}

//		//
//		if(pLen)
//		{
//			HAL_UART_Transmit(&huart4, pData, pLen, 100);
//		}
	}
}

void HandleDeviceSendFrame(uint8_t devID, uint8_t buffer_len, uint8_t *buffer)
{
	uint8_t pData[FP_F_MAX_LEN], pLen = 0;
	uint8_t pIndex = 0;
	// | H1 | H2 |
	pData[pIndex++] = FP_H1; pData[pIndex++] = FP_H2;
	pData[pIndex++] = devID; // | CMD |

	switch (devID) {
		case DEV_0_ID:
			//
			pLen = FP_F_LEN(buffer_len);
			pData[pIndex++] = buffer_len; // | DATA LEN |

			//
			uint8_t DATA[buffer_len];
			for(uint8_t i = 0;i < buffer_len;i++)
				DATA[i] = buffer[i];

			//
			for(uint8_t i = 0;i < buffer_len;i++)
			{
				//				(DATA & (0xFF << i)) >> i; // | DATA |
				pData[pIndex++] = DATA[i];
			}

			//
			pData[pIndex++] = FP_T1; pData[pIndex++] = FP_T2; // | T1 | T2 |

			// | CRC |
			uint16_t *pCRC = (uint16_t*)(pData + pIndex);
			*pCRC = Compute_CRC16(DATA, buffer_len);

			break;
//		default:
//			break;
	}

	//
	if(pLen)
	{
		HAL_UART_Transmit(&huart4, pData, pLen, 2000);
	}
}

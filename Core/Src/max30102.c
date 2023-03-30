/*
 * max30102.c
 *
 *  Created on: Nov 24, 2022
 *      Author: ADMIN
 */

#include "max30102.h"

// FIFO Registers
static const uint8_t MAX30105_FIFOWRITEPTR = 	0x04u;
static const uint8_t MAX30105_FIFOOVERFLOW = 	0x05u;
static const uint8_t MAX30105_FIFOREADPTR = 	0x06u;
static const uint8_t MAX30105_FIFODATA =		0x07u;

static const uint8_t MAX_30105_EXPECTEDPARTID = 0x15u;

// Part ID Registers
static const uint8_t MAX30105_REVISIONID = 		0xFEu;
static const uint8_t MAX30105_PARTID = 			0xFFu;
// Configuration Registers
static const uint8_t MAX30105_FIFOCONFIG = 		0x08u;
static const uint8_t MAX30105_MODECONFIG = 		0x09u;
static const uint8_t MAX30105_PARTICLECONFIG = 	0x0A;// Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30105_LED1_PULSEAMP = 	0x0Cu;
static const uint8_t MAX30105_LED2_PULSEAMP = 	0x0Du;
static const uint8_t MAX30105_LED3_PULSEAMP = 	0x0Eu;
static const uint8_t MAX30105_LED_PROX_AMP = 	0x10u;
static const uint8_t MAX30105_MULTILEDCONFIG1 = 0x11u;
static const uint8_t MAX30105_MULTILEDCONFIG2 = 0x12u;

static const uint8_t MAX30105_RESET_MASK = 		0xBFu;
static const uint8_t MAX30105_RESET = 			0x40u;

static const uint8_t MAX30105_MODE_MASK = 		0xF8u;
static const uint8_t MAX30105_MODE_REDONLY = 	0x02u;
static const uint8_t MAX30105_MODE_REDIRONLY = 	0x03u;
static const uint8_t MAX30105_MODE_MULTILED = 	0x07u;

static const uint8_t MAX30105_SAMPLEAVG_MASK =	(uint8_t)~0b11100000u;
static const uint8_t MAX30105_SAMPLEAVG_1 = 	0x00u;
static const uint8_t MAX30105_SAMPLEAVG_2 = 	0x20u;
static const uint8_t MAX30105_SAMPLEAVG_4 = 	0x40u;
static const uint8_t MAX30105_SAMPLEAVG_8 = 	0x60u;
static const uint8_t MAX30105_SAMPLEAVG_16 = 	0x80u;
static const uint8_t MAX30105_SAMPLEAVG_32 = 	0xA0u;

static const uint8_t MAX30105_ROLLOVER_MASK = 	0xEFu;
static const uint8_t MAX30105_ROLLOVER_ENABLE = 0x10u;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30105_ADCRANGE_MASK = 	0x9Fu;
static const uint8_t MAX30105_ADCRANGE_2048 = 	0x00u;
static const uint8_t MAX30105_ADCRANGE_4096 = 	0x20u;
static const uint8_t MAX30105_ADCRANGE_8192 = 	0x40u;
static const uint8_t MAX30105_ADCRANGE_16384 = 	0x60u;

static const uint8_t MAX30105_SAMPLERATE_MASK = 0xE3u;
static const uint8_t MAX30105_SAMPLERATE_50 = 	0x00u;
static const uint8_t MAX30105_SAMPLERATE_100 = 	0x04u;
static const uint8_t MAX30105_SAMPLERATE_200 = 	0x08u;
static const uint8_t MAX30105_SAMPLERATE_400 = 	0x0Cu;
static const uint8_t MAX30105_SAMPLERATE_800 = 	0x10u;
static const uint8_t MAX30105_SAMPLERATE_1000 = 0x14u;
static const uint8_t MAX30105_SAMPLERATE_1600 = 0x18u;
static const uint8_t MAX30105_SAMPLERATE_3200 = 0x1Cu;

static const uint8_t MAX30105_PULSEWIDTH_MASK = 0xFCu;
static const uint8_t MAX30105_PULSEWIDTH_69 = 	0x00u;
static const uint8_t MAX30105_PULSEWIDTH_118 = 	0x01u;
static const uint8_t MAX30105_PULSEWIDTH_215 = 	0x02u;
static const uint8_t MAX30105_PULSEWIDTH_411 = 	0x03u;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30105_SLOT1_MASK = 		0xF8u;
static const uint8_t MAX30105_SLOT2_MASK = 		0x8Fu;
static const uint8_t MAX30105_SLOT3_MASK = 		0xF8u;
static const uint8_t MAX30105_SLOT4_MASK = 		0x8Fu;

static const uint8_t SLOT_NONE = 				0x00u;
static const uint8_t SLOT_RED_LED = 			0x01u;
static const uint8_t SLOT_IR_LED = 				0x02u;
static const uint8_t SLOT_GREEN_LED = 			0x03u;
static const uint8_t SLOT_NONE_PILOT = 			0x04u;
static const uint8_t SLOT_RED_PILOT =			0x05u;
static const uint8_t SLOT_IR_PILOT = 			0x06u;
static const uint8_t SLOT_GREEN_PILOT = 		0x07u;

//

void I2C_WriteBuffer(MAX30105_t *MAX30105_handler, uint8_t address, uint8_t *reg, uint8_t sizeBuf)
{
	while(HAL_I2C_Master_Transmit(MAX30105_handler->hi2c, address, reg, sizeBuf, 100))
	{
		if(HAL_I2C_GetError(MAX30105_handler->hi2c) != HAL_I2C_ERROR_AF)
		{

		}
	}
}

void I2C_ReadBuffer(MAX30105_t *MAX30105_handler, uint8_t address, uint8_t *pData, uint8_t sizeBuf)
{
	while(HAL_I2C_Master_Receive(MAX30105_handler->hi2c, address, pData, sizeBuf, 100))
	{
		if(HAL_I2C_GetError(MAX30105_handler->hi2c) != HAL_I2C_ERROR_AF)
		{

		}
	}
}
void MAX30105_Init(MAX30105_t *MAX30105_handler, I2C_HandleTypeDef *hi2c)
{
	MAX30105_handler->hi2c = hi2c;
}

uint8_t MAX30105_readPartID(MAX30105_t *MAX30105_handler)
{
	return MAX30105_readRegister8(MAX30105_handler, MAX30105_handler->_i2caddr, MAX30105_PARTID);
}

uint8_t MAX30105_begin(MAX30105_t *MAX30105_handler, uint8_t i2caddr)
{
	MAX30105_handler->_i2caddr = i2caddr;

	// Step 1: Initial Communication and Verification
	// Check that a MAX30105 is connected
	if(MAX30105_readPartID(MAX30105_handler) != MAX_30105_EXPECTEDPARTID)
		// Error -- Part ID read from MAX30105 does not match expected part ID.
		// This may mean there is a physical connectivity problem (broken wire, unpowered, etc).
		return MAX30105_FAIL;

	// Populate revision ID
	MAX30105_readRevisionID(MAX30105_handler);

	return MAX30105_OK;
}

// Report the most recent red value
uint32_t MAX30105_getRed(MAX30105_t *MAX30105_handler)
{
	// Check the sensor for new data for 250ms
	if (MAX30105_safeCheck(MAX30105_handler, 250))
		return (MAX30105_handler->sense.red[MAX30105_handler->sense.head]);
	else
		return (0); // Sensor failed to find new data
}

// Report the most recent IR value
int32_t MAX30105_getIR(MAX30105_t *MAX30105_handler)
{
	// Check the sensor for new data for 250ms
	if (MAX30105_safeCheck(MAX30105_handler, 250))
		return (MAX30105_handler->sense.IR[MAX30105_handler->sense.head]);
	else
		return (0); // Sensor failed to find new data
}

// Check for new data but give up after a certain amount of time
// Returns true if new data was found
// Returns false if new data was not found
uint8_t MAX30105_safeCheck(MAX30105_t *MAX30105_handler, uint8_t maxTimeToCheck)
{
	uint32_t markTime = HAL_GetTick();
	while(1)
	{
		if(HAL_GetTick() - markTime > maxTimeToCheck)
			return 0;
		if(MAX30105_check(MAX30105_handler) == 1) // We found new data!
			return 1;

		HAL_Delay(1);
	}
}

//End Interrupt configuration
void MAX30105_softReset(MAX30105_t *MAX30105_handler)
{
	MAX30105_bitMask(MAX30105_handler, MAX30105_MODECONFIG, MAX30105_RESET_MASK, MAX30105_RESET);

	// Poll for bit to clear, reset is then complete
	// Timeout after 100ms
	for(uint8_t i = 0;i < 100;i++)
	{
		uint8_t response = MAX30105_readRegister8(MAX30105_handler, MAX30105_handler->_i2caddr, MAX30105_MODECONFIG);
		if ((response & MAX30105_RESET) == 0)
			break; //We're done!
		HAL_Delay(1); //Let's not over burden the I2C bus
	}
}

void MAX30105_setLEDMode(MAX30105_t *MAX30105_handler, uint8_t mode)
{
	// Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
	// See datasheet, page 19
	MAX30105_bitMask(MAX30105_handler, MAX30105_MODECONFIG, MAX30105_MODE_MASK, mode);
}

void MAX30105_setADCRange(MAX30105_t *MAX30105_handler, uint8_t adcRange)
{
	// adcRange: one of MAX30105_ADCRANGE_2048, _4096, _8192, _16384
	MAX30105_bitMask(MAX30105_handler, MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adcRange);
}

void MAX30105_setSampleRate(MAX30105_t *MAX30105_handler, uint8_t sampleRate)
{
	// sampleRate: one of MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
	MAX30105_bitMask(MAX30105_handler, MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, sampleRate);
}

void MAX30105_setPulseWidth(MAX30105_t *MAX30105_handler, uint8_t pulseWidth)
{
	// pulseWidth: one of MAX30105_PULSEWIDTH_69, _188, _215, _411
	MAX30105_bitMask(MAX30105_handler, MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void MAX30105_setPulseAmplitudeRed(MAX30105_t *MAX30105_handler, uint8_t value)
{
	MAX30105_writeRegister8(MAX30105_handler, MAX30105_handler->_i2caddr, MAX30105_LED1_PULSEAMP, value);
}

void MAX30105_setPulseAmplitudeIR(MAX30105_t *MAX30105_handler, uint8_t value)
{
	MAX30105_writeRegister8(MAX30105_handler, MAX30105_handler->_i2caddr, MAX30105_LED2_PULSEAMP, value);
}

void MAX30105_setPulseAmplitudeGreen(MAX30105_t *MAX30105_handler, uint8_t value)
{
	MAX30105_writeRegister8(MAX30105_handler, MAX30105_handler->_i2caddr, MAX30105_LED3_PULSEAMP, value);
}

void MAX30105_setPulseAmplitudeProximity(MAX30105_t *MAX30105_handler, uint8_t value)
{
	MAX30105_writeRegister8(MAX30105_handler, MAX30105_handler->_i2caddr, MAX30105_LED_PROX_AMP, value);
}

void MAX30105_enableSlot(MAX30105_t *MAX30105_handler, uint8_t slotNumber, uint8_t device)
{
//	uint8_t originalContents;

	switch (slotNumber) {
	case 1:
		MAX30105_bitMask(MAX30105_handler, MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
		break;
	case 2:
		MAX30105_bitMask(MAX30105_handler, MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4);
		break;
	case 3:
		MAX30105_bitMask(MAX30105_handler, MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device);
		break;
	case 4:
		MAX30105_bitMask(MAX30105_handler, MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK, device << 4);
		break;
	default:
		//Shouldn't be here!
		break;
	}
}

//
// FIFO Configuration
//

//Set sample average (Table 3, Page 18)
void MAX30105_setFIFOAverage(MAX30105_t *MAX30105_handler, uint8_t samples)
{
	MAX30105_bitMask(MAX30105_handler, MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, samples);
}

//Enable roll over if FIFO over flows
void MAX30105_enableFIFORollover(MAX30105_t *MAX30105_handler)
{
	MAX30105_bitMask(MAX30105_handler, MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
}

//
// Data Collection
//

//Polls the sensor for new data
//Call regularly
//If new data is available, it updates the head and tail in the main struct
//Returns number of new samples obtained
uint16_t MAX30105_check(MAX30105_t *MAX30105_handler)
{
	//Read register FIDO_DATA in (3-byte * number of active LED) chunks
	//Until FIFO_RD_PTR = FIFO_WR_PTR
	uint8_t readPointer = MAX30105_getReadPointer(MAX30105_handler);
	uint8_t writePointer = MAX30105_getWritePointer(MAX30105_handler);

	int numberOfSamples = 0;

	//Do we have new data?
	if (readPointer != writePointer)
	{
		//Calculate the number of readings we need to get from sensor
		numberOfSamples = writePointer - readPointer;
		if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

		//We now have the number of readings, now calc bytes to read
		//For this example we are just doing Red and IR (3 bytes each)
		int bytesLeftToRead = numberOfSamples * MAX30105_handler->activeLEDs * 3;

		//Get ready to read a burst of data from the FIFO register
		uint8_t reg = MAX30105_FIFODATA;
		I2C_WriteBuffer(MAX30105_handler, MAX30105_handler->_i2caddr, &reg, 1);

		//We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
		//I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
		//Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
		while (bytesLeftToRead > 0)
		{
			int toGet = bytesLeftToRead;
			if (toGet > I2C_BUFFER_LENGTH)
			{
				//If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
				//32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
				//32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.

				toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (MAX30105_handler->activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
			}

			bytesLeftToRead -= toGet;

			uint8_t pData[I2C_BUFFER_LENGTH] = {0};
			I2C_ReadBuffer(MAX30105_handler, MAX30105_handler->_i2caddr, pData, (uint8_t)toGet);

			while (toGet > 0)
			{
				MAX30105_handler->sense.head++; //Advance the head of the storage struct
				MAX30105_handler->sense.head %= STORAGE_SIZE; //Wrap condition

				uint8_t temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
				uint32_t tempLong;

				uint8_t pIndex = 0;
				//Burst read three bytes - RED
				temp[3] = 0;
				temp[2] = pData[pIndex++];
				temp[1] = pData[pIndex++];
				temp[0] = pData[pIndex++];

				// Convert array to long
				memcpy(&tempLong, temp, sizeof(tempLong));

				tempLong &= 0x3FFFFu; // Zero out all but 18 bits

				MAX30105_handler->sense.red[MAX30105_handler->sense.head] = tempLong; // Store this reading into the sense array

				if (MAX30105_handler->activeLEDs > 1)
				{
					// Burst read three more bytes - IR
					temp[3] = 0;
					temp[2] = pData[pIndex++];
					temp[1] = pData[pIndex++];
					temp[0] = pData[pIndex++];

					// Convert array to long
					memcpy(&tempLong, temp, sizeof(tempLong));

					tempLong &= 0x3FFFF; // Zero out all but 18 bits

					MAX30105_handler->sense.IR[MAX30105_handler->sense.head] = tempLong;
				}

				if (MAX30105_handler->activeLEDs > 2)
				{
					// Burst read three more bytes - Green
					temp[3] = 0;
					temp[2] = pData[pIndex++];
					temp[1] = pData[pIndex++];
					temp[0] = pData[pIndex++];

					// Convert array to long
					memcpy(&tempLong, temp, sizeof(tempLong));

					tempLong &= 0x3FFFF; // Zero out all but 18 bits

					MAX30105_handler->sense.green[MAX30105_handler->sense.head] = tempLong;
				}

				toGet -= MAX30105_handler->activeLEDs * 3;
			}
		}// End while (bytesLeftToRead > 0)

	}// End readPtr != writePtr

	return (numberOfSamples); // Let the world know how much new data we found
}

//Tell caller how many samples are available
uint8_t MAX30105_available(MAX30105_t *MAX30105_handler)
{
	int8_t numberOfSamples = MAX30105_handler->sense.head - MAX30105_handler->sense.tail;
	if (numberOfSamples < 0) numberOfSamples += STORAGE_SIZE;

	return (numberOfSamples);
}

// Advance the tail
void MAX30105_nextSample(MAX30105_t *MAX30105_handler)
{
	if (MAX30105_available(MAX30105_handler)) // Only advance the tail if new data is available
	  {
	    MAX30105_handler->sense.tail++;
	    MAX30105_handler->sense.tail %= STORAGE_SIZE; // Wrap condition
	  }
}

//Read the FIFO Write Pointer
uint8_t MAX30105_getWritePointer(MAX30105_t *MAX30105_handler)
{
	return (MAX30105_readRegister8(MAX30105_handler, MAX30105_handler->_i2caddr, MAX30105_FIFOWRITEPTR));
}

//Read the FIFO Read Pointer
uint8_t MAX30105_getReadPointer(MAX30105_t *MAX30105_handler)
{
	return (MAX30105_readRegister8(MAX30105_handler, MAX30105_handler->_i2caddr, MAX30105_FIFOREADPTR));
}

void MAX30105_clearFIFO(MAX30105_t *MAX30105_handler)
{
	//Resets all points to start in a known state
	//Page 15 recommends clearing FIFO before beginning a read
	MAX30105_writeRegister8(MAX30105_handler, MAX30105_handler->_i2caddr, MAX30105_FIFOWRITEPTR, 0);
	MAX30105_writeRegister8(MAX30105_handler, MAX30105_handler->_i2caddr, MAX30105_FIFOOVERFLOW, 0);
	MAX30105_writeRegister8(MAX30105_handler, MAX30105_handler->_i2caddr, MAX30105_FIFOREADPTR, 0);
}

//
// Low-level I2C Communication
//
uint8_t MAX30105_readRegister8(MAX30105_t *MAX30105_handler, uint8_t address, uint8_t reg)
{
//	HAL_I2C_Master_Transmit(MAX30105_handler->hi2c, address, &reg, 1, 100);
	I2C_WriteBuffer(MAX30105_handler, address | 0x01, &reg, 1);
	//
	while(HAL_I2C_GetState(MAX30105_handler->hi2c) != HAL_I2C_STATE_READY);
	uint8_t pData = 0;
//	HAL_I2C_Master_Receive(MAX30105_handler->hi2c, address, &pData, 1, 100);
	I2C_ReadBuffer(MAX30105_handler, address, &pData, 1);

	return pData;
}

void MAX30105_writeRegister8(MAX30105_t *MAX30105_handler, uint8_t address, uint8_t reg, uint8_t value)
{
	uint8_t pData[] = {reg, value};
	I2C_WriteBuffer(MAX30105_handler, address, pData, 2);
//	I2C_WriteBuffer(MAX30105_handler, address, value, 1);
//	HAL_I2C_Master_Transmit(MAX30105_handler->hi2c, address, &reg, 1, 100);
//	HAL_I2C_Master_Transmit(MAX30105_handler->hi2c, address, &value, 1, 100);
}

void MAX30105_readRevisionID(MAX30105_t *MAX30105_handler)
{
	MAX30105_handler->revisionID = MAX30105_readRegister8(MAX30105_handler, MAX30105_handler->_i2caddr, MAX30105_REVISIONID);
}

//Given a register, read it, mask it, and then set the thing
void MAX30105_bitMask(MAX30105_t *MAX30105_handler, uint8_t reg, uint8_t mask, uint8_t thing)
{
	// Grab current register context
	uint8_t originalContents = MAX30105_readRegister8(MAX30105_handler, MAX30105_handler->_i2caddr, reg);

	// Zero-out the portions of the register we're interested in
	originalContents = originalContents & mask;

	// Change contents
	MAX30105_writeRegister8(MAX30105_handler, MAX30105_handler->_i2caddr, reg, originalContents | thing);

	uint8_t response = MAX30105_readRegister8(MAX30105_handler, MAX30105_handler->_i2caddr, reg);
}

//Setup the sensor
//The MAX30105 has many settings. By default we select:
// Sample Average = 4
// Mode = MultiLED
// ADC Range = 16384 (62.5pA per LSB)
// Sample rate = 50
//Use the default setup if you are just getting started with the MAX30105 sensor
void MAX30105_setup(MAX30105_t *MAX30105_handler, uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange)
{
	MAX30105_softReset(MAX30105_handler); //Reset all configuration, threshold, and data registers to POR values

	//FIFO Configuration
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	//The chip will average multiple samples of same type together if you wish
	if (sampleAverage == 1) MAX30105_setFIFOAverage(MAX30105_handler, MAX30105_SAMPLEAVG_1); //No averaging per FIFO record
	else if (sampleAverage == 2) MAX30105_setFIFOAverage(MAX30105_handler, MAX30105_SAMPLEAVG_2);
	else if (sampleAverage == 4) MAX30105_setFIFOAverage(MAX30105_handler, MAX30105_SAMPLEAVG_4);
	else if (sampleAverage == 8) MAX30105_setFIFOAverage(MAX30105_handler, MAX30105_SAMPLEAVG_8);
	else if (sampleAverage == 16) MAX30105_setFIFOAverage(MAX30105_handler, MAX30105_SAMPLEAVG_16);
	else if (sampleAverage == 32) MAX30105_setFIFOAverage(MAX30105_handler, MAX30105_SAMPLEAVG_32);
	else MAX30105_setFIFOAverage(MAX30105_handler, MAX30105_SAMPLEAVG_4);

	//setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
	MAX30105_enableFIFORollover(MAX30105_handler); //Allow FIFO to wrap/roll over
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	//Mode Configuration
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	if (ledMode == 3) MAX30105_setLEDMode(MAX30105_handler, MAX30105_MODE_MULTILED); //Watch all three LED channels
	else if (ledMode == 2) MAX30105_setLEDMode(MAX30105_handler, MAX30105_MODE_REDIRONLY); //Red and IR
	else MAX30105_setLEDMode(MAX30105_handler, MAX30105_MODE_REDONLY); //Red only

	MAX30105_handler->activeLEDs = ledMode; //Used to control how many bytes to read from FIFO buffer
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	//Particle Sensing Configuration
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	if(adcRange < 4096) MAX30105_setADCRange(MAX30105_handler, MAX30105_ADCRANGE_2048); //7.81pA per LSB
	else if(adcRange < 8192) MAX30105_setADCRange(MAX30105_handler, MAX30105_ADCRANGE_4096); //15.63pA per LSB
	else if(adcRange < 16384) MAX30105_setADCRange(MAX30105_handler, MAX30105_ADCRANGE_8192); //31.25pA per LSB
	else if(adcRange == 16384) MAX30105_setADCRange(MAX30105_handler, MAX30105_ADCRANGE_16384); //62.5pA per LSB
	else MAX30105_setADCRange(MAX30105_handler, MAX30105_ADCRANGE_2048);

	if (sampleRate < 100) MAX30105_setSampleRate(MAX30105_handler, MAX30105_SAMPLERATE_50); //Take 50 samples per second
	else if (sampleRate < 200) MAX30105_setSampleRate(MAX30105_handler, MAX30105_SAMPLERATE_100);
	else if (sampleRate < 400) MAX30105_setSampleRate(MAX30105_handler, MAX30105_SAMPLERATE_200);
	else if (sampleRate < 800) MAX30105_setSampleRate(MAX30105_handler, MAX30105_SAMPLERATE_400);
	else if (sampleRate < 1000) MAX30105_setSampleRate(MAX30105_handler, MAX30105_SAMPLERATE_800);
	else if (sampleRate < 1600) MAX30105_setSampleRate(MAX30105_handler, MAX30105_SAMPLERATE_1000);
	else if (sampleRate < 3200) MAX30105_setSampleRate(MAX30105_handler, MAX30105_SAMPLERATE_1600);
	else if (sampleRate == 3200) MAX30105_setSampleRate(MAX30105_handler, MAX30105_SAMPLERATE_3200);
	else MAX30105_setSampleRate(MAX30105_handler, MAX30105_SAMPLERATE_50);

	//The longer the pulse width the longer range of detection you'll have
	//At 69us and 0.4mA it's about 2 inches
	//At 411us and 0.4mA it's about 6 inches
	if (pulseWidth < 118) MAX30105_setPulseWidth(MAX30105_handler, MAX30105_PULSEWIDTH_69); //Page 26, Gets us 15 bit resolution
	else if (pulseWidth < 215) MAX30105_setPulseWidth(MAX30105_handler, MAX30105_PULSEWIDTH_118); //16 bit resolution
	else if (pulseWidth < 411) MAX30105_setPulseWidth(MAX30105_handler, MAX30105_PULSEWIDTH_215); //17 bit resolution
	else if (pulseWidth == 411) MAX30105_setPulseWidth(MAX30105_handler, MAX30105_PULSEWIDTH_411); //18 bit resolution
	else MAX30105_setPulseWidth(MAX30105_handler, MAX30105_PULSEWIDTH_69);
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	//LED Pulse Amplitude Configuration
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	//Default is 0x1F which gets us 6.4mA
	//powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
	//powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
	//powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
	//powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

	MAX30105_setPulseAmplitudeRed(MAX30105_handler, powerLevel);
	MAX30105_setPulseAmplitudeIR(MAX30105_handler, powerLevel);
	MAX30105_setPulseAmplitudeGreen(MAX30105_handler, powerLevel);
	MAX30105_setPulseAmplitudeProximity(MAX30105_handler, powerLevel);

	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	//Multi-LED Mode Configuration, Enable the reading of the three LEDs
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	MAX30105_enableSlot(MAX30105_handler, 1, SLOT_RED_LED);
	if (ledMode > 1) MAX30105_enableSlot(MAX30105_handler, 2, SLOT_IR_LED);
	if (ledMode > 2) MAX30105_enableSlot(MAX30105_handler, 3, SLOT_GREEN_LED);
	//enableSlot(1, SLOT_RED_PILOT);
	//enableSlot(2, SLOT_IR_PILOT);
	//enableSlot(3, SLOT_GREEN_PILOT);
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	MAX30105_clearFIFO(MAX30105_handler);
}

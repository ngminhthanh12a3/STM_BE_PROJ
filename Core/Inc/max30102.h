#ifndef _MAX30102_H__
#define _MAX30102_H__
#include "main.h"

//
#define MAX30105_ADDRESS (0x57u << 1)

//
//The catch-all default is 32
#define I2C_BUFFER_LENGTH 32

//
enum MAX30105_STATUS {
	MAX30105_FAIL,
	MAX30105_OK
};


//
#define STORAGE_SIZE 4 //Each long is 4 bytes so limit this to fit on your micro
typedef struct Record
{
	uint32_t red[STORAGE_SIZE];
	uint32_t IR[STORAGE_SIZE];
	uint32_t green[STORAGE_SIZE];
	uint8_t head;
	uint8_t tail;
} sense_struct; //This is our circular buffer of readings from the sensor
typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t _i2caddr;

	//activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
	uint8_t activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO

	uint8_t revisionID;

	sense_struct sense;
}MAX30105_t;

void MAX30105_Init(MAX30105_t *MAX30105_handler, I2C_HandleTypeDef *hi2c);
uint8_t MAX30105_begin(MAX30105_t *MAX30105_handler, uint8_t i2caddr);

uint32_t MAX30105_getRed(MAX30105_t *MAX30105_handler); //Returns immediate red value
int32_t MAX30105_getIR(MAX30105_t *MAX30105_handler); //Returns immediate IR value
uint8_t MAX30105_safeCheck(MAX30105_t *MAX30105_handler, uint8_t maxTimeToCheck); //Given a max amount of time, check for new data

// Configuration
void MAX30105_softReset(MAX30105_t *MAX30105_handler);

void MAX30105_setLEDMode(MAX30105_t *MAX30105_handler, uint8_t mode);

void MAX30105_setADCRange(MAX30105_t *MAX30105_handler, uint8_t adcRange);
void MAX30105_setSampleRate(MAX30105_t *MAX30105_handler, uint8_t sampleRate);
void MAX30105_setPulseWidth(MAX30105_t *MAX30105_handler, uint8_t pulseWidth);

void MAX30105_setPulseAmplitudeRed(MAX30105_t *MAX30105_handler, uint8_t value);
void MAX30105_setPulseAmplitudeIR(MAX30105_t *MAX30105_handler, uint8_t value);
void MAX30105_setPulseAmplitudeGreen(MAX30105_t *MAX30105_handler, uint8_t value);
void MAX30105_setPulseAmplitudeProximity(MAX30105_t *MAX30105_handler, uint8_t value);

//Multi-led configuration mode (page 22)
void MAX30105_enableSlot(MAX30105_t *MAX30105_handler, uint8_t slotNumber, uint8_t device); //Given slot number, assign a device to slot

//FIFO Configuration (page 18)
void MAX30105_setFIFOAverage(MAX30105_t *MAX30105_handler, uint8_t samples);
void MAX30105_enableFIFORollover(MAX30105_t *MAX30105_handler);

//FIFO Reading
uint16_t MAX30105_check(MAX30105_t *MAX30105_handler); //Checks for new data and fills FIFO
uint8_t MAX30105_available(MAX30105_t *MAX30105_handler); //Tells caller how many new samples are available (head - tail)
void MAX30105_nextSample(MAX30105_t *MAX30105_handler); //Advances the tail of the sense array

uint8_t MAX30105_getWritePointer(MAX30105_t *MAX30105_handler);
uint8_t MAX30105_getReadPointer(MAX30105_t *MAX30105_handler);
void MAX30105_clearFIFO(MAX30105_t *MAX30105_handler); //Sets the read/write pointers to zero

// Low-level I2C communication
uint8_t MAX30105_readRegister8(MAX30105_t *MAX30105_handler, uint8_t address, uint8_t reg);
void MAX30105_writeRegister8(MAX30105_t *MAX30105_handler, uint8_t address, uint8_t reg, uint8_t value);

//
uint8_t MAX30105_readPartID(MAX30105_t *MAX30105_handler);
void MAX30105_readRevisionID(MAX30105_t *MAX30105_handler);

void MAX30105_bitMask(MAX30105_t *MAX30105_handler, uint8_t reg, uint8_t mask, uint8_t thing);

void MAX30105_setup(MAX30105_t *MAX30105_handler, uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange);
#endif // _MAX30102_H__

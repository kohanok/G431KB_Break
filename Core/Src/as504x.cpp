#include "as504x.h"
#include "main.h"
#include <math.h>

/* Private defines -----------------------------------------------------------*/

const int AS5048A_CLEAR_ERROR_FLAG              = 0x0001;
const int AS5048A_PROGRAMMING_CONTROL           = 0x0003;
const int AS5048A_OTP_REGISTER_ZERO_POS_HIGH    = 0x0016;
const int AS5048A_OTP_REGISTER_ZERO_POS_LOW     = 0x0017;
const int AS5048A_DIAG_AGC                      = 0x3FFD;
const int AS5048A_MAGNITUDE                     = 0x3FFE;
const int AS5048A_ANGLE                         = 0x3FFF;


uint8_t errorFlag = 0;
uint16_t position;

extern SPI_HandleTypeDef hspi1;
/**
 * Constructor
 */
/*
as504x_AS5048A(SPI_HandleTypeDef* hspi, GPIO_TypeDef* arg_ps, uint16_t arg_cs){
	_cs = arg_cs;
	_ps = arg_ps;
	&hspi1 = hspi;
	errorFlag = 0;
	position = 0;
}
*/
#define EN_SPI HAL_GPIO_WritePin(CE0_GPIO_Port, CE0_Pin, GPIO_PIN_RESET);
#define DIS_SPI HAL_GPIO_WritePin(CE0_GPIO_Port, CE0_Pin, GPIO_PIN_SET);
#define PI	3.14159265358979323846
/**
 * Initialiser
 * Sets up the SPI interface
 */
void as504x_init(){

	//You can write here various checking functions

	as504x_close();
	as504x_open();


}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void as504x_close(){
	if (HAL_SPI_DeInit(&hspi1) != HAL_OK)
	{
		//User error function
	}
}

/**
 * Openthe SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void as504x_open(){
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		//User error function
	}
}

/**
 * Utility function used to calculate even parity of word
 */
uint8_t as504x_spiCalcEvenParity(uint16_t value){
	uint8_t cnt = 0;
	uint8_t i;

	for (i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
}

/*
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit word
 * Returns the value of the register
 */
uint16_t as504x_read(uint16_t registerAddress){

	uint8_t data[2];

	uint16_t command = 0b0100000000000000; // PAR=0 R/W=R
	command = command | registerAddress;

	//Add a parity bit on the the MSB
	command |= ((uint16_t)as504x_spiCalcEvenParity(command)<<15);

	//Split the command into two bytes
	data[1] = command & 0xFF;
	data[0] = ( command >> 8 ) & 0xFF;

	EN_SPI;
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 2, 0xFFFF);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {}
	DIS_SPI;

	EN_SPI;
	HAL_SPI_Receive(&hspi1, (uint8_t *)&data, 2, 0xFFFF);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {}
	DIS_SPI;

	if (data[1] & 0x40) {
		errorFlag = 1;
	} else {
		errorFlag = 0;
	}

	//Return the data, stripping the parity and error bits
	return (( ( data[1] & 0xFF ) << 8 ) | ( data[0] & 0xFF )) & ~0xC000;
}

/*
 * Write to a register
 * Takes the 16-bit  address of the target register and the 16 bit word of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
uint16_t as504x_write(uint16_t registerAddress, uint16_t data) {

	uint8_t dat[2];

	uint16_t command = 0b0000000000000000; // PAR=0 R/W=W
	command |= registerAddress;

	//Add a parity bit on the the MSB
	command |= ((uint16_t)as504x_spiCalcEvenParity(command)<<15);

	//Split the command into two bytes
	dat[1] = command & 0xFF;
	dat[0] = ( command >> 8 ) & 0xFF;

	//Start the write command with the target address
	EN_SPI;
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&dat, 2, 0xFFFF);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {}
	DIS_SPI;

	uint16_t dataToSend = 0b0000000000000000;
	dataToSend |= data;

	//Craft another packet including the data and parity
	dataToSend |= ((uint16_t)as504x_spiCalcEvenParity(dataToSend)<<15);
	dat[1] = command & 0xFF;
	dat[0] = ( command >> 8 ) & 0xFF;

	//Now send the data packet
	EN_SPI;
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&dat, 2, 0xFFFF);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {}
	DIS_SPI;

	//Send a NOP to get the new data in the register
	dat[1] = 0x00;
	dat[0] = 0x00;
	EN_SPI;
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&dat, 2, 0xFFFF);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {}
	HAL_SPI_Receive(&hspi1, (uint8_t *)&dat, 2, 0xFFFF);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {}
	DIS_SPI;

	//Return the data, stripping the parity and error bits
	return (( ( dat[1] & 0xFF ) << 8 ) | ( dat[0] & 0xFF )) & ~0xC000;
}

/**
 * Returns the raw angle directly from the sensor
 */
uint16_t as504x_getRawRotation(){
	return as504x_read(AS5048A_ANGLE);
}

/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int} between -2^13 and 2^13
 */
int as504x_getRotation(){
	uint16_t data;
	int rotation;

	data = as504x_getRawRotation();
	rotation = (int)data - (int)position;
	if(rotation > 8191) rotation = -((0x3FFF)-rotation); //more than -180
	//if(rotation < -0x1FFF) rotation = rotation+0x3FFF;

	return rotation;
}

/**
 * returns the value of the state register
 * @return 16 bit word containing flags
 */
uint16_t as504x_getState(){
	return as504x_read(AS5048A_DIAG_AGC);
}

/*
 * Check if an error has been encountered.
 */
uint8_t as504x_error(){
	return errorFlag;
}

/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 */
uint8_t as504x_getGain(){
	uint16_t data = as504x_getState();
	return (uint8_t) data & 0xFF;
}

/*
 * Get and clear the error register by reading it
 */
uint16_t as504x_getErrors(){
	return as504x_read(AS5048A_CLEAR_ERROR_FLAG);
}

/*
 * Set the zero position
 */
void as504x_setZeroPosition(uint16_t arg_position){
	position = arg_position % 0x3FFF;
}

/*
 * Returns the current zero position
 */
uint16_t as504x_getZeroPosition(){
	return position;
}

/*
 * Returns normalized angle value
 */
float as504x_normalize(float angle) {
	// http://stackoverflow.com/a/11498248/3167294
	#ifdef ANGLE_MODE_1
		angle += 180;
	#endif
	angle = fmod(angle, 360);
	if (angle < 0) {
		angle += 360;
	}
	#ifdef ANGLE_MODE_1
		angle -= 180;
	#endif
	return angle;
}

/*
 * Returns caalculated angle value
 */
float as504x_read2angle(uint16_t angle) {
	/*
	 * 14 bits = 2^(14) - 1 = 16.383
	 *
	 * https://www.arduino.cc/en/Reference/Map
	 *
	 */
	return (float)angle * ((float)360 / 16383);
};


int as504x_degrees(uint16_t sensor_result)
{
    return as504x_mask(sensor_result) * 36000 / 0x4000;
}


float as504x_radian(float angle)
{
    return (angle * PI) / 180;
}

int as504x_mask(uint16_t sensor_result)
{
    return sensor_result & 0x3FFF; // return lowest 14-bits
}

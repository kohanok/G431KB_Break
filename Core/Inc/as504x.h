/**
  ******************************************************************************
  * @file           : AS5048A.h
  * @brief          : Header for as5048a.c file.
  *                   This file contains the common defines of the application.
  * @version		: 0.1
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 Raimondas Pomarnacki.
  * All rights reserved.</center></h2>
  *
  * This software component is not licensed,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __as504x__H
#define __as504x__H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/


/* Private includes ----------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include <math.h>


/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/


uint16_t as504x_transaction(uint16_t data);


/**
 * Initialiser
 * Sets up the SPI interface
 */
void as504x_init();

/**
 * Closes the SPI connection
 */
void as504x_close();

/**
 * Open the SPI connection
 */
void as504x_open();

/*
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit word
 * Returns the value of the register
 */
uint16_t as504x_read(uint16_t registerAddress);

/*
 * Write to a register
 * Takes the 16-bit  address of the target register and the 16 bit word of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
uint16_t as504x_write(uint16_t registerAddress, uint16_t data);

/**
 * Returns the raw angle directly from the sensor
 */
uint16_t as504x_getRawRotation();

/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int} between -2^13 and 2^13
 */
int as504x_getRotation();

/**
 * returns the value of the state register
 * @return 16 bit word containing flags
 */
uint16_t as504x_getState();

/*
 * Check if an error has been encountered.
 */
uint8_t as504x_error();


/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 */
uint8_t as504x_getGain();


/*
 * Get and clear the error register by reading it
 */
uint16_t as504x_getErrors();

/*
 * Set the zero position
 */
void as504x_setZeroPosition(uint16_t arg_position);

/*
 * Returns the current zero position
 */
uint16_t as504x_getZeroPosition();

/*
 * Returns normalized angle value
 */
float as504x_normalize(float angle);

/*
 * Returns calculated angle value
 */
float as504x_read2angle(uint16_t angle);
uint8_t as504x_spiCalcEvenParity(uint16_t value);
int as504x_degrees(uint16_t sensor_result);
float as504x_radian(float angle);
int as504x_mask(uint16_t sensor_result);
/* Private defines -----------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

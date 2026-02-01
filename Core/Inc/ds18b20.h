/*****************************************************************************
* Copyright (C) 2026 by Taher Ujjainwala
*
* Redistribution, modification or use of this software in source or binary
* forms is permitted as long as the files maintain this copyright. Users are
* permitted to modify this and use it to learn about the field of embedded
* software. Taher Ujjainwala is not liable for
* any misuse of this material.
*
*****************************************************************************/
/**
* @file ds18b20.h
* @brief An abstraction for function definitions
*
* This header file provides an abstraction of
* changing the Led status via function calls.
*
* @author Taher Ujjainwala
* @date Jan 31 2026
* @version 1.0
*
*/

/*************************
 *
 *
 *
 *    File name   : ds18b20.h
 *    Description : This file is responsible for all functions which are implemented in ds18b20.c for program execution

 *    Author: TAHER S UJJAINWALA
 * 	  Tools : STM32Cube IDE 1.12.1, STM32Cube MX
 *    References: Datasheet, Deep Blue Embedded
 *    Date  : 01/31/2026
 *
 *
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include <stdio.h>
#include <stdint.h>

/*************************************************************************************************************
 *
 * Name :		  	void DS18B20_SetPinOutput(void);
 *
 *
 *
 * Description :	This function sets the IO as Output with predefined output states
 * Inputs: NONE
 *
 * Return: NONE
 *************************************************************************************************************/

void DS18B20_SetPinOutput(void);

/*************************************************************************************************************
 *
 * Name :		  	void DS18B20_SetPinInput(void);
 *
 *
 *
 * Description :	This function sets the IO as Input with predefined input states
 * Inputs: NONE
 *
 * Return: NONE
 *************************************************************************************************************/


void DS18B20_SetPinInput(void);

/*************************************************************************************************************
 *
 * Name :		  	void DS18B20_WriteBit(uint8_t bit);
 *
 *
 *
 * Description :	This function writes a bit on the 1-wire bus.
 * 					This information is the register address as mentioned by the datasheet that enables the sensor
 * 					into defined state like set the ROM, Send the Convert T functions -etc
 * Inputs: BIT
 *
 * Return: NONE
 *************************************************************************************************************/


void DS18B20_WriteBit(uint8_t bit);

/*************************************************************************************************************
 *
 * Name :		  	void DS18B20_WriteByte(uint8_t byte);
 *
 *
 *
 * Description :	This function writes a byte on the 1-wire bus.
 * 					This information is the register address as mentioned by the datasheet that enables the sensor
 * 					into defined state like set the ROM, Send the Convert T functions -etc
 * Inputs: BYTE
 *
 * Return: NONE
 *************************************************************************************************************/

void DS18B20_WriteByte(uint8_t byte);

/*************************************************************************************************************
 *
 * Name :		  	uint8_t DS18B20_ReadBit(void);
 *
 *
 *
 * Description :	This function reads bit-by bit from 1-wire bus. It stores the information and returns the bit
 * Inputs: NONE
 *
 * Return: Bit
 *************************************************************************************************************/

uint8_t DS18B20_ReadBit(void);

/*************************************************************************************************************
 *
 * Name :		  	uint8_t DS18B20_ReadByte(void);
 *
 *
 *
 * Description :	This function reads a byte from 1-wire bus. It stores the information into a 8-bit integer
 * Inputs: NONE
 *
 * Return: Byte
 *************************************************************************************************************/

uint8_t DS18B20_ReadByte(void);

/*************************************************************************************************************
 *
 * Name :		  	float DS18B20_ReadTemp(void);
 *
 *
 *
 * Description :	This function reads 12-bit temperature using the write commands defined by the datasheet.
 * 					Once the temperature is received, it is converted from raw format into float
 *
 * Inputs: NONE
 *
 * Return: Temperature in Celcius
 *************************************************************************************************************/

float DS18B20_ReadTemp(void);

/*************************************************************************************************************
 *
 * Name :		  	uint8_t DS18B20_Init(void)
 *
 *
 *
 * Description :	This function enables Initialization of DS18B20 sensor. It sends reset pulse and waits for presence
 * 					pulse. If the presence pulse is received, the sensor is initialized
 *
 * Inputs: NONE
 *
 * Return: Presence Status
 *************************************************************************************************************/

uint8_t DS18B20_Init(void);

#endif /* INC_DS18B20_H_ */

/*
 * ds18b20.h
 *
 *  Created on: Jan 24, 2026
 *      Author: ujjai
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include <stdio.h>
#include <stdint.h>

void DS18B20_SetPinOutput(void);
void DS18B20_SetPinInput(void);
void DS18B20_WriteBit(uint8_t bit);
void DS18B20_WriteByte(uint8_t byte);
uint8_t DS18B20_ReadBit(void);
uint8_t DS18B20_ReadByte(void);
float DS18B20_ReadTemp(void);
uint8_t DS18B20_Init(void);

#endif /* INC_DS18B20_H_ */

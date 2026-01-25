/*
 * ds18b20.c
 *
 *  Created on: Jan 24, 2026
 *      Author: ujjai
 */

//Including the .h header files
#include "main.h"
#include <stdio.h>
#include <stdint.h>

//Port and Pin Definition - Configurable Parameters
//Currently only supporting 12-bit resolution
#define DS18B20_GPIO_PORT 		GPIOA
#define DS18B20_GPIO_PIN 		GPIO_PIN_9
//#define DS18B20_RESOLUTION		BIT_12

//ROM Command Macros
#define CMD_READ_ROM			0x33
#define CMD_MATCH_ROM 			0x55
#define CMD_SKIP_ROM			0xCC
#define CMD_ALARM_SEARCH		0xEC

//Function Command Macros
#define CMD_CONVERT_T			0x44
#define CMD_WRITE_SCRATCHPAD	0x4E
#define CMD_READ_SCRATCHPAD 	0xBE
#define CMD_COPY_SCRATCHPAD		0x48
#define CMD_RECALL 				0xB8
#define CMD_READ_POWERSUPPLY	0xB4

void DS18B20_SetPinOutput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // Open-Drain for one-wire
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DS18B20_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief Sets the data pin as input.
 */
void DS18B20_SetPinInput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DS18B20_GPIO_PORT, &GPIO_InitStruct);
}

uint8_t DS18B20_Init(void) {
    uint8_t presence_status = 0;

    DS18B20_SetPinOutput();
    HAL_GPIO_WritePin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN, GPIO_PIN_RESET);
    delay_us(480); // 480us reset pulse

    DS18B20_SetPinInput();
    delay_us(70); // 70us wait for presence pulse

    if (HAL_GPIO_ReadPin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN) == GPIO_PIN_RESET) {
    	presence_status = 1; // Presence pulse detected
    } else {
    	presence_status = 0; // No presence pulse
    }

    delay_us(410); // Wait for the end of the time slot

    return presence_status;
}


void DS18B20_WriteBit(uint8_t bit) {
    DS18B20_SetPinOutput();
    if (bit) {
        // Write '1'
        HAL_GPIO_WritePin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN, GPIO_PIN_RESET);
        delay_us(6); // 6us low
        HAL_GPIO_WritePin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN, GPIO_PIN_SET);
        delay_us(64); // 64us high
    } else {
        // Write '0'
        HAL_GPIO_WritePin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN, GPIO_PIN_RESET);
        delay_us(60); // 60us low
        HAL_GPIO_WritePin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN, GPIO_PIN_SET);
        delay_us(10); // 10us high
    }
}

void DS18B20_WriteByte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        DS18B20_WriteBit(byte & 0x01);
        byte >>= 1;
    }
}

uint8_t DS18B20_ReadBit(void) {
    uint8_t bit = 0;
    DS18B20_SetPinOutput();
    HAL_GPIO_WritePin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN, GPIO_PIN_RESET);
    delay_us(2); // 2us low

    DS18B20_SetPinInput();
    delay_us(13); // Wait for sensor to respond
    bit = HAL_GPIO_ReadPin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN);
    delay_us(45); // Wait for the end of the time slot

    return bit;
}

uint8_t DS18B20_ReadByte(void) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte >>= 1;
        if (DS18B20_ReadBit()) {
            byte |= 0x80;
        }
    }
    return byte;
}


float DS18B20_ReadTemp(void) {
    uint8_t temp_lsb, temp_msb;
    float temp_celsius;
    int16_t temp_raw;

    if (!DS18B20_Init()) {
        return -127.0; // Error
    }

    DS18B20_WriteByte(CMD_SKIP_ROM);
    DS18B20_WriteByte(CMD_CONVERT_T);

    // Wait for conversion to complete
//    switch(DS18B20_RESOLUTION)
//    {
//      case BIT_9:
//        HAL_Delay(94);
//        break;
//      case BIT_10:
//        HAL_Delay(188);
//        break;
//      case BIT_11:
//        HAL_Delay(375);
//        break;
//      default:
//        HAL_Delay(750);
//        break;
//    }

    HAL_Delay(750); // 750ms for 12-bit resolution

    if (!DS18B20_Init()) {
        return -127.0; // Error
    }

    DS18B20_WriteByte(CMD_SKIP_ROM);
    DS18B20_WriteByte(CMD_READ_SCRATCHPAD);

    temp_lsb = DS18B20_ReadByte();
    temp_msb = DS18B20_ReadByte();

    // Combine the bytes to get the raw temperature value
    temp_raw = (int16_t)(((uint16_t)temp_msb << 8) | temp_lsb);

    // Convert raw value to Celsius
    // The temperature is stored as a 16-bit signed integer.
    // Resolution is 1/16th of a degree Celsius (0.0625).
    temp_celsius = (float)temp_raw * 0.0625;

    return temp_celsius;
}

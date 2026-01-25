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

void DS18B20_SetPinOutput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // Open-Drain for one-wire
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief Sets the data pin as input.
 */
void DS18B20_SetPinInput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

uint8_t DS18B20_Init(void) {
    uint8_t presence_status = 0;

    DS18B20_SetPinOutput();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    delay_us(480); // 480us reset pulse

    DS18B20_SetPinInput();
    delay_us(70); // 70us wait for presence pulse

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET) {
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
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        delay_us(6); // 6us low
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        delay_us(64); // 64us high
    } else {
        // Write '0'
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        delay_us(60); // 60us low
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
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
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    delay_us(2); // 2us low

    DS18B20_SetPinInput();
    delay_us(13); // Wait for sensor to respond
    bit = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
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

    DS18B20_WriteByte(0xCC);
    DS18B20_WriteByte(0x44);

    // Wait for conversion to complete
    HAL_Delay(750); // 750ms for 12-bit resolution

    if (!DS18B20_Init()) {
        return -127.0; // Error
    }

    DS18B20_WriteByte(0xCC);
    DS18B20_WriteByte(0xBE);

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

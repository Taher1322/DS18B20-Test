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


//This function sets the IO as Output with predefined output states
void DS18B20_SetPinOutput(void) {

	//Init the structure that holds all the defined field
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    //Define the Sensor Pin
    GPIO_InitStruct.Pin = DS18B20_GPIO_PIN;

    //Open-Drain for One-wire
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;

    //Define the State
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    //Define the Frequency
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    //Call Init
    HAL_GPIO_Init(DS18B20_GPIO_PORT, &GPIO_InitStruct);
}

//This function sets the IO as Input with predefined input states
void DS18B20_SetPinInput(void) {

	//Init the structure that holds all the defined field
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    //Define the Sensor Pin
    GPIO_InitStruct.Pin = DS18B20_GPIO_PIN;

    //Input for One-wire
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

    //Define the State
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    //Define the Frequency
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    //Call Init
    HAL_GPIO_Init(DS18B20_GPIO_PORT, &GPIO_InitStruct);
}


//This function enables Initialization of DS18B20 sensor
uint8_t DS18B20_Init(void) {

	//Define variables
    uint8_t presence_status = 0;

    //Set the Pin as Output
    DS18B20_SetPinOutput();

    //Send the Reset Pulse - 480us as defined by Datasheet Page 10
    //https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf
    HAL_GPIO_WritePin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN, GPIO_PIN_RESET);
    delay_us(480);

    //Set the Pin as Input
    DS18B20_SetPinInput();

    //Wait for Presence Pulse - It can be seen between 60us to 240us - Defined by Datasheet Page 15
    //https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf
    delay_us(70); // 70us wait for presence pulse

    //Read the Presence Pulse
    if (HAL_GPIO_ReadPin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN) == GPIO_PIN_RESET) {
    	//Presence pulse detected
    	presence_status = 1;
    } else {
    	//No presence pulse
    	presence_status = 0;
    }

    //Wait for the End
    delay_us(410);

    //Return Sensor Init Status
    return presence_status;
}

//This function writes a bit on the 1-wire bus
void DS18B20_WriteBit(uint8_t bit) {

	 //Set the Pin as Output
    DS18B20_SetPinOutput();
    if (bit) {
        //Condition to write 1
        HAL_GPIO_WritePin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN, GPIO_PIN_RESET);

        //Min delay as defined by the Datasheet - Page 16
        //https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf
        delay_us(6);
        HAL_GPIO_WritePin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN, GPIO_PIN_SET);

        //Min delay as defined by the Datasheet - Page 16
        //https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf
        delay_us(64);

    } else {

        //Condition to write 0
        HAL_GPIO_WritePin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN, GPIO_PIN_RESET);

        //Min delay as defined by the Datasheet - Page 16
        //https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf
        delay_us(60);

        //Min delay as defined by the Datasheet - Page 16
        //https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf
        HAL_GPIO_WritePin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN, GPIO_PIN_SET);
        delay_us(10);
    }
}


//This function writes a byte on the 1-wire bus
void DS18B20_WriteByte(uint8_t byte) {

	//Loop to extract Bytes into Bits and write it
    for (int i = 0; i < 8; i++) {
        DS18B20_WriteBit(byte & 0x01);
        byte >>= 1;
    }
}

//This function reads bit-by bit from 1-wire bus
uint8_t DS18B20_ReadBit(void) {

	//Define variable to read the bit
	uint8_t bit = 0;

	 //Set the Pin as Output
    DS18B20_SetPinOutput();

    //Min delay as defined by the Datasheet - Page 16
    //https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf
    HAL_GPIO_WritePin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN, GPIO_PIN_RESET);
    delay_us(2);

    //Set the Pin as Input
    DS18B20_SetPinInput();

    //Wait for the Sensor to Respond
    delay_us(13);

    //Read the bit value and store it
    bit = HAL_GPIO_ReadPin(DS18B20_GPIO_PORT, DS18B20_GPIO_PIN);

    //Min delay as defined by the Datasheet - Page 16
    //https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf
    delay_us(45);

    //Return the bit
    return bit;
}


//This function reads a byte from 1-wire bus
uint8_t DS18B20_ReadByte(void) {
    uint8_t byte = 0;

    //Loop to read bytes and store it
    for (int i = 0; i < 8; i++) {
        byte >>= 1;
        if (DS18B20_ReadBit()) {
            byte |= 0x80;
        }
    }
    return byte;
}


//This function reads 12-bit temperature using the write commands defined by the datasheet Page 18
//Using the Operation Example 1 sequence
float DS18B20_ReadTemp(void) {

	//Local variables to read raw temp data and convert into float value
    uint8_t temp_lsb, temp_msb;
    float temp_celsius;
    int16_t temp_raw;

    //Init and Error Check - Ensure the connections are good and the sensor is responding
    if (!DS18B20_Init()) {
    	//Return Error if Sensor is not connected or Doesn't work
        return -127.0;
    }

    //Send the Skip ROM Command
    DS18B20_WriteByte(CMD_SKIP_ROM);

    //Send the Convert T Command
    DS18B20_WriteByte(CMD_CONVERT_T);

    //Wait 750ms for 12-bit Resolution
    HAL_Delay(750);

    //Init and Error Check - Ensure the connections are good and the sensor is responding
    if (!DS18B20_Init()) {
    	//Return Error if Sensor does not respond
        return -127.0;
    }

    //Send the Skip ROM Command
    DS18B20_WriteByte(CMD_SKIP_ROM);

    //Send the Scratchpad command
    DS18B20_WriteByte(CMD_READ_SCRATCHPAD);

    //Read the Raw Temperature Values and Store LSB and MSB
    temp_lsb = DS18B20_ReadByte();
    temp_msb = DS18B20_ReadByte();

    //Combine the raw bytes into single temperature value
    temp_raw = (int16_t)(((uint16_t)temp_msb << 8) | temp_lsb);

    // Convert Raw value to Celsius
    // The temperature is stored as a 16-bit signed integer
    //Since 12-bit resolution is used - The resolution increments by 0.0625 degrees - Defined by Datasheet Page 5
    temp_celsius = (float)temp_raw * 0.0625;

    //Return Temperature
    return temp_celsius;
}

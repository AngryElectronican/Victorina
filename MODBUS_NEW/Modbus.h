#pragma once
#include <avr/interrupt.h>
#include <inttypes.h>
#include <stdlib.h>
#include <avr/io.h>

#include "Timer.h"
#include "CircleBuffer.h"
#include "USART.h"

//Slave Address
#define DEVICE_ADDRESS 10
//Error_codes
#define FUNCTION_UNSUPPORTED_ERROR 0x01
#define ADDRESSING_ERROR 0x02
#define QUANTILY_ERROR 0x03
#define MULTUPLY_UNSUPPORTED_ERROR 0x04
#define ADDRESS_NOT_MATCH_ERROR 0x05
//REG QUANTILY
#define REG_QUANTILY 16
//States
enum states{
    ADDRESS=0,
    COMMAND,
    READ_MULTIPLY_BITS,
    READ_MULTIPLY_REGISTERS,
    WRITE_SINGLE_BIT,
    WRITE_MULTIPLY_BIT,
    END_RESPONSE,
    ERROR_CYCLE
};

uint16_t ModRTU_CRC(uint8_t* buf, uint8_t len);
void ModRTU_Read_Bits(uint8_t* rx_mass, uint8_t* bits);
void ModRTU_Write_Bit(uint8_t* rx_mass);
void ModRTU_Write_Multiply_Bits(uint8_t* rx_mass);
void ModRTU_Init(void);
void ModRTU_TX(void);
void ModRTU_RX(void);
void ModRTU_Handler(void);

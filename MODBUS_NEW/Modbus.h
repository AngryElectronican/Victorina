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
    READ_REGISTER,
    WRITE_BIT,
    END_RESPONSE,
    ERROR_CYCLE
};

uint16_t ModRTU_CRC(uint8_t* buf, uint8_t len);
void ModRTU_Write_Bits(uint32_t* bits);
void ModRTU_Init(void);
void ModRTU_TX(void);
void ModRTU_RX(void);
void ModRTU_Handler(void);


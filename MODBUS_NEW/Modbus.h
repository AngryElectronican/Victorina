#pragma once
#include <inttypes.h>
#include <avr/io.h>

//Slave Address
#define DEVICE_ADDRESS 10
//Error_codes
#define FUNCTION_UNSUPPORTED_ERROR 0x01
#define ADDRESSING_ERROR 0x02
#define QUANTILY_ERROR 0x03
#define MULTUPLY_UNSUPPORTED_ERROR 0x04
#define ADDRESS_NOT_MATCH_ERROR 0x05
//States
#define ADDRESS 0
#define COMMAND 1
#define READ_REGISTER 2
#define WRITE_BIT 3
#define END_RESPONSE 4
#define ERROR_CYCLE 5


uint16_t ModRTU_CRC(uint8_t* buf, uint8_t len);
void ModRTU_Init(void);
void ModRTU_TX(void);
void ModRTU_RX(void);


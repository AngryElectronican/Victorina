#pragma once
#include <inttypes.h>
#include <avr/io.h>

#define DEVICE_ADDRESS 10

#define FUNCTION_UNSUPPORTED_ERROR 0x01
#define ADDRESSING_ERROR 0x02
#define QUANTILY_ERROR 0x03
#define MULTUPLY_UNSUPPORTED_ERROR 0x04
#define ADDRESS_NOT_MATCH_ERROR 0x05

uint16_t ModRTU_CRC(uint8_t* buf, uint8_t len);

void ModRTU_Init(void);
void ModRTU_TX(void);
void ModRTU_RX(void);


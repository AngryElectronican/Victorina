#ifndef USART_H_
#define USART_H_

#include <inttypes.h>
#include <avr/io.h>

void USART_Init();
void USART_Write(uint8_t* data);
uint8_t USART_Read(void);

#endif 

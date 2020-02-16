#include "USART.h"

void USART_Init(){
  	DDRD|=1<<PD1;
	UCSR0B|=(1<<RXCIE0)|(1<<TXEN0)|(1<<RXEN0);
	UCSR0C|=(1<<UCSZ01)|(1<<UCSZ00)|(1<<UPM01);
	UBRR0L = 103;
}
void USART_Write(uint8_t* data){
	while (!(UCSR0A&(1<<UDRE0)));
	UDR0 = *data;
}
uint8_t USART_Read(void){
	while (!(UCSR0A&(1<<RXC0)));
	return UDR0;
}

#include "ModbusCRC.h"

uint16_t ModRTU_CRC(uint8_t* buf, uint8_t len)
{
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
        crc >>= 1;
    }
  }
  return crc;
}
void ModRTU_Init(){
  DDRD|=1<<PD2;
  DDRB|=1<<PB5;
}
void ModRTU_TX(){
  PORTD|=1<<PD2;
  PORTB|=1<<PB5;
}
void ModRTU_RX(){
  PORTD&=~(1<<PD2);
  PORTB&=~(1<<PB5);
}
#include "Modbus.h"

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
void ModRTU_Write_Bits(uint32_t* bits){
  for(uint8_t i=3;i<8;i++){ // PD3,PD4,PD5,PD6,PD7
    DDRD|=1<<i;
    if((((*bits)>>i)&0x01)==1){
      PORTD|=1<<i;
    }else{
      PORTD&=~(1<<i);
    }
  for(uint8_t i=0;i<6;i++){ //PB0,PB1,PB2,PB3,PB4,PB5
    DDRB|=1<<i;
    if((((*bits)>>(i+5))&0x01)==1){
      PORTB|=1<<i;
    }else{
      PORTB&=~(1<<i);
    }
  }
  for(uint8_t i=0;i<6;i++){
    DDRC|=1<<i;
    if((((*bits)>>(i+11))&0x01)==1){
      PORTC|=1<<i;
    }else{
      PORTC&=~(1<<i);
    }
  }
  }
}

void ModRTU_Init(){
  DDRD|=1<<PD2;
  DDRB|=1<<PB5;
}
void ModRTU_TX(){
  PORTD|=1<<PD2;
  //PORTB|=1<<PB5;
}
void ModRTU_RX(){
  PORTD&=~(1<<PD2);
  //PORTB&=~(1<<PB5);
}
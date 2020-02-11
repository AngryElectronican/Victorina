#include "Modbus.h"

uint8_t frame_time=4;
Time timer1=0;
Time timer2=0;
CircleBUF FIFO;


uint16_t REGISTER[REG_QUANTILY]={
  0x0000,0x0101,0x0202,0x0303,0x0404,0x0505,0x0606,0x0707,
  0x0808,0x0909,0x0A0A,0x0B0B,0x0C0C,0x0D0D,0x0E0E,0x0F0F
};
//uint16_t REGISTER[16]={0};
uint32_t output_bits=0x0000;

uint8_t rx_data[16];
uint8_t state=ADDRESS;
uint8_t rx_counter=0;
uint8_t error_code=0;

ISR(USART_RX_vect){
  UCSR0B&=~(1<<RXCIE0);
  buf_push(&FIFO,UDR0);
  UCSR0B|=1<<RXCIE0;
}
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
uint32_t ModRTU_Read_Bits(uint8_t* address,uint8_t* quantilly){
  uint32_t bits=0;
  uint8_t out_bit=0;
  for(uint8_t i=*address;i<(*address)+(*quantilly);i++){
    if((*address)>0 && (*address)<5){
        if((PORTD>>i+3)&0x01){
          bits|=1<<out_bit;
        }else{
          bits&=~(1<<out_bit);
        }
      }
    if((*address)>5 && (*address)<11){
        if((PORTB>>i)&0x01){
          bits|=1<<out_bit;
        }else{
          bits&=~(1<<out_bit);
        }
    }
    if((*address)>11 && (*address)<17){
        if((PORTC>>i)&0x01){
          bits|=1<<out_bit;
        }else{
          bits&=~(1<<out_bit);
        }
    }
    out_bit++;
  }
  return bits;
}
void ModRTU_Write_Bits(uint32_t* bits){
  for(uint8_t i=3;i<8;i++){ // PD3,PD4,PD5,PD6,PD7
    DDRD|=1<<i;
    if((((*bits)>>i)&0x01)==1){
      PORTD|=1<<i;
    }else{
      PORTD&=~(1<<i);
    }
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
void ModRTU_Handler(void){
switch(state){
  case ADDRESS:
    if(buf_available(&FIFO)){
    Timer0_StartTimer(&timer1);
    rx_data[rx_counter]=buf_pull(&FIFO);
    if(rx_data[rx_counter]!=DEVICE_ADDRESS){
      error_code=ADDRESS_NOT_MATCH_ERROR;
      }else{
        rx_counter++;
        state=COMMAND;
        }
    }
  break;
  case COMMAND:
    if(buf_available(&FIFO)){
    Timer0_StartTimer(&timer1);
    rx_data[rx_counter]=buf_pull(&FIFO);
    switch(rx_data[rx_counter]){
      case 0x01:
        state=READ_MULTIPLY_COILS;
      break;
      case 0x03:
        state=READ_MULTIPLY_REGISTERS;
      break;
      case 0x05:
        state=WRITE_BIT;
      break;
      default:
        error_code=0x01;
        state=ERROR_CYCLE;
      }
    rx_counter++;
    }
  break;
  case READ_MULTIPLY_COILS:
    if(buf_available(&FIFO)){
      Timer0_StartTimer(&timer1);
      rx_data[rx_counter]=buf_pull(&FIFO);
      rx_counter++;
      if(rx_counter>=8){
        uint16_t CRC_rx=(uint16_t)(rx_data[rx_counter-1]<<8 | rx_data[rx_counter-2]);
        uint16_t CRC_calc=ModRTU_CRC(rx_data,rx_counter-2);
        if(CRC_calc==CRC_rx){
          uint32_t read_bits=ModRTU_Read_Bits(&rx_data[3],&rx_data[5]);
          uint8_t byte_count=0;
          if(rx_data[5]%8){
            byte_count=(rx_data[5]/8)+1;
          }else{
            byte_count=rx_data[5]/8;
          }
          uint8_t tx_size=5+byte_count;
          uint8_t *tx_data=(uint8_t*)malloc(tx_size);
          tx_data[0]=rx_data[0];
          tx_data[1]=rx_data[1];
          tx_data[2]=byte_count;
          for(uint8_t i=0;i<byte_count;i++){
            tx_data[i+3]=(read_bits>>(i*8))&0xFF;
          }
          uint16_t CRC_tx=ModRTU_CRC(tx_data,tx_size-2);
          tx_data[tx_size-2]=CRC_tx & 0xFF; //LOW byte
          tx_data[tx_size-1]=(CRC_tx>>8) & 0xFF; //HIGH byte
          ModRTU_TX();
          for(uint8_t i=0;i<tx_size;i++){
            USART_Write(tx_data+i);
            Timer0_StartTimer(&timer1);
            }
          free(tx_data);
          Timer0_StartTimer(&timer2);
          state=END_RESPONSE;
        }
      }
    }
  break;
  case READ_MULTIPLY_REGISTERS:
  if(buf_available(&FIFO)){
    Timer0_StartTimer(&timer1);
    rx_data[rx_counter]=buf_pull(&FIFO);
    rx_counter++;
    if(rx_counter>=8){
      uint16_t CRC_rx=(uint16_t)(rx_data[rx_counter-1]<<8 | rx_data[rx_counter-2]);
      uint16_t CRC_calc=ModRTU_CRC(rx_data,rx_counter-2);
      if(CRC_calc==CRC_rx){

        if(rx_data[5]<1 || rx_data[5]>REG_QUANTILY){
          state=ERROR_CYCLE;
          error_code=0x03;
          break;
        }
        if((rx_data[3]>REG_QUANTILY) || ((rx_data[3]+rx_data[5])>REG_QUANTILY)){
          state=ERROR_CYCLE;
          error_code=0x02;
          break; 
        }
        uint8_t tx_size=5+rx_data[5]*2;
        uint8_t *tx_data=(uint8_t*)malloc(tx_size);
        tx_data[0]=rx_data[0];
        tx_data[1]=rx_data[1];
        tx_data[2]=rx_data[5]*2;
        for(uint8_t i=3;i<(tx_size-2);i=i+2){
        uint8_t reg_index=((i-3)>>1)+rx_data[3];
        tx_data[i]=(REGISTER[reg_index]>>8) & 0xFF; //HIGH byte
        tx_data[i+1]=REGISTER[reg_index] & 0xFF; //LOW byte
        }
        uint16_t CRC_tx=ModRTU_CRC(tx_data,tx_size-2);
        tx_data[tx_size-2]=CRC_tx & 0xFF; //LOW byte
        tx_data[tx_size-1]=(CRC_tx>>8) & 0xFF; //HIGH byte
        ModRTU_TX();
        for(uint8_t i=0;i<tx_size;i++){
          USART_Write(tx_data+i);
          Timer0_StartTimer(&timer1);
          }
        free(tx_data);
        Timer0_StartTimer(&timer2);
        state=END_RESPONSE;
        }
      }
    }
  break;
  case WRITE_BIT:
    if(buf_available(&FIFO)){
      Timer0_StartTimer(&timer1);
      rx_data[rx_counter]=buf_pull(&FIFO);
      rx_counter++;
      if(rx_counter>=8){
        uint16_t CRC_rx=(uint16_t)(rx_data[rx_counter-1]<<8 | rx_data[rx_counter-2]);
        uint16_t CRC_calc=ModRTU_CRC(rx_data,rx_counter-2);
          if(CRC_calc==CRC_rx){
            uint8_t tx_size=8;
            uint8_t *tx_data=(uint8_t*)malloc(tx_size);
            uint16_t pin_address=rx_data[2]<<8 | rx_data[3];//6 max outputs
            if(rx_data[4]==0xFF && rx_data[5]==0x00 ){
              output_bits|=1<<pin_address;
            }else if(rx_data[4]==0x00 && rx_data[5]==0x00){
              output_bits&=~(1<<pin_address);
            }
            DDRD|=1<<PD3;
            PORTD|=1<<PD3;
            ModRTU_Write_Bits(&output_bits);
            for(uint8_t i=0;i<8;i++){
              tx_data[i]=rx_data[i];
            }
            ModRTU_TX();
            for(uint8_t i=0;i<8;i++){
              USART_Write(tx_data+i);
              Timer0_StartTimer(&timer1);
              }
            free(tx_data);
            Timer0_StartTimer(&timer2);
            state=END_RESPONSE;
        }
      }
    }
  break;
  case ERROR_CYCLE:
    if(buf_available(&FIFO)){
      Timer0_StartTimer(&timer1);
      uint8_t temp=buf_pull(&FIFO);
      }
    if(Timer0_TimeIsOut(&timer1,frame_time)){
      if(error_code!=ADDRESS_NOT_MATCH_ERROR){
        
        uint8_t *tx_data=(uint8_t*)malloc(5);
        tx_data[0]=rx_data[0];
        tx_data[1]=0x80+rx_data[1];
        tx_data[2]=error_code;
        error_code=0;
        uint16_t CRC_tx=ModRTU_CRC(tx_data,3);
        tx_data[3]=CRC_tx & 0xFF; //LOW byte
        tx_data[4]=(CRC_tx>>8) & 0xFF; //HIGH byte
        ModRTU_TX();
        for(uint8_t i=0;i<5;i++){
          USART_Write(tx_data+i);
          Timer0_StartTimer(&timer1);
          }
        free(tx_data);
        Timer0_StartTimer(&timer2);
        state=END_RESPONSE;
        }
      }
  break;
  case END_RESPONSE:
    if(Timer0_TimeIsOut(&timer2,frame_time)){
      state=ADDRESS;
      rx_counter=0;
      ModRTU_RX();
      }
  break;
  }
   if(Timer0_TimeIsOut(&timer1,frame_time*2)){
    state=0;
    rx_counter=0;
    ModRTU_RX();
    } 
}
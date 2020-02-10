#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "Timer.h"
#include "CircleBuffer.h"
#include "USART.h"
#include "ModbusCRC.h"
#include "States.h"

uint8_t frame_time=4;
Time timer1=0;
Time timer2=0;
CircleBUF FIFO;

uint8_t rx_data[16];
uint16_t REGISTER[16]={
  0x0000,0x0101,0x0202,0x0303,0x0404,0x0505,0x0606,0x0707,
  0x0808,0x0909,0x0A0A,0x0B0B,0x0C0C,0x0D0D,0x0E0E,0x0F0F
};
uint8_t state=ADDRESS;
uint8_t rx_counter=0;
uint8_t error_code=0;

ISR(USART_RX_vect){
  UCSR0B&=~(1<<RXCIE0);
  buf_push(&FIFO,UDR0);
  UCSR0B|=1<<RXCIE0;
}

int main(void){
  USART_Init();
  Timer0_Init();
  ModRTU_Init();
  sei();
  Timer0_StartTimer(&timer1);
  while(1){
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
            case 0x03:
              state=READ_REGISTER;
            break;
            default:
              error_code=0x01;
              state=ERROR_CYCLE;
            }
          rx_counter++;
          }
        break;
        case READ_REGISTER:
        if(buf_available(&FIFO)){
          Timer0_StartTimer(&timer1);
          rx_data[rx_counter]=buf_pull(&FIFO);
          rx_counter++;
          if(rx_counter>=8){
            uint16_t CRC_rx=(uint16_t)(rx_data[rx_counter-1]<<8 | rx_data[rx_counter-2]);
            uint16_t CRC_calc=ModRTU_CRC(rx_data,rx_counter-2);
            if(CRC_calc==CRC_rx){
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
            REGISTER[0]++;
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
}

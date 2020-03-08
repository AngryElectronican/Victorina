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

uint8_t rx_data[32];
uint8_t state=ADDRESS;
uint8_t rx_counter=0;
uint8_t error_code=0;

ISR(USART_RX_vect){
  UCSR0B&=~(1<<RXCIE0);
  buf_push(&FIFO,UDR0);
  UCSR0B|=1<<RXCIE0;
}
void ModRTU_Init(){
  DDRD|=1<<PD2; // TX/RX pin
  USART_Init();
  Timer0_Init();
  sei();
}
void ModRTU_TX(){
  PORTD|=1<<PD2;
}
void ModRTU_RX(){
  PORTD&=~(1<<PD2);
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
void ModRTU_Read_Bits(uint8_t* rx_data, uint8_t* bits){
  uint16_t address=rx_data[2]<<8 | rx_data[3];
  uint16_t quantily=(rx_data[4]<<8) | rx_data[5];
  uint8_t out_bit=0;
  uint8_t byte_count=0;
  for(uint8_t i=address;i<address+quantily;i++){
    byte_count=out_bit/8;
    if(i>=0 && i<5){
        if((PIND>>(i+3))&0x01){
          bits[byte_count]|=1<<(out_bit-8*byte_count);
        }else{
          bits[byte_count]&=~(1<<(out_bit-8*byte_count));
        }
      }
    else if(i>=5 && i<11){
        if((PINB>>(i-5))&0x01){
          bits[byte_count]|=1<<(out_bit-8*byte_count);
        }else{
          bits[byte_count]&=~(1<<(out_bit-8*byte_count));
        }
      }
    else if(i>=11 && i<17){
        if((PINC>>(i-11))&0x01){
          bits[byte_count]|=1<<(out_bit-8*byte_count);
        }else{
          bits[byte_count]&=~(1<<(out_bit-8*byte_count));
        }
      }
    out_bit++;
  }
}
void ModRTU_Write_Bit(uint8_t* rx_data){
  uint16_t address=rx_data[2]<<8 | rx_data[3];//6 max outputs
  uint8_t value=0;
  if(rx_data[4]==0xFF && rx_data[5]==0x00 ){
    value=1;
    }else if(rx_data[4]==0x00 && rx_data[5]==0x00){
      value=0;
      }
if(address>=0 && address<5){
  if(value){
    DDRD|=1<<(address+3);
    PORTD|=1<<(address+3);
    }else{
      DDRD|=1<<(address+3);
      PORTD&=~(1<<(address+3));
      }
    }
  if(address>=5 && address<11){
  if(value){
    DDRB|=1<<(address-5);
    PORTB|=1<<(address-5);
    }else{
      DDRB|=1<<(address-5);
      PORTB&=~(1<<(address-5));
      }
    }
  if(address>=11 && address<17){
  if(value){
    DDRC|=1<<(address-11);
    PORTC|=1<<(address-11);
    }else{
      DDRC|=1<<(address-11);
      PORTC&=~(1<<(address-11));
      }
  }
}
void ModRTU_Write_Multiply_Bits(uint8_t* rx_mass){ //////////
  uint16_t address=rx_data[2]<<8 | rx_data[3];
  uint16_t quantily=(rx_data[4]<<8)|rx_data[5];
  uint8_t byte_count=rx_data[6];
  uint8_t* bits_buf=malloc(byte_count);
  for(uint8_t i=0;i<byte_count;i++){
    bits_buf[i]=rx_mass[i+7];
  }
    for(uint8_t i=address;i<quantily+address;i++){
      uint8_t byte_num=(i-address)/8;
      if(i>=0 && i<5){
        if((bits_buf[byte_num]>>(quantily-byte_num*8-1))&0x01){
        DDRD|=1<<(i+3);
        PORTD|=1<<(i+3);
        }else{
          DDRD|=1<<(i+3);
          PORTD&=~(1<<(i+3));
          }
        }
      if(i>=5 && i<11){
        if((bits_buf[byte_num]>>(quantily-byte_num*8-1))&0x01){
          DDRB|=1<<(i-5);
          PORTB|=1<<(i-5);
          }else{
            DDRB|=1<<(i-5);
            PORTB&=~(1<<(i-5));
            }
        }
      if(i>=11 && i<17){
        if((bits_buf[byte_num]>>(quantily-byte_num*8-1))&0x01){
          DDRC|=1<<(i-11);
          PORTC|=1<<(i-11);
          }else{
            DDRC|=1<<(i-11);
            PORTC&=~(1<<(i-11));
            }
        }
      }
      free(bits_buf);
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
        state=READ_MULTIPLY_BITS;
      break;
      case 0x03:
        state=READ_MULTIPLY_REGISTERS;
      break;
      case 0x05:
        state=WRITE_SINGLE_BIT;
      break;
      case 0x0F:
        state=WRITE_MULTIPLY_BIT;
      break;
      case 0x10:
        state=WRITE_MULTIPLY_REGISTER;
      break;
      default:
        error_code=0x01;
        state=ERROR_CYCLE;
      }
    rx_counter++;
    }
  break;
  case READ_MULTIPLY_BITS:
    if(buf_available(&FIFO)){
      Timer0_StartTimer(&timer1);
      rx_data[rx_counter]=buf_pull(&FIFO);
      rx_counter++;
      if(rx_counter>=8){
        uint16_t CRC_rx=(uint16_t)(rx_data[rx_counter-1]<<8 | rx_data[rx_counter-2]);
        uint16_t CRC_calc=ModRTU_CRC(rx_data,rx_counter-2);
        if(CRC_calc==CRC_rx){
		  uint16_t bits_quantily=(rx_data[4]<<8) | (rx_data[5]);
		  if((bits_quantily==0) || (bits_quantily>32)){
			state=ERROR_CYCLE;
			error_code=0x03;
			break;
		  }
		  uint16_t starting_address=(rx_data[2]<<8) | (rx_data[3]);
		  if((starting_address>32) || (bits_quantily+starting_address>32)){
			state=ERROR_CYCLE;
			error_code=0x02;
			break;
		  }
          uint8_t byte_count=0;
          if(((rx_data[4]<<8) | rx_data[5])%8){
            byte_count=(((rx_data[4]<<8) | rx_data[5])/8)+1;
          }else{
            byte_count=((rx_data[4]<<8) | rx_data[5])/8;
          }
          uint8_t* read_bits=malloc(byte_count);
          for(uint8_t i=0;i<byte_count;i++){
            read_bits[i]=0x00;
          }
          ModRTU_Read_Bits(rx_data,read_bits);
          uint8_t tx_size=5+byte_count;
          uint8_t *tx_data=(uint8_t*)malloc(tx_size);
          tx_data[0]=rx_data[0];
          tx_data[1]=rx_data[1];
          tx_data[2]=byte_count;
          for(uint8_t i=0;i<byte_count;i++){
            tx_data[i+3]=read_bits[i];
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
          free(read_bits);
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
		uint16_t bits_quantily=(rx_data[4]<<8) | (rx_data[5]);
        if(bits_quantily<1 || bits_quantily>REG_QUANTILY){
          state=ERROR_CYCLE;
          error_code=0x03;
          break;
        }
		uint16_t starting_address=(rx_data[2]<<8) | (rx_data[3]);
        if((starting_address>REG_QUANTILY) || ((bits_quantily+starting_address)>REG_QUANTILY)){
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
  case WRITE_SINGLE_BIT:
    if(buf_available(&FIFO)){
      Timer0_StartTimer(&timer1);
      rx_data[rx_counter]=buf_pull(&FIFO);
      rx_counter++;
      if(rx_counter>=8){
        uint16_t CRC_rx=(uint16_t)(rx_data[rx_counter-1]<<8 | rx_data[rx_counter-2]);
        uint16_t CRC_calc=ModRTU_CRC(rx_data,rx_counter-2);
          if(CRC_calc==CRC_rx){
			if(!(rx_data[4]==0xFF && rx_data[5]==0x00)&& !(rx_data[4]==0x00 && rx_data[5]==0x00) ){
				state=ERROR_CYCLE;
				error_code=0x03;
				break;
				}
			uint16_t starting_address=(rx_data[2]<<8) | (rx_data[3]);
			if(starting_address>17){
				state=ERROR_CYCLE;
				error_code=0x02;
				break;
				}
            uint8_t tx_size=8;
            uint8_t *tx_data=(uint8_t*)malloc(tx_size);
            ModRTU_Write_Bit(rx_data);
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
  case WRITE_MULTIPLY_BIT:
    if(buf_available(&FIFO)){
    Timer0_StartTimer(&timer1);
    rx_data[rx_counter]=buf_pull(&FIFO);
    rx_counter++;
    static uint8_t quantily_bytes=0;
    if(rx_counter==7){
      quantily_bytes=rx_data[6]+9;
    }
    if(rx_counter>=quantily_bytes){
      uint16_t CRC_rx=(uint16_t)(rx_data[rx_counter-1]<<8 | rx_data[rx_counter-2]);
      uint16_t CRC_calc=ModRTU_CRC(rx_data,rx_counter-2);
        if(CRC_calc==CRC_rx){
		  uint16_t bits_quantily=(rx_data[4]<<8) | (rx_data[5]);
		  uint8_t byte_count=0;
		  if(bits_quantily%8){
			  byte_count=(bits_quantily/8)+1;
		  }else{
			  byte_count=bits_quantily/8;
		  }
		  if(bits_quantily<1 || bits_quantily>17 || byte_count!=rx_data[6]){
				state=ERROR_CYCLE;
				error_code=QUANTILY_ERROR;
				break;
		  }
		  uint16_t starting_address=(rx_data[2]<<8) | (rx_data[3]);
		  if(starting_address>32 || starting_address+bits_quantily>32){
				state=ERROR_CYCLE;
				error_code=ADDRESSING_ERROR;
				break;  
		  }
          uint8_t tx_size=8;
          uint8_t *tx_data=(uint8_t*)malloc(tx_size);
          ModRTU_Write_Multiply_Bits(rx_data);
          for(uint8_t i=0;i<6;i++){
            tx_data[i]=rx_data[i];
          }
          uint16_t CRC_tx=ModRTU_CRC(tx_data,6);
          tx_data[tx_size-2]=CRC_tx & 0xFF; //LOW byte
          tx_data[tx_size-1]=(CRC_tx>>8) & 0xFF; //HIGH byte
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
  case WRITE_MULTIPLY_REGISTER:
    if(buf_available(&FIFO)){
      Timer0_StartTimer(&timer1);
      rx_data[rx_counter]=buf_pull(&FIFO);
      rx_counter++;
      static uint8_t quantily_bytes=0;
      if(rx_counter==7){
        quantily_bytes=rx_data[6]+9;
      }
      if(rx_counter>=quantily_bytes){
        uint16_t CRC_rx=(uint16_t)(rx_data[rx_counter-1]<<8 | rx_data[rx_counter-2]);
        uint16_t CRC_calc=ModRTU_CRC(rx_data,rx_counter-2);
          if(CRC_calc==CRC_rx){
            uint16_t quantily_of_registers=rx_data[4]<<8 | rx_data[5];
            if((quantily_of_registers<1 || quantily_of_registers>16)||((quantily_of_registers*2) != (rx_data[6]))){
              state=ERROR_CYCLE;
              error_code=QUANTILY_ERROR;
              break;
              }
            uint16_t starting_address=(rx_data[2]<<8) | (rx_data[3]);
            if((starting_address>16) || (starting_address+quantily_of_registers>16)){
              state=ERROR_CYCLE;
              error_code=ADDRESSING_ERROR;
              break;
              }
              for(uint16_t i=starting_address;i<starting_address+quantily_of_registers;i++){
                REGISTER[i]=rx_data[7+i*2]<<8 |rx_data[8+i*2];
              }
            uint8_t tx_size=8;
            uint8_t *tx_data=(uint8_t*)malloc(tx_size);
            for(uint8_t i=0;i<5;i++){
              tx_data[i]=rx_data[i];
            }
                        uint16_t CRC_tx=ModRTU_CRC(tx_data,6);
            tx_data[tx_size-2]=CRC_tx & 0xFF; //LOW byte
            tx_data[tx_size-1]=(CRC_tx>>8) & 0xFF; //HIGH byte
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

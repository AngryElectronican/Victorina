#include "CircleBuffer.h"

void buf_push(CircleBUF* buffer, uint8_t data){
  buffer->buf[buffer->head]=data;
  if((buffer->head==buffer->tail) && (buffer->count)){
    DDRB|=1<<PB5;
    PORTB^=1<<PB5;
    buffer->tail++;
    buffer->tail=buffer->tail&MASK;
    
    buffer->head++;
    buffer->head=buffer->head&MASK;
   }else{
     buffer->head++;
     buffer->head=buffer->head&MASK;
     
     if(buffer->count>=SIZE){
      buffer->count=SIZE;
      }else{
        buffer->count++;
      }
   }
}
uint8_t buf_pull(CircleBUF* buffer){
    UCSR0B&=~(1<<RXCIE0);
    uint8_t data=buffer->buf[buffer->tail];
    buffer->tail=(buffer->tail+1)&MASK;
    buffer->count--;
    UCSR0B|=1<<RXCIE0;
    return data;
}
uint8_t buf_available(CircleBUF* buffer){
    return buffer->count;
}

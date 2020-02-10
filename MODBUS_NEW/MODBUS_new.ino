#include <inttypes.h>
#include "Modbus.h"

int main(void){
  USART_Init();
  Timer0_Init();
  ModRTU_Init();
  sei();
  while(1){
    ModRTU_Handler();
  }
}

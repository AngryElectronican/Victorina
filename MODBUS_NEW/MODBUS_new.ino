#include <inttypes.h>
#include "Modbus.h"

int main(void){
  ModRTU_Init();
  while(1){
    ModRTU_Handler();
  }
}

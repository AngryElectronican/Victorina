#include "Timer.h"

static volatile Time GlobalTime=0;

ISR(TIMER0_COMPA_vect){
	GlobalTime++;
}
void Timer0_Init(){ // 1 ms 16 000 000 Hz
	TCCR0A|=(1<<WGM01);
	TCCR0B|=(1<<CS00)|(1<<CS01); //devider 64
	TIMSK0|=(1<<OCIE0A);
	OCR0A=250;
	sei();
}
Time Timer0_GetTime(){
	return GlobalTime;
}
void Timer0_StartTimer(Time* StartTime){
	*StartTime=GlobalTime;
}
uint8_t Timer0_TimeIsOut(Time* StartTime,Time Delay){
	return (GlobalTime-(*StartTime)>=Delay);
}

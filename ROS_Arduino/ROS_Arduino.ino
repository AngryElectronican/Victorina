#include <PID_v1.h>
#define enc1pinA 21
#define enc1pinB 20

#define enc2pinA 19
#define enc2pinB 18

#define EN1 3
#define M1 4

#define EN2 5
#define M2 6

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

volatile long int enc1Ticks=0,enc2Ticks=0;
unsigned long currentTime,prevTime,

void setup() {
pinMode(enc1pinA, INPUT);
pinMode(enc1pinB, INPUT);

pinMode(enc2pinA, INPUT);
pinMode(enc2pinB, INPUT);

attachInterrupt(2, Enc1A, CHANGE);
attachInterrupt(3, Enc1B, CHANGE);
 
attachInterrupt(4, Enc2A, CHANGE);
attachInterrupt(5, Enc2B, CHANGE); 

Input = 0;
Setpoint = 0;
myPID.SetMode(AUTOMATIC);
}

void loop() {
//get velocity
//input=velocity

}
void Enc1A(){
  if(digitalRead(enc1pinA)){
      !digitalRead(enc1pinB)?enc1Ticks++:enc1Ticks--;
    }else{
      digitalRead(enc1pinB)?enc1Ticks++:enc1Ticks--;
    }
}
void Enc1B(){
  if(digitalRead(enc1pinB)){
      digitalRead(enc1pinA)?enc1Ticks++:enc1Ticks--;
    }else{
      !digitalRead(enc1pinA)?enc1Ticks++:enc1Ticks--;
    }
}
void Enc2A(){
  if(digitalRead(enc2pinA)){
      !digitalRead(enc2pinB)?enc2Ticks++:enc2Ticks--;
    }else{
      digitalRead(enc2pinB)?enc2Ticks++:enc2Ticks--;
    }
}
void Enc2B(){
  if(digitalRead(enc2pinB)){
      digitalRead(enc2pinA)?enc2Ticks++:enc2Ticks--;
    }else{
      !digitalRead(enc2pinA)?enc2Ticks++:enc2Ticks--;
    }
}

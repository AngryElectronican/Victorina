#include <PID_v1.h>
#include <Math.h>
#define R_WHELL 0.005 //meters
#define WHELL_LEN (2*PI*R_WHELL)
#define TICKS_PER_REVOLUTION 1450
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
long int ticks1=0,prev_ticks1=0, ticks2=0, prev_ticks2=0;
long int dTicks1=0, dTicks2=0;
unsigned long currentTime=0,prevTime=0,dT=0;

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
prevTime=millis();
}

void loop() {
  currentTime=millis();
  if(currentTime-prevTime>=100){
      dT=currentTime-prevTime; //dT
      prevTime=currentTime;
      
      ticks1=enc1Ticks;
      dTicks1=ticks1-prev_ticks1; //DTicks1
      prev_ticks1=ticks1;
      
      ticks2=enc2Ticks;
      dTicks2=ticks2-prev_ticks2; //DTicks2
      prev_ticks2=ticks2;
      
      double velocity1=(dTicks1*WHELL_LEN)/TICKS_PER_REVOLUTION;
      velocity1=velocity1/dT;

      double velocity2=(dTicks2*WHELL_LEN)/TICKS_PER_REVOLUTION;
      velocity2=velocity2/dT;
      Serial.println("VELOCITYS");
      Serial.print(velocity1);Serial.print("\t");Serial.println(velocity2);
      Serial.println("TICKS");
      Serial.print(ticks1);Serial.print("\t");Serial.println(ticks2);
      Serial.println("");
    }
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
void MotorSpeed(boolean Motor,int Motor_Speed){
  if(Motor){
      (Motor_Speed<0) ? digitalWrite(M2,LOW) :digitalWrite(M2,HIGH);
      analogWrite(EN2,Motor_Speed);
    }else{
       (Motor_Speed<0) ? digitalWrite(M1,LOW) :digitalWrite(M1,HIGH);
       analogWrite(EN1,Motor_Speed);
    }
  }

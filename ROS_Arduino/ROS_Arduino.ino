#define ROS
#ifdef ROS
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#endif

#include "PID_v1.h"
#include <math.h>
#define R_WHELL 0.0485 //meters
//#define WHELL_LEN (2*PI*R_WHELL)
#define WHELL_LEN 0.3047344873982099
#define TICKS_PER_REVOLUTION 1450
#define enc1pinA 21
#define enc1pinB 20

#define enc2pinA 19
#define enc2pinB 18

#define EN1 3
#define M1 4

#define EN2 5
#define M2 6

int Kp=200,Ki=1200,Kd=0;

double Setpoint1, Input1, Output1;
PID myPID1(&Input1, &Output1, &Setpoint1,Kp,Ki,Kd, DIRECT);

double Setpoint2, Input2, Output2;
PID myPID2(&Input2, &Output2, &Setpoint2,Kp,Ki,Kd, DIRECT);

volatile long int enc1Ticks=0,enc2Ticks=0;
long int ticks1=0,prev_ticks1=0, ticks2=0, prev_ticks2=0;
long int dTicks1=0, dTicks2=0;
long double currentTime=0,prevTime=0,dT=0;

int SampleTime=10;

#ifdef ROS
ros::NodeHandle nh;
geometry_msgs::Twist motors_speed;
void velCallback(  const geometry_msgs::Twist &vel)
{
  Setpoint1 = vel.linear.x;
  Setpoint2=Setpoint1;
}

ros::Publisher actual_speed("actual_speed", &motors_speed);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);
#endif

void setup() {
#ifdef ROS
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(actual_speed);

#endif
  //Serial.begin(9600);
  
  pinMode(enc1pinA, INPUT);
  pinMode(enc1pinB, INPUT);
  
  pinMode(enc2pinA, INPUT);
  pinMode(enc2pinB, INPUT);
  
  attachInterrupt(2, Enc1A, CHANGE);
  attachInterrupt(3, Enc1B, CHANGE);
   
  attachInterrupt(4, Enc2A, CHANGE);
  attachInterrupt(5, Enc2B, CHANGE); 
  
  Input1 = 0;
  Setpoint1 = 0.0;
  Input2 = 0;
  Setpoint2 = 0.0;
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetSampleTime(SampleTime);
  myPID1.SetOutputLimits(-255,255);
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetSampleTime(SampleTime);
  myPID2.SetOutputLimits(-255,255);
  prevTime=millis();
}

void loop() {
  currentTime=millis();
  if(currentTime-prevTime>=10){
      dT=currentTime-prevTime; //dT
      prevTime=currentTime;
      
      ticks1=enc1Ticks;
      dTicks1=ticks1-prev_ticks1; //DTicks1
      prev_ticks1=ticks1;
      
      ticks2=enc2Ticks;
      dTicks2=ticks2-prev_ticks2; //DTicks2
      prev_ticks2=ticks2;
      
      double velocity1=(dTicks1*1000*WHELL_LEN)/TICKS_PER_REVOLUTION;
      velocity1=velocity1/dT;

      double velocity2=(dTicks2*1000*WHELL_LEN)/TICKS_PER_REVOLUTION;
      velocity2=velocity2/dT;
      Input1=velocity1;
      Input2=velocity2;
      motors_speed.linear.x=velocity1;
      motors_speed.linear.y=velocity2;
      myPID1.Compute();
      myPID2.Compute();
      if(Setpoint1==0 && Setpoint2==0){
        MotorSpeed(1,0);
        MotorSpeed(0,0);
      }else{
      MotorSpeed(1,Output1);
      MotorSpeed(0,Output2);
      }
      //Serial.println("VELOCITYS");
      //Serial.print(velocity1);Serial.print("\t");Serial.println(velocity2);
      //Serial.println("TICKS");
      //Serial.print(ticks1);Serial.print("\t");Serial.println(ticks2);
      //Serial.println("");
#ifdef ROS
motors_speed.publish( &motors_speed );

nh.spinOnce();
#endif
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
      (Motor_Speed>=0) ? digitalWrite(M2,HIGH) :digitalWrite(M2,LOW);
      analogWrite(EN2,abs(Motor_Speed));
    }else{
      (Motor_Speed>=0) ? digitalWrite(M1,LOW) :digitalWrite(M1,HIGH);
       analogWrite(EN1,abs(Motor_Speed));
    }
  }

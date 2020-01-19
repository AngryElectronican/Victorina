
#include "DFRobotDFPlayerMini.h"

DFRobotDFPlayerMini myDFPlayer;
uint8_t led_pin=13;
uint8_t in_pin=2;
uint8_t counter=1;
void setup(){
  pinMode(led_pin,OUTPUT);
  pinMode(in_pin,INPUT);
  Serial.begin(9600);
  if (!myDFPlayer.begin(Serial)){
    while(true){
      delay(50);
      digitalWrite(led_pin,HIGH);
      delay(50);
      digitalWrite(led_pin,LOW);
    }
  }
  myDFPlayer.volume(20);  //Set volume value. From 0 to 30
}

void loop(){
  if(digitalRead(in_pin)){
    myDFPlayer.play(counter);
    counter++;
    if(counter>4){
      counter=1;
      }
    while(digitalRead(in_pin));
    delay(500);
    }
}

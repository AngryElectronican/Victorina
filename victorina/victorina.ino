#define R1 3
#define G1 5
#define B1 6

#define R2 9
#define G2 10
#define B2 11

#define IN1 A1
#define IN2 A0

#define NOT_CORRECT 0
#define Player1 1
#define Player2 2
#define NO_ONE 3

#define BLINC_TIMES 10
#define SPEED_COLOUR 15

uint8_t loop_state=0;
uint8_t state_player=0;
unsigned long time1=0,time2=0;
uint8_t counter=0;
uint16_t delay_time=10000;

void writeRGB1(uint8_t R,uint8_t G,uint8_t B);
void writeRGB2(uint8_t R,uint8_t G,uint8_t B);

void setup() {
pinMode(R1,OUTPUT);
pinMode(G1,OUTPUT);
pinMode(B1,OUTPUT);
pinMode(IN1,INPUT_PULLUP);

pinMode(R2,OUTPUT);
pinMode(G2,OUTPUT);
pinMode(B2,OUTPUT);
pinMode(IN2,INPUT_PULLUP);

time1=millis()+5000;
time2=millis();
}

void loop() {
  static uint8_t but1_state=HIGH;
  static uint8_t but2_state=HIGH;
  if(millis()-time1>=delay_time){
    but1_state=digitalRead(IN1);
    but2_state=digitalRead(IN2);
    if(but1_state==LOW && but2_state==LOW){
        loop_state=NOT_CORRECT;
        state_player=0;
        time1=millis();
      }else if(but1_state==LOW && but2_state==HIGH){
        loop_state=Player1;
        state_player=0;
        time1=millis();
        time2=millis();
      }else if(but1_state==HIGH && but2_state==LOW){
        loop_state=Player2;
        state_player=0;
        time1=millis();
      }else if(but1_state==HIGH && but2_state==HIGH){
        loop_state=NO_ONE;
      }
    }

  switch(loop_state){
    case NOT_CORRECT:
      writeRGB1(255,255,0);
      writeRGB2(255,255,0);
    break;
    case Player1:
    static uint8_t blink_counter1=0;
      switch(state_player){
        case 0:
        if(millis()-time2>=400){
          time2=millis();
          blink_counter1++;
          writeRGB1(0,255*((blink_counter1+1)%2),255*(blink_counter1%2));
          writeRGB2(255,0,0);
          if(blink_counter1>=BLINC_TIMES){
            state_player=1;
            blink_counter1=0;
            }
        }
        break;
        case 1:
          writeRGB1(0,255,0);
        break;
      }
    break;
    case Player2:
    static uint8_t blink_counter2=0;
      switch(state_player){
        case 0:
        if(millis()-time2>=400){
          time2=millis();
          blink_counter2++;
          writeRGB2(0,255*((blink_counter2+1)%2),255*(blink_counter2%2));
          writeRGB1(255,0,0);
          if(blink_counter2>=BLINC_TIMES){
            state_player=1;
            blink_counter2=0;
            }
        }
        break;
        case 1:
          writeRGB2(0,255,0);
        break;
      }
    break;
    case NO_ONE:
      switch(state_player){
        case 0:
        if(millis()-time2>=SPEED_COLOUR){
          time2=millis();
          counter++;
          writeRGB1(0,255-counter,counter);
          writeRGB2(0,255-counter,counter);
        }
          if(counter>=255){
            counter=0;
            state_player=1;
          }
        break;
        case 1:
        if(millis()-time2>=SPEED_COLOUR){
          time2=millis();
          counter++;
          writeRGB1(counter,0,255-counter);
          writeRGB2(counter,0,255-counter);
        }
          if(counter>=255){
            counter=0;
            state_player=2;
          }
        break;
        case 2:
        if(millis()-time2>=SPEED_COLOUR){
          time2=millis();
          counter++;
          writeRGB1(255-counter,counter,0);
          writeRGB2(255-counter,counter,0);
        }
          if(counter>=255){
            counter=0;
            state_player=0;
          }
        break;
      }
    break;
    }
}

void writeRGB1(uint8_t R,uint8_t G,uint8_t B){
  analogWrite(R1,R);
  analogWrite(G1,G);
  analogWrite(B1,B);
}
void writeRGB2(uint8_t R,uint8_t G,uint8_t B){
  analogWrite(R2,R);
  analogWrite(G2,G);
  analogWrite(B2,B);
}

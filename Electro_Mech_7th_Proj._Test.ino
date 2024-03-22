/***********************************************************************************/
/**********************************Arduino UNO*************************************/
/*************************************Ver:01***************************************/
/******************************Author:Eng.Fares Sherif*******************************/
/************************************21-3-2024***************************************/
/***********************************************************************************/
//This Code implemented on Arduino Uno Just For testing The Encoded Dc Motor Position Control Manually using Precision Potentiometer and Displaying the Angle on LCD+i2c without PID Control

#include <LiquidCrystal_I2C.h>

#define ENCODER_A_1 2                 //SHOULDER ENCODER PIN A (BASE JOINT) "INTERRUPT"
#define ENCODER_B_1 22               //SHOULDER ENCODER PIN B (BASE JOINT)
#define MOTOR_1_FORWARD  5      //SHOULDER MOTOR (BASE JOINT)
#define MOTOR_1_BACKWARD 4     //SHOULDER MOTOR (BASE JOINT)
#define POT_1 A0          //SHOULDER POSITION CONTROL (BASE JOINT)
#define MOTOR_SPEED 9   // MOTOR_1 ENABLE PIN 9

volatile long POS_1;
int MIN_ANGLE=0,MAX_ANGLE=360, OLD_ANGLE_1;
LiquidCrystal_I2C LCD(0x27,16,2);

void MOVE_MOTOR_ANGLE_1(int ANGLE) {

  int DIRECTION = (ANGLE > OLD_ANGLE_1) ? HIGH : LOW;

  while (OLD_ANGLE_1 != ANGLE) {

    digitalWrite(MOTOR_1_FORWARD, DIRECTION);
    digitalWrite(MOTOR_1_BACKWARD,!DIRECTION);

    OLD_ANGLE_1 += (DIRECTION == HIGH) ? 1 : -1;
    delay(10);
      }
  digitalWrite(MOTOR_1_FORWARD, LOW);
  digitalWrite(MOTOR_1_BACKWARD, LOW);
}

void READ_ENCODER_1(){
  int B = digitalRead(ENCODER_B_1);
  if(B > 0){
    POS_1++;
  }
  else{
    POS_1--;
  }

}

void setup() {
Serial.begin(9600);
pinMode(ENCODER_A_1,INPUT);
pinMode(ENCODER_B_1,INPUT);
pinMode(MOTOR_1_FORWARD, OUTPUT);
pinMode(MOTOR_1_BACKWARD, OUTPUT);
pinMode(MOTOR_SPEED, OUTPUT);
attachInterrupt(digitalPinToInterrupt(ENCODER_A_1),READ_ENCODER_1,CHANGE);
LCD.init(); //LCD I2C DATA PIN 20 , CLK PIN 21
LCD.backlight();
LCD.clear();
}

void loop() {
  analogWrite(MOTOR_SPEED,  100);
int POT_1_VALUE=analogRead(POT_1);
int ANGLE_MOTOR_1=map(POT_1_VALUE,0,1023,MIN_ANGLE,MAX_ANGLE);
constrain(ANGLE_MOTOR_1, 0, 360);
  LCD.setCursor(0,1);
  LCD.print("1st ");
  LCD.print(ANGLE_MOTOR_1);
  LCD.print((char)223); // DEGREE SYMBOL
  delay(100);  
MOVE_MOTOR_ANGLE_1(ANGLE_MOTOR_1);
Serial.println(ANGLE_MOTOR_1);

}



#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include"TCA9548.h"

#define ENCODER_A_1 2 // YELLOW - Encoder for Motor 1
#define ENCODER_B_1 3 // WHITE - Encoder for Motor 1

#define ENCODER_A_2 18 // Encoder for Motor 2
#define ENCODER_B_2 19 // Encoder for Motor 2

#define ENCODER_A_3 20 // Encoder for Motor 2
#define ENCODER_B_3 21 // Encoder for Motor 2

#define MOTOR_SPEED_1  4
#define MOTOR_1_FORWARD 10
#define MOTOR_1_BACKWARD 11

#define MOTOR_SPEED_2 5
#define MOTOR_2_FORWARD 6
#define MOTOR_2_BACKWARD 9

#define MOTOR_SPEED_3 7
#define MOTOR_3_FORWARD 8
#define MOTOR_3_BACKWARD 13

float GEAR_RATIO = 514.8; 
int PULSE_PER_REV = 11;
int POS_1,POS_2 , MIN_ANGLE=0,MAX_ANGLE=360, OLD_ANGLE_1, OLD_ANGLE_2, OLD_ANGLE_3,ANGLE_MOTOR_1,ANGLE_MOTOR_2,ANGLE_MOTOR_3;
String receivedData = "";
float angleX = 0;
float angleY = 0;
LiquidCrystal_I2C LCD(0x27,16,2);
int tiltangle[3];  
int receivedDataArray[3];
int newArray[3];
int MOTOR_SPEED[3];

void readEncoder1() {
  int b = digitalRead(ENCODER_B_1);
  if (b > 0) {
    POS_1++;
  } else {
    POS_1--;
  }
}
void readEncoder2() {
  int b = digitalRead(ENCODER_A_2); 
    
  if (b > 0) {
    POS_2++;
  } else {
    POS_2--;
  }
}
void readEncoder3()
{
}
void parseData(String data) {
  // Find the comma that separates the angles
  int commaIndex = data.indexOf(',');

  // Extract the angles as strings
  String angleXString = data.substring(0, commaIndex);
  String angleYString = data.substring(commaIndex + 1);

  // Convert the strings to float
  angleX = angleXString.toFloat();
  angleY = angleYString.toFloat();

  // Print the angles to the serial monitor (for debugging)
  Serial.print("AngleX: ");
  Serial.print(angleX);
  Serial.print(", AngleY: ");
  Serial.println(angleY);
}
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
void MOVE_MOTOR_ANGLE_2(int ANGLE) {

  int DIRECTION = (ANGLE > OLD_ANGLE_2) ? HIGH : LOW;

  while (OLD_ANGLE_2 != ANGLE) {

    digitalWrite(MOTOR_2_FORWARD, DIRECTION);
    digitalWrite(MOTOR_2_BACKWARD,!DIRECTION);

    OLD_ANGLE_2 += (DIRECTION == HIGH) ? 1 : -1;
    delay(10);
      }
  digitalWrite(MOTOR_2_FORWARD, LOW);
  digitalWrite(MOTOR_2_BACKWARD, LOW);
}
void MOVE_MOTOR_ANGLE_3(int ANGLE) {

  int DIRECTION = (ANGLE > OLD_ANGLE_3) ? HIGH : LOW;

  while (OLD_ANGLE_3 != ANGLE) {

    digitalWrite(MOTOR_3_FORWARD, DIRECTION);
    digitalWrite(MOTOR_3_BACKWARD,!DIRECTION);

    OLD_ANGLE_3 += (DIRECTION == HIGH) ? 1 : -1;
    delay(10);
      }
  digitalWrite(MOTOR_3_FORWARD, LOW);
  digitalWrite(MOTOR_3_BACKWARD, LOW);
}
void copyData() {
  for (int i = 0; i < 3; i++) {
    tiltangle[i] = receivedDataArray[i];
  }
}
/*void parseData(String data) {
  int index = 0;
  int lastIndex = 0;
  for (int i = 0; i < 3; i++) {
    index = data.indexOf(',', lastIndex);
    if (index == -1) index = data.length();
    receivedDataArray[i] = data.substring(lastIndex, index).toInt();
    lastIndex = index + 1;
  }
}*/
void setup() {
  Serial.begin(9600);
  pinMode(ENCODER_A_1, INPUT);
  pinMode(ENCODER_B_1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), readEncoder1, RISING);

  pinMode(ENCODER_A_2, INPUT);
  pinMode(ENCODER_B_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_2), readEncoder2, RISING);

  pinMode(ENCODER_A_3, INPUT);
  pinMode(ENCODER_B_3, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_3), readEncoder3, RISING);

  pinMode(MOTOR_SPEED_1, OUTPUT);
  pinMode(MOTOR_1_FORWARD, OUTPUT);
  pinMode(MOTOR_1_BACKWARD, OUTPUT);

  pinMode(MOTOR_SPEED_2, OUTPUT);
  pinMode(MOTOR_2_FORWARD, OUTPUT);
  pinMode(MOTOR_2_BACKWARD, OUTPUT);

  pinMode(MOTOR_SPEED_3, OUTPUT);
  pinMode(MOTOR_3_FORWARD, OUTPUT);
  pinMode(MOTOR_3_BACKWARD, OUTPUT);

  LCD.init(); //LCD I2C DATA PIN 20 , CLK PIN 21
  LCD.backlight();
  LCD.clear();
}

void loop() {
 // Check if data is available to read
  if (Serial.available() > 0) {
     // for (int i = 0; i < 3; i++){
    // Read the incoming data
while (Serial.available()) {
    // Read the incoming data
    char incomingChar = (char)Serial.read();
    if (incomingChar == '\n') {
      // Data reception is complete, process the received data
      parseData(receivedData);
      receivedData = "";
    } else {
      receivedData += incomingChar;
    }
  } 
  //Serial.print(i);
  //Serial.print("Tilt Angle: ");
  Serial.println(receivedData);
  //MOTOR_SPEED[i] = map(receivedData[i], -90, 90, -125, 125);
      }
   
    //int MOTOR_SPEED2 = map(receivedData[2], -90, 90, -125, 125);
    //int MOTOR_SPEED3 = map(receivedData[3], -90, 90, -125, 125);
 
if (MOTOR_SPEED[1] > 0) {
    digitalWrite(MOTOR_1_FORWARD, HIGH);
    digitalWrite(MOTOR_1_BACKWARD, LOW);
  } else {
    digitalWrite(MOTOR_1_FORWARD, LOW);
    digitalWrite(MOTOR_1_BACKWARD, HIGH);
    MOTOR_SPEED[1] = -MOTOR_SPEED[1];
  }
  analogWrite(MOTOR_SPEED_1, MOTOR_SPEED[1]);
  if (MOTOR_SPEED_2 > 0) {
    digitalWrite(MOTOR_2_FORWARD, HIGH);
    digitalWrite(MOTOR_2_BACKWARD, LOW);
  } else {
    digitalWrite(MOTOR_2_FORWARD, LOW);
    digitalWrite(MOTOR_2_BACKWARD, HIGH);
    MOTOR_SPEED[2] = -MOTOR_SPEED[2];
  }
  analogWrite(MOTOR_SPEED_2, MOTOR_SPEED[2]);
  if (MOTOR_SPEED_3 > 0) {
    digitalWrite(MOTOR_3_FORWARD, HIGH);
    digitalWrite(MOTOR_3_BACKWARD, LOW);
  } else {
    digitalWrite(MOTOR_3_FORWARD, LOW);
    digitalWrite(MOTOR_3_BACKWARD, HIGH);
   MOTOR_SPEED[3] = -MOTOR_SPEED[3];
  }
  analogWrite(MOTOR_SPEED_3, MOTOR_SPEED[3]);
    
  LCD.setCursor(0,0);
  LCD.print("1st ");
  LCD.print(receivedData);
  LCD.print((char)223);
  LCD.setCursor(0,1);
  LCD.print("2nd ");
  LCD.print(ANGLE_MOTOR_2);
  LCD.print((char)223);
  LCD.setCursor(7,0);
  LCD.print("3rd ");
  LCD.print(ANGLE_MOTOR_3);
  LCD.print((char)223);

  }



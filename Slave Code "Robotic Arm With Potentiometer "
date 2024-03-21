#include <LiquidCrystal_I2C.h>

#define ENCODER_A_1 2                 //SHOULDER ENCODER PIN A (BASE JOINT) "INTERRUPT"
#define ENCODER_B_1 22               //SHOULDER ENCODER PIN B (BASE JOINT)
#define ENCODER_A_2 3               //ELBOW ENCODER PIN A (MID JOINT) "INTERRUPT"
#define ENCODER_B_2 23             //ELBOW ENCODER PIN B (MID JOINT)
#define ENCODER_A_3 18            //WRIST ENCODER PIN A (END JOINT) "INTERRUPT"
#define ENCODER_B_3 24           //WRIST ENCODER PIN B (END JOINT)
#define MOTOR_1_FORWARD  4      //SHOULDER MOTOR (BASE JOINT)
#define MOTOR_1_BACKWARD 5     //SHOULDER MOTOR (BASE JOINT)
#define MOTOR_2_FORWARD  6    //ELBOW MOTOR (MID JOINT)
#define MOTOR_2_BACKWARD 7   //ELBOW MOTOR (MID JOINT)
#define MOTOR_3_FORWARD  8  //WRIST MOTOR (END JOINT)
#define MOTOR_3_BACKWARD 9 //WRIST MOTOR (END JOINT)
#define POT_1 A0          //SHOULDER POSITION CONTROL (BASE JOINT)
#define POT_2 A1         //ELBOW POSITION CONTROL (MID JOINT)
#define POT_3 A2        //WRIST POSITION CONTROL (END JOINT)
#define LCD_SDA 20     //LCD I2C DATA PIN
#define LCD_SCL 21    //LCD I2C CLK PIN

int MIN_ANGLE=0,MAX_ANGLE=360, OLD_ANGLE_1, OLD_ANGLE_2, OLD_ANGLE_3,POS_1,POS_2,POS_3;
LiquidCrystal_I2C LCD(0x27,16,2);
byte HEART[8] ={
    0b00001010,
    0b00010101,
    0b00010001,
    0b00001010,
    0b00000100,
    0b00000000,
    0b00000000,
    0b00000000
}; //heart shape character

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
void READ_ENCODER_1(){
  int B = digitalRead(ENCODER_B_1);
  if(B > 0){
    POS_1++;
  }
  else{
    POS_1--;
  }
}
void READ_ENCODER_2(){
  int B = digitalRead(ENCODER_B_2);
  if(B > 0){
    POS_2++;
  }
  else{
    POS_2--;
  }
}
void READ_ENCODER_3(){
  int B = digitalRead(ENCODER_B_3);
  if(B > 0){
    POS_3++;
  }
  else{
    POS_3--;
  }
}
void setup() {
  Serial.begin(9600);
  pinMode(ENCODER_A_1,INPUT);
  pinMode(ENCODER_A_2,INPUT);
  pinMode(ENCODER_A_3,INPUT);
  pinMode(ENCODER_B_1,INPUT);
  pinMode(ENCODER_B_2,INPUT);
  pinMode(ENCODER_B_3,INPUT);
  pinMode(LCD_SDA,OUTPUT);
  pinMode(LCD_SCL,OUTPUT);
  pinMode(MOTOR_1_FORWARD, OUTPUT);
  pinMode(MOTOR_2_FORWARD, OUTPUT);
  pinMode(MOTOR_3_FORWARD, OUTPUT);
  pinMode(MOTOR_1_BACKWARD, OUTPUT);
  pinMode(MOTOR_2_BACKWARD, OUTPUT);
  pinMode(MOTOR_3_BACKWARD, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_1),READ_ENCODER_1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_2),READ_ENCODER_2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_3),READ_ENCODER_3,CHANGE);
}

void loop() {
  int POT_1_VALUE=analogRead(POT_1);
  int POT_2_VALUE=analogRead(POT_2);
  int POT_3_VALUE=analogRead(POT_3);
  int ANGLE_MOTOR_1=map(POT_1_VALUE,0,1023,MIN_ANGLE,MAX_ANGLE);
  int ANGLE_MOTOR_2=map(POT_2_VALUE,0,1023,MIN_ANGLE,MAX_ANGLE);
  int ANGLE_MOTOR_3=map(POT_3_VALUE,0,1023,MIN_ANGLE,MAX_ANGLE);

  MOTOR_ANGLE_MOVE_1(ANGLE_MOTOR_1);
  MOVE_MOTOR_ANGLE_2(ANGLE_MOTOR_2);
  MOVE_MOTOR_ANGLE_3(ANGLE_MOTOR_3);

 //OLD_ANGLE_1=ANGLE_MOTOR_1;
 //OLD_ANGLE_2=ANGLE_MOTOR_2;
 //OLD_ANGLE_3=ANGLE_MOTOR_3;


  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("1st ");
  LCD.print(ANGLE_MOTOR_1);
  LCD.print((char)223); // Degree symbol
  LCD.setCursor(0, 1);
  LCD.print("2nd");
  LCD.print(ANGLE_MOTOR_2);
  LCD.print((char)223); // Degree symbol
  LCD.print("2nd");
  LCD.print(ANGLE_MOTOR_3);
  LCD.print((char)223); // Degree symbol

  delay(100); // Adjust delay as needed for display refresh rate

}

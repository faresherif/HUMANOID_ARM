//#include <SPI.h>
//#include <nRF24L01.h>
//#include <RF24.h>
#include <Encoder.h>
#include <PID_v1.h>                                        
#include <MPU6050_tockn.h>
#include <Wire.h>
//RF24 radio(4, 8);
const byte address[6] = "00001";

#define enc_outAp 2  
#define enc_outBp 3
#define MOT_DIR1p 10
#define MOT_DIR2p 11

#define enc_outAr 18  
#define enc_outBr 19
#define MOT_DIR1r 6
#define MOT_DIR2r 9

#define enc_outAy 20  
#define enc_outBy 21
#define MOT_DIR1y 4
#define MOT_DIR2y 5

#define ENC_RESOLUTION_roll 435/4
#define ENC_RESOLUTION_pitch 435/4
#define ENC_RESOLUTION_yaw 435/4


double kp = 3;
double ki = 0;
double kd = 20;


double input_p, output_p, setpoint_p;
float lastError_p;

double input_r, output_r, setpoint_r;
float lastError_r;
 

double input_y, output_y, setpoint_y;
float lastError_y;

double alpha = 0.2;
double filteredAngle_p, prevFilteredAngle_p;
double filteredAngle_r, prevFilteredAngle_r;
double filteredAngle_y, prevFilteredAngle_y;

Encoder encoder_pitch(enc_outAp, enc_outBp);
Encoder encoder_roll(enc_outAr, enc_outBr);
Encoder encoder_yaw(enc_outAy, enc_outBy);
PID pid_pitch(&input_p, &output_p, &setpoint_p, kp, ki, kd, DIRECT);
PID pid_roll(&input_r, &output_r, &setpoint_r, kp, ki, kd, DIRECT);
PID pid_yaw(&input_y, &output_y, &setpoint_y, kp, ki, kd, DIRECT);

unsigned long prevTime_p, prevTime_r,prevTime_y;
double prevError_p,prevError_r,prevError_y;

volatile int count_p ;
volatile int count_r;
volatile int count_y;

int pwmValue_p;
int pwmValue_r;
int pwmValue_y;

double roll = 0, pitch = 0, yaw = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(enc_outAp,INPUT);
  pinMode(enc_outBp,INPUT);
  pinMode(enc_outAr,INPUT);
  pinMode(enc_outBr,INPUT);
  pinMode(enc_outAy,INPUT);
  pinMode(enc_outBy,INPUT);
  pinMode(MOT_DIR1p,OUTPUT);
  pinMode(MOT_DIR2p,OUTPUT);
  pinMode(MOT_DIR1r,OUTPUT);
  pinMode(MOT_DIR2r,OUTPUT);
  pinMode(MOT_DIR1y,OUTPUT);
  pinMode(MOT_DIR2y,OUTPUT);

  encoder_pitch.write(0);
  encoder_roll.write(0);
  encoder_yaw.write(0);

  input_p,input_r,input_y = 0;
  output_p,output_r,output_y = 0;
  setpoint_p,setpoint_r,setpoint_y = 0;
  
  pid_pitch.SetControllerDirection(AUTOMATIC);
  pid_pitch.SetSampleTime(10);
  pid_pitch.SetOutputLimits(-255, 255);

  pid_roll.SetControllerDirection(AUTOMATIC);
  pid_roll.SetSampleTime(10);
  pid_roll.SetOutputLimits(0, 255);

  pid_yaw.SetControllerDirection(AUTOMATIC);
  pid_yaw.SetSampleTime(10);
  pid_yaw.SetOutputLimits(0, 255);

  prevError_p,prevError_r,prevError_y = 0;
  prevTime_p, prevTime_r,prevTime_y = millis();

/*
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  delay(5000);
*/
}

void loop() {

     if (Serial.available()>0) 
   {  
    byte angles[3];
    //Serial.read(&angles, sizeof(angles));
  for (int i = 0; i < 3; i++) {
   angles[i] = Serial.read();  // Read the bytes into the array
  }
     pitch=(double)angles[0];
     roll=(double)angles[1];
     yaw=(double)angles[2];
    }

  delay(200);
 
  pid_p ();
  set_motor1();
   
  pid_r ();
  set_motor2();
  
  pid_y ();
  set_motor3();
 
}

   void pid_p ()
{
 
   
  filteredAngle_p = alpha * pitch + (1 - alpha) * prevFilteredAngle_p;
  prevFilteredAngle_p = filteredAngle_p;
  setpoint_p = filteredAngle_p;

  long count_p = encoder_pitch.read();
  double angle_p =  (double)count_p -50 / ENC_RESOLUTION_pitch * 360;

  double error = setpoint_p - angle_p;
  unsigned long currTime_p = millis();
  double dt = (double)(currTime_p - prevTime_p) / 1000;
  input_p = angle_p;
  float P = error;
  float I = I + error;
  float D = error - lastError_p;
  lastError_p = error;
  output_p =( P * kp + I * ki + D * kd);
  /*
  if(output_p>1000)
  output_p=1000;
  else if(output_p<-1000)
  output_p=-1000;
  */
  
  pwmValue_p = map(abs(output_p), 0, 1000, 0, 255);

  prevError_p = error;
  prevTime_p = currTime_p;

}
void pid_r ()
{
 
  filteredAngle_r = alpha * roll + (1 - alpha) * prevFilteredAngle_r;
  prevFilteredAngle_r = filteredAngle_r;
  setpoint_r = filteredAngle_r;
  
  long count_r = encoder_roll.read();
  double angle_r = (double)count_r -50 / ENC_RESOLUTION_roll * 360;

  double error = setpoint_r - angle_r;
  unsigned long currTime_r = millis();
  double dt = (double)(currTime_r - prevTime_r) / 1000;
  input_r = angle_r;
  float P = error;
  float I = I + error;
  float D = error - lastError_r;
  lastError_r = error;
  output_r =( P * kp + I * ki + D * kd);
  /*
    if(output_r>1000)
  output_r=1000;
  else if(output_r<-1000)
  output_r=-1000;
  */
  
  pwmValue_r = map(abs(output_r), 0, 1000 , 0, 255);

  prevError_r = error;
  prevTime_r = currTime_r;
}
void pid_y ()
{
 

  filteredAngle_y = alpha * yaw + (1 - alpha) * prevFilteredAngle_y;
  prevFilteredAngle_y = filteredAngle_y;
 
  setpoint_y = filteredAngle_y;

  long count_y = encoder_yaw.read();
  double angle_y =180+  (double)count_y / ENC_RESOLUTION_yaw * 360;

  double error = setpoint_y - angle_y;
  unsigned long currTime_y = millis();
  double dt = (double)(currTime_y - prevTime_y) / 1000;
  input_y = angle_y;
  float P = error;
  float I = I + error;
  float D = error - lastError_y;
  lastError_y = error;
  
  output_y =( P * 6 + I * 0 + D * 20);
  /*
   if(output_y>1000)
   output_y=1000;
   else if(output_y<-1000)
   output_y=-1000;
  */

  pwmValue_y = map(abs(output_y) , 0, 1000, 0, 255);
  prevError_y = error;
  prevTime_y = currTime_y;
}

void set_motor1()
{
  
if (output_p > 0) {
analogWrite(MOT_DIR1p, pwmValue_p/7);
analogWrite(MOT_DIR2p,0);

} 
else if (output_p < 0) {
analogWrite(MOT_DIR1p, 0);
analogWrite(MOT_DIR2p, pwmValue_p/7);
} else {
analogWrite(MOT_DIR1p, 0);
analogWrite(MOT_DIR2p, 0);
}
}

void set_motor2()
{
  
if (output_r > 0) {
analogWrite(MOT_DIR1r,0 );
 analogWrite(MOT_DIR2r,pwmValue_r/7); 
} 
else if (output_r < 0) {
analogWrite(MOT_DIR1r, pwmValue_r/7);
analogWrite(MOT_DIR2r, 0 );
} 
else {
analogWrite(MOT_DIR1r, 0);
analogWrite(MOT_DIR2r, 0);
}
}
void set_motor3()
{ 
if (output_y > 0) {
analogWrite(MOT_DIR1y,0 );
analogWrite(MOT_DIR2y,pwmValue_y/4); 
} 
else if (output_y < 0) {
analogWrite(MOT_DIR1y, pwmValue_y/4);
analogWrite(MOT_DIR2y, 0 );
} 
else {
analogWrite(MOT_DIR1y, 0);
analogWrite(MOT_DIR2y, 0);
}
}

#include <MPU6050_tockn.h>
#include <Wire.h>
//#include <SPI.h>
//#include <nRF24L01.h>
//#include <RF24.h>


MPU6050 mpu6050(Wire);  

//RF24 radio(7, 8); // Create an instance of the radio module using pin 9 for CE and pin 10 for CSN
//const byte address[6] = "00001"; // Set the address for the radio module

// Variables to store the filtered angle readings
double roll = 0, pitch = 0, yaw = 0;

// Define variables for the PID control loop
unsigned long prevTime;
double prevError;

volatile int pos ;

// Filter coefficients
const double alpha = 0.50;
const double dt = 0.01;
double filteredAngle, prevFilteredAngle;

void setup() {
  Serial.begin(9600);
/*
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
*/
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();
 /*
  int angle1 = map(mpu6050.getAngleY(),-90,90,0,180);/// pitch
  int angle2 = map(mpu6050.getAngleX(),-90,90,0,180);///roll
  int angle3 = map( mpu6050.getAngleZ(),-90,90,0,360);///yaw
  */
  int angle1 = mpu6050.getAngleY();/// pitch
  int angle2 = mpu6050.getAngleX();///roll
  int angle3 = mpu6050.getAngleZ();///yaw
  

  // Create a byte array to store the angle values
      if (angle1>255)
    angle1=255;
      else if (angle1<0)
    angle1=0;
      if (angle2>255)
    angle2=255;
     else if (angle2<0)
    angle2=0;
      if (angle3>255)
    angle3=255;
     else if (angle3<0)
    angle3=0;
   
  byte angles[3];
  angles[0] = (byte)angle1;
  angles[1] = (byte)angle2;
  angles[2] = (byte)angle3;

  // Send the angle values using the radio module
 // radio.write(&angles, sizeof(angles));
  Serial.println(angles[0]);
  Serial.println(angles[1]);
  Serial.println(angles[2]);

  delay(200);
}

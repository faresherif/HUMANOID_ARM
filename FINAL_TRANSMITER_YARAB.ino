#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
const int TCA_ADDR = 0x70; // Address of TCA9548A

void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  // Initialize each MPU6050
  for (uint8_t i = 0; i < 3; i++) {
    tcaSelect(i);
    mpu.initialize();
    if (!mpu.testConnection()) {
      Serial.print("MPU6050 #");
      Serial.print(i);
      Serial.println(" connection failed");
    } else {
      Serial.print("MPU6050 #");
      Serial.print(i);
      Serial.println(" connection successful");
    }
  }
}

void loop() {
  float tiltangle[3];
  for (uint8_t i = 0; i < 3; i++) {
    tcaSelect(i);
    int ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    tiltangle[i] = atan2(-ay, az) * 180 / M_PI;
    Serial.print(tiltangle[i]);
    Serial.print(",");
  }
  Serial.println();
  delay(1000);
}
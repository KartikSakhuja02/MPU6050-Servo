#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("Adafruit MPU6050 test!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float xAcceleration = a.acceleration.x;
  int servoAngle = map(xAcceleration, -10, 10, 0, 180);
  servoAngle = constrain(servoAngle, 0, 180);

  myServo.write(servoAngle);

  Serial.print("X-axis Acceleration: ");
  Serial.print(xAcceleration);
  Serial.print(" m/s^2, Servo Angle: ");

  delay(100);
}

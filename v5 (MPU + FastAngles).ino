// Y angle not coming and Z is drifting
// Servo also not moving properly

#include <mpu6050_FastAngles.h>
#include <Servo.h>

mpu6050_FastAngles mpu;
Servo myServo;

void setup() {
  Serial.begin(9600);
  myServo.attach(10); // Attach servo to pin 10

  // Initialize the sensor with a specific gyroscope scale
  mpu.begin(MPU_MODE_250); // Options: MPU_MODE_250, MPU_MODE_500, MPU_MODE_1000, MPU_MODE_2000

  // Optional, adjust the correction factor of the complementary filter
  mpu.setComplementaryFactor(0.95); // Value between 0.0 and 1.0, standard is 0.98

  // Optional, adjust the parameters of the Kalman filter
  mpu.setKalmanQangle(0.001);
  mpu.setKalmanQbias(0.003);
  mpu.setKalmanRmeasure(0.03);

  // Optional, display the current settings
  mpu.printSettings();
}

void loop() {
  // Get the angles, standard filter is COMPLEMENTARY
  float angleX = mpu.getAngle('X');
  float angleY = mpu.getAngle('Y');
  float angleZ = mpu.getAngle('Z', KALMAN);

  // Map angleX to servo range (0 to 180 degrees)
  int servoPos = map(angleX, -90, 90, 0, 180);
  servoPos = constrain(servoPos, 0, 180); // Ensure values stay within servo range

  myServo.write(servoPos); // Move the servo

  Serial.print("Angle X: "); Serial.print(angleX);
  Serial.print("\tAngle Y: "); Serial.print(angleY);
  Serial.print("\tAngle Z: "); Serial.println(angleZ);
  Serial.print("\tServo Position: "); Serial.println(servoPos);

  delay(100);
}

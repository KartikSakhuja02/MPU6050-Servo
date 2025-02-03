#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();

  // Check if the sensor is connected
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 connected successfully!");
}

void loop() {
  // Variables to store accelerometer and gyroscope data
  int16_t ax, ay, az, gx, gy, gz;

  // Get raw data from MPU6050
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  // Print accelerometer data
  Serial.print("Acceleration X: "); Serial.print(ax); 
  Serial.print(" | Y: "); Serial.print(ay);
  Serial.print(" | Z: "); Serial.println(az);

  // Print gyroscope data
  Serial.print("Gyroscope X: "); Serial.print(gx); 
  Serial.print(" | Y: "); Serial.print(gy);
  Serial.print(" | Z: "); Serial.println(gz);

  // Delay for a bit before taking next reading
  delay(500);
}

// MPU6050 Complementary filtering code

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

float pitch = 0;
float roll = 0;
float alpha = 0.98; // Complementary filter coefficient
unsigned long lastTime = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }

    Serial.println("MPU6050 initialized");
}

void loop() {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    // Get raw accelerometer and gyroscope data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert to degrees per second
    float gyroX = gx / 131.0; // Gyro sensitivity for ±250°/s
    float gyroY = gy / 131.0;

    // Calculate time difference
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // Convert ms to seconds
    lastTime = currentTime;

    // Calculate pitch and roll from accelerometer
    float accelPitch = atan2(ay, az) * 180.0 / PI;
    float accelRoll = atan2(ax, az) * 180.0 / PI;

    // Apply complementary filter
    pitch = alpha * (pitch + gyroX * dt) + (1 - alpha) * accelPitch;
    roll = alpha * (roll + gyroY * dt) + (1 - alpha) * accelRoll;

    // Print the results
    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print(" | Roll: ");
    Serial.println(roll);

    delay(100); // Adjust delay as needed
}

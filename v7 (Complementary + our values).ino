#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Complementary filter variables
float angle = 0.0; // Initial angle
const float alpha = 0.9; // Complementary filter constant
unsigned long lastTime;

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

    lastTime = millis(); // Initialize lastTime

    Serial.println("");
    delay(100);
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate the angle from the accelerometer
    float accelAngle = atan2(a.acceleration.y, a.acceleration.z) * (180.0 / PI);

    // Get current time and calculate delta time
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime;

    // Calculate the angle from the gyroscope
    float gyroRate = g.gyro.x; // Gyro rate in degrees/second
    angle += gyroRate * dt; // Integrate to get angle

    // Apply complementary filter
    angle = alpha * angle + (1 - alpha) * accelAngle;

    // Print the filtered angle and raw readings
    Serial.print("Filtered Angle: ");
    Serial.print(angle);
    
    Serial.print(", Acceleration X: ");
    Serial.print(a.acceleration.x);
    
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    
    Serial.println(" rad/s");

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    
    Serial.println(" degC");
    
    Serial.println("");

    delay(100); // Loop delay of 100 ms (10 Hz)
}

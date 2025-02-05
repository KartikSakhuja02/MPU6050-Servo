// MPU6050 Kalman filter code with our readings

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Kalman filter variables
float q_angle = 0.009; // Process noise covariance for angle
float q_bias = 0.007;  // Process noise covariance for bias
float r = 0.03;        // Measurement noise covariance
float angle = 0;       // Estimated angle
float bias = 0;        // Estimated bias
float rate = 0;        // Rate of change of the angle
float p[2][2] = { {1, 0}, {0, 1} }; // Error covariance matrix

void setup(void) {
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

    Serial.println("");
    delay(100);
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate the angle from the accelerometer
    float accAngle = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;

    // Update rate and angle from gyroscope
    rate = g.gyro.x - bias;
    angle += rate * 0.01; // Assuming loop runs every 10ms

    // Kalman filter equations
    float s = p[0][0] + r; // Estimate error
    float k[2]; // Kalman gain
    k[0] = p[0][0] / s; // Kalman gain for angle
    k[1] = p[1][0] / s; // Kalman gain for bias

    // Update angle estimate with accelerometer measurement
    angle += k[0] * (accAngle - angle);
    
    // Update bias estimate
    bias += k[1] * (accAngle - angle);

    // Update error covariance matrix
    float p00_temp = p[0][0];
    float p01_temp = p[0][1];

    p[0][0] -= k[0] * p00_temp;
    p[0][1] -= k[0] * p01_temp;
    
    p[1][0] -= k[1] * p00_temp;
    
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

// MPU + Servo
// The servo is moving very jittery due to noise
// Error - Servo is not moving accurately

#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>  

MPU6050 mpu;
Servo myServo1;  

float ax, ay, az, gx, gy, gz;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
unsigned long lastTime = 0;
float pitch = 0, pitch_prev = 0; 

void calibrateGyro() {
    int numSamples = 1000;
    float sumX = 0, sumY = 0, sumZ = 0;

    Serial.println("Calibrating gyroscope...");
    for (int i = 0; i < numSamples; i++) {
        int16_t raw_gx, raw_gy, raw_gz;
        mpu.getRotation(&raw_gx, &raw_gy, &raw_gz);

        sumX += raw_gx / 131.0;
        sumY += raw_gy / 131.0;
        sumZ += raw_gz / 131.0;
    }

    gyroBiasX = sumX / numSamples;
    gyroBiasY = sumY / numSamples;
    gyroBiasZ = sumZ / numSamples;

    Serial.println("Calibration complete.");
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }

    Serial.println("MPU6050 initialized");
    calibrateGyro();
    mpu.setDLPFMode(6);

    myServo1.attach(10);
    myServo1.write(180);
    lastTime = micros();
}

void loop() {
    int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;

    mpu.getAcceleration(&raw_ax, &raw_ay, &raw_az);
    mpu.getRotation(&raw_gx, &raw_gy, &raw_gz);

    gx = (raw_gx / 131.0) - gyroBiasX;
    gy = (raw_gy / 131.0) - gyroBiasY;
    gz = (raw_gz / 131.0) - gyroBiasZ;

    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0;  
    lastTime = currentTime;

    pitch += gx * dt;

    int servoAngle = map(pitch, -90, 90, 0, 180);
    servoAngle = constrain(servoAngle, 0, 180);

    myServo1.write(servoAngle);
    Serial.print("Pitch Angle: ");
    Serial.print(pitch);
    Serial.print(" | Servo Angle: ");
    Serial.println(servoAngle);
}

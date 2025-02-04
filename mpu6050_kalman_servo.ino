#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo myServo1;
Servo myServo2;

float pitch = 0;
unsigned long lastTime = 0;

float Q_angle = 0.001;
float Q_bias = 0.003;
float R_measure = 0.03;
float angle = 0;
float bias = 0;
float P[2][2] = {{0, 0}, {0, 0}};

float kalmanFilter(float newAngle, float newRate, float dt) {
    angle += dt * (newRate - bias);
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];

    return angle;
}

void calibrateGyro() {
    int numSamples = 1000;
    float sumX = 0;

    Serial.println("Calibrating gyroscope...");
    for (int i = 0; i < numSamples; i++) {
        int16_t raw_gx;
        mpu.getRotation(&raw_gx, nullptr, nullptr);
        sumX += raw_gx / 131.0;
        delay(3);
    }
    bias = sumX / numSamples;
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
    myServo1.attach(9);
    myServo2.attach(10);
    myServo1.write(90);
    myServo2.write(90);
    lastTime = millis();
}

void loop() {
    int16_t raw_ax, raw_ay, raw_az, raw_gx;
    mpu.getAcceleration(&raw_ax, &raw_ay, &raw_az);
    mpu.getRotation(&raw_gx, nullptr, nullptr);

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    float accel_pitch = atan2(raw_ay, sqrt(raw_ax * raw_ax + raw_az * raw_az)) * 180.0 / PI;
    float gyro_rate = (raw_gx / 131.0) - bias;

    pitch = kalmanFilter(accel_pitch, gyro_rate, dt);
    int servoAngle = constrain(map(pitch, -90, 90, 0, 180), 0, 180);

    myServo1.write(servoAngle);
    myServo2.write(servoAngle);
    Serial.print("Pitch Angle: ");
    Serial.print(pitch);
    Serial.print(" | Servo Angle: ");
    Serial.println(servoAngle);

    delay(20);
}
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>  

MPU6050 mpu;
Servo myServo1;
Servo myServo2;  

float ax, ay, az;
float pitch = 0; 
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
    mpu.setDLPFMode(6);

    myServo1.attach(9);
    myServo2.attach(10);
    myServo1.write(180);
    myServo2.write(180);  
}

void loop() {
    int16_t raw_ax, raw_ay, raw_az;

    mpu.getAcceleration(&raw_ax, &raw_ay, &raw_az);

    ax = raw_ax / 16384.0;
    ay = raw_ay / 16384.0;
    az = raw_az / 16384.0;

    pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

    int servoAngle = map(pitch, -90, 90, 0, 180);
    servoAngle = constrain(servoAngle, 0, 180);

    myServo1.write(servoAngle);
    myServo2.write(servoAngle);
    Serial.print("Pitch Angle: ");
    Serial.print(pitch);
    Serial.print(" | Servo Angle: ");
    Serial.println(servoAngle);

    delay(20);  
}

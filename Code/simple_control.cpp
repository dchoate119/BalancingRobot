// Simple Proportial Control 

// Code use for prototype video


#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <math.h>  // For defining PI


MPU9250 mpu;


// Define pins for motor control
const int ENA = 14;  // Pin for controlling the speed of the left motor (A)
const int IN1 = 19;  // Pin for direction of the left motor (A)
const int IN2 = 18;  // Pin for direction of the left motor (A)


const int ENB = 12;  // Pin for controlling the speed of the right motor (B)
const int IN3 = 17;  // Pin for direction of the right motor (B)
const int IN4 = 16;  // Pin for direction of the right motor (B)


// Gyro offsets
float gyroXOffset = 0.0, gyroYOffset = 0.0, gyroZOffset = 0.0;


// Angles
float angleX = 0.0, angleY = 0.0, angleZ = 0.0;


// Timing variables
unsigned long previousTime = 0;
const float dt = 0.005; // 5 ms for 200 Hz


// PWM setup
const int ledcChannelA = 14; // PWM channel for left motor
const int ledcChannelB = 12; // PWM channel for right motor
const int ledcFreq = 5000;   // PWM frequency
const int ledcResolution = 8; // 8-bit resolution (0-255)


// Function to control motors based on speed and direction
void moveMotors(int speedA, int speedB) {
    if (speedA > 0) {
        digitalWrite(IN1, HIGH); // Direction for the left motor
        digitalWrite(IN2, LOW);
        ledcWrite(ledcChannelA, speedA); // Set speed for left motor
    } else {
        digitalWrite(IN1, LOW); // Reverse direction for left motor
        digitalWrite(IN2, HIGH);
        ledcWrite(ledcChannelA, -speedA); // Set speed for left motor
    }


    if (speedB > 0) {
        digitalWrite(IN3, LOW); // Direction for the right motor
        digitalWrite(IN4, HIGH);
        ledcWrite(ledcChannelB, speedB); // Set speed for right motor
    } else {
        digitalWrite(IN3, HIGH); // Reverse direction for right motor
        digitalWrite(IN4, LOW);
        ledcWrite(ledcChannelB, -speedB); // Set speed for right motor
    }
}


// Function to stop the motors
void stopMotors() {
    ledcWrite(ledcChannelA, 0); // Stop the left motor
    ledcWrite(ledcChannelB, 0); // Stop the right motor
}


void setup() {
    Serial.begin(9600);
    Wire.begin();


    // Initialize MPU9250
    if (!mpu.setup(0x68)) {  // Check if the address 0x68 is correct for your sensor
        Serial.println("Failed to initialize MPU9250");
        while (1);
    }
    Serial.println("MPU9250 initialized");


    // Set motor pins as outputs
    // pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    // pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);


    // Configure PWM for motors
    ledcSetup(ledcChannelA, ledcFreq, ledcResolution);
    ledcAttachPin(ENA, ledcChannelA);
    ledcSetup(ledcChannelB, ledcFreq, ledcResolution);
    ledcAttachPin(ENB, ledcChannelB);


    // Calibrate gyroscope
    const int calibrationSamples = 1000;


    // Calibration loop
    for (int i = 0; i < calibrationSamples; i++) {
        mpu.update();
        gyroXOffset += mpu.getGyroX();
        gyroYOffset += mpu.getGyroY();
        gyroZOffset += mpu.getGyroZ();
        delay(1); // Short delay to avoid flooding the sensor
    }
    gyroXOffset /= calibrationSamples;
    gyroYOffset /= calibrationSamples;
    gyroZOffset /= calibrationSamples;


    // Print the offsets for reference
    Serial.print("Gyro X Offset: "); Serial.println(gyroXOffset);
    Serial.print("Gyro Y Offset: "); Serial.println(gyroYOffset);
    Serial.print("Gyro Z Offset: "); Serial.println(gyroZOffset);


    // Stop the motors at the beginning
    stopMotors();
}


void loop() {
    unsigned long currentTime = millis();


    // Check if 5 milliseconds have passed
    if (currentTime - previousTime >= 5) {
        previousTime = currentTime; // Update previousTime


        // Update sensor data
        mpu.update();


        // Get gyroscope data (angular rates) and subtract offsets
        float gyroX = mpu.getGyroX() - gyroXOffset; // Adjusted for bias
        float gyroY = mpu.getGyroY() - gyroYOffset; // Adjusted for bias
        float gyroZ = mpu.getGyroZ() - gyroZOffset; // Adjusted for bias


        // Integrate gyroscope data to get angles
        angleX += (gyroX * dt)*10; // Integrate to get angle
        angleY += (gyroY * dt)*10; // Integrate to get angle
        angleZ += (gyroZ * dt)*10; // Integrate to get angle


        // Print the angles and angular rates (comment/uncomment as needed)
        Serial.print("Angle X: "); Serial.print(angleX);
        Serial.print(" | Angle Y: "); Serial.print(angleY);
        Serial.print(" | Angle Z: "); Serial.println(angleZ);


        // Serial.print("Gyro X Rate: "); Serial.print(gyroX);
        // Serial.print(" | Gyro Y Rate: "); Serial.print(gyroY);
        // Serial.print(" | Gyro Z Rate: "); Serial.println(gyroZ);


        // Calculate motor speeds based on angleY
        int motorSpeedA = 0; // Speed for motor A
        int motorSpeedB = 0; // Speed for motor B


        if (abs(angleY) < 45) {
            motorSpeedA = map(abs(angleY), 0, 45, 0, 255); // Speed increases as angle approaches 45 degrees
            motorSpeedB = map(abs(angleY), 0, 45, 0, 255); // Same mapping for motor B
        } else {
            motorSpeedA = 0; // Stop if angle is beyond threshold
            motorSpeedB = 0; // Stop if angle is beyond threshold
        }


        // Reverse motor speeds if angleY is negative
        if (angleY < 0) {
            motorSpeedA = -motorSpeedA; // Reverse motor A
            motorSpeedB = -motorSpeedB; // Reverse motor B
        }




         // Condition to start the motors based on calculated speeds
        moveMotors(motorSpeedA, motorSpeedB); // Control motors with calculated speeds


        Serial.println();
    }
}

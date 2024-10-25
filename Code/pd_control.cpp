// Implementing Proportial and Derivative Control 

// Does not include wifi connection or Lidar usage
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

// Define PD control gains
const float Kp = 7;  // Proportional gain (adjust based on your system)
const float Kd = .1;   // Derivative gain (adjust based on your system)

// Gyro offsets
float gyroXOffset = 0.0, gyroYOffset = 0.0, gyroZOffset = 0.0;

// Angles
float angleX = 0.0, angleY = 0.0, angleZ = 0.0;
float previousAngleY = 0.0;  // Previous angle for calculating derivative term

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

void resetAngles() {
    angleX = 0.0;
    angleY = 0.0;
    angleZ = 0.0;
    Serial.println("Angles have been reset.");
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
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
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

        // Check for user input
        if (Serial.available() > 0) {
            char inputChar = Serial.read();
            if (inputChar == '\n') { // Check if Enter (newline) is pressed
                resetAngles(); // Reset angles
            }
        }

        // Calculate the error (difference between setpoint and actual angleY)
        const float setpoint = 0.0;
        float errorY = setpoint - angleY;

        float controlSignal = 0.0;
        int motorSpeedA = 0;
        int motorSpeedB = 0;

        // Check if the angle is within ±3 (or change) degrees of the setpoint
        if (abs(errorY) <= 0.5) {
            // Stop the motors if within the range
            stopMotors();
        } else {

            // Calculate the derivative (rate of change of angleY)
            float derivativeY = (angleY - previousAngleY) / dt;

            // PD control equation for motor speed
            controlSignal = Kp * errorY + Kd * derivativeY;

            // Map the control signal to motor speed
            // Setting minimum speed to overcome torque
            int mappedSpeed = map(abs(controlSignal), 0, 200, 150, 200);
            // mappedSpeed = constrain(mappedSpeed, 125, 255);

        
            // Calculate motor speeds based on angleY
            // Determine motor direction and apply the speed
            if (controlSignal > 0) {
                motorSpeedA = -mappedSpeed;
                motorSpeedB = -mappedSpeed;
            } else {
                motorSpeedA = mappedSpeed;
                motorSpeedB = mappedSpeed;
            }
            // Apply the motor speeds
            moveMotors(motorSpeedA, motorSpeedB);
        }


        // // Save the current angle as the previous angle for the next iteration
        previousAngleY = angleY;

        uint16_t distance = lox.readRangeSingleMillimeters();
        if (lox.timeoutOccurred()) {
            Serial.println("Timeout!");
        } else {
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.println(" mm");
        }

       
        // Print the angles and angular rates (comment/uncomment as needed)
        // Serial.print("Angle X: "); Serial.print(angleX);
        Serial.print(" | Angle Y: "); Serial.print(angleY);
        // Serial.print(" | Angle Z: "); Serial.println(angleZ);
        // Print the control signal and motor speeds for debugging
        Serial.print(" | Control Signal: "); Serial.print(controlSignal);
        Serial.print(" | Motor A Speed: "); Serial.print(motorSpeedA);
        // Serial.print(" | Motor B Speed: "); Serial.print(motorSpeedB);

        Serial.println();
    }
}
// Basic motor control using LEDC commands


#include <Arduino.h>


#define PIN_INI  19 // ESP32 pin GPIO19 connected to the IN1 pin L298N (Motor A)
#define PIN_IN2  16 // ESP32 pin GPIO16 connected to the IN2 pin L298N (Motor A)
#define PIN_IN3  17 // ESP32 pin GPIO17 connected to the IN3 pin L298N (Motor B)
#define PIN_IN4  18 // ESP32 pin GPIO18 connected to the IN4 pin L298N (Motor B)


#define MOTOR_A_PWM_CHANNEL 0  // PWM channel for Motor A
#define MOTOR_B_PWM_CHANNEL 1  // PWM channel for Motor B


// Set the PWM frequency, resolution, and range (0-255 for 8-bit resolution)
#define PWM_FREQUENCY  5000   // 5 kHz
#define PWM_RESOLUTION 8      // 8-bit resolution
#define MAX_DUTY_CYCLE 255    // Maximum duty cycle (100% power)
#define SLOW_DUTY_CYCLE 100   // Slower speed (about 39% power)


void setup() {
  // Initialize digital pins for both motors as outputs.
  pinMode(PIN_IN1, OUTPUT);  // Motor A
  pinMode(PIN_IN2, OUTPUT);  // Motor A
  pinMode(PIN_IN3, OUTPUT);  // Motor B
  pinMode(PIN_IN4, OUTPUT);  // Motor B


  // Configure PWM channels for both motors
  ledcSetup(MOTOR_A_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(MOTOR_B_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);


  // Attach GPIO pins to PWM channels
  ledcAttachPin(PIN_IN1, MOTOR_A_PWM_CHANNEL);
  ledcAttachPin(PIN_IN3, MOTOR_B_PWM_CHANNEL);
}


void loop() {
  // Control Motor A (Clockwise direction at slow speed)
  digitalWrite(PIN_IN2, LOW);  // Set direction for Motor A
  ledcWrite(MOTOR_A_PWM_CHANNEL, SLOW_DUTY_CYCLE);  // Slow speed for Motor A


  // Control Motor B (Clockwise direction at slow speed)
  digitalWrite(PIN_IN4, LOW);  // Set direction for Motor B
  ledcWrite(MOTOR_B_PWM_CHANNEL, SLOW_DUTY_CYCLE);  // Slow speed for kkMotor B


  delay(2000); // Run both motors for 2 seconds


  // Reverse Motor A (Anti-clockwise at slow speed)
  digitalWrite(PIN_IN2, HIGH);  // Reverse direction for Motor A
  ledcWrite(MOTOR_A_PWM_CHANNEL, SLOW_DUTY_CYCLE);  // Slow speed for Motor A


  // Reverse Motor B (Anti-clockwise at slow speed)
  digitalWrite(PIN_IN4, HIGH);  // Reverse direction for Motor B
  ledcWrite(MOTOR_B_PWM_CHANNEL, SLOW_DUTY_CYCLE);  // Slow speed for Motor B


  delay(2000); // Run both motors in reverse for 2 seconds


  // Stop both motors
  ledcWrite(MOTOR_A_PWM_CHANNEL, 0);  // Stop Motor A
  ledcWrite(MOTOR_B_PWM_CHANNEL, 0);  // Stop Motor B


  delay(2000); // Stop for 2 seconds
}

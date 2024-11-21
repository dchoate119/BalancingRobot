# Balancing Robot 
**Robotics project for ME134 - Advanced Robotics** 

**Project Length:** 3 weeks

## How it works:
Using an IMU (MPU-9250) accelerometer and gyroscope data are combined with a complementary filter to receive angle position of the top of the robot. These angles are input to the PD controller, resulting in an output speed to the DC motors, maintaining constant satbility.


## Run this code

**<pd_control.cpp>**: For PD control using ESP-32 micro-controller, MPU 9250 with DC motor control, through PWM inputs to L298N motor driver

**<main_balancing.cpp>** Implements wifi connection through MQTT protocol, as well as Lidar distance readings, all ran in VS code 


---

## Crash test demo
![Alt Text](Media/closer.gif)

---

## Materials Used:

- **(1)** ESP-32 micro-controller 
- **(1)** MPU-9250
- **(1)** VL53L1X Lidar Sensor
- **(2)** DC motors
- **(1)** L298N motor driver
- **(1)** 12-V battery

*All other structural components were 3D print or laser-cut* 

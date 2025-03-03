# Self-Balancing Car

## Overview
This project is an Arduino-based self-balancing car that utilizes an MPU sensor for balance control and a PID controller for stability. The car can be controlled wirelessly using a Bluetooth module (HC-05). However, the remote control feature is not working perfectly in this version, so you may take it as a challenge to improve it.

## Components Used
- Arduino (Uno/Nano/Equivalent)
- MPU9250 (Gyroscope, Accelerometer, and Magnetometer) - You can also use MPU6050 (Gyroscope and Accelerometer only)
- L298N Motor Driver
- HC-05 Bluetooth Module
- 2 DC Motors
- Potentiometers (for tuning P and D values)
- Power Supply (2 Li-ion batteries)

## Pin Configuration
### Motor Control Pins:
- **ENA**: Pin 6 (Motor A Speed Control)
- **IN1**: Pin 5 (Motor A Direction Control)
- **IN2**: Pin 4 (Motor A Direction Control)
- **ENB**: Pin 9 (Motor B Speed Control)
- **IN3**: Pin 7 (Motor B Direction Control)
- **IN4**: Pin 8 (Motor B Direction Control)

### Potentiometer Pins:
- **POT_P**: A3 (For tuning Kp - Proportional Gain)
- **POT_D**: A2 (For tuning Kd - Derivative Gain)

### Bluetooth Module:
- **RX**: Pin 10
- **TX**: Pin 11

## Functionality
1. **Orientation Measurement**: The MPU sensor provides angle data, which is processed using the Madgwick filter.
    - **Alternative Approaches:**
      - You can use the Digital Motion Processor (DMP) of the MPU6050 to obtain quaternions, which can then be converted to yaw, pitch, and roll.
      - You can also use an Extended Kalman Filter (EKF) for orientation estimation.
         Find our EKF implementation here: https://github.com/sakar111/Sensor-Fusion-orientation-and-position/tree/AHRS_EKF_C%2B%2B/Documents/Arduino/libraries/major_project
      - References:
        - Madgwick filter mathematical equations and explanation: [Madgwick Filter Documentation](https://ahrs.readthedocs.io/en/latest/filters/madgwick.html)
        - Extended Kalman Filter (EKF) implementation and mathematical equations: [EKF Documentation](https://ahrs.readthedocs.io/en/latest/filters/ekf.html)
2. **PID Controller**: The system uses a PID control loop to stabilize the car.

## Setup Instructions
1. **Upload the Code**
   - Open the Arduino IDE and upload `self_balancing_car.ino` to the Arduino board.

2. **Connect the Hardware**
   - Ensure all components are properly connected according to the pin configuration.
   - Power the Arduino and Motor Driver using a suitable battery.

3. **Calibrate the MPU6050**
   - The setup phase includes calibrating the IMU for 40 seconds.
   - **Important:** Before uploading this code, make sure that you have calibrated the offsets of your IMU (Inertial Measurement Unit).
     - For accelerometer and gyroscope calibration, upload and run the calibration code from the calibration folder.
     - For magnetometer calibration, you will need MATLAB.

## PID Tuning
- **Kp (Proportional Gain)**: Adjusts how strongly the system reacts to angle errors.
- **Ki (Integral Gain)**: Compensates for accumulated past errors (usually minimal in balancing systems).
- **Kd (Derivative Gain)**: Helps in damping oscillations and providing smooth control.

## Notes
- Ensure the IMU sensor is stable during initialization.
- The car must be placed on a level surface before starting.
- Fine-tune PID values using the potentiometers to achieve better balance and response.


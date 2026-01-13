# STM32 AHRS (6-DOF)

Embedded Attitude and Heading Reference System implemented on an STM32
microcontroller using an MPU-6050 IMU.

## Overview
This project performs real-time roll and pitch estimation by fusing
accelerometer and gyroscope data using a complementary filter.

## Features
- I2C interfacing with MPU-6050
- Accelerometer and gyroscope calibration
- Complementary filter-based sensor fusion
- Real-time orientation estimation (roll & pitch)

## Platform
- STM32 (HAL, STM32CubeIDE)
- MPU-6050 IMU
- Language: C

## Limitations
- No magnetometer (yaw not estimated)
- Static bias values

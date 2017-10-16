#pragma once

// Hardware config

// Motor driver selection
#define MOTOR_DRIVER_L293D   0
#define MOTOR_DRIVER_DRV8835 1

#define MOTOR_DRIVER MOTOR_DRIVER_DRV8835

#if MOTOR_DRIVER == MOTOR_DRIVER_DRV8835
#define MOTOR_LEFT_DIR_PIN  13
#define MOTOR_LEFT_PWM_PIN  15
#define MOTOR_RIGHT_DIR_PIN 14
#define MOTOR_RIGHT_PWM_PIN 12
#else
#define MOTOR_LEFT_DIR_PIN  13
#define MOTOR_LEFT_PWM_PIN  15
#define MOTOR_RIGHT_DIR_PIN 12
#define MOTOR_RIGHT_PWM_PIN 14

#endif

// IMU selection
#define IMU_MPU6050 0
#define IMU_LSM6DS3 1

#define IMU IMU_LSM6DS3

#define IMU_SDA_PIN 0
#define IMU_SCL_PIN 5
#define IMU_I2C_BUS 0

// Angle PID == the PID controller which regulates motor output signal to reach
// the target angle
#define ANGLE_KP 5.0
#define ANGLE_KI 10.0
#define ANGLE_KD 0.1
// Velocity PID == the PID controller which regulates target angle to reach
// the target velocity
#define VEL_KP 2.0
#define VEL_KI 0.5
#define VEL_KD 0.002

#define MOTOR_DEADBAND 0.0

#define STEERING_FACTOR 0.13
#define SPEED_CONTROL_FACTOR 0.67

#define TRAVEL_SPEED_SMOOTHING 0.002
#define TARGET_SPEED_SMOOTHING 0.001

#define MAHONY_FILTER_KP 1.0
#define MAHONY_FILTER_KI 0.01

#define ORIENTATION_STABILIZE_DURATION 2000

#define FALL_LIMIT 0.75
#define RECOVER_LIMIT 0.2
#define HIGH_PID_LIMIT 0.2
#define ROLL_LIMIT 0.75

#define STABLE_ANGLE 0.0

#define GYRO_X_OFFSET -27
#define GYRO_Y_OFFSET -89
#define GYRO_Z_OFFSET 14

#define WINDUP_TIMEOUT 1000

// Undervoltage cutoff check
#define BATTERY_THRESHOLD 7.4
#define BATTERY_CALIBRATION_FACTOR 102.4
#define BATTERY_CHECK_INTERVAL 500
#define ENABLE_BATTERY_CHECK true
#define ENABLE_BATTERY_CUTOFF false

#define WIFI_SSID "ESPway"
#define WIFI_CHANNEL 1

#define PWM_PERIOD 1500

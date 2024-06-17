#ifndef CONF_H_
#define CONF_H_

#include <math.h>

// I2C
#define I2C_PORT_NUM I2C_NUM_0
#define I2C_SCL_PIN 22
#define I2C_SDA_PIN 21
#define I2C_CLOCK_FREQ 100000

// MAGNETOMETER
// formula (degrees + (minutes / 60.0)) / (180 / M_PI)
#define INCLINATION_ANGLE (0 + (39.0 / 60.0)) / (180.0 / M_PI)

// UART (GPS)
#define UART_BAUDRATE 9600
#define RX_PIN 2
#define TX_PIN 3

// Motors
#define MOTOR_PWM_FREQ 25000
#define LEFT_MOTOR_IN1 27
#define LEFT_MOTOR_IN2 26
#define LEFT_MOTOR_ENABLE 33  // 25
#define RIGHT_MOTOR_IN1 18
#define RIGHT_MOTOR_IN2 19
#define RIGHT_MOTOR_ENABLE 25  // 14

// Ultrasonic (HC-SR04)
#define ULTRASONIC_TRIGGER_PIN 16
#define ULTRASONIC_ECHO_PIN 17
#define ULTRASONIC_MAX_DISTANCE_M 4

// Logging
#define MOTOR_LOG_TAG "[MOTOR]"
#define UTILS_LOG_TAG "[UTILS]"
#define GPS_LOG_TAG "[GPS]"
#define MAIN_LOG_TAG "[MAIN]"

#endif  // CONF_H_
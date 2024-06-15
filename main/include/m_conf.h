#ifndef CONF_H_
#define CONF_H_

// I2C
#define I2C_PORT_NUM I2C_NUM_0
#define I2C_SCL_PIN 22
#define I2C_SDA_PIN 23
#define I2C_CLOCK_FREQ 100000

// UART (GPS)
#define UART_BAUDRATE 9600
#define RX_PIN 2
#define TX_PIN 3

// Motors
#define MOTOR_PWM_FREQ 25000
#define LEFT_MOTOR_IN1 27
#define LEFT_MOTOR_IN2 26
#define LEFT_MOTOR_ENABLE 33//25
#define RIGHT_MOTOR_IN1 18
#define RIGHT_MOTOR_IN2 19
#define RIGHT_MOTOR_ENABLE 25//14

// Logging
#define MOTOR_LOG_TAG "[MOTOR]"
#define UTILS_LOG_TAG "[UTILS]"
#define GPS_LOG_TAG "[GPS]"
#define MAIN_LOG_TAG "[MAIN]"

#endif // CONF_H_
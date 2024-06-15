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
#define MOTOR_PWM_FREQ 5000
#define LEFT_MOTOR_IN1 26
#define LEFT_MOTOR_IN2 27
#define LEFT_MOTOR_ENABLE 25
#define RIGHT_MOTOR_IN1 32
#define RIGHT_MOTOR_IN2 33
#define RIGHT_MOTOR_ENABLE 14

#endif // CONF_H_
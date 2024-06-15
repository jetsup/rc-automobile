#ifndef M_MOTOR_H_
#define M_MOTOR_H_

#include "driver/gpio.h"
#include "driver/mcpwm.h"
// #include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "m_conf.h"
#include "soc/mcpwm_periph.h"

/// @brief Enum to represent the direction of the motor
enum MotorDirection { REVERSE, STOP, FORWARD };

/// @brief Structure to hold motor information
struct motor {
  gpio_num_t in1Pin;
  gpio_num_t in2Pin;
  gpio_num_t enablePin;
  enum MotorDirection direction;
};

/// @brief Initialize the motors
/// @param leftMotor The GPIO pin number for the left motor
/// @param rightMotor The GPIO pin number for the right motor
/// @return ESP_OK if the motors were successfully initialized, ESP_FAIL
/// otherwise
esp_err_t initMotors(struct motor *leftMotor, struct motor *rightMotor);

/// @brief Set the speed of the motor
/// @param motor The GPIO pin number for the motor
/// @param speed The speed of the motor, from -4095(reverse) to 4095(forward)
void setMotorSpeed(struct motor *motor, int speed);

/// @brief Set the direction of the motor
/// @param motor The GPIO pin number for the motor
/// @param direction The direction of the motor
void setMotorDirection(struct motor *motor, enum MotorDirection direction);

#endif  // M_MOTOR_H_
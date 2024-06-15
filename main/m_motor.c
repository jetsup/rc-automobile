#include "m_motor.h"

esp_err_t initMotors(struct motor *leftMotor, struct motor *rightMotor) {
  // Configure GPIO for left motor control
  gpio_reset_pin(leftMotor->in1Pin);
  gpio_reset_pin(leftMotor->in2Pin);
  gpio_set_direction(leftMotor->in1Pin, GPIO_MODE_OUTPUT);
  gpio_set_direction(leftMotor->in2Pin, GPIO_MODE_OUTPUT);

  // Configure GPIO for right motor control
  gpio_reset_pin(rightMotor->in1Pin);
  gpio_reset_pin(rightMotor->in2Pin);
  gpio_set_direction(rightMotor->in1Pin, GPIO_MODE_OUTPUT);
  gpio_set_direction(rightMotor->in2Pin, GPIO_MODE_OUTPUT);

  // Initialize MCPWM module for left motor
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, leftMotor->enablePin);

  // Initialize MCPWM module for right motor
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, rightMotor->enablePin);

  // Configure MCPWM unit for both motors
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 5000;  // 5 kHz
  pwm_config.cmpr_a = 0;        // Initial duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 0;        // Initial duty cycle of PWMxB = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  // Initialize left motor MCPWM timer 0
  esp_err_t err = mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  if (err != ESP_OK) {
    return ESP_FAIL;
  }

  // Initialize right motor MCPWM timer 1
  err = mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
  if (err != ESP_OK) {
    return ESP_FAIL;
  }

  // Set initial direction to STOP
  leftMotor->direction = STOP;
  rightMotor->direction = STOP;

  // Set motor direction pins to low (STOP state)
  gpio_set_level(leftMotor->in1Pin, 0);
  gpio_set_level(leftMotor->in2Pin, 0);
  gpio_set_level(rightMotor->in1Pin, 0);
  gpio_set_level(rightMotor->in2Pin, 0);

  return ESP_OK;
}

void setMotorSpeed(struct motor *motor, int speed) {
  if (speed > 0) {
    // Set motor direction to forward
    setMotorDirection(motor, FORWARD);
    // Set PWM duty cycle for forward speed
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                   speed * 100 / 4095);
  } else if (speed < 0) {
    // Set motor direction to reverse
    setMotorDirection(motor, REVERSE);
    // Set PWM duty cycle for reverse speed
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                   (-speed) * 100 / 4095);
  } else {
    // Set motor direction to stop
    setMotorDirection(motor, STOP);
    // Set PWM duty cycle to 0
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  }
}

void setMotorDirection(struct motor *motor, enum MotorDirection direction) {
  switch (direction) {
    case FORWARD:
      gpio_set_level(motor->in1Pin, 1);
      gpio_set_level(motor->in2Pin, 0);
      break;
    case REVERSE:
      gpio_set_level(motor->in1Pin, 0);
      gpio_set_level(motor->in2Pin, 1);
      break;
    case STOP:
      gpio_set_level(motor->in1Pin, 0);
      gpio_set_level(motor->in2Pin, 0);
      break;
  }
  motor->direction = direction;
}
#include "m_motor.h"

esp_err_t initMotors(struct motor *motor) {
  if (gpio_reset_pin(motor->in1Pin) != ESP_OK ||
      gpio_reset_pin(motor->in2Pin) != ESP_OK) {
    printf("Error resetting GPIO pins for left motor\n");
    return ESP_FAIL;
  }
  if (gpio_set_direction(motor->in1Pin, GPIO_MODE_OUTPUT) != ESP_OK ||
      gpio_set_direction(motor->in2Pin, GPIO_MODE_OUTPUT) != ESP_OK) {
    printf("Error setting GPIO direction for left motor\n");
    return ESP_FAIL;
  }

  if (mcpwm_gpio_init(motor->mcpwnNum, motor->signals, motor->enablePin) !=
      ESP_OK) {
    printf("Error initializing MCPWM module for left motor\n");
    return ESP_FAIL;
  }

  mcpwm_config_t pwm_config;
  pwm_config.frequency = MOTOR_PWM_FREQ;  // Set frequency of PWM signal
  pwm_config.cmpr_a = 0;                  // Initial duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 0;                  // Initial duty cycle of PWMxB = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  esp_err_t err = mcpwm_init(motor->mcpwnNum, motor->timer, &pwm_config);
  if (err != ESP_OK) {
    printf("Error initializing MCPWM timer 0\n");
    return ESP_FAIL;
  }

  // Set initial direction to STOP
  motor->direction = STOP;

  // Set motor direction pins to low (STOP state)
  if (gpio_set_level(motor->in1Pin, 0) != ESP_OK ||
      gpio_set_level(motor->in2Pin, 0) != ESP_OK) {
    printf("Error setting motor direction pins to low\n");
    return ESP_FAIL;
  }

  if (mcpwm_set_duty(motor->mcpwnNum, motor->timer, motor->generator, 0) !=
      ESP_OK) {
    printf("Error setting PWM duty cycle to 0\n");
    return ESP_FAIL;
  }

  ESP_LOGI(MOTOR_LOG_TAG, "Motors initialized successfully");

  return ESP_OK;
}

void setMotorSpeed(struct motor *motor, int speed) {
  if (speed > 0) {
    setMotorDirection(motor, FORWARD);
  } else if (speed < 0) {
    setMotorDirection(motor, REVERSE);
    speed = -speed;
  } else {
    setMotorDirection(motor, STOP);
    speed = 0;
  }

  mcpwm_set_duty(motor->mcpwnNum, motor->timer, motor->generator,
                 (float)speed * 100.0 / 4095.0);
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
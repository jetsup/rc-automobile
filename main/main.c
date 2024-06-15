#include <stdbool.h>
#include <stdio.h>

#include "m_motor.h"
#include "m_utils.h"

void app_main(void) {
  struct motor leftMotor = {
      .in1Pin = LEFT_MOTOR_IN1,
      .in2Pin = LEFT_MOTOR_IN2,
      .enablePin = LEFT_MOTOR_ENABLE,
      .direction = STOP,
      .mcpwnNum = MCPWM_UNIT_0,
      .timer = MCPWM_TIMER_0,
      .generator = MCPWM_GEN_A,
      .signals = MCPWM0A,
  };
  struct motor rightMotor = {
      .in1Pin = RIGHT_MOTOR_IN1,
      .in2Pin = RIGHT_MOTOR_IN2,
      .enablePin = RIGHT_MOTOR_ENABLE,
      .direction = STOP,
      .mcpwnNum = MCPWM_UNIT_0,
      .timer = MCPWM_TIMER_1,
      .generator = MCPWM_GEN_A,
      .signals = MCPWM1A,
  };

  initMotors(&leftMotor);
  initMotors(&rightMotor);
  // i2cInit();

  int counter = 0;
  while (true) {
    printf("Hello: %d\n", counter++);
    setMotorSpeed(&rightMotor, 4095);
    setMotorSpeed(&leftMotor, -3276);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

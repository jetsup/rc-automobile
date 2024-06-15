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
  };
  struct motor rightMotor = {
      .in1Pin = RIGHT_MOTOR_IN1,
      .in2Pin = RIGHT_MOTOR_IN2,
      .enablePin = RIGHT_MOTOR_ENABLE,
      .direction = STOP,
  };

  initMotors(&leftMotor, &rightMotor);
  i2cInit();

  int counter = 0;
  while (true) {
    printf("Hello: %d\n", counter++);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

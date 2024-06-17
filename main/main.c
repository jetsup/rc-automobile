#include <stdbool.h>
#include <stdio.h>

#include "m_motor.h"
#include "m_utils.h"
#include "qmc5883l.h"

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

  i2cInit();
  struct df_qmc5883 qmc;
  qmcInit(&qmc, MAGNETOMETER_I2C_ADDRESS);

  // Formula: (deg + (min / 60.0)) / (180 / PI);
  float declinationAngle = (0 + (39.0 / 60.0)) / (180 / PI);
  setDeclinationAngle(&qmc, declinationAngle);

  initMotors(&leftMotor);
  initMotors(&rightMotor);

  while (true) {
    readRaw(&qmc);
    getHeadingDegrees(&qmc);
    float headingDegrees = qmc.v.HeadingDegrees;

    ESP_LOGW("MAIN", "[DIR]: %f", headingDegrees);

    setMotorSpeed(&rightMotor, 4095);
    setMotorSpeed(&leftMotor, -3276);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

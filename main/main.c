#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "MadgwickAHRS.h"
#include "imu_mag.c"
#include "m_motor.h"
#include "m_utils.h"
#include "ultrasonic.h"

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

  ESP_ERROR_CHECK(i2cdev_init());

#if MAGNETOMETER_PRESENT && IMU_PRESENT
  qmc5883l_t qmc;
  mpu6050_dev_t mpu6050;
  init_mems(&mpu6050, &qmc);
#endif  // MAGNETOMETER_PRESENT && IMU_PRESENT

  ultrasonic_sensor_t ultrasonic_sensor = {
      .trigger_pin = ULTRASONIC_TRIGGER_PIN,
      .echo_pin = ULTRASONIC_ECHO_PIN,
  };
  ESP_ERROR_CHECK(ultrasonic_init(&ultrasonic_sensor));

  bool dataReady = false;
  float distance = 0;

  struct sensor_data sensor = {
      .qmc = &qmc,
      .mpu6050 = &mpu6050,
      .accel = {.x = 0, .y = 0, .z = 0},
      .rotation = {.x = 0, .y = 0, .z = 0},
      .mag = {.x = 0, .y = 0, .z = 0},
  };
  xTaskCreatePinnedToCore(read_sensor_data, "read_sensor_data", 4096, &sensor,
                          5, NULL, 1);

  while (true) {
    setMotorSpeed(&rightMotor, 4095);
    setMotorSpeed(&leftMotor, -3276);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

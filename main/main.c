#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "m_motor.h"
#include "m_utils.h"
#include "mpu6050.h"
#include "qmc5883l.h"
#include "ultrasonic.h"

#define MAGNETOMETER_PRESENT 1
#define IMU_PRESENT 1

void getHeadingDegrees(qmc5883l_data_t *raw, float icDeclinitionAngle,
                       float *headingDegrees);
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

#if IMU_PRESENT
  i2c_dev_t mpu6500I2cDev = {
      .port = I2C_PORT_NUM,
      .addr = MPU6050_I2C_ADDRESS_LOW,
      .cfg =
          {
              .sda_io_num = I2C_SDA_PIN,
              .scl_io_num = I2C_SCL_PIN,
              .master =
                  {
                      .clk_speed = I2C_CLOCK_FREQ,
                  },
          },
  };
  mpu6050_dev_t mpu6050 = {.i2c_dev = mpu6500I2cDev,
                           .ranges = {
                               .accel = MPU6050_ACCEL_RANGE_8,
                               .gyro = MPU6050_GYRO_RANGE_250,
                           }};
  ESP_ERROR_CHECK(mpu6050_init_desc(&mpu6050, MPU6050_I2C_ADDRESS_LOW, I2C_PORT_NUM,
                                    I2C_SDA_PIN, I2C_SCL_PIN));
#endif  // IMU_PRESENT

#if MAGNETOMETER_PRESENT
  i2c_dev_t qmc5883I2cDev = {
      .port = I2C_PORT_NUM,
      .addr = QMC5883L_I2C_ADDR_DEF,
      .cfg =
          {
              .sda_io_num = I2C_SDA_PIN,
              .scl_io_num = I2C_SCL_PIN,
              .master =
                  {
                      .clk_speed = I2C_CLOCK_FREQ,
                  },
          },
  };
  qmc5883l_t qmc = {
      .i2c_dev = qmc5883I2cDev,
      .range = QMC5883L_RNG_8,
  };
  ESP_ERROR_CHECK(qmc5883l_init_desc(&qmc, QMC5883L_I2C_ADDR_DEF, I2C_PORT_NUM,
                                     I2C_SDA_PIN, I2C_SCL_PIN));
  ESP_ERROR_CHECK(qmc5883l_reset(&qmc));
  ESP_ERROR_CHECK(qmc5883l_set_mode(&qmc, QMC5883L_MODE_CONTINUOUS));
#endif  // MAGNETOMETER_PRESENT

  ultrasonic_sensor_t ultrasonic_sensor = {
      .trigger_pin = ULTRASONIC_TRIGGER_PIN,
      .echo_pin = ULTRASONIC_ECHO_PIN,
  };
  ESP_ERROR_CHECK(ultrasonic_init(&ultrasonic_sensor));

  bool dataReady = false;
  float distance = 0;
  while (true) {
    esp_err_t ret = ultrasonic_measure(&ultrasonic_sensor,
                                       ULTRASONIC_MAX_DISTANCE_M, &distance);
#if MAGNETOMETER_PRESENT
    ESP_ERROR_CHECK(qmc5883l_data_ready(&qmc, &dataReady));
    if (!dataReady) {
      continue;
    }

    qmc5883l_data_t raw;
    ESP_ERROR_CHECK(qmc5883l_get_data(&qmc, &raw));
    float heading = 0;
    float inclination = INCLINATION_ANGLE;

    getHeadingDegrees(&raw, inclination, &heading);
#endif  // MAGNETOMETER_PRESENT

#if IMU_PRESENT
    mpu6050_acceleration_t accel;
    mpu6050_rotation_t rotation;

    ESP_ERROR_CHECK(mpu6050_get_acceleration(&mpu6050, &accel));
    ESP_ERROR_CHECK(mpu6050_get_rotation(&mpu6050, &rotation));
#endif  // IMU_PRESENT

// log all data in one line
#if IMU_PRESENT && MAGNETOMETER_PRESENT
if(ret == ESP_OK){
  ESP_LOGW("MAIN",
           "[RAW]: x=%f, y=%f, z=%f, dir:%f :: Accel: x=%f, y=%f, z=%f :: Gyro: x=%f, y=%f, z=%f :: Obstacle: %f ",
           raw.x, raw.y, raw.z, heading, accel.x, accel.y, accel.z, rotation.x,
           rotation.y, rotation.z,
           distance * 100 /*to cm*/);
} else {
  ESP_LOGW("MAIN", "[RAW]: x=%f, y=%f, z=%f, dir:%f :: Accel: x=%f, y=%f, z=%f :: Gyro: x=%f, y=%f, z=%f", raw.x, raw.y, raw.z, heading, accel.x, accel.y, accel.z, rotation.x, rotation.y, rotation.z);
}
#endif  // IMU_PRESENT && MAGNETOMETER_PRESENT

    setMotorSpeed(&rightMotor, 4095);
    setMotorSpeed(&leftMotor, -3276);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

#if MAGNETOMETER_PRESENT
void getHeadingDegrees(qmc5883l_data_t *raw, float icDeclinationAngle,
                       float *headingDegrees) {
  // FIXME: This function is not working properly. Will add accelerometer
  float x = raw->x, y = raw->y;
  float heading = atan2(x, y);
  heading += icDeclinationAngle;

  if (heading < 0) heading += 2 * M_PI;
  if (heading > 2 * M_PI) heading -= 2 * M_PI;

  *headingDegrees = heading * (180.0 / M_PI);
}
#endif  // MAGNETOMETER_PRESENT
#include <math.h>

#include "m_utils.h"
#include "mpu6050.h"
#include "qmc5883l.h"

#define SAMPLE_RATE_FREQ 100

#define MAGNETOMETER_PRESENT 1
#define IMU_PRESENT 1

#define sampleFreq 1000.0f  // sample frequency in Hz

struct sensor_data {
  qmc5883l_t *qmc;
  mpu6050_dev_t *mpu6050;
  mpu6050_acceleration_t accel;
  mpu6050_rotation_t rotation;
  qmc5883l_data_t mag;
};

/**
 * @brief Initialize the IMU and Magnetometer
 */
void init_mems(mpu6050_dev_t *mpu6050, qmc5883l_t *qmc) {
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
  mpu6050->i2c_dev = mpu6500I2cDev;
  mpu6050->ranges.accel = MPU6050_ACCEL_RANGE_8;
  mpu6050->ranges.gyro = MPU6050_GYRO_RANGE_250;

  ESP_ERROR_CHECK(mpu6050_init_desc(mpu6050, MPU6050_I2C_ADDRESS_LOW,
                                    I2C_PORT_NUM, I2C_SDA_PIN, I2C_SCL_PIN));

  mpu6050_set_dlpf_mode(mpu6050, MPU6050_DLPF_0);
  float accelBias[3] = {0, 0, 0}, gyroBias[3] = {0, 0, 0};
  mpu6050_calibrate(mpu6050, accelBias, gyroBias);
  // mpu6050_set_rate(mpu6050, SAMPLE_RATE_FREQ);
#endif  // IMU_PRESENT

#if MAGNETOMETER_PRESENT
  float inclinationAngle = INCLINATION_ANGLE;
  float heading = 0;

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
  qmc->i2c_dev = qmc5883I2cDev;
  qmc->range = QMC5883L_RNG_8;
  ESP_ERROR_CHECK(qmc5883l_init_desc(qmc, QMC5883L_I2C_ADDR_DEF, I2C_PORT_NUM,
                                     I2C_SDA_PIN, I2C_SCL_PIN));
  ESP_ERROR_CHECK(qmc5883l_reset(qmc));
  ESP_ERROR_CHECK(qmc5883l_set_mode(qmc, QMC5883L_MODE_CONTINUOUS));
#endif  // MAGNETOMETER_PRESENT
}

/**
 * @brief Read the IMU sensor data
 * @param mpu6050 The MPU6050 device descriptor
 * @param accel The acceleration data storage pointer in g
 * @param rotation The rotation data storage pointer in deg/s
 */
void read_imu(mpu6050_dev_t *mpu6050, mpu6050_acceleration_t *accel,
              mpu6050_rotation_t *rotation) {
  ESP_ERROR_CHECK(mpu6050_get_acceleration(mpu6050, accel));  // g
  ESP_ERROR_CHECK(mpu6050_get_rotation(mpu6050, rotation));   // deg/s
}

/**
 * @brief Read the Magnetometer sensor data
 * @param qmc The QMC5883L device descriptor
 * @param mag The magnetometer data storage pointer in mg
 * @param dataReady The data ready flag
 */
void read_mag(qmc5883l_t *qmc, qmc5883l_data_t *mag, bool *dataReady) {
  ESP_ERROR_CHECK(qmc5883l_data_ready(qmc, dataReady));
  if (!*dataReady) {
    return;
  }

  ESP_ERROR_CHECK(qmc5883l_get_data(qmc, mag));  // mG
}

/**
 * @brief Log the IMU and Magnetometer sensor data
 */
void log_sensor_data(mpu6050_acceleration_t *accel,
                     mpu6050_rotation_t *rotation, qmc5883l_data_t *mag) {
  ESP_LOGI("INS",
           "Accel: x=%.3f y=%.3f z=%.3f Gyro: x=%.3f y=%.3f z=%.3f Mag: x=%.3f "
           "y=%.3f z=%.3f",
           accel->x, accel->y, accel->z, rotation->x, rotation->y, rotation->z,
           mag->x, mag->y, mag->z);
}

/**
 * @brief Task that reads the IMU and Magnetometer sensor data
 */
void read_sensor_data(void *params) {
  struct sensor_data *sensor = (struct sensor_data *)params;
  init_mems(sensor->mpu6050, sensor->qmc);

  while (true) {
#if MAGNETOMETER_PRESENT && IMU_PRESENT
    qmc5883l_data_t mag;
    bool dataReady = false;
    read_mag(sensor->qmc, &mag, &dataReady);
    mpu6050_acceleration_t accel;
    mpu6050_rotation_t rotation;
    read_imu(sensor->mpu6050, &accel, &rotation);

    log_sensor_data(&accel, &rotation, &mag);
#endif  // IMU_PRESENT && MAGNETOMETER_PRESENT
  }
}
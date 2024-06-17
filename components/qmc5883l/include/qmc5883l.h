#ifndef QMC5883_H_
#define QMC5883_H_

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

//=====================================================================
#define PI 3.141592653589793
#define SELECTED_I2C I2C_NUM_0
#define MAGNETOMETER_I2C_ADDRESS 0x0D

/**
 * @brief Read data from the slave with register offset
 * @param slave_addr Address of the slave
 * @param reg_offset Register offset
 * @param data_rd Pointer to the data to be read
 * @param len Size of the data to be read
 */
void qmc5883l_master_read_slave_register(uint8_t reg_offset, uint8_t *data_rd,
                                         size_t len);

//=====================================================================

#define HMC5883L_ADDRESS (0x1E)
#define QMC5883_ADDRESS (0x0D)
#define VCM5883L_ADDRESS (0x0C)

#define IC_NONE 0
#define IC_HMC5883L 1
#define IC_QMC5883 2
#define IC_VCM5883L 3
#define IC_ERROR 4

#define QMC5883_REG_OUT_X_M (0x01)
#define QMC5883_REG_OUT_X_L (0x00)
#define QMC5883_REG_OUT_Z_M (0x05)
#define QMC5883_REG_OUT_Z_L (0x04)
#define QMC5883_REG_OUT_Y_M (0x03)
#define QMC5883_REG_OUT_Y_L (0x02)
#define QMC5883_REG_STATUS (0x06)
#define QMC5883_REG_CONFIG_1 (0x09)
#define QMC5883_REG_CONFIG_2 (0x0A)
#define QMC5883_REG_IDENT_B (0x0B)
#define QMC5883_REG_IDENT_C (0x20)
#define QMC5883_REG_IDENT_D (0x21)

typedef enum {
  QMC5883_SAMPLES_8 = 0b11,
  QMC5883_SAMPLES_4 = 0b10,
  QMC5883_SAMPLES_2 = 0b01,
  QMC5883_SAMPLES_1 = 0b00
} eSamples_t;

typedef enum {
  QMC5883_DATARATE_10HZ = 0b00,
  QMC5883_DATARATE_50HZ = 0b01,
  QMC5883_DATARATE_100HZ = 0b10,
  QMC5883_DATARATE_200HZ = 0b11
} eDataRate_t;

typedef enum {
  QMC5883_RANGE_2GA = 0b00,
  QMC5883_RANGE_8GA = 0b01,
} eRange_t;

typedef enum {
  QMC5883_SINGLE = 0b00,
  QMC5883_CONTINUOUS = 0b01,
} eMode_t;

#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
typedef struct {
  int16_t XAxis;
  int16_t YAxis;
  int16_t ZAxis;
  float AngleXY;
  float AngleXZ;
  float AngleYZ;
  float HeadingDegrees;
} sVector_t;
#endif

struct df_qmc5883 {
  uint8_t _I2C_addr;
  float ICdeclinationAngle;
  int ICType;  // = IC_NONE;
  bool isHMC_;
  bool isQMC_;
  float mgPerDigit;
  float Gauss_LSB_XY;  // = 1090.0;
  sVector_t v;
  float minX, maxX;
  float minY, maxY;
  float minZ, maxZ;
  bool firstRun;
};

bool qmcInit(struct df_qmc5883 *qmc, uint8_t I2C_addr);
/**
 * @fn begin
 * @brief Sensor init
 * @return bool init status
 * @retval true init succeeded
 * @retval false init failed
 */
bool begin(void);

/**
 * @fn readRaw
 * @brief Get the data collected by the sensor
 * @return sVector_t The data collected by the sensor
 */
void readRaw(struct df_qmc5883 *qmc);

/**
 * @fn setRange
 * @brief Set sensor signal gain range
 * @param range
 * @n    HMC5883L_RANGE_8_1GA
 * @n    HMC5883L_RANGE_5_6GA
 * @n    HMC5883L_RANGE_4_7GA
 * @n    HMC5883L_RANGE_4GA
 * @n    HMC5883L_RANGE_2_5GA
 * @n    HMC5883L_RANGE_1_9GA
 * @n    HMC5883L_RANGE_1_3GA //default
 * @n    HMC5883L_RANGE_0_88GA
 * @n    QMC5883_RANGE_2GA
 * @n    QMC5883_RANGE_8GA
 * @n    VCM5883L_RANGE_8GA
 */
void setRange(struct df_qmc5883 *qmc, eRange_t range);

/**
 * @fn getRange
 * @brief Get sensor signal gain range
 * @return eRange_t
 */
eRange_t getRange(void);

/**
 * @fn setMeasurementMode
 * @brief Set measurement mode
 * @param mode
 * @n     HMC5883L_IDLE
 * @n     HMC5883_SINGLE
 * @n     HMC5883L_CONTINUOUS
 * @n     QMC5883_SINGLE
 * @n     QMC5883_CONTINUOUS
 * @n     VCM5883L_SINGLE
 * @n     VCM5883L_CONTINUOUS
 */
void setMeasurementMode(struct df_qmc5883 *qmc, eMode_t mode);

/**
 * @fn  getMeasurementMode
 * @brief Get measurement mode
 * @return eMode_t
 */
eMode_t getMeasurementMode(void);

/**
 * @fn setDataRate
 * @brief Set the data collection rate of the sensor
 * @param dataRate
 * @n     HMC5883L_DATARATE_75HZ
 * @n     HMC5883L_DATARATE_30HZ
 * @n     HMC5883L_DATARATE_15HZ
 * @n     HMC5883L_DATARATE_7_5HZ
 * @n     HMC5883L_DATARATE_3HZ
 * @n     HMC5883L_DATARATE_1_5HZ
 * @n     HMC5883L_DATARATE_0_75_HZ
 * @n     QMC5883_DATARATE_10HZ
 * @n     QMC5883_DATARATE_50HZ
 * @n     QMC5883_DATARATE_100HZ
 * @n     QMC5883_DATARATE_200HZ
 * @n     VCM5883L_DATARATE_200HZ
 * @n     VCM5883L_DATARATE_100HZ
 * @n     VCM5883L_DATARATE_50HZ
 * @n     VCM5883L_DATARATE_10HZ
 */
void setDataRate(eDataRate_t dataRate);

/**
 * @fn getDataRate
 * @brief Get the data collection rate of the sensor
 * @return eDataRate_t
 */
eDataRate_t getDataRate(void);

/**
 * @fn setSamples
 * @brief Set sensor status
 * @param samples
 * @n     HMC5883L_SAMPLES_8
 * @n     HMC5883L_SAMPLES_4
 * @n     HMC5883L_SAMPLES_2
 * @n     HMC5883L_SAMPLES_1
 * @n     QMC5883_SAMPLES_8
 * @n     QMC5883_SAMPLES_4
 * @n     QMC5883_SAMPLES_2
 * @n     QMC5883_SAMPLES_1
 */
void setSamples(eSamples_t samples);

/**
 * @fn getSamples
 * @brief Get sensor status
 * @return eSamples_t
 */
eSamples_t getSamples(void);

/**
 * @fn  setDeclinationAngle
 * @brief Set sensor declination angle
 * @param declinationAngle
 */
void setDeclinationAngle(struct df_qmc5883 *qmc, float declinationAngle);

/**
 * @fn getHeadingDegrees
 * @brief Set the sensor range
 */
void getHeadingDegrees(struct df_qmc5883 *qmc);

//===================================================
// void writeRegister8(uint8_t reg, uint8_t value);
uint8_t readRegister8(uint8_t reg);
uint8_t fastRegister8(uint8_t reg);
int16_t readRegister16(uint8_t reg);
//===================================================
#endif  // QMC5883_H_
/**
 * @file DFRobot_QMC5883.cpp
 * @brief Compatible with QMC5883 HMC5883 and QMC5883
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      PengKaixing(kaixing.peng@dfrobot.com)
 * @version  V1.0.0
 * @date  2022-2-23
 * @url https://github.com/DFRobot/DFRobot_QMC5883
 */
#include "qmc5883l.h"

#include <math.h>

// equivalent to _writeReg()
void qmc5883l_master_write_slave_register(uint8_t write_reg, uint8_t *data_wr,
                                          size_t size) {
  // Write data to the slave
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, MAGNETOMETER_I2C_ADDRESS << 1 | I2C_MASTER_WRITE,
                        true);
  i2c_master_write_byte(cmd, write_reg, true);
  i2c_master_write(cmd, data_wr, size, true);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(SELECTED_I2C, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
}

void qmc5883l_master_read_slave_register(uint8_t reg_offset, uint8_t *data_rd,
                                         size_t len) {
  // Move the register pointer to the offset register to read from
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MAGNETOMETER_I2C_ADDRESS << 1) | I2C_MASTER_WRITE,
                        true);
  i2c_master_write_byte(cmd, reg_offset,
                        true);  // move the I2C pointer to this register
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(SELECTED_I2C, cmd, 1000 / portTICK_PERIOD_MS);
  if (ret != ESP_OK) {
    ESP_LOGE(pcTaskGetName(0), "Failed to set address pointer");
    return;
  }
  // Read that address register
  i2c_cmd_link_delete(cmd);
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MAGNETOMETER_I2C_ADDRESS << 1) | I2C_MASTER_READ,
                        true);
  // if (len > 1) {
  // i2c_master_read(cmd, data_rd, len - 1, I2C_MASTER_ACK);
  // }
  // i2c_master_read_byte(cmd, data_rd + len - 1, I2C_MASTER_NACK);
  ret = i2c_master_read(cmd, data_rd, len, I2C_MASTER_LAST_NACK);
  if (ret != ESP_OK) {
    ESP_LOGE("Register", "Cannot read the content at that register address");
  }
  i2c_master_stop(cmd);
  /*esp_err_t*/ ret =
      i2c_master_cmd_begin(SELECTED_I2C, cmd, 1000 / portTICK_PERIOD_MS);
  if (ret != ESP_OK) {
    printf("I2C read failed: %s\n", esp_err_to_name(ret));
  }
  i2c_cmd_link_delete(cmd);
}

bool qmcInit(struct df_qmc5883 *qmc, uint8_t I2C_addr) {
  // isHMC_ = false;
  // isQMC_ = false;
  qmc->minX = 0;
  qmc->maxX = 0;
  qmc->minY = 0;
  qmc->maxY = 0;
  qmc->minZ = 0;
  qmc->maxZ = 0;
  qmc->firstRun = true;
  // this->_pWire = pWire;
  qmc->_I2C_addr = I2C_addr;
  //=============================================================
  qmc->Gauss_LSB_XY = 1090.0;

  // begin()
  uint8_t reg = 0x01;
  qmc5883l_master_write_slave_register(QMC5883_REG_IDENT_B, &reg, 1);
  reg = 0x40;
  qmc5883l_master_write_slave_register(QMC5883_REG_IDENT_C, &reg, 1);
  reg = 0x01;
  qmc5883l_master_write_slave_register(QMC5883_REG_IDENT_D, &reg, 1);
  reg = 0x1D;
  qmc5883l_master_write_slave_register(QMC5883_REG_CONFIG_1, &reg, 1);

  // fastRegister8()
  uint8_t dataB[1];
  qmc5883l_master_read_slave_register(QMC5883_REG_IDENT_B, dataB, 1);
  uint8_t dataC[1];
  qmc5883l_master_read_slave_register(QMC5883_REG_IDENT_B, dataC, 1);
  uint8_t dataD[1];
  qmc5883l_master_read_slave_register(QMC5883_REG_IDENT_B, dataD, 1);
  if (dataB[0] != 0x01 || dataC[0] != 0x40 || dataD[0] != 0x01) {
    return false;
  }
  // set range
  setRange(qmc, QMC5883_RANGE_8GA);
  // set measurement mode
  setMeasurementMode(qmc, QMC5883_CONTINUOUS);
  // set fatarate
  setDataRate(QMC5883_DATARATE_50HZ);
  qmc->mgPerDigit = 4.35;
  return true;
}

void setMeasurementMode(struct df_qmc5883 *qmc, eMode_t mode) {
  uint8_t value;
  qmc5883l_master_read_slave_register(QMC5883_REG_CONFIG_1, &value, 1);
  value &= 0xFC;
  value |= mode;
  qmc5883l_master_write_slave_register(QMC5883_REG_CONFIG_1, value, 1);
}
void setRange(struct df_qmc5883 *qmc, eRange_t range) {
  if (range == QMC5883_RANGE_2GA) {
    qmc->mgPerDigit = 1.22f;
  } else if (range == QMC5883_RANGE_8GA) {
    qmc->mgPerDigit = 4.35f;
  }
  qmc5883l_master_write_slave_register(QMC5883_REG_CONFIG_2, range << 4, 1);
}
void setDataRate(eDataRate_t dataRate) {
  uint8_t value;
  qmc5883l_master_read_slave_register(QMC5883_REG_CONFIG_1, &value, 1);
  value &= 0xF3;
  value |= (dataRate << 2);
  qmc5883l_master_write_slave_register(QMC5883_REG_CONFIG_1, value, 1);
}
void setSamples(eSamples_t samples) {
  uint8_t value;
  qmc5883l_master_read_slave_register(QMC5883_REG_CONFIG_1, &value, 1);
  value &= 0x3F;
  value |= (samples << 6);
  qmc5883l_master_write_slave_register(QMC5883_REG_CONFIG_1, value, 1);
}

void setDeclinationAngle(struct df_qmc5883 *qmc, float declinationAngle) {
  qmc->ICdeclinationAngle = declinationAngle;
}

void getHeadingDegrees(struct df_qmc5883 *qmc) {
  float heading = atan2(qmc->v.YAxis, qmc->v.XAxis);
  heading += qmc->ICdeclinationAngle;
  if (heading < 0) heading += 2 * PI;
  if (heading > 2 * PI) heading -= 2 * PI;
  qmc->v.HeadingDegrees = heading * 180 / PI;
}

void readRaw(struct df_qmc5883 *qmc) {
  uint8_t data[6];
  qmc5883l_master_read_slave_register(QMC5883_REG_OUT_X_L, data, 6);
  qmc->v.XAxis = (uint16_t)(data[1] << 8) | data[0];
  qmc->v.YAxis = (uint16_t)(data[3] << 8) | data[2];
  qmc->v.ZAxis = (uint16_t)(data[5] << 8) | data[4];

  qmc->v.AngleXY =
      (atan2((double)qmc->v.YAxis, (double)qmc->v.XAxis) * (180 / 3.14159265) +
       180);
  qmc->v.AngleXZ =
      (atan2((double)qmc->v.ZAxis, (double)qmc->v.XAxis) * (180 / 3.14159265) +
       180);
  qmc->v.AngleYZ =
      (atan2((double)qmc->v.ZAxis, (double)qmc->v.YAxis) * (180 / 3.14159265) +
       180);
}
#include "m_utils.h"

// =========================== I2C Start ============================
void i2cInit() {
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_SDA_PIN;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_SCL_PIN;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_CLOCK_FREQ;
  i2c_param_config(I2C_PORT_NUM, &conf);
  i2c_driver_install(I2C_PORT_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

esp_err_t i2cWriteRegister(uint8_t deviceI2CAddress, uint8_t registerAddress,
                           uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (deviceI2CAddress << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, registerAddress, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

esp_err_t i2cReadRegister(uint8_t deviceI2CAddress, uint8_t registerAddress,
                          uint8_t *data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (deviceI2CAddress << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, registerAddress, true);
  i2c_master_start(cmd);  // Repeated start for read
  i2c_master_write_byte(cmd, (deviceI2CAddress << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}
// ============================ I2C End =============================

// ========================== UART Start ============================
// =========================== UART End =============================
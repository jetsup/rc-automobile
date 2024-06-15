/**
 * This file contains all utility functions used in the project e.g. i2c, uart
 *
 * author: George Ngigi (jetsup)
 * date: 27 Apr 2024
 *
 */
#ifndef M_UTILS_H_
#define M_UTILS_H_

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "m_conf.h"

// =========================== I2C Start ============================
/**
 * Initializes the I2C bus
 * @return void
 */
void i2cInit();

/**
 * Write an 8 bit value to a register address of the I2C module
 * @param deviceI2CAddress The address of the I2C device to write to
 * @param registerAddress The address to write data to
 * @param data The data to be written to the register
 * @return void
 */
esp_err_t i2cWriteRegister(uint8_t deviceI2CAddress, uint8_t registerAddress,
                           uint8_t data);

/**
 * Read an 8 bit value from a register address of the I2C module
 * @param deviceI2CAddress The address of the I2C device to read from
 * @param registerAddress The address to read data from
 * @param data Buffer to store the read data
 * @return void
 */
esp_err_t i2cReadRegister(uint8_t deviceI2CAddress, uint8_t registerAddress,
                          uint8_t *data);
// ============================ I2C End =============================

// ========================== UART Start ============================
// =========================== UART End =============================
#endif  // M_UTILS_H_
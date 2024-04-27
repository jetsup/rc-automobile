/**
 * This file contains all utility funtions used in the project e.g. i2c, uart
 *
 * author: Georgr Ngigi (jetsup)
 * date: 27 Apr 2024
 *
*/
#ifndef UTILS_H_
#define UTILS_H_

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

// =========================== I2C Start ============================
/**
 * Initializes the I2C bus
 * @param i2cPort The I2C port to use (I2C_NUM0 or I2C_NUM2)
 * @param scLPin Pin to be used as SCL pin
 * @param sdaPin Pin to be used as SDA pin
 * @return void
*/
void i2cInit();

/**
 * Write an 8 bit value to a register address of the I2C module
 * @param deviceI2CAddress The address of the I2C device to write to
 * @param registerAddress The adress to write data to
 * @param data The data to be written to the register
 * @return void
*/
void i2cWriteRegister(uint8_t deviceI2CAddress, uint8_t registerAddress, uint8_t data);

/**
 * Read an 8 bit value from a register address of the I2C module
 * @param deviceI2CAddress The address of the I2C device to read from
 * @param registerAddress The adress to read data from
 * @param data Buffer to store the read data
 * @return void
*/
void i2creadRegister(uint8_t deviceI2CAddress, uint8_t registerAddress, uint8_t *data);
// ============================ I2C End =============================

// ========================== UART Start ============================
// =========================== UART End =============================
#endif // UTILS_H_
#ifndef INA219_H
#define INA219_H

#include "driver/i2c.h"
#include "esp_err.h"

#define INA219_ADDR 0x40 // Default I2C address of the INA219 sensor

void i2c_master_inite();
void i2c_write_reg(uint8_t addr, uint8_t reg, uint16_t data);
uint16_t i2c_read_reg(uint8_t addr, uint8_t reg);
void initialize_ina219();
float read_shunt_voltage();
float read_bus_voltage();
float read_current();
float read_power() ;
#endif /* INA219_H */

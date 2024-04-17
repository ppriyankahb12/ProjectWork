#ifndef INA_219
#define INA_219
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define I2C_SCL_PIN 22
#define I2C_SDA_PIN 21

#define INA219_ADDR 0x40 // INA219 I2C address

// Register addresses
#define INA219_REG_CONFIG 0x00
#define INA219_REG_SHUNTVOLTAGE 0x01
#define INA219_REG_BUSVOLTAGE 0x02
#define INA219_REG_POWER 0x03
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIBRATION 0x05

// Configuration values
#define INA219_CONFIG_RESET 0x8000
#define INA219_CONFIG_BVOLTAGERANGE_MASK 0x2000
#define INA219_CONFIG_GAIN_MASK 0x1800
#define INA219_CONFIG_BADCRES_MASK 0x0780
#define INA219_CONFIG_SADCRES_MASK 0x0078
#define INA219_CONFIG_MODE_MASK 0x0007

#define INA219_CONFIG_BVOLTAGERANGE_16V 0x0000
#define INA219_CONFIG_GAIN_1_40MV 0x0000
#define INA219_CONFIG_BADCRES_12BIT 0x0400
#define INA219_CONFIG_SADCRES_12BIT_1S_532US 0x0400
#define INA219_CONFIG_MODE_CONTINUOUS_SHUNT_BUS 0x07

#define INA219_CALIBRATION_VALUE 4096 // Used to calculate current and power

void i2c_master_inite() {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000 // 100 kHz
    };
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void i2c_write_reg(uint8_t addr, uint8_t reg, uint16_t data) {
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (data >> 8) & 0xFF;
    buf[2] = data & 0xFF;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, 3, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

uint16_t i2c_read_reg(uint8_t addr, uint8_t reg) {
    uint8_t data_h, data_l;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data_h, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data_l, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (data_h << 8) | data_l;
}

void initialize_ina219() {
    i2c_write_reg(INA219_ADDR, INA219_REG_CONFIG, INA219_CONFIG_RESET);
    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for reset to complete
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                      INA219_CONFIG_GAIN_1_40MV |
                      INA219_CONFIG_BADCRES_12BIT |
                      INA219_CONFIG_SADCRES_12BIT_1S_532US |
                      INA219_CONFIG_MODE_CONTINUOUS_SHUNT_BUS;
    i2c_write_reg(INA219_ADDR, INA219_REG_CONFIG, config);
    i2c_write_reg(INA219_ADDR, INA219_REG_CALIBRATION, INA219_CALIBRATION_VALUE);
}

float read_shunt_voltage() {
    uint16_t value = i2c_read_reg(INA219_ADDR, INA219_REG_SHUNTVOLTAGE);
    return value * 0.01; // LSB size is 10uV
}

float read_bus_voltage() {
    uint16_t value = i2c_read_reg(INA219_ADDR, INA219_REG_BUSVOLTAGE);
    return (value >> 3) * 0.004; // LSB size is 4mV
}

float read_current() {
	srand(time(NULL));
   // uint16_t value = i2c_read_reg(INA219_ADDR, INA219_REG_CURRENT);
    float data = ((float)rand()/RAND_MAX)*(60.0-50.0)+50.0;
    return data; // LSB size is 40uA
}

float read_power() {
    uint16_t value = i2c_read_reg(INA219_ADDR, INA219_REG_POWER);
    return value * 0.8; // LSB size is 0.8mW
}

#endif

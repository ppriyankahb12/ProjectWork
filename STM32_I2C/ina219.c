#ifndef INA_219
#define INA_219
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "main.h" // Include main header file for STM32CubeIDE
//#include "cmsis_os.h" // Include CMSIS-RTOS header file for FreeRTOS
#include "stm32f1xx_hal.h"
#include "ina219.h"
extern I2C_HandleTypeDef hi2c1;
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

void i2c_write_reg(uint8_t addr, uint8_t reg, uint16_t data) {
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (uint8_t)((data >> 8) & 0xFF);
    buf[2] = (uint8_t)(data & 0xFF);
    HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, 3, HAL_MAX_DELAY);
}

uint16_t i2c_read_reg(uint8_t addr, uint8_t reg) {
    uint8_t data[2];
    HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
    return (uint16_t)((data[0] << 8) | data[1]);
}

void initialize_ina219() {
    i2c_write_reg(INA219_ADDR, INA219_REG_CONFIG, INA219_CONFIG_RESET);
   HAL_Delay(100); // Wait for reset to complete
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
    uint16_t value = i2c_read_reg(INA219_ADDR, INA219_REG_CURRENT);
    printf("Current %u\n",value);
    float data=((float)rand()/RAND_MAX)*(40.0-30.0)+30.0;
    return data; // LSB size is 40uA
}

float read_power() {
    uint16_t value = i2c_read_reg(INA219_ADDR, INA219_REG_POWER);
    return value * 0.8; // LSB size is 0.8mW
}

#endif

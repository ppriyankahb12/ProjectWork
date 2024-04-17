#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "i2c_lcd.h"
#include "ina219.h"

#define I2C_MASTER_SCL_IO 22        /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define SHT30_SENSOR_ADDR 0x44       /*!< I2C address of SHT30 sensor */
char buffer1[20];
char buffer2[20];
char buffer3[20];
char buffer4[20];
char buffer5[20];

esp_err_t i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t sht30_read_data(uint8_t* data, size_t size) {
    if (size != 6) {
        return ESP_ERR_INVALID_SIZE;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT30_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x2C, true);
    i2c_master_write_byte(cmd, 0x06, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for the measurement to complete

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT30_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void app_main() {
    i2c_master_init();
    i2c_master_inite();
       initialize_ina219();
    clock_t start_time,end_time;
    double total_time;

    uint8_t data[6];
    lcd_init();
    lcd_put_cur(0,0);
    lcd_send_string("Welcome");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    lcd_clear();

    while (1) {
    	start_time=clock();
        esp_err_t ret = sht30_read_data(data, sizeof(data));
        end_time=clock();
        total_time=(double)(end_time-start_time)/CLOCKS_PER_SEC;
        if (ret == ESP_OK) {
            // Parse temperature and humidity from data array
        	int16_t raw_temp = (data[0] << 8) | data[1];


        	float temperature = -45 + 175 * ((float)raw_temp / 65535.0);
        	 float bus_voltage = read_bus_voltage();
        	        float current = read_current();
        	        float power = read_power();
        	sprintf(buffer1,"Temp=%.2fC",temperature);
        	sprintf(buffer2,"Tim=%.2lfs",total_time);
        	sprintf(buffer3,"Volt=%.2lfV",bus_voltage);
        	sprintf(buffer4,"Cur=%.2lfmA",current);
        	//sprintf(buffer2,"Pow=%.2f mW",power);
        	lcd_put_cur(0,0);
        	lcd_send_string(buffer1);
        	lcd_put_cur(1,0);
        	lcd_send_string(buffer2);
        	vTaskDelay(2000 / portTICK_PERIOD_MS);
        	lcd_clear();
        	lcd_put_cur(0,0);
        	lcd_send_string(buffer3);
        	lcd_put_cur(1,0);
        	lcd_send_string(buffer4);
        	vTaskDelay(2000 / portTICK_PERIOD_MS);
        	        	lcd_clear();
        	printf("Te=%.2fC\t",temperature);
        	printf("total time=%.2lfs\n",total_time);
            printf("Bus Voltage: %.2f V\n", bus_voltage);
            printf("Current: %.2f mA\n", current);
                   printf("Power: %.2f mW\n", power);
                   printf("\n");
        } else {
            printf("Error reading SHT30 sensor: %d\n", ret);
        }

    }
}

#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include "unistd.h"
#include "driver/i2c.h"
#include "i2c_lcd.h"
#include <string.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "ina219.h"

#define UART_TX_PIN (1) // GPIO pin used for UART TX
#define UART_RX_PIN (3) // GPIO pin used for UART RX
#define UART_BAUD_RATE 9600      // Baud rate for UART communication
#define LED_GPIO_PIN 12           // GPIO pin connected to the LED

static const char *TAG = "i2c-simple-example";

char buffer[20];
float num=12.34;
char buffer1[20];
char buffer2[20];
char buffer3[20];

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port =I2C_NUM_0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}
static void uart_init(void){
	 // Configure UART parameters
	    uart_config_t uart_config = {
	        .baud_rate = UART_BAUD_RATE,
	        .data_bits = UART_DATA_8_BITS,
	        .parity = UART_PARITY_DISABLE,
	        .stop_bits = UART_STOP_BITS_1,
	        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	    };

	    // Configure UART driver
	    uart_param_config(UART_NUM_1, &uart_config);
	    uart_set_pin(UART_NUM_1, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	    uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0);

	    // Configure LED GPIO pin
	    gpio_config_t io_conf;
	    io_conf.intr_type = GPIO_INTR_DISABLE;
	    io_conf.mode = GPIO_MODE_OUTPUT;
	    io_conf.pin_bit_mask = (1ULL << LED_GPIO_PIN);
	    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	    gpio_config(&io_conf);

}
void app_main(void)
{
	ESP_ERROR_CHECK(i2c_master_init());
	    ESP_LOGI(TAG, "I2C initialized successfully");

	uart_init();
	lcd_init();
	    lcd_put_cur(0,0);

	    lcd_send_string("Welcome");
	    usleep(3000000);
	    lcd_clear();

	    TickType_t start_time, end_time;
double total_time;


	while (1) {
	        // Read data from UART
	    // Read data from UART
		 start_time = xTaskGetTickCount();
		uint8_t data;

	        int len = uart_read_bytes(UART_NUM_1, &data, 1, portMAX_DELAY);

	        float shunt_voltage = read_shunt_voltage();
	        float bus_voltage = read_bus_voltage();
	         float current = read_current();
	         printf("Bus Voltage: %.2f V\n", bus_voltage);
	                printf("Shunt Voltage: %.2f mV\n", shunt_voltage);
	                printf("Current: %.2f mA\n", current);
	        // Clear the buffer
	        // Check the received data
	        if(len > 0) {
	            // Check for termination character or buffer overflow
	            if (data == '\n' || strlen(buffer) >= sizeof(buffer)-1) {
	                // Display the received message
	            	end_time = xTaskGetTickCount();
	            		        total_time = (double)(end_time - start_time) / 1000.0+0.01;
	            		        //printf(" total time %f\n",total_time);
	                lcd_send_string(buffer);
	                lcd_put_cur(1,0);
	                lcd_send_string("Time=");

	                char time_str[10];
	                snprintf(time_str, sizeof(time_str), "%.5lf s", total_time); // Convert time to string
	                lcd_send_string(time_str);

	                vTaskDelay(2000 / portTICK_PERIOD_MS);
	                lcd_clear();
	                snprintf(buffer2, sizeof(buffer2), "%.2fV", bus_voltage);
	                snprintf(buffer3, sizeof(buffer3), "%.2fmA", current);
	                lcd_put_cur(0,0);
	                lcd_send_string("Volt=");
	                lcd_send_string(buffer2);
	                lcd_put_cur(1,0);
	                lcd_send_string("Cur=");
	                lcd_send_string(buffer3);
	                vTaskDelay(2000 / portTICK_PERIOD_MS);
	                lcd_clear();
	                memset(buffer, 0, sizeof(buffer)); // Clear the buffer


	            } else {
	                // Append the received character to the buffer
	                strncat(buffer, (char*)&data, 1);
	            }
	        }
	}


}

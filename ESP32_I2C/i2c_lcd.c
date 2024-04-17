#ifndef LCD_H
#define LCD_H
#include "i2c-lcd.h"
#include "stm32f1xx_hal.h"
extern I2C_HandleTypeDef hi2c2;
#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_clear()
{
lcd_send_cmd(0x80);
for(int i=0;i<70;i++){
	lcd_send_data(' ');
}
}

void lcd_put_cur(int row,int col)
{
	switch(row)
{
	case 0:
		col=0x80;
		break;
	case 1:
		col =0xC0;
		break;
}
	lcd_send_cmd(col);
}

void lcd_init(void)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}
void lcd_printfloat(float number, uint8_t decimalPlaces) {
  // Extract the integer part of the number
  int integerPart = (int)number;

  // Extract the decimal part of the number
  float decimalPart = number - integerPart;

  // Print the integer part
  LCD_PrintInt(integerPart);

  // Print the decimal point
  lcd_send_string(".");

  // Print the decimal part
  int multiplier = 1;
  for (int i = 0; i < decimalPlaces; ++i) {
    multiplier *= 10;
  }
  int roundedDecimalPart = (int)(decimalPart * multiplier + 0.5); // Round the decimal part
  LCD_PrintInt(roundedDecimalPart);
}
void LCD_PrintInt(int number) {
  char buffer[16]; // Assuming a maximum of 16 characters for the buffer
  sprintf(buffer, "%d", number); // Convert integer to string
 lcd_send_string(buffer); // Print the string on LCD
}
#endif

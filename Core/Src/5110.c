#include "5110.h"
#include "font.h"
#include "main.h"

//Define the LCD Operation function
void LCD5110_LCD_write_byte(unsigned char dat, unsigned char LCD5110_MOde);
void LCD5110_LCD_HAL_Delay_ms(unsigned int t);

//Define the hardware operation function
void LCD5110_SCK(unsigned char temp);
void LCD5110_DIN(unsigned char temp);
void LCD5110_CS(unsigned char temp);
void LCD5110_RST(unsigned char temp);
void LCD5110_DC(unsigned char temp);

/**
 * Initialize LCD module
 *
 * Input parameters : none
 * Return value		: none
 */
void LCD5110_init() {

	// Set pin initial state
	LCD5110_Led(0); //Turn back light off
	LCD5110_RST(0); //Set LCD reset = 0;
	LCD5110_DC(1); //Mode = command;
	LCD5110_DIN(1); //Set In at high level;
	LCD5110_SCK(1); //Set CLK high;
	LCD5110_CS(1); //Unselect chip;

	//Keep reset pin low for 10 ms
	HAL_Delay(10);
	//Release Reset Pin
	LCD5110_RST(1); //LCD_RST = 1;

	//Configure LCD module
	LCD5110_LCD_write_byte(0x21, LCD_COMMAND); //Extended instruction set selected
	LCD5110_LCD_write_byte(0xB7, LCD_COMMAND); //Set LCD voltage (defined by experimentation...)
	LCD5110_LCD_write_byte(0x14, LCD_COMMAND); //Set Bias for 1/48
	LCD5110_LCD_write_byte(0x06, LCD_COMMAND); //Set temperature control (TC2)
	LCD5110_LCD_write_byte(0x20, LCD_COMMAND); //Revert to standard instruction set
	LCD5110_clear(); //Clear display (still off)
	LCD5110_LCD_write_byte(0x0c, LCD_COMMAND); //Set display on in "normal" mode (not inversed)

}
/**
 * Write byte to the module.
 *
 * @param dat  	data to write
 * @param mode  0 if command, 1 if data
 *
 * @retval		None
 */
void LCD5110_LCD_write_byte(unsigned char dat, unsigned char mode) {
	unsigned char i;
	LCD5110_CS(0); //SPI_CS = 0;

	if (0 == mode)
		LCD5110_DC(0); //LCD_DC = 0;
	else
		LCD5110_DC(1); //LCD_DC = 1;

	for (i = 0; i < 8; i++) {
		LCD5110_DIN(dat & 0x80); //SPI_MO = dat & 0x80;
 		dat = dat << 1;
		LCD5110_SCK(0); //SPI_SCK = 0;
		LCD5110_SCK(1); //SPI_SCK = 1;
	}

	LCD5110_CS(1); //SPI_CS = 1;

}

/**
 * Write character to LCD at current position
 *
 * @param c: char to write
 * @retval None
 */
void LCD5110_write_char(unsigned char c) {
	unsigned char line;
	unsigned char ch = 0;

	c = c - 32;

	for (line = 0; line < 6; line++) {
		ch = font6_8[c][line];
		LCD5110_LCD_write_byte(ch, LCD_DATA);

	}
}

/**
 * Write character to LCD in inverse video at current location
 *
 * @param c: char to write
 * @retval None
 */
void LCD5110_write_char_inv(unsigned char c) {
	unsigned char line;
	unsigned char ch = 0;

	c = c - 32;

	for (line = 0; line < 6; line++) {
		ch = ~font6_8[c][line];
		LCD5110_LCD_write_byte(ch, LCD_DATA);

	}
}

/**
 * Write string to LCD at current position. String must be null terminated.
 *
 * @param s: string pointer
 * @retval None
 */
void LCD5110_write_string(char *s, int color) {
	unsigned char ch;
	while (*s != '\0') {
		ch = *s;
		if (color == 0)
			LCD5110_write_char(ch);
		else 
			if (color)
			LCD5110_write_char_inv(ch);
		s++;
	}
}

/**
 * Clear display. Write 0 in all memory location.
 *
 * @param None
 * @retval None
 */
void LCD5110_clear() {
	unsigned char i, j;
	for (i = 0; i < 6; i++)
		for (j = 0; j < 84; j++)
			LCD5110_LCD_write_byte(0, LCD_DATA);
}

/**
 * Set memory current location for characters (set coordinates).
 * Applies only for Fonts with a 6 pixels width.
 *
 * @param X: Column (range from 0 to 13)
 * @param Y: Row (range from 0 to 5)
 * @retval None
 *
 */
void LCD5110_set_XY(unsigned char Y, unsigned char X) {
	unsigned char x;
	x = 6 * X;

	LCD5110_LCD_write_byte(0x40 | Y, LCD_COMMAND);
	LCD5110_LCD_write_byte(0x80 | x, LCD_COMMAND);
}

/**
 * Write integer to LCD
 *
 * @param b: integer to write
 * @retval None
 */
void LCD5110_Write_Dec(unsigned int buffer, int amount_num, int color) {

	unsigned char datas[4];

	datas[0] = buffer / 1000;
	buffer = buffer - datas[0] * 1000;
	datas[1] = buffer / 100;
	buffer = buffer - datas[1] * 100;
	datas[2] = buffer / 10;
	buffer = buffer - datas[2] * 10;
	datas[3] = buffer;

	datas[0] += 48;
	datas[1] += 48;
	datas[2] += 48;
	datas[3] += 48;

	if (amount_num == 1)
	{
			if (color == 0)
			{
				LCD5110_write_char(datas[3]);

			} else 
			if (color == 1)
			{
				LCD5110_write_char_inv(datas[3]);
			}
	} else
	if (amount_num == 2)
	{
		if (color == 0)
		{
			LCD5110_write_char(datas[2]);
			LCD5110_write_char(datas[3]);
		} else 
		if (color == 1)
		{
			LCD5110_write_char_inv(datas[2]);
			LCD5110_write_char_inv(datas[3]);
		}
	}  else
	if (amount_num == 3)
	{
		if (color == 0)
		{
			LCD5110_write_char(datas[1]);
			LCD5110_write_char(datas[2]);
			LCD5110_write_char(datas[3]);
		} else 
		if (color == 1)
		{
			LCD5110_write_char_inv(datas[1]);
			LCD5110_write_char_inv(datas[2]);
			LCD5110_write_char_inv(datas[3]);
		}
	} else
	if (amount_num == 4)
	{
		if (color == 0)
		{
			LCD5110_write_char(datas[0]);
			LCD5110_write_char(datas[1]);
			LCD5110_write_char(datas[2]);
			LCD5110_write_char(datas[3]);
		} else 
		if (color == 1)
		{
			LCD5110_write_char_inv(datas[0]);
			LCD5110_write_char_inv(datas[1]);
			LCD5110_write_char_inv(datas[2]);
			LCD5110_write_char_inv(datas[3]);
		}
	} 
}

/**
 * Manage CS pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
void LCD5110_CS(unsigned char state) {
	if (state == 0)
		HAL_GPIO_WritePin(LCD5110__CS_GPIO_Port, LCD5110__CS_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LCD5110__CS_GPIO_Port, LCD5110__CS_Pin, GPIO_PIN_SET);
}

/**
 * Manage Reset pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
void LCD5110_RST(unsigned char state) {
	if (state == 0)
		HAL_GPIO_WritePin(LCD5110__RST_GPIO_Port, LCD5110__RST_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LCD5110__RST_GPIO_Port, LCD5110__RST_Pin, GPIO_PIN_SET);
}

/**
 * Manage DC pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
void LCD5110_DC(unsigned char state) {
	if (state == 0)
		HAL_GPIO_WritePin(LCD5110__DC_GPIO_Port, LCD5110__DC_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LCD5110__DC_GPIO_Port, LCD5110__DC_Pin, GPIO_PIN_SET);
}

/**
 * Manage DIN pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
void LCD5110_DIN(unsigned char state) {
	if (state == 0)
		HAL_GPIO_WritePin(LCD5110__DIN_GPIO_Port, LCD5110__DIN_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LCD5110__DIN_GPIO_Port, LCD5110__DIN_Pin, GPIO_PIN_SET);
}

/**
 * Manage CLK pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
void LCD5110_SCK(unsigned char state) {
	if (state == 0)
		HAL_GPIO_WritePin(LCD5110__SCK_GPIO_Port, LCD5110__SCK_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LCD5110__SCK_GPIO_Port, LCD5110__SCK_Pin, GPIO_PIN_SET);
}

/**
 * Manage LED pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
void LCD5110_Led(unsigned char state) {
	if (state == 0)
		HAL_GPIO_WritePin(LCD5110__LED_GPIO_Port, LCD5110__LED_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LCD5110__LED_GPIO_Port, LCD5110__LED_Pin, GPIO_PIN_SET);
}

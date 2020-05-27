

#include "HD44780.h"


void _LCD_OutNibble(unsigned char nibbleToWrite)
{

	if(nibbleToWrite & 0x01)
	LCD_DB4_PORT |= LCD_DB4;
	else
	LCD_DB4_PORT  &= ~LCD_DB4;

	if(nibbleToWrite & 0x02)
	LCD_DB5_PORT |= LCD_DB5;
	else
	LCD_DB5_PORT  &= ~LCD_DB5;

	if(nibbleToWrite & 0x04)
	LCD_DB6_PORT |= LCD_DB6;
	else
	LCD_DB6_PORT  &= ~LCD_DB6;

	if(nibbleToWrite & 0x08)
	LCD_DB7_PORT |= LCD_DB7;
	else
	LCD_DB7_PORT  &= ~LCD_DB7;
}

unsigned char _LCD_InNibble(void)
{
	unsigned char tmp = 0;

	if(LCD_DB4_PIN & LCD_DB4)
	tmp |= (1 << 0);
	if(LCD_DB5_PIN & LCD_DB5)
	tmp |= (1 << 1);
	if(LCD_DB6_PIN & LCD_DB6)
	tmp |= (1 << 2);
	if(LCD_DB7_PIN & LCD_DB7)
	tmp |= (1 << 3);
	return tmp;
}

void _LCD_Write(unsigned char dataToWrite)
{
	LCD_DB4_DIR |= LCD_DB4;
	LCD_DB5_DIR |= LCD_DB5;
	LCD_DB6_DIR |= LCD_DB6;
	LCD_DB7_DIR |= LCD_DB7;

	LCD_RW_PORT &= ~LCD_RW;
	LCD_E_PORT |= LCD_E;
	_LCD_OutNibble(dataToWrite >> 4);
	LCD_E_PORT &= ~LCD_E;
	LCD_E_PORT |= LCD_E;
	_LCD_OutNibble(dataToWrite);
	LCD_E_PORT &= ~LCD_E;
	while(LCD_ReadStatus()&0x80);
}


unsigned char _LCD_Read(void)
{
	unsigned char tmp = 0;
	LCD_DB4_DIR &= ~LCD_DB4;
	LCD_DB5_DIR &= ~LCD_DB5;
	LCD_DB6_DIR &= ~LCD_DB6;
	LCD_DB7_DIR &= ~LCD_DB7;

	LCD_RW_PORT |= LCD_RW;
	LCD_E_PORT |= LCD_E;
	tmp |= (_LCD_InNibble() << 4);
	LCD_E_PORT &= ~LCD_E;
	LCD_E_PORT |= LCD_E;
	tmp |= _LCD_InNibble();
	LCD_E_PORT &= ~LCD_E;
	return tmp;
}


void LCD_WriteCommand(unsigned char commandToWrite)
{
	LCD_RS_PORT &= ~LCD_RS;
	_LCD_Write(commandToWrite);
}


unsigned char LCD_ReadStatus(void)
{
	LCD_RS_PORT &= ~LCD_RS;
	return _LCD_Read();
}

void LCD_WriteData(unsigned char dataToWrite)
{
	LCD_RS_PORT |= LCD_RS;
	_LCD_Write(dataToWrite);
}

unsigned char LCD_ReadData(void)
{
	LCD_RS_PORT |= LCD_RS;
	return _LCD_Read();
}

void LCD_WriteText(char * text)
{
	while(*text)
	LCD_WriteData(*text++);
}

void LCD_GoTo(unsigned char x, unsigned char y)
{
	LCD_WriteCommand(HD44780_DDRAM_SET | (x + (0x40 * y)));
}

void LCD_Clear(void)
{
	LCD_WriteCommand(HD44780_CLEAR);
	_delay_ms(2);
}

void LCD_Home(void)
{
	LCD_WriteCommand(HD44780_HOME);
	_delay_ms(2);
}

void LCD_Initalize(void)
{
	unsigned char i;
	LCD_DB4_DIR |= LCD_DB4; 
	LCD_DB5_DIR |= LCD_DB5; 
	LCD_DB6_DIR |= LCD_DB6; 
	LCD_DB7_DIR |= LCD_DB7; 
	LCD_E_DIR 	|= LCD_E;  
	LCD_RS_DIR 	|= LCD_RS; 
	LCD_RW_DIR 	|= LCD_RW; 
	_delay_ms(15);
	LCD_RS_PORT &= ~LCD_RS; 
	LCD_E_PORT &= ~LCD_E;  
	LCD_RW_PORT &= ~LCD_RW;
	for(i = 0; i < 3; i++) 
	{
		LCD_E_PORT |= LCD_E; 
		_LCD_OutNibble(0x03);
		LCD_E_PORT &= ~LCD_E; 
		_delay_ms(5); 
	}

	LCD_E_PORT |= LCD_E; 
	_LCD_OutNibble(0x02);
	LCD_E_PORT &= ~LCD_E; 

	_delay_ms(1); 
	LCD_WriteCommand(HD44780_FUNCTION_SET | HD44780_FONT5x7 | HD44780_TWO_LINE | HD44780_4_BIT); 
	LCD_WriteCommand(HD44780_DISPLAY_ONOFF | HD44780_DISPLAY_OFF); 
	LCD_WriteCommand(HD44780_CLEAR); 
	LCD_WriteCommand(HD44780_ENTRY_MODE | HD44780_EM_SHIFT_CURSOR | HD44780_EM_INCREMENT);
	LCD_WriteCommand(HD44780_DISPLAY_ONOFF | HD44780_DISPLAY_ON | HD44780_CURSOR_OFF | HD44780_CURSOR_NOBLINK); 
}


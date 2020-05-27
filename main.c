/*
 * GccApplication1.c
 *
 * Created: 12.03.2020 23:06:29
 * Author : Test
 */ 

#define F_CPU 1000000UL
#define LED (1 << PB0)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#include "HD44780.h"
#include "i2cmaster.h"




int main(void)
{
		LCD_Initalize();

		int adres = 0xb4;         // Adres czujnika MLX90614
		int erc = 0;              // Zmienna przechowuj?ca warto?? PEC
		int daneH = 0;            // Drugi bajt danych
		int daneL = 0;            // Pierwszy bajt danych
		double tempnalsb = 0.02;  // Zmienna, przez któr? wymna?ana b?dzie warto?? cyfrowa
		double temperatura = 0;   // Zmienna przechowuj?ca temperatur?
		LCD_WriteText("test");
		 i2c_init();  
		_delay_ms(500);
		LCD_WriteText("dupa");
		     // Inicjalizacja magistrali I2C	
	
	//void displayTemperature(double )
		
		
	while (1)
	{
		
		
		i2c_start_wait(adres+I2C_WRITE);
		i2c_write(0x07);                     // Zapis warto?ci 0x07 (wybranie rejestru Tobj1)
		i2c_rep_start(adres+I2C_READ);

		// Ponowny start komunikacji I2C pod adresem do odczytu
		daneL = i2c_readAck();               // Odczyt pierwszego bajtu danych
		daneH = i2c_readAck();               // Odczyt drugiego bajtu danych
		erc = i2c_readNak();                 // Odczyt trzeciego (nieistotnego) bajtu danych
		i2c_stop();



		temperatura = (double)(((daneH & 0x007F) << 8) + daneL); // Utworzenie 16-sto bitowej zmiennej sk?adaj?cej si? z dwóch zmiennych jedno-bajtowych
		temperatura = temperatura*tempnalsb; // Na jeden bit przypada 0.02 K, wynikiem tej operacji jest temperatura w Kelvinach

		temperatura = temperatura - 273.15; // Konwersja na stopnie Celsiusza
		double test = 9.33;
		char *temp[sizeof(test)];
		memcpy(temp,&test,sizeof(test));
		LCD_Clear();
		
		char bb[7] = "xxxxxxx";
		itoa(temperatura*100,bb,10);
		LCD_WriteText(bb);
		_delay_ms(500);
	}
}

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


char min[7];
char max[7];
char average[7];


void convertDoubleToCharDisplay(double temperature, char *display){
	char displayStr[7] = "1234567"; 
	int total = temperature;
	int mantis = abs(temperature * 1000);
	mantis = mantis % 1000;
	char totalStr[] = "123" ;
	char mantisStr[] = "123" ;
	itoa(abs(total),totalStr,10);
	itoa(mantis,mantisStr,10);

	displayStr[0]= ' ';
	displayStr[1]= ' ';
	displayStr[2]= ' ';
	if(abs(total) >= 100){
		displayStr[1]=totalStr[0];
		displayStr[2]=totalStr[1];
		displayStr[3]=totalStr[2];
		if(total < 0)
		displayStr[0]= '-';
		
		
	}else if(abs(total) >= 10){
		displayStr[0] = ' ';
		displayStr[2]=totalStr[0];
		displayStr[3]=totalStr[1];
		if(abs(total) < 0)
			displayStr[1]= '-';
	}else{
		displayStr[0] = ' ';
		displayStr[0] = ' ';
		displayStr[3]=totalStr[0];
		if(abs(total) < 0)
			displayStr[2]= '-';
	}
		
	displayStr[4]= '.';
	displayStr[5]= mantisStr[0];
	displayStr[6]= mantisStr[1];
	
	strcpy(display,displayStr);
	return;
}


void displayTemperature(short int *change){
	//++(*change);

	

/*	
	if((*change) < 25)
	{
		LCD_GoTo(0,0);
		LCD_WriteText("MIN:");
		LCD_GoTo(4,0);
		LCD_WriteText(displayMIN);
	}
	else{
		LCD_GoTo(0,0);
		LCD_WriteText("MAX:");
		LCD_GoTo(4,0);
		LCD_WriteText(displayMAX);
	}
	if((*change) > 50)
		(*change) = 0;*/
	
	LCD_GoTo(0,1);
	LCD_WriteText("AVERAGE:");
	LCD_GoTo(8,1);
	LCD_WriteText("11");
	LCD_GoTo(15,1);
	LCD_WriteText("C");
	_delay_ms(100);
	
		return;
}

void getData(double Temperatures[21]){
	
		int adres = 0xb4;         // Adres czujnika MLX90614
		int erc = 0;              // Zmienna przechowuj?ca warto?? PEC
		int daneH = 0;            // Drugi bajt danych
		int daneL = 0;            // Pierwszy bajt danych
		double tempnalsb = 0.02;  // Zmienna, przez któr? wymna?ana b?dzie warto?? cyfrowa
		double temperature = 0;   // Zmienna przechowuj?ca temperatur?
		
	for(int index = 0; index < 21; ++index){
		temperature = 0;
		daneH = 0;   
		daneL = 0; 
		i2c_start_wait(adres+I2C_WRITE);
		i2c_write(0x07);                     // Zapis warto?ci 0x07 (wybranie rejestru Tobj1)
		i2c_rep_start(adres+I2C_READ);
		daneL = i2c_readAck();               // Odczyt pierwszego bajtu danych
		daneH = i2c_readAck();               // Odczyt drugiego bajtu danych
		erc = i2c_readNak();                 // Odczyt trzeciego (nieistotnego) bajtu danych
		i2c_stop();
		
		temperature = (double)(((daneH & 0x007F) << 8) + daneL); // Utworzenie 16-sto bitowej zmiennej sk?adaj?cej si? z dwóch zmiennych jedno-bajtowych
		temperature = temperature*tempnalsb; // Na jeden bit przypada 0.02 K, wynikiem tej operacji jest temperatura w Kelvinach

		temperature = temperature - 273.15; // Konwersja na stopnie Celsiusza
		Temperatures[index] = temperature;
		
	}
	LCD_Clear();
		return;
}

void dataProcesing(double Temperatures[21]){
	
	
	char displayMIN[7];
	char displayMAX[7];
	
	char displayAVE[7] = "XXXXXXX";
	convertDoubleToCharDisplay(23.54,displayAVE);
	/*convertDoubleToCharDisplay(temperature,displayMIN);
	convertDoubleToCharDisplay(temperature,displayMAX);*/
	//for(int index = 0; index < 7;++index){
		strcpy(average , displayAVE);

	//}
		return;
}

int main(void)
{
		
		LCD_Initalize();
		LCD_WriteText("INIT");
		_delay_ms(1000);
		i2c_init();  
		
		double Temperatures[21];
		
		DDRD = _BV(DDD7);
		PORTD = _BV(PD7);		short int change = 0;
		
		
	while (1)
	{
		getData(Temperatures);
		
		dataProcesing(Temperatures);
		
		displayTemperature(&change);

	}
}

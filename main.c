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
#include <math.h>

#include "HD44780.h"
#include "i2cmaster.h"


void convertDoubleToCharDisplay(double temperature,char display[7]){
	
	char displayStr[7] = "XXXXXXX";
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
}


void displayTemperature(char averageStr[7]){

	LCD_Clear();
	LCD_GoTo(0,1);
	LCD_WriteText("AVERAGE:");
	LCD_GoTo(8,1);
	LCD_WriteText(averageStr);
	LCD_GoTo(15,1);
	LCD_WriteText("C");
	_delay_ms(100);
}



void getData(double Temperatures[21]){
	
	int adress = 0xb4;          
	int dateH = 0;           
	int dataL = 0;            
	double tempnalsb = 0.02;  
	double temperature = 0;  
	
	for(int index = 0; index < 21; ++index){
		temperature = 0;
		dateH = 0;   
		dataL = 0; 
		i2c_start_wait(adress+I2C_WRITE);
		i2c_write(0x07);                     
		i2c_rep_start(adress+I2C_READ);
		dataL = i2c_readAck();           
		dateH = i2c_readAck();             
		i2c_readNak();                
		i2c_stop();
		
		temperature = (double)(((dateH & 0x007F) << 8) + dataL); 
		temperature = temperature*tempnalsb; 

		temperature = temperature - 273.15; 
		Temperatures[index] = temperature;	
	}
}


double approximation(double temp){

	return temp - (0.0038*temp*temp + -0.3371 * temp + 7.7529);
}
	
		
double approximation2(double temp){

	return 5.952*0.00001 * temp * temp	+1.027*temp - 1.634;

	
}
double approximation3(double temp){

	return -0.003561*temp * temp * temp+0.4113*temp * temp - 14.77*temp + 200.3;
}

double approximation4(double temp){

	return 13.35*exp(0.02713*temp);

}	
	
void dataProcesing(double Temperatures[21], char *average){
		
		double averageTMP = 0;
		for(int x = 0; x < 21; ++x){
			averageTMP += Temperatures[x];
		}
			
		averageTMP = averageTMP/20;
		averageTMP = approximation(averageTMP);
		convertDoubleToCharDisplay(averageTMP,average);

	 
}


void dataProcesing2(double Temperatures[21], char *average){
	
	double averageTMP = 0;
	for(int x = 0; x < 21; ++x){
		averageTMP += Temperatures[x];
	}
	
	averageTMP = averageTMP/20;
	if(averageTMP > 38.5)
		averageTMP = approximation2b(averageTMP);
		else
		averageTMP = approximation2a(averageTMP);
	convertDoubleToCharDisplay(averageTMP,average);

	
}

void dataProcesing3(double Temperatures[21], char *average){
	
	double averageTMP = 0;
	for(int x = 0; x < 21; ++x){
		averageTMP += Temperatures[x];
	}
	
	averageTMP = averageTMP/20;
	averageTMP = approximation3(averageTMP);
	convertDoubleToCharDisplay(averageTMP,average);

}

void dataProcesing4(double Temperatures[21], char *average){
	
	double averageTMP = 0;
	for(int x = 0; x < 21; ++x){
		averageTMP += Temperatures[x];
	}
	
	averageTMP = averageTMP/20;
	averageTMP = approximation4(averageTMP);
	convertDoubleToCharDisplay(averageTMP,average);

	
}

int main(void)
{
		
		LCD_Initalize();
		LCD_WriteText("INIT");
		i2c_init();  

		double Temperatures[21];
		
		DDRD = _BV(DDD7);
		PORTD = _BV(PD7);
		char average[7];
		char average2[7];
		char average3[7];
		char average4[7];
		while (1)
		{
			
			if(!(PIND & _BV(PIND7)))
			{
				getData(Temperatures);
				dataProcesing(Temperatures,average);
				dataProcesing2(Temperatures,average2);
				dataProcesing3(Temperatures,average3);
				dataProcesing4(Temperatures,average4);
				_delay_ms(20);
			}

		LCD_Clear();
		LCD_GoTo(0,0);
		LCD_WriteText(average);
		LCD_GoTo(9,0);
		LCD_WriteText(average2);
		LCD_GoTo(0,1);
		LCD_WriteText(average3);
		LCD_GoTo(9,1);
		LCD_WriteText(average4);
		_delay_ms(100);

		//	displayTemperature(average);
			_delay_ms(200);

		}
}

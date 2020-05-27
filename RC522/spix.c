/*
 * spi.c
 * 
 * Copyright 2013 Shimon <shimon@monistit.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */
#include "spix.h"

void spi_init()
{
	uint32_t baud = 115200;
	UBRR1 = 0;
	/* Setting the XCKn port pin as output, enables master
	mode. */
	DDRD |= (1<<SPI_SCK) | (1<<SPI_SS);
	PORTD |= (1<<SPI_SCK) | (1<<SPI_SS);


	// Set the resetPowerDownPin as digital output, do not reset or power down.
	DDR(READER_RESET_PORT) |= (1<<READER_RESET_PIN);

	/* Set MSPI mode of operation and SPI data mode 0. */
	// UCSR1C = (1<<UMSEL11)|(1<<UMSEL10) |(0<<UCPHA1)|(0<<UCPOL1);
	UCSR1C = (1<<UMSEL11)|(1<<UMSEL10) |(0<<UCSZ10)|(0<<UCPOL1);
	/* Enable receiver and transmitter. */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	/* Set baud rate. */
	/* IMPORTANT: The Baud Rate must be set after the
	transmitter is enabled */
	if (baud)
		UBRR1 = (F_CPU/(2*baud)) - 1;
	else
		UBRR1 = 0x0000;
}

void spi_off() {
	PORTD &= ~(1<<SPI_SCK);
	PORTD &= ~(1<<SPI_SS);
	DDRD &= ~(1<<SPI_SCK);
	DDRD &= ~(1<<SPI_SS);

	PORT(READER_RESET_PORT) &= ~(1<<READER_RESET_PIN);
	DDR(READER_RESET_PORT) &= ~(1<<READER_RESET_PIN);
}

uint8_t spi_transmit(uint8_t data)
{
	loop_until_bit_is_set(UCSR1A,UDRE1);

	UDR1 = data;

	loop_until_bit_is_set(UCSR1A,RXC1);

	return UDR1;
}


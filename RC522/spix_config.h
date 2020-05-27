/*
SPI MASTER AVR CONFIG
*/
#ifndef SPIX_CONFIG_H
#define SPIX_CONFIG_H

#include "../lib/common.h"

#define SPI_DDR		DDRD
#define SPI_PORT	PORTD
#define SPI_PIN		PIND
#define SPI_SS		PORTD5
#define SPI_MOSI	PORTD3
#define SPI_MISO	PORTD2
#define SPI_SCK		PORTD4

#define READER_RESET_PORT D
#define READER_RESET_PIN 6

#define ENABLE_CHIP() (SPI_PORT &= (~(1<<SPI_SS)))
#define DISABLE_CHIP() (SPI_PORT |= (1<<SPI_SS))

#define IS_RESET() (bit_is_clear(PORT(READER_RESET_PORT), READER_RESET_PIN))
#define DISABLE_RESET() (PORT(READER_RESET_PORT) |= (1<<READER_RESET_PIN))

#endif

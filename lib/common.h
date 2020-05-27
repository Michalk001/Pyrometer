/*
 * common.h
 *
 *  Created on: 8 cze 2015
 *      Author: g.ostapiuk
 */

#ifndef LIB_COMMON_H_
#define LIB_COMMON_H_

#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>

#define DEBUG_UART 0

#define PORT(x) XPORT(x)
#define XPORT(x) (PORT##x)

#define PIN(x) XPIN(x)
#define XPIN(x) (PIN##x)

#define DDR(x) XDDR(x)
#define XDDR(x) (DDR##x)

typedef enum { FALSE = 0, TRUE = 1 } BOOL;

//typedef struct {
//	uint16_t	year;	/* 2000..2099 */
//	uint8_t	month;	/* 1..12 */
//	uint8_t	day;	/* 1.. 31 */
//	//BYTE	wday;	/* 1..7 */
//	uint8_t	hour;	/* 0..23 */
//	uint8_t	min;	/* 0..59 */
//	uint8_t	sec;	/* 0..59 */
//} RTC;

typedef struct {
	uint8_t *cid;
	uint64_t sum;
	uint32_t last_read_date;
	//RTC *last_read_date;
} CARD;

typedef struct {
	//RTC rtc;
	long count;
} POSITION;

uint16_t byte_array_to_uint_16(uint8_t *array);

uint32_t byte_array_to_uint_32(uint8_t *array);

uint64_t byte_array_to_uint_64(uint8_t *array);

void uint_16_to_byte_array(uint8_t *array, uint16_t value);

void uint_32_to_byte_array(uint8_t *array, uint32_t value);

void uint_64_to_byte_array(uint8_t *array, uint64_t value);

uint32_t get_date_time(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec);

uint8_t get_year(uint32_t *rtc);

uint8_t get_month(uint32_t *rtc);

uint8_t get_day(uint32_t *rtc);

uint8_t get_hour(uint32_t *rtc);

uint8_t get_min(uint32_t *rtc);

uint8_t get_sec(uint32_t *rtc);

BOOL check_if_in_time(uint32_t *from_rtc, uint32_t *to_rtc, uint32_t *seconds, uint16_t *seconds_to_next_read);

#endif /* LIB_COMMON_H_ */

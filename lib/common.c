/*
 * common.c
 *
 *  Created on: 8 cze 2015
 *      Author: g.ostapiuk
 */

#include "common.h"

#define SECONDSPERDAY 86400UL

const uint16_t days_to_month_365[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 };
const uint16_t days_to_month_366[] = { 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366 };

uint16_t byte_array_to_uint_16(uint8_t *array) {
	uint16_t out_value;
	out_value = *(array + 1) | ((uint16_t)*array << 8);
	return out_value;
}

uint32_t byte_array_to_uint_32(uint8_t *array) {
	uint32_t out_value;
	out_value = *(array + 3)
			| ((uint32_t)*(array + 2) << 8)
			| ((uint32_t)*(array + 1) << 16)
			| ((uint32_t)*(array + 0) << 24);
	return out_value;
}

uint64_t byte_array_to_uint_64(uint8_t *array) {
	uint64_t out_value;
	out_value = *(array + 7)
			| ((uint64_t)*(array + 6) << 8)
			| ((uint64_t)*(array + 5) << 16)
			| ((uint64_t)*(array + 4) << 24)
			| ((uint64_t)*(array + 3) << 32)
			| ((uint64_t)*(array + 2) << 40)
			| ((uint64_t)*(array + 1) << 48)
			| ((uint64_t)*(array + 0) << 56);
	return out_value;
}

void uint_16_to_byte_array(uint8_t *array, uint16_t value) {
	*(array + 1) = value & 0xff;
	*(array + 0) = (value >> 8) & 0xff;
}

void uint_32_to_byte_array(uint8_t *array, uint32_t value) {
	*(array + 3) = value & 0xff;
	*(array + 2) = (value >> 8) & 0xff;
	*(array + 1) = (value >> 16) & 0xff;
	*(array + 0) = (value >> 24) & 0xff;
}

void uint_64_to_byte_array(uint8_t *array, uint64_t value) {
	*(array + 7) = value & 0xff;
	*(array + 6) = (value >> 8) & 0xff;
	*(array + 5) = (value >> 16) & 0xff;
	*(array + 4) = (value >> 24) & 0xff;
	*(array + 3) = (value >> 32) & 0xff;
	*(array + 2) = (value >> 40) & 0xff;
	*(array + 1) = (value >> 48) & 0xff;
	*(array + 0) = (value >> 56) & 0xff;
}

uint32_t get_date_time(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
	uint32_t time = (((uint32_t)year) << 25) |
			   (((uint32_t)month) << 21) |
			   (((uint32_t)day) << 16) |
			   (((uint32_t)hour) << 11) |
			   (((uint32_t)min) << 5) |
			   (((uint32_t)sec) >> 1);
	return time;
}

uint8_t get_year(uint32_t *rtc) {
	return (*rtc >> 25);
}

uint8_t get_month(uint32_t *rtc) {
	return *rtc >> 21 & 0x0f;
}

uint8_t get_day(uint32_t *rtc) {
	return *rtc >> 16 & 0x1f;
}

uint8_t get_hour(uint32_t *rtc) {
	return *rtc >> 11 & 0x1f;
}

uint8_t get_min(uint32_t *rtc) {
	return *rtc >> 5 & 0x3f;
}

uint8_t get_sec(uint32_t *rtc) {
	return *rtc << 1 & 0x3f;
}

BOOL is_leap_year(uint16_t year) {
	return year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
}

uint32_t get_time_seconds(uint8_t hour, uint8_t min, uint8_t sec) {
	uint32_t total_seconds = (uint32_t)hour * 3600UL + (uint32_t)min * 60UL + (uint32_t)sec;
	return total_seconds;
}

uint64_t get_date_seconds(uint16_t year, uint8_t month, uint8_t day) {
	const uint16_t *days;
	days = is_leap_year(year) ? days_to_month_366 : days_to_month_365;
	uint16_t y = year - 1;
	uint64_t n = (uint32_t)(y * 365UL) + (uint32_t)(y / 4UL) - (uint32_t)(y / 100UL) + (uint32_t)(y / 400UL) + (uint32_t)days[month - 1] + (uint32_t)day - 1UL;
	return n * SECONDSPERDAY;
}

uint64_t get_total_seconds_from_datetime(uint32_t *rtc) {
	uint16_t year = get_year(rtc) + 2000;
	uint8_t month = get_month(rtc);
	uint8_t day = get_day(rtc);
	uint8_t hour = get_hour(rtc);
	uint8_t min = get_min(rtc);
	uint8_t sec = get_sec(rtc);

	uint64_t date_seconds = get_date_seconds(year, month, day);
	uint32_t time_seconds = get_time_seconds(hour, min, sec);

	return date_seconds + time_seconds;
}

BOOL check_if_in_time(uint32_t *from_rtc, uint32_t *to_rtc, uint32_t *seconds, uint16_t *seconds_to_next_read) {
	*seconds_to_next_read = 0;
	if (*from_rtc == 0) {
		return FALSE;
	}

	uint64_t from_total_seconds = get_total_seconds_from_datetime(from_rtc);
	uint64_t to_total_seconds = get_total_seconds_from_datetime(to_rtc);
	uint64_t total_seconds_passed = to_total_seconds - from_total_seconds;

	if ((int64_t)(total_seconds_passed - *seconds) >= 0) {
		return FALSE;
	}

	*seconds_to_next_read = *seconds - total_seconds_passed;
	return TRUE;
}

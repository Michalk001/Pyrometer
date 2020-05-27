/*
 * reader.c
 *
 *  Created on: 29 maj 2015
 *      Author: g.ostapiuk
 */

#include <util/delay.h>

#include "reader.h"
#include "spix.h"

#define RETURNFREE(result, obj1, obj2) free(obj1); free(obj2); return result;

#define SECTOR_LEN 48
#define BLOCK_LEN 16
#define BUFFER_LEN 18
#define SECTOR_COUNT 4

#define READER_PORT D
#define READER_ON_OFF_PIN 7

void reader_init() {
	DDR(READER_PORT) |= (1<<READER_ON_OFF_PIN);
	PORT(READER_PORT) |= (1<<READER_ON_OFF_PIN);
	_delay_ms(5);

	PCD_Init();
}

void reader_off() {
	PCD_Off();
	PORT(READER_PORT) &= ~(1<<READER_ON_OFF_PIN);
	DDR(READER_PORT) &= ~(1<<READER_ON_OFF_PIN);
}

uint8_t reader_read_block_by_addr(uint8_t *block_data, uint8_t block_addr, Uid *card_id, MIFARE_Key *key) {
	uint8_t block_count = BUFFER_LEN * sizeof(uint8_t);

	uint8_t first_block = block_addr - block_addr % 4;

	uint8_t status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_B, first_block, key, card_id);
	if (status != STATUS_OK) {
		return status;
	}

	status = MIFARE_Read(block_addr, block_data, &block_count);
	if (status != STATUS_OK) {
		return status;
	}

	return status;
}

uint8_t reader_read_block(uint8_t *block_data, uint8_t sector, uint8_t block_nr, Uid *card_id, MIFARE_Key *key) {
	uint8_t block_addr = sector * 4 + block_nr;
	return reader_read_block_by_addr(block_data, block_addr, card_id, key);
}

uint8_t reader_write_block(uint8_t *block_data, uint8_t sector, uint8_t block_nr, Uid *card_id, MIFARE_Key *key) {
	uint8_t block_addr = sector * 4 + block_nr;
	uint8_t block_count = BLOCK_LEN * sizeof(uint8_t);

	uint8_t status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_B, block_addr, key, card_id);
	if (status != STATUS_OK) {
		return status;
	}

	status = MIFARE_Write(block_addr, block_data, block_count);
	if (status != STATUS_OK) {
			return status;
	}

	return status;
}

BOOL reader_init_reading_card(Uid *uid) {
	if (!PICC_IsNewCardPresent()) {
		return FALSE;
	}

	if (PICC_ReadCardSerial(uid)) {
		return TRUE;
	}

	return FALSE;
}

void reader_clean_card() {
	PICC_HaltA();
	PCD_StopCrypto1();
}

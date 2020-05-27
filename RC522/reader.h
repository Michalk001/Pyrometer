/*
 * reader.h
 *
 *  Created on: 29 maj 2015
 *      Author: g.ostapiuk
 */

#ifndef MFRC522_READER_H_
#define MFRC522_READER_H_

#include "mfrc522.h"
#include "../lib/common.h"

void reader_init();

void reader_off();

BOOL reader_init_reading_card(Uid *uid);

void reader_clean_card();

uint8_t reader_read_block_by_addr(uint8_t *block_data, uint8_t block_addr, Uid *card_id, MIFARE_Key *key);

uint8_t reader_read_block(uint8_t *block_data, uint8_t sector, uint8_t block_nr, Uid *card_id, MIFARE_Key *key);

uint8_t reader_write_block(uint8_t *block_data, uint8_t sector, uint8_t block_nr, Uid *card_id, MIFARE_Key *key);

#endif /* MFRC522_READER_H_ */

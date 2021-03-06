/*
 * zionEeprom.h
 *
 *  Created on: Feb 23, 2021
 *      Author: auphilli
 */

#ifndef INC_ZIONEEPROM_H_
#define INC_ZIONEEPROM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c4;


#define SOC_ADDRESS			0x53 << 1
#define ASIC_ADDRESS		0x54 << 1
#define DISPLAY_ADDRESS		0x55 << 1
#define EEPROM_I2C			hi2c4
#define EEPROM_SIZE_KBIT	64

#define SPARE_I2C 			hi2c2
#define SPARE_ADDRESS		0x53 << 1
#define EEPROM_SIZE_KBIT	64

int * zionEEPROMPresence();
void writeDataToEEPROM(uint8_t * data, uint8_t chipAddress, uint16_t memoryAddress, int len, uint16_t timeout);
void readDataFromEEPROM(uint8_t * data, uint8_t chipAddress, uint16_t memoryAddress, int len, uint16_t timeout);
void clearEEPROM(uint8_t chipAddress);
void writeZionBinaries();
int * parseZionEEPROM(uint8_t chipAddress);

#if 1
void writeDataToSpareEEPROM(uint8_t * data, uint8_t chipAddress, uint16_t memoryAddress, int len, uint16_t timeout);
void readDataFromSpareEEPROM(uint8_t * data, uint8_t chipAddress, uint16_t memoryAddress, int len, uint16_t timeout);
#endif

#if 1
static uint8_t zionEEPROMTrident[2220] = {
			// Offset 0x00000000 to 0x00002219
			0x5A, 0x45, 0x46, 0x01, 0x01, 0x01, 0x00, 0x00, 0x4E, 0x00, 0x00, 0x00,
			0x98, 0x08, 0x00, 0x00, 0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x43, 0x69, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
			0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x48, 0x42, 0x7A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x42, 0x74, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41,
			0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x48, 0x42, 0x6A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x71, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
			0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xA0, 0x41, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x85, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
			0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x48, 0x43, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x46, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
			0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xC8, 0x42, 0x6B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x43, 0x8D, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41,
			0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x48, 0x42, 0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0x43, 0x9A, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x43,
			0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x48, 0x43, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0x43, 0x87, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x42,
			0x8F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x48, 0x42, 0x95, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x2D, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x42,
			0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x48, 0x43, 0x9D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x43, 0x8E, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x43,
			0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xA0, 0x41, 0x75, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41, 0x73, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41,
			0x7B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x20, 0x41, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x40, 0x6E, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x43,
			0x96, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xA0, 0x40, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x93, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x42,
			0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x48, 0x43, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41, 0x6D, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41,
			0x91, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xC8, 0x42, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41, 0x83, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41,
			0x8B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x20, 0x41, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x42, 0x98, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x42,
			0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xA0, 0x41, 0x8A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x44, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
			0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x48, 0x42, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x0C, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x42,
			0x6F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xA0, 0x41, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41, 0x88, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41,
			0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xA0, 0x41, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41, 0x17, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41,
			0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x48, 0x42, 0x4E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x2E, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x43,
			0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x48, 0x43, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0x43, 0x1E, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x43,
			0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xC0, 0x40, 0x0A, 0xD7, 0x23, 0x3C, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x48, 0x43, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x6C, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x42,
			0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x48, 0x43, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x7E, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
			0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x48, 0x42, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x42, 0x2F, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
			0x8C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x20, 0x41, 0x97, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41, 0x84, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x40,
			0x0A, 0xD7, 0x23, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x43,
			0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0x23, 0x3C, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x48, 0x43, 0x5A, 0x45, 0x46, 0x01, 0x01, 0x04, 0x00, 0x00,
			0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x01
};
#endif

#if 1
static uint8_t zionEEPROMToga[1408] = {
		// Offset 0x00000000 to 0x00001407
		0x5A, 0x45, 0x46, 0x01, 0x01, 0x01, 0x00, 0x00, 0x31, 0x00, 0x00, 0x00,
		0x6C, 0x05, 0x00, 0x00, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x42, 0x2A, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41,
		0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x20, 0x41, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x43, 0x0F, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0x43,
		0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xFA, 0x43, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x42, 0x2C, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x42,
		0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xA0, 0x41, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x42, 0x0D, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41,
		0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x20, 0x41, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41, 0x21, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41,
		0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x20, 0x41, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41, 0x57, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0x23, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41,
		0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xA0, 0x41, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41, 0x30, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41,
		0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x40, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x20, 0x41, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41, 0x52, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x40,
		0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x20, 0x41, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41, 0x42, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
		0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x40, 0x3A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x40, 0x39, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x40,
		0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x40, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x20, 0x41, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41, 0x0E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41,
		0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x20, 0x41, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x36, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41,
		0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xA0, 0x41, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x5A, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41,
		0x4B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xA0, 0x41, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41, 0x58, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41,
		0x4A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x20, 0x41, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x37, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x40,
		0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x20, 0x41, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x42, 0x3D, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
		0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xC8, 0x42, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x42, 0x5A, 0x45, 0x46, 0x01,
		0x01, 0x04, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x01, 0x00, 0x01, 0x01
};
#endif
#if 1
static uint8_t zionEEPROMKanu[792] = {
		// Offset 0x00000000 to 0x00000791
		0x5A, 0x45, 0x46, 0x01, 0x01, 0x01, 0x00, 0x00, 0x1B, 0x00, 0x00, 0x00,
		0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41, 0x01, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
		0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x40, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41, 0x05, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
		0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x40, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x40, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x0A, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
		0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41, 0x11, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41,
		0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x20, 0x41, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x1A, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41,
		0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x20, 0x41, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41, 0x1D, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x43,
		0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x40, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x45, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
		0x4C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x48, 0x42, 0x4D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41, 0x5B, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
		0x5C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x40, 0x5E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41, 0x5F, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
		0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x41,
		0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x40, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x40, 0x5A, 0x45, 0x46, 0x01, 0x01, 0x04, 0x00, 0x00,
		0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x01
	};
#endif
#if 1
static uint8_t zionEEPROMCarabao[464] = {
	// Offset 0x00000000 to 0x00000463
	0x5A, 0x45, 0x46, 0x01, 0x01, 0x01, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x81, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
	0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
	0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x48, 0x42, 0x83, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x84, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
	0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
	0x85, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x48, 0x42, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x87, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
	0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
	0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x48, 0x42, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x8A, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
	0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
	0x8B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x48, 0x42, 0x8C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42, 0x8D, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
	0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42,
	0x8E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x48, 0x42, 0x8F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x0A, 0xD7, 0xA3, 0x3B,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x42
};
#endif

#endif /* INC_ZIONEEPROM_H_ */

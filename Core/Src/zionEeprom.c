/*
 * zionEeprom.c
 *
 *  Created on: Feb 23, 2021
 *      Author: auphilli
 */

#include "zionEeprom.h"

int * zionEEPROMPresence(){
	static int eepromPresent[3];
	memset(eepromPresent, 0x00, sizeof(eepromPresent));
	if(HAL_I2C_IsDeviceReady(&EEPROM_I2C, SOC_ADDRESS, 2, 100)== HAL_OK){
		eepromPresent[0]=1;
	}
	if(HAL_I2C_IsDeviceReady(&EEPROM_I2C, ASIC_ADDRESS, 2, 100)== HAL_OK){
		eepromPresent[1]=1;
	}
	if(HAL_I2C_IsDeviceReady(&EEPROM_I2C, DISPLAY_ADDRESS, 2, 100)== HAL_OK){
		eepromPresent[2]=1;
	}
	return eepromPresent;
}

void writeDataToEEPROM(uint8_t * data, uint8_t chipAddress, uint16_t memoryAddress, int len,uint16_t timeout){
	int bytes = 0;
	int writeSize=32;
	while(bytes < len){
		if((bytes+32)>len){
			writeSize=len-bytes;
		}
		HAL_I2C_Mem_Write(&EEPROM_I2C,chipAddress,memoryAddress+bytes, I2C_MEMADD_SIZE_16BIT,data+bytes,writeSize,timeout);
		HAL_Delay(10);
		bytes+=32;
	}

}
void readDataFromEEPROM(uint8_t * data, uint8_t chipAddress, uint16_t memoryAddress, int len,uint16_t timeout){
	HAL_I2C_Mem_Read(&EEPROM_I2C,chipAddress,memoryAddress, I2C_MEMADD_SIZE_16BIT,data,len,timeout);
}
void clearEEPROM(uint8_t chipAddress){
	  const uint8_t eraseData[32] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF\
	    , 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	  uint32_t bytes = 0;
	  while ( bytes < (EEPROM_SIZE_KBIT * 128))
	  {
	    writeDataToEEPROM((uint8_t*)eraseData, chipAddress,  bytes,sizeof(eraseData), 100);
	    bytes += sizeof(eraseData);
	  }
}

void writeZionBinaries(){
	  int* zionEEPROM;
	  zionEEPROM = zionEEPROMPresence();
 	  clearEEPROM(SOC_ADDRESS);
 	  clearEEPROM(ASIC_ADDRESS);
 	  clearEEPROM(DISPLAY_ADDRESS);
	  if(*zionEEPROM){
		  writeDataToEEPROM((uint8_t*)zionEEPROMTrident,SOC_ADDRESS,0x0,sizeof(zionEEPROMTrident), 100);
	  }
	  if(*(zionEEPROM+1)){
		 writeDataToEEPROM((uint8_t*)zionEEPROMToga,ASIC_ADDRESS,0x0,sizeof(zionEEPROMToga), 100);
	  }
	  if(*(zionEEPROM+2)){
		 writeDataToEEPROM((uint8_t*)zionEEPROMKanu,DISPLAY_ADDRESS,0x0,sizeof(zionEEPROMKanu), 500);

	  }
}

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
	int writeSize=32; //32 bytes of data write are allowed per Page Write command
	while(bytes < len){
		if((bytes+32)>len){
			writeSize=len-bytes; //partial page writes are allowed.
		}
		HAL_I2C_Mem_Write(&EEPROM_I2C,chipAddress,memoryAddress+bytes, I2C_MEMADD_SIZE_16BIT,data+bytes,writeSize,timeout);
		HAL_Delay(10); //give delay for EEPROM to write data to memory
		bytes+=32;
	}

}
//reads from the EEPROM can be continuous
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
	  if(*zionEEPROM){ // only do the actions if the SOC EEPROM is detected
	 	  clearEEPROM(SOC_ADDRESS);
		  writeDataToEEPROM((uint8_t*)zionEEPROMTrident,SOC_ADDRESS,0x0,sizeof(zionEEPROMTrident), 100);
	  }
	  if(*(zionEEPROM+1)){ // only do the actions if the ASIC EEPROM is detected
	 	  clearEEPROM(ASIC_ADDRESS);
		 writeDataToEEPROM((uint8_t*)zionEEPROMToga,ASIC_ADDRESS,0x0,sizeof(zionEEPROMToga), 100);
	  }
	  if(*(zionEEPROM+2)){ //only do the actions if the DISPLAY EEPROM is detected
	 	 clearEEPROM(DISPLAY_ADDRESS);
		 writeDataToEEPROM((uint8_t*)zionEEPROMKanu,DISPLAY_ADDRESS,0x0,sizeof(zionEEPROMKanu), 500);

	  }
}

int * parseZionEEPROM(uint8_t chipAddress){
	int foundTheEnd = 0;
	int x=0;
	int zeroWasFF=0;
	int outOfSyncFF=0;
	int previousByteOfFF[6];
	int index=0;
	int indexSubtractor = 0;
	uint8_t letsParseSomeBytes[100];
	uint8_t pastParsedBytes[100];
	static int deviceHeaderBytes[6];

	memset(previousByteOfFF,0,sizeof(previousByteOfFF));
	memset(pastParsedBytes,0,sizeof(pastParsedBytes));
	memset(letsParseSomeBytes,0,sizeof(letsParseSomeBytes));
	readDataFromEEPROM((uint8_t*)letsParseSomeBytes,chipAddress,0x00,sizeof(letsParseSomeBytes),100);
	while(!foundTheEnd){
		if(!(index%100) & (index>0)){
			for(x=0;x<100;x++){
				pastParsedBytes[x] = letsParseSomeBytes[x];
			}
			readDataFromEEPROM((uint8_t*)letsParseSomeBytes,chipAddress,index,sizeof(letsParseSomeBytes),100);
			indexSubtractor+=100;
		}
		//if((letsParseSomeBytes[index] == 0xff)){
		if((letsParseSomeBytes[index-indexSubtractor] == 0xff)){
			if((previousByteOfFF[0] == 0) & (!zeroWasFF)){
				previousByteOfFF[0] = index;
				if(index==0){
					zeroWasFF=1;
				}
			}
			else{
				for(x=0; x<5; x++){
					if(previousByteOfFF[x] == index-1){
						previousByteOfFF[x+1] = index;
						if((x+1)==5){
							foundTheEnd=1;
						}
						outOfSyncFF=0;
						break;
					}
					else{
						outOfSyncFF=1;
					}
				}
				if(outOfSyncFF){
					memset(previousByteOfFF,0,sizeof(previousByteOfFF));
					zeroWasFF=0;
					previousByteOfFF[0] = index;
					outOfSyncFF=0;
				}
			}
		}
		index++;
	}
	int counter=0;
	if(previousByteOfFF[0]>6){
		int remainder = previousByteOfFF[0]%100;
		if(remainder >4){
			for(x=0;x<5;x++){
				//deviceHeaderBytes[x] = letsParseSomeBytes[previousByteOfFF[0]-5+x];
				deviceHeaderBytes[x] = letsParseSomeBytes[remainder-5+x];
			}
		}
		else{
			for(x=100 -5 +remainder;x<100;x++){
				deviceHeaderBytes[counter] = pastParsedBytes[x];
				counter++;
			}
			for(x=0;x<remainder;x++){
				deviceHeaderBytes[counter] = letsParseSomeBytes[x];
				counter++;
			}
		}
	}
	else{
		memset(deviceHeaderBytes,0xff,sizeof(deviceHeaderBytes));
	}
	deviceHeaderBytes[5] = previousByteOfFF[0];
	return deviceHeaderBytes;
}

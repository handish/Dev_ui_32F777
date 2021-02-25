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
		  writeDataToEEPROM((uint8_t*)zionEEPROMCarabao,SOC_ADDRESS,0x0,sizeof(zionEEPROMCarabao), 100);
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
	int size = 100;
	int counter=6;
	int zeroWas5A=0;
	int validHeader=0;
	int previousByteOfHeader[20];
	int index=0;
	int indexSubtractor = 0;
	uint8_t letsParseSomeBytes[size];
	uint8_t pastParsedBytes[size];
	static int deviceHeaderBytes[5];

	memset(previousByteOfHeader,0,sizeof(previousByteOfHeader));
	memset(pastParsedBytes,0,sizeof(pastParsedBytes));
	memset(letsParseSomeBytes,0,sizeof(letsParseSomeBytes));
	readDataFromEEPROM((uint8_t*)letsParseSomeBytes,chipAddress,0x00,sizeof(letsParseSomeBytes),100);
	//if the eeprom is uninitialized and/or improperly formated, just end.
	if(letsParseSomeBytes[0] == 0xff){
		foundTheEnd=1;
		previousByteOfHeader[0]=index;
	}
	while(!foundTheEnd){
		//if the eeprom is uninitialized and/or improperly formated, just end.
		//every time we reach the end of our data, store it in the past buffer and get more!
		if(!(index%size) & (index>0)){
			for(x=0;x<size;x++){
				pastParsedBytes[x] = letsParseSomeBytes[x];
			}
			readDataFromEEPROM((uint8_t*)letsParseSomeBytes,chipAddress,index,sizeof(letsParseSomeBytes),100);
			indexSubtractor+=size;
		}
		//if 5 consecutive bits in a read operation are 0xff, time to give up finding the legit header
		if((letsParseSomeBytes[0] == 0xff) && (letsParseSomeBytes[1] == 0xff) && (letsParseSomeBytes[2] == 0xff) && (letsParseSomeBytes[3] == 0xff) && (letsParseSomeBytes[5] == 0xff)){
			previousByteOfHeader[0]=-1;
			foundTheEnd=1;
		}
		//if((letsParseSomeBytes[index] == 0xff)){
		if(((letsParseSomeBytes[index-indexSubtractor] == 0x5a))|| ((letsParseSomeBytes[previousByteOfHeader[0]%size] == 0x5a) && (index < previousByteOfHeader[0]+20)) || ((pastParsedBytes[previousByteOfHeader[0]%size] == 0x5a) && (index < previousByteOfHeader[0]+20))){
			if((previousByteOfHeader[0] == 0) & (!zeroWas5A) & ((letsParseSomeBytes[index-indexSubtractor] == 0x5a))){
				previousByteOfHeader[0] = index;
				if(index==0){
					zeroWas5A=1;
				}
			}
			//if validHeader was set, we just need the next 14 bytes of data
			else if(validHeader){
				previousByteOfHeader[counter]=index;
				if(counter==19){
					foundTheEnd=1;
				}
				counter++;
			}
			else{
				if((letsParseSomeBytes[index-indexSubtractor] == 0x45)){
					previousByteOfHeader[1] = index;
				}
				else if (((letsParseSomeBytes[index-indexSubtractor] == 0x46)) && (previousByteOfHeader[1] == index -1)){
					previousByteOfHeader[2] = index;
				}
				else if (((letsParseSomeBytes[index-indexSubtractor] == 0x01)) && (previousByteOfHeader[2] == index -1)){
					previousByteOfHeader[3] = index;
				}
				else if (((letsParseSomeBytes[index-indexSubtractor] == 0x01)) && (previousByteOfHeader[3] == index -1)){
					previousByteOfHeader[4] = index;
				}
				else if (((letsParseSomeBytes[index-indexSubtractor] == 0x04)) && (previousByteOfHeader[4] == index -1)){
					previousByteOfHeader[5] = index;
					validHeader=1;
				}
				//if things didn't look good, erase it!
				else if (!validHeader){
					memset(previousByteOfHeader,0,sizeof(previousByteOfHeader));
					zeroWas5A=0;
				}

			}
		}
		index++;
	}
	//if the eemprom is initialized
	if(previousByteOfHeader[0]>6){
		//figure out on which index our data started
		int remainder = previousByteOfHeader[16]%size;
		//if some of our data is split between past read and present read
		if(remainder >((size-1)-4)){
			//amount of bytes in the previous read
			int bytesInPreviousRead = (size-1) - remainder;
			//grab those bytes and store them in the buffer
			for(x=0;x<bytesInPreviousRead;x++){
				//deviceHeaderBytes[x] = letsParseSomeBytes[previousByteOfFF[0]-5+x];
				deviceHeaderBytes[x] = pastParsedBytes[previousByteOfHeader[16+x]%size];
			}
			//grab the remaining bytes from the present buffer
			for(x=bytesInPreviousRead;x<4;x++){
				deviceHeaderBytes[x] = letsParseSomeBytes[previousByteOfHeader[16+x]%size];
			}
		}
		else{
			//easy! all bytes are in the recent array. Grab them and store them.
			for(x=0;x<4;x++){
				deviceHeaderBytes[x] = letsParseSomeBytes[previousByteOfHeader[16+x]%size];
			}
		}
	}
	else{
		//send invalid data
		if(previousByteOfHeader[0] == -1){
			//if eeprom is initialized but no device header data
			for(x=0;x<sizeof(deviceHeaderBytes)-1;x++){
				deviceHeaderBytes[x] = 400;
			}
		}
		//if eeprom is uninitialized
		else{
			for(x=0;x<sizeof(deviceHeaderBytes)-1;x++){
				deviceHeaderBytes[x] = 600;
			}
		}
	}
	//deviceHeaderBytes[4] = previousByteOfHeader[0];
	return deviceHeaderBytes;
}

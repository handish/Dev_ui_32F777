/*
 * menu.c
 *
 *  Created on: Feb 18, 2021
 *      Author: auphilli
 */
#include "menu.h"

int previousMenu = 0;
float displayAdcValues[21];
int setIndicator=0;

void initializeDisplay(){
	HAL_StatusTypeDef Status = HAL_OK;
	SMLCD_InitGPIO();
	SMLCD_Init(hspi4);
	SMLCD_Enable();
	SMLCD_Clear();
	if (Status != HAL_OK)
	{
		DevUI_Error_Handler("SPI LCD Clear Command Failed.", Status, 0, 0, true);
	}
#define ORI 0
	uint8_t ori;
	if (ORI == 0){
		ori = LCD_ORIENT_NORMAL;
	}
	SMLCD_Orientation(ori);
	LCD_Clear();
	memset(displayAdcValues,0,sizeof(displayAdcValues));
}

void drawMainMenu(int indicator){
	int i,j;
	previousMenu=0;
	getLatestADC();
	LCD_Clear();
	LCD_PixelMode = LCD_PSET;
	LCD_Rect(0, 0, scr_width - 1, scr_height - 1);
	LCD_Rect(2, 2, scr_width - 3, scr_height - 3);

	// RTC :)
	i  = 10;
	j  = 10;
	printFaults(i,j);
	//i += LCD_PutStr(i, j, "FAULTS:", fnt7x10);
	//i += LCD_PutStr(i, j, "So many!", fnt7x10);
	i=150;
	i += LCD_PutStr(i, j, "MODE:", fnt7x10);
	if(displayAdcValues[Adc.adc0] >3.5){
		//errorLED.vsysPMIFault=0;
		switch(bootButtons.bootMode){
		case UNINITIALIZED:
			LCD_PutStr(i, j, "OFF", fnt7x10);
			//errorLED.standard_boot = 0;
			//errorLED.uefi_boot  = 0;
			//errorLED.edl_boot  = 0;
			break;
		case STANDARD:
			LCD_PutStr(i, j, "OS", fnt7x10);
			//errorLED.standard_boot = 1;
			//errorLED.uefi_boot  = 0;
			//errorLED.edl_boot  = 0;
			break;
		case UEFI:
			LCD_PutStr(i, j, "UEFI", fnt7x10);
			//errorLED.standard_boot = 0;
			//errorLED.uefi_boot  = 1;
			//errorLED.edl_boot  = 0;
			break;
		case EDL:
			LCD_PutStr(i, j, "EDL", fnt7x10);
			//errorLED.standard_boot = 0;
			//errorLED.uefi_boot  = 0;
			//errorLED.edl_boot  = 1;
			break;
		case MASS_STORAGE:
			LCD_PutStr(i, j, "MASS", fnt7x10);
			//errorLED.standard_boot = 1;
			//errorLED.uefi_boot  = 1;
			//errorLED.edl_boot  = 0;
			break;
		case RECOVERY:
			LCD_PutStr(i, j, "FFU", fnt7x10);
			//errorLED.standard_boot = 0;
			//errorLED.uefi_boot  = 1;
			//errorLED.edl_boot  = 1;
			break;
		}
	}
	else{
		LCD_PutStr(i, j, "OFF", fnt7x10);
		//errorLED.vsysPMIFault=1;
		//errorLED.standard_boot = 0;
		//errorLED.uefi_boot  = 0;
		//errorLED.edl_boot  = 0;

	}
	i  = 135;
	j += 14;

	// Horizontal divider
	LCD_FillRect(2, j, scr_width - 94, j + 3);

	// Vertical divider
	LCD_FillRect(i + 5, 2, i + 8, j);
	LCD_FillRect(scr_width-97, 2, scr_width-94,scr_height-1);
	i  = scr_width-90;
	j  = 3;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=20;
	LCD_PutStr(i, j, "ZION FLT", fnt7x10);
	j+=22;
	i  = scr_width-90;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=20;
	LCD_PutStr(i, j, "VSYS FLT", fnt7x10);
	i  = scr_width-90;
	j  += 22;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=20;
	LCD_PutStr(i, j, "FAULT3", fnt7x10);
	j+=22;
	i  = scr_width-90;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=20;
	LCD_PutStr(i, j, "FAULT4", fnt7x10);
	i  = scr_width-90;
	j  += 22;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=20;
	LCD_PutStr(i, j, "FAULT5", fnt7x10);
	j+=22;
	i  = scr_width-90;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=20;
	LCD_PutStr(i, j, "FAULT6", fnt7x10);
	i  = scr_width-90;
	j  +=22;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=20;
	LCD_PutStr(i, j, "FAULT7", fnt7x10);
	j+=22;
	i  = scr_width-90;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=20;
	LCD_PutStr(i, j, "FAULT8", fnt7x10);
	i  = scr_width-90;
	j  +=22;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=20;
	LCD_PutStr(i, j, "FAULT9", fnt7x10);
	j = scr_height-20;
	i  = scr_width-90;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=20;
	LCD_PutStr(i, j, "MODE", fnt7x10);

	i=100;
	j=75;
	i+=20;
	LCD_PutStr(i, j, "BOOT MODE", fnt7x10);
	j+=15;
	LCD_FillRect(i-3, j - 2, i + 71, j + 2);

	i=100;
	j=110;
	i+=20;
	LCD_PutStr(i, j, "STATUS", fnt7x10);
	j+=15;
	LCD_FillRect(i-3, j - 2, i + 50, j + 2);

	i=100;
	j=145;
	i+=20;
	LCD_PutStr(i, j, "SYSTEM INFO", fnt7x10);
	j+=15;
	LCD_FillRect(i-3, j - 2, i + 85, j + 2);
	i=100;
	switch(indicator){
	case FIRST:
	{
		j=75;
		break;
	}
	case SECOND:
	{
		j=110;
		break;
	}
	case THIRD:
	{
		j=145;
		break;
	}
	default:
	{
		j=75;
		break;
	}
	}
	LCD_FillRect(i, j, i + 12, j + 10);
	SMLCD_Flush();
}

void drawStatusMenu(int indicator){
	int i,j;
	//float *adcValuePointer;
	int convertedFloat;
	int adjacentSpacing = 20;
	int inputGpioAlignment=245;
	int daughterCardAlignment=240;
	int i2cCheck;
	int arrowUp = 0;
	int arrowDown=1;
	int arrowSize=3;
	previousMenu=0;
	LCD_Clear();
	getLatestADC();
	drawMenuHeader();
	i  = 135;
	j = 24;
	//horizontal divider
	LCD_FillRect(2, j, scr_width - 2, j + 3);
//	LCD_PixelMode = LCD_PSET;
//
//	LCD_Rect(0, 0, scr_width - 1, scr_height - 1);
//	LCD_Rect(2, 2, scr_width - 3, scr_height - 3);
//
//	// RTC :)
//	i  = 10;
//	j  = 10;
//	i += LCD_PutStr(i, j, "FAULTS:", fnt7x10);
//	i += LCD_PutStr(i, j, "So many!", fnt7x10);
//	i+=170;
//	i += LCD_PutStr(i, j, "MODE:", fnt7x10);
//	i += LCD_PutStr(i, j, "QED", fnt7x10);
//	i  = 135;
//	j += 12;
//
//	//horizontal divider
//	LCD_FillRect(2, j, scr_width - 2, j + 3);
	switch(indicator){
	case 1:{
		i=35;
		j=90;
		LCD_FillRect(2, j - 2, scr_width-2, j + 2);
		j=75;
		i=62;
		i+=LCD_PutStr(i, j, "FAULTS:", fnt7x10);
		i+=145;
		LCD_PutStr(i, j, "DAUGHTER CARDS:", fnt7x10);

		i=10;
		j=95;

		i+= LCD_PutStr(i, j, "ZION FLT: ", fnt7x10);
		if(errorLED.zionFault){
			LCD_PutStr(i, j, "SOC ZION ERROR", fnt7x10);
		}
		else{
			LCD_PutStr(i, j, "Clear", fnt7x10);
		}
		i=daughterCardAlignment;
		i+= LCD_PutStr(i, j, "SOC: ", fnt7x10);
		if(ZION.SOC_EEPROM_Detected){
			LCD_PutStr(i,j,"Detected", fnt7x10);
		}
		else{
			LCD_PutStr(i,j,"Undetected", fnt7x10);
		}
		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "VSYS_PMI: ", fnt7x10);
		if(errorLED.vsysPMIFault){
			LCD_PutStr(i, j, "VSYS PMI LOW", fnt7x10);
		}
		else{
			LCD_PutStr(i, j, "Clear", fnt7x10);
		}
		i=daughterCardAlignment;
		i+= LCD_PutStr(i, j, "ASIC: ", fnt7x10);
		if(ZION.ASIC_EEPROM_Detected){
			LCD_PutStr(i,j,"Detected", fnt7x10);
		}
		else{
			LCD_PutStr(i,j,"Undetected", fnt7x10);
		}
		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "Fault3: ", fnt7x10);
		if(errorLED.fault3){
			LCD_PutStr(i, j, "FAULT 3 Triggered", fnt7x10);
		}
		else{
			LCD_PutStr(i, j, "Clear", fnt7x10);
		}
		i=daughterCardAlignment;
		i+= LCD_PutStr(i, j, "Display: ", fnt7x10);
		if(ZION.DISPLAY_EEPROM_Detected){
			LCD_PutStr(i,j,"Detected", fnt7x10);
		}
		else{
			LCD_PutStr(i,j,"Undetected", fnt7x10);
		}

		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "Fault4: ", fnt7x10);
		if(errorLED.fault4){
			LCD_PutStr(i, j, "FAULT 4 Triggered", fnt7x10);
		}
		else{
			LCD_PutStr(i, j, "Clear", fnt7x10);
		}

		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "Fault5: ", fnt7x10);
		if(errorLED.fault5){
			LCD_PutStr(i, j, "FAULT 5 Triggered", fnt7x10);
		}
		else{
			LCD_PutStr(i, j, "Clear", fnt7x10);
		}

		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "Fault6: ", fnt7x10);
		if(errorLED.fault6){
			LCD_PutStr(i, j, "FAULT 6 Triggered", fnt7x10);
		}
		else{
			LCD_PutStr(i, j, "Clear", fnt7x10);
		}

		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "Fault7: ", fnt7x10);
		if(errorLED.fault7){
			LCD_PutStr(i, j, "FAULT 7 Triggered", fnt7x10);
		}
		else{
			LCD_PutStr(i, j, "Clear", fnt7x10);
		}

		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "Fault8: ", fnt7x10);
		if(errorLED.fault8){
			LCD_PutStr(i, j, "FAULT 8 Triggered", fnt7x10);
		}
		else{
			LCD_PutStr(i, j, "Clear", fnt7x10);
		}

		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "Fault9: ", fnt7x10);
		if(errorLED.fault9){
			LCD_PutStr(i, j, "FAULT 9 Triggered", fnt7x10);
		}
		else{
			LCD_PutStr(i, j, "Clear", fnt7x10);
		}


		//horizontal divider
		j=65;
		LCD_FillRect(2, j-1, scr_width-2, j+1);
		//vertical divider
		i=230;
		LCD_FillRect(i-3, j, i+3, scr_height-2);

		i=200;
		j=220;
		drawUpDownArrow(i, j, arrowSize, arrowDown);
		i=180;
		drawUpDownArrow(i, j, arrowSize, arrowDown);
		i=220;
		drawUpDownArrow(i, j, arrowSize, arrowDown);
		break;
	}
	case 2:{
		i=35;
		j=90;
		LCD_FillRect(2, j - 2, scr_width-2, j + 2);
		j=75;
		i=42;
		i+=LCD_PutStr(i, j, "AI VOLTAGES:", fnt7x10);
		i+=125;
		LCD_PutStr(i, j, "GPIO INPUTS:", fnt7x10);

		i=10;
		j=95;
		i+= LCD_PutStr(i, j, "AI0: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc0];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
		i+= LCD_PutStr(i, j, "AI9: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc9];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
		i=inputGpioAlignment;
		i+= LCD_PutStr(i, j, "In0: ", fnt7x10);
		i+=LCD_PutInt(i,j,gpioInputBuf[inputGPIOs.input0], fnt7x10)+adjacentSpacing;
		i+= LCD_PutStr(i, j, "In9: ", fnt7x10);
		i+=LCD_PutInt(i,j,gpioInputBuf[inputGPIOs.input9], fnt7x10);


		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "AI1: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc1];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
		i+= LCD_PutStr(i, j, "AI10: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc10];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
		i=inputGpioAlignment;
		i+= LCD_PutStr(i, j, "In1: ", fnt7x10);
		i+=LCD_PutInt(i,j,gpioInputBuf[inputGPIOs.input1], fnt7x10)+adjacentSpacing;
		i+= LCD_PutStr(i, j, "In10: ", fnt7x10);
		i+=LCD_PutInt(i,j,gpioInputBuf[inputGPIOs.input10], fnt7x10);

		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "AI2: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc2];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
		i+= LCD_PutStr(i, j, "AI11: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc11];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
		i=inputGpioAlignment;
		i+= LCD_PutStr(i, j, "In2: ", fnt7x10);
		i+=LCD_PutInt(i,j,gpioInputBuf[inputGPIOs.input2], fnt7x10)+adjacentSpacing;
		i+= LCD_PutStr(i, j, "In11: ", fnt7x10);
		i+=LCD_PutInt(i,j,gpioInputBuf[inputGPIOs.input11], fnt7x10);

		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "AI3: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc3];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
		i+= LCD_PutStr(i, j, "AI12: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc12];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
		i=inputGpioAlignment;
		i+= LCD_PutStr(i, j, "In3: ", fnt7x10);
		LCD_PutInt(i,j,gpioInputBuf[inputGPIOs.input3], fnt7x10);


		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "AI4: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc4];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
		i+= LCD_PutStr(i, j, "AI13: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc13];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
		i=inputGpioAlignment;
		i+= LCD_PutStr(i, j, "In4: ", fnt7x10);
		LCD_PutInt(i,j,gpioInputBuf[inputGPIOs.input4], fnt7x10);

		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "AI5: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc5];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
		i+= LCD_PutStr(i, j, "AI14: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc14];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
		i=inputGpioAlignment;
		i+= LCD_PutStr(i, j, "In5: ", fnt7x10);
		LCD_PutInt(i,j,gpioInputBuf[inputGPIOs.input5], fnt7x10);

		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "AI6: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc6];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
		i+= LCD_PutStr(i, j, "AI15: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc15];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
		i=inputGpioAlignment;
		i+= LCD_PutStr(i, j, "In6: ", fnt7x10);
		LCD_PutInt(i,j,gpioInputBuf[inputGPIOs.input6], fnt7x10);

		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "AI7: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc7];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
		i=inputGpioAlignment;
		i+= LCD_PutStr(i, j, "In7: ", fnt7x10);
		LCD_PutInt(i,j,gpioInputBuf[inputGPIOs.input7], fnt7x10);

		i=10;
		j+=15;
		i+= LCD_PutStr(i, j, "AI8: ", fnt7x10);
		convertedFloat = 1000 * displayAdcValues[Adc.adc8];
		i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
		i=inputGpioAlignment;
		i+= LCD_PutStr(i, j, "In8: ", fnt7x10);
		LCD_PutInt(i,j,gpioInputBuf[inputGPIOs.input8], fnt7x10);
		//horizontal divider
		j=65;
		LCD_FillRect(2, j-1, scr_width-2, j+1);
		//vertical divider
		i=230;
		LCD_FillRect(i-3, j, i+3, scr_height-2);

		i=155;
		j=35;
		LCD_PutStr(i,j,"YOUR AD HERE!",fnt7x10);
		i=155;
		j=50;
		i+=LCD_PutStr(i,j,"Monthly fee: $",fnt7x10)+8;
		j=28;
		LCD_PutIntF(i,j,3999, 2,fnt_dig_big);

		//draw the additional pages available indicators
		i=200;
		j=220;
		drawUpDownArrow(i, j, arrowSize, arrowDown);
		j=15;
		drawUpDownArrow(i, j, arrowSize, arrowUp);
		i=180;
		drawUpDownArrow(i, j, arrowSize, arrowUp);
		j=220;
		drawUpDownArrow(i, j, arrowSize, arrowDown);
		i=220;
		drawUpDownArrow(i, j, arrowSize, arrowDown);
		j=15;
		drawUpDownArrow(i, j, arrowSize, arrowUp);
		break;
	}
	case 3:{
		i=35;
		j=75;
		LCD_FillRect(2, j - 2, scr_width-2, j + 2);
		j=55;
		i=150;
		i+=LCD_PutStr(i, j, "DEV UI HEALTH:", fnt7x10);

		i=40;
		j=85;
		i+= LCD_PutStr(i, j, "LCD: ", fnt7x10);
		LCD_PutStr(i, j, "Do you see me? Must be working!", fnt7x10);

		i=40;
		j+=25;
		i+= LCD_PutStr(i, j, "LED Driver: ", fnt7x10);
		if(!(errorLED.ledDriver)){
			LCD_PutStr(i, j, "Present", fnt7x10);
		}
		else{
			LCD_PutStr(i, j, "Unresponsive", fnt7x10);
		}

		i=40;
		j+=25;
		i+= LCD_PutStr(i, j, "SOC UART: ", fnt7x10);
		LCD_PutStr(i, j, "Am I reading this now?", fnt7x10);

		i=40;
		j+=25;
		i+= LCD_PutStr(i, j, "SOC I2C: ", fnt7x10);
		i2cCheck=writeI2CRegister(socI2cVoltageMux.address, 0x11, 0x00,1,socI2cVoltageMux.i2cBank);
		if(i2cCheck == HAL_OK){
			LCD_PutStr(i, j, "Present", fnt7x10);
			// Clear the HAL fault LED.
			errorLED.fault9 = false;
		}
		else{
			LCD_PutStr(i, j, "Undetected", fnt7x10);
		}


		i=40;
		j+=25;
		i+= LCD_PutStr(i, j, "Dev UI Runtime: ", fnt7x10);
		//GetTick provides runtime in milliseconds
		int runtime = (HAL_GetTick()/1000);
		i+= LCD_PutInt(i, j, runtime, fnt7x10);
		i+= LCD_PutStr(i, j, " seconds", fnt7x10);

		i=200;
		j=15;
		drawUpDownArrow(i, j, arrowSize, arrowUp);
		i=180;
		drawUpDownArrow(i, j, arrowSize, arrowUp);
		i=220;
		drawUpDownArrow(i, j, arrowSize, arrowUp);

		break;
	}
	}

	SMLCD_Flush();



}

void drawSystemInfoMenu(int indicator){
	int i,j;
	//float *adcValuePointer;
	int convertedFloat;
	int adjacentSpacing = 20;
	int indentAlignment=20;
	int arrowUp = 0;
	int arrowDown=1;
	int arrowSize=3;
	int otherBoardAlignment = 230;
	previousMenu=0;
	LCD_Clear();
	getLatestADC();
	drawMenuHeader();
	i  = 135;
	j = 24;
	//horizontal divider
	LCD_FillRect(2, j, scr_width - 2, j + 3);
//	LCD_PixelMode = LCD_PSET;
//
//	LCD_Rect(0, 0, scr_width - 1, scr_height - 1);
//	LCD_Rect(2, 2, scr_width - 3, scr_height - 3);
//
//	// RTC :)
//	i  = 10;
//	j  = 10;
//	i += LCD_PutStr(i, j, "FAULTS:", fnt7x10);
//	i += LCD_PutStr(i, j, "So many!", fnt7x10);
//	i+=170;
//	i += LCD_PutStr(i, j, "MODE:", fnt7x10);
//	i += LCD_PutStr(i, j, "QED", fnt7x10);
//	i  = 135;
//	j += 12;
//	//horizontal divider
//	LCD_FillRect(2, j, scr_width - 2, j + 3);

	switch(indicator){

	case 1:{
		//i=35;
		//j=90;
		//LCD_FillRect(2, j - 2, scr_width-2, j + 2);

		j=45;
		i=42;
		i+=LCD_PutStr(i, j, "FFU Version:", fnt7x10);
		LCD_PutStr(i, j, "Unknown", fnt7x10);

		j+=15;
		i=42;
		i+=LCD_PutStr(i, j, "UI Firmware Version: ", fnt7x10);
		LCD_PutStr(i, j, "V0.0.1", fnt7x10);

		j+=30;
		i=10;
		LCD_PutStr(i, j, "Project:", fnt7x10);

		j+=15;
		i=indentAlignment;
		if((ZION.SOC_BoardID==1) || (ZION.ASIC_BoardID==1) || (ZION.DISPLAY_BoardID==1)){
			LCD_PutStr(i, j, "Atlas", fnt7x10);
		}
		else{
			LCD_PutStr(i, j, "Unknown", fnt7x10);
		}

		j+=15;
		i=10;
		i+=LCD_PutStr(i, j, "Board Versions: ", fnt7x10);
		i=otherBoardAlignment;
		LCD_PutStr(i, j, "Other Boards: ", fnt7x10);

		j+=15;
		i=indentAlignment;
		i+=LCD_PutStr(i, j, "SOC: ", fnt7x10);
		if(ZION.SOC_BoardID==1){
			i+=LCD_PutStr(i, j, "TRIDENT ", fnt7x10);
			switch(ZION.SOC_BoardFab){
			case 1:{
				LCD_PutStr(i, j, "FAB A", fnt7x10);
				break;
			}
			case 2:{
				LCD_PutStr(i, j, "FAB B", fnt7x10);
				break;
			}
			case 3:{
				LCD_PutStr(i, j, "FAB C", fnt7x10);
				break;
			}
			case 4:{
				LCD_PutStr(i, j, "FAB D", fnt7x10);
				break;
			}
			default:{
				LCD_PutStr(i, j, "FAB NA", fnt7x10);
				break;
			}
			}
		}
		else{
			if(ZION.SOC_EEPROM_Detected){
				if(ZION.SOC_BoardFab == -2){
					LCD_PutStr(i, j, "EEPROM-NO DEVICE DATA", fnt7x10);
				}
				else if(ZION.SOC_BoardFab ==-1){
					LCD_PutStr(i, j, "EEPROM-UNINITIALIZED", fnt7x10);
				}
			}
			else{
				LCD_PutStr(i, j, "EEPROM not detected", fnt7x10);
			}
		}
		i=otherBoardAlignment+indentAlignment;
		i+=LCD_PutStr(i, j, "ZION: ", fnt7x10);
		if(ZION.zionSwitch){
			LCD_PutStr(i, j, "Not detected", fnt7x10);
		}
		else{
			LCD_PutStr(i, j, "Detected", fnt7x10);
		}

		j+=15;
		i=indentAlignment;
		i+=LCD_PutStr(i, j, "ASIC: ", fnt7x10);
		if(ZION.ASIC_BoardID==1){
			i+=LCD_PutStr(i, j, "TOGA ", fnt7x10);
			switch(ZION.ASIC_BoardFab){
			case 1:{
				LCD_PutStr(i, j, "FAB A", fnt7x10);
				break;
			}
			case 2:{
				LCD_PutStr(i, j, "FAB B", fnt7x10);
				break;
			}
			case 3:{
				LCD_PutStr(i, j, "FAB C", fnt7x10);
				break;
			}
			case 4:{
				LCD_PutStr(i, j, "FAB D", fnt7x10);
				break;
			}
			default:{
				LCD_PutStr(i, j, "FAB NA", fnt7x10);
				break;
			}
			}
		}
		else{
			if(ZION.ASIC_EEPROM_Detected){
				if(ZION.ASIC_BoardFab == -2){
					LCD_PutStr(i, j, "EEPROM-NO DEVICE DATA", fnt7x10);
				}
				else if(ZION.ASIC_BoardFab ==-1){
					LCD_PutStr(i, j, "EEPROM-UNINITIALIZED", fnt7x10);
				}
			}
			else{
				LCD_PutStr(i, j, "EEPROM not detected", fnt7x10);
			}
		}

		j+=15;
		i=indentAlignment;
		i+=LCD_PutStr(i, j, "DISPLAY: ", fnt7x10);
		if(ZION.DISPLAY_BoardID==1){
			i+=LCD_PutStr(i, j, "KANU ", fnt7x10);
			switch(ZION.DISPLAY_BoardFab){
			case 1:{
				LCD_PutStr(i, j, "FAB A", fnt7x10);
				break;
			}
			case 2:{
				LCD_PutStr(i, j, "FAB B", fnt7x10);
				break;
			}
			case 3:{
				LCD_PutStr(i, j, "FAB C", fnt7x10);
				break;
			}
			case 4:{
				LCD_PutStr(i, j, "FAB D", fnt7x10);
				break;
			}
			default:{
				LCD_PutStr(i, j, "FAB NA", fnt7x10);
				break;
			}
			}
		}
		else{
			if(ZION.DISPLAY_EEPROM_Detected){
				if(ZION.DISPLAY_BoardFab == -2){
					LCD_PutStr(i, j, "EEPROM-NO DEVICE DATA", fnt7x10);
				}
				else if(ZION.DISPLAY_BoardFab ==-1){
					LCD_PutStr(i, j, "EEPROM-UNINITIALIZED", fnt7x10);
				}
			}
			else{
				LCD_PutStr(i, j, "EEPROM not detected", fnt7x10);
			}
		}

		i=200;
		j=220;
		drawUpDownArrow(i, j, arrowSize, arrowDown);
		i=180;
		drawUpDownArrow(i, j, arrowSize, arrowDown);
		i=220;
		drawUpDownArrow(i, j, arrowSize, arrowDown);
		break;
	}
	case 2:{
		j=45;
		i=10;
		LCD_PutStr(i, j, "DEV UI BANK VOLTAGES:", fnt7x10);

		j+=25;
		i=indentAlignment;
		i+=LCD_PutStr(i, j, "SPARE UART: ", fnt7x10);
		convertedFloat = 10.0 * displayAdcValues[Adc.spareUartADC];
		LCD_PutIntF(i, j, convertedFloat, 1, fnt7x10);

		j+=25;
		i=indentAlignment;
		i+=LCD_PutStr(i, j, "SPARE I2C: ", fnt7x10);
		convertedFloat = 10.0 * displayAdcValues[Adc.spareI2cADC];
		LCD_PutIntF(i, j, convertedFloat, 1, fnt7x10);

		j+=25;
		i=indentAlignment;
		i+=LCD_PutStr(i, j, "SPARE SPI: ", fnt7x10);
		convertedFloat = 10.0 * displayAdcValues[Adc.spareSpiADC];
		LCD_PutIntF(i, j, convertedFloat, 1, fnt7x10);

		j+=25;
		i=indentAlignment;
		i+=LCD_PutStr(i, j, "CONFIG: ", fnt7x10);
		convertedFloat = 10.0 * displayAdcValues[Adc.configADC];
		LCD_PutIntF(i, j, convertedFloat, 1, fnt7x10);

		j+=25;
		i=indentAlignment;
		i+=LCD_PutStr(i, j, "ZION: ", fnt7x10);
		convertedFloat = 10.0 * displayAdcValues[Adc.zionADC];
		i+=LCD_PutIntF(i, j, convertedFloat, 1, fnt7x10) + adjacentSpacing;
		i+=LCD_PutStr(i, j, "Zion Switch: ", fnt7x10);
		if(ZION.zionSwitch ==1){
			LCD_PutStr(i, j, "ACTIVE", fnt7x10);
		}
		else{
			LCD_PutStr(i, j, "INACTIVE", fnt7x10);
		}



		i=200;
		j=15;
		drawUpDownArrow(i, j, arrowSize, arrowUp);
		i=180;
		drawUpDownArrow(i, j, arrowSize, arrowUp);
		i=220;
		drawUpDownArrow(i, j, arrowSize, arrowUp);

		break;
	}
	}
	SMLCD_Flush();
}

void drawBootMenu(int indicator, uint8_t button, int menu){
	int i,j;
		int adjacentSpacing = 20;
		int indentAlignment=50;


		LCD_Clear();
		getLatestADC();
		drawMenuHeader();
		i  = 135;
		j = 24;
		//horizontal divider
		LCD_FillRect(2, j, scr_width - 2, j + 3);
		if(displayAdcValues[Adc.adc0] >3.5){
			if(ZION.zionFinished){
				if((ZION.SOC_BoardFab == ATLAS) || (ZION.ASIC_BoardFab == ATLAS) || (ZION.DISPLAY_BoardFab == ATLAS)){
					j=45;
					i=5;
					LCD_PutStr(i,j, "ATLAS RECOGNIZED. PROVIDING ATLAS BOOT MODES:", fnt7x10);
					j+=30;
					i=42;
					LCD_PutStr(i, j, "Please Select Boot Mode:", fnt7x10);
					i=indentAlignment;
					j+=20;
					LCD_PutStr(i, j, "STANDARD", fnt7x10);
					j+=20;
					LCD_PutStr(i, j, "EMERGENCY DOWNLOAD", fnt7x10);
					j+=20;
					LCD_PutStr(i, j, "RECOVERY", fnt7x10);
					j+=20;
					LCD_PutStr(i, j, "MASS STORAGE", fnt7x10);
					j+=20;
					LCD_PutStr(i, j, "UEFI", fnt7x10);
					i-= 17;
					if(setIndicator==0){
						switch(indicator){
						case FIRST:
						{
							j=95;
							if((button == SEL) & (previousMenu == menu)){
								i=140;
								drawUpDownArrow(i, j+5, 3, 3);
								bootButtons.btn0=1;
								bootButtons.bootModeSet=1;
								setIndicator=1;
							}

							break;
						}
						case SECOND:
						{
							j=115;
							if((button == SEL) & (previousMenu == menu)){
								i=220;
								drawUpDownArrow(i, j+5, 3, 3);
								bootButtons.edl_sw=1;
								bootButtons.bootModeSet=1;
								setIndicator=2;
							}

							break;
						}
						case THIRD:
						{
							j=135;
							if((button == SEL) & (previousMenu == menu)){
								i=130;
								drawUpDownArrow(i, j+5, 3, 3);
								bootButtons.btn1=1;
								bootButtons.bootModeSet=1;
								setIndicator=3;
							}

							break;
						}
						case FOURTH:
						{
							j=155;
							if((button == SEL) & (previousMenu == menu)){
								i=160;
								drawUpDownArrow(i, j+5, 3, 3);
								bootButtons.btn2=1;
								bootButtons.bootModeSet=1;
								setIndicator=4;
							}

							break;
						}
						case FIFTH:
						{
							j=175;
							if((button == SEL) & (previousMenu == menu)){
								i=100;
								drawUpDownArrow(i, j+5, 3, 3);
								bootButtons.btn3=1;
								bootButtons.bootModeSet=1;
								setIndicator=5;
							}

							break;
						}
						default:
						{
							j=95;
							break;
						}
						}
					}
					else{
						switch(setIndicator){
						case FIRST:
						{
							j=95;
							i=140;
							drawUpDownArrow(i, j+5, 3, 3);
							if(bootButtons.modeClear){
								setIndicator=0;
							}
							break;
						}
						case SECOND:
						{
							j=115;
							i=220;
							drawUpDownArrow(i, j+5, 3, 3);
							if(bootButtons.modeClear){
								setIndicator=0;
							}
							break;
						}
						case THIRD:
						{
							j=135;
							i=130;
							drawUpDownArrow(i, j+5, 3, 3);
							if(bootButtons.modeClear){
								setIndicator=0;
							}
							break;
						}
						case FOURTH:
						{
							j=155;
							i=160;
							drawUpDownArrow(i, j+5, 3, 3);
							if(bootButtons.modeClear){
								setIndicator=0;
							}
							break;
						}
						case FIFTH:
						{
							j=175;
							i=100;
							drawUpDownArrow(i, j+5, 3, 3);
							if(bootButtons.modeClear){
								setIndicator=0;
							}
							break;
						}
						default:
						{
							drawUpDownArrow(i, j+5, 3, 3);
							break;
						}
						}
					}
				}
				else{
					j=45;
					i=5;
					LCD_PutStr(i,j, "UNKNOWN SYSTEM. STANDARD MODE ONLY:", fnt7x10);
					j+=30;
					i=42;
					LCD_PutStr(i, j, "Please Select Boot Mode:", fnt7x10);
					i=indentAlignment;
					j+=20;
					LCD_PutStr(i, j, "STANDARD", fnt7x10);
					if(setIndicator==0){
						switch(indicator){
						case FIRST:
						{
							j=95;
							if((button == SEL) & (previousMenu == menu)){
								i=140;
								drawUpDownArrow(i, j+5, 3, 3);
								bootButtons.btn0=1;
								bootButtons.bootModeSet=1;
								setIndicator=1;
							}
							break;
						}
						default:
						{
							j=95;
							if((button == SEL) & (previousMenu == menu)){
								i=140;
								drawUpDownArrow(i, j+5, 3, 3);
								bootButtons.btn0=1;
								bootButtons.bootModeSet=1;
								setIndicator=1;
							}
							break;
						}
						}
					}
					else{
						switch(setIndicator){
						case FIRST:
						{
							j=95;
							i=140;
							drawUpDownArrow(i, j+5, 3, 3);
							if(bootButtons.modeClear){
								setIndicator=0;
							}
							break;
						}
						default:
						{
							j=95;
							i=140;
							drawUpDownArrow(i, j+5, 3, 3);
							if(bootButtons.modeClear){
								setIndicator=0;
							}
							break;
						}
						}
					}
				}
			}
			else{
				j=45;
				i=15;
				LCD_PutStr(i,j, "WAITING ON ZION INFO. STANDARD MODE ONLY:", fnt7x10);
				j+=30;
				i=42;
				LCD_PutStr(i, j, "Please Select Boot Mode:", fnt7x10);
				i=indentAlignment;
				j+=20;
				LCD_PutStr(i, j, "STANDARD", fnt7x10);
				if(setIndicator==0){
					switch(indicator){
					case FIRST:
					{
						j=95;
						if((button == SEL) & (previousMenu == menu)){
							i=140;
							drawUpDownArrow(i, j+5, 3, 3);
							bootButtons.btn0=1;
							bootButtons.bootModeSet=1;
							setIndicator=1;
						}
						break;
					}
					default:
					{
						j=95;
						if((button == SEL) & (previousMenu == menu)){
							i=140;
							drawUpDownArrow(i, j+5, 3, 3);
							bootButtons.btn0=1;
							bootButtons.bootModeSet=1;
							setIndicator=1;
						}
						break;
					}
					}
				}
				else{
					switch(setIndicator){
					case FIRST:
					{
						j=95;
						i=140;
						drawUpDownArrow(i, j+5, 3, 3);
						if(bootButtons.modeClear){
							setIndicator=0;
						}
						break;
					}
					default:
					{
						j=95;
						i=140;
						drawUpDownArrow(i, j+5, 3, 3);
						if(bootButtons.modeClear){
							setIndicator=0;
						}
						break;
					}
					}
				}
			}
		}
		else{
			j=110;
			i=120;
			bootButtons.bootMode=0;
			LCD_PutStr(i,j, "POWER SWITCH DISABLED!", fnt7x10);
			j+=14;
			i=75;
			LCD_PutStr(i,j, "Flip Switch to enable Boot Options!", fnt7x10);
		}
		previousMenu=menu;
		i=25;
		LCD_FillRect(i, j, i + 12, j + 10);
		SMLCD_Flush();
}

void drawMenuHeader(){
	int i, j;
	LCD_PixelMode = LCD_PSET;

	LCD_Rect(0, 0, scr_width - 1, scr_height - 1);
	LCD_Rect(2, 2, scr_width - 3, scr_height - 3);

	// RTC :)
	i  = 10;
	j  = 10;
	printFaults(i,j);
	i=275;
	i += LCD_PutStr(i, j, "MODE:", fnt7x10);
	if(displayAdcValues[Adc.adc0] >3.5){
		switch(bootButtons.bootMode){
		case UNINITIALIZED:
			LCD_PutStr(i, j, "OFF", fnt7x10);
			break;
		case STANDARD:
			LCD_PutStr(i, j, "OS", fnt7x10);
			break;
		case UEFI:
			LCD_PutStr(i, j, "UEFI", fnt7x10);
			break;
		case EDL:
			LCD_PutStr(i, j, "EDL", fnt7x10);
			break;
		case MASS_STORAGE:
			LCD_PutStr(i, j, "MASS", fnt7x10);
			break;
		case RECOVERY:
			LCD_PutStr(i, j, "FFU", fnt7x10);
			break;
		}
	}
	else{
		LCD_PutStr(i, j, "OFF", fnt7x10);
	}

	i  = 135;
	j += 14;


}

void getLatestADC(){
	int i;
	float * adcValuePointer;
	 if (adcRestart[0] & adcRestart[1] & adcRestart[2]){
		  adcValuePointer = getADCValues();
		  for(i=0;i<20;i++){
	  		  displayAdcValues[i]=*adcValuePointer;
	  		  adcValuePointer++;
		  }
	 }
}
//print only the critical faults. Order is VSYS_PMI, ZION, FAULT3-9
int printFaults(int i, int j){
	int x = i;
	int y = j;
	x += LCD_PutStr(x, y, "FAULTS:", fnt7x10);
	if(errorLED.vsysPMIFault){
		x += LCD_PutStr(x, y, "VSYS", fnt7x10);
	}
	else if(errorLED.zionFault){
		x += LCD_PutStr(x, y, "ZION", fnt7x10);
	}
	else if(errorLED.fault3){
		x += LCD_PutStr(x, y, "FAULT3", fnt7x10);
	}
	else if(errorLED.fault4){
		x += LCD_PutStr(x, y, "FAULT4", fnt7x10);
	}
	else if(errorLED.fault5){
		x += LCD_PutStr(x, y, "FAULT5", fnt7x10);
	}
	else if(errorLED.fault6){
		x += LCD_PutStr(x, y, "FAULT6", fnt7x10);
	}
	else if(errorLED.fault7){
		x += LCD_PutStr(x, y, "FAULT7", fnt7x10);
	}
	else if(errorLED.fault8){
		x += LCD_PutStr(x, y, "FAULT8", fnt7x10);
	}
	else if(errorLED.fault9){
		x += LCD_PutStr(x, y, "FAULT9", fnt7x10);
	}
	else if(errorLED.ledDriver){
		x += LCD_PutStr(x, y, "LED DVR", fnt7x10);
	}
	else{
		x += LCD_PutStr(x, y, "NONE!", fnt7x10);
	}
	return x;
}

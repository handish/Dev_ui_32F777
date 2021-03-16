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
int socI2cCheck=0;
int checkedEEPROM=0;
int i2cCheck=1;

#define VSYS_ADC_VAL		displayAdcValues[Adc.adc0]

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
	i=150;
	i += LCD_PutStr(i, j, "MODE:", fnt7x10);
	int weAreAtlas = (ZION.SOC_BoardID==ATLAS) || (ZION.ASIC_BoardID==ATLAS) || (ZION.DISPLAY_BoardID==ATLAS);
	if(VSYS_ADC_VAL >VSYS_FLT){
		if(ZION.SOC_EEPROM_Detected){
			//Add an if/else if for your project name and call the method
			if(weAreAtlas){
				atlasMainMenuBootModes(i,j);
			}
			else{
				defaultMainMenuBootModes(i,j);
			}
		}
		else{
			defaultMainMenuBootModes(i,j);
		}
	}
	else{
		LCD_PutStr(i, j, "OFF", fnt7x10);
		bootButtons.bootMode=0;
		checkedEEPROM=0;

	}
	i  = 135;
	j += 14;

	// Horizontal divider
	LCD_FillRect(2, j, scr_width - 94, j + 3);

	// Vertical divider
	LCD_FillRect(i + 5, 2, i + 8, j);
	LCD_FillRect(scr_width-97, 2, scr_width-94,scr_height-1);

	//
	//Add Project Main Menu Fault call here.
	//
	if(weAreAtlas){
		atlasMainMenuFaultLedLabels();
	}
	else{
		defaultMainMenuFaultLedLabels();
	}
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
	int arrowUp = 0;
	int arrowDown=1;
	int arrowSize=3;
	previousMenu=0;
	LCD_Clear();
	getLatestADC();
	drawMenuHeader();
	int weAreAtlas = (ZION.SOC_BoardID==ATLAS) || (ZION.ASIC_BoardID==ATLAS) || (ZION.DISPLAY_BoardID==ATLAS);
	i  = 135;
	j = 24;
	//horizontal divider
	LCD_FillRect(2, j, scr_width - 2, j + 3);

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

		//Add an if/else if for your project name and call the method

		if(weAreAtlas){
			atlasStatusFaults();
		}
		else{
			defaultStatusFaults();
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
		i=35;
		j=90;
		LCD_FillRect(2, j - 2, scr_width-2, j + 2);
		j=75;
		i=42;
		i+=LCD_PutStr(i, j, "AI VOLTAGES:", fnt7x10);
		i+=125;
		LCD_PutStr(i, j, "GPIO INPUTS:", fnt7x10);

		//Add an if/else if for your project and call the method

		if(weAreAtlas){
			atlasStatusADCsAndGPIOs();
		}
		else{
			defaultStatusADCsAndGPIOs();
		}


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
		//check the I2C state once per switch ran. Multiple runs can cause hard faults
		if((VSYS_ADC_VAL > VSYS_FLT) && !(checkedEEPROM)){
			i2cCheck = writeI2CRegister(socI2cVoltageMux.address, 0x11, 0x00,1,socI2cVoltageMux.i2cBank);
			checkedEEPROM=1;
		}
		else{
			checkedEEPROM=0;
		}
		if(i2cCheck == HAL_OK){
			LCD_PutStr(i, j, "Present", fnt7x10);
			// Clear the HAL fault LED.
			//errorLED.fault9 = false;
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
	int otherBoardAlignment = 225;
	int verticalSpacing = 15;
	previousMenu=0;
	LCD_Clear();
	getLatestADC();
	drawMenuHeader();
	i  = 135;
	j = 24;
	//horizontal divider
	LCD_FillRect(2, j, scr_width - 2, j + 3);

	switch(indicator){

	case 1:{
		//i=35;
		//j=90;
		//LCD_FillRect(2, j - 2, scr_width-2, j + 2);

		j=45;
		i=42;
		i+=LCD_PutStr(i, j, "FFU Version:", fnt7x10);
		LCD_PutStr(i, j, "Unknown", fnt7x10);

		j+=verticalSpacing;
		i=42;
		i+=LCD_PutStr(i, j, "UI Firmware Version: ", fnt7x10);
		LCD_PutStr(i, j, "V0.0.1", fnt7x10);

		j+=30;
		i=10;
		LCD_PutStr(i, j, "Project:", fnt7x10);

		j+=verticalSpacing;
		i=indentAlignment;
		//add your project definition here. example is ATLAS below. ATLAS is defined as finding zion information of 1 in the device header.
		int weAreAtlas = (ZION.SOC_BoardFab == ATLAS) || (ZION.ASIC_BoardFab == ATLAS) || (ZION.DISPLAY_BoardFab == ATLAS);
//Add an if/else if for your project name

		if(weAreAtlas){
			LCD_PutStr(i, j, "Atlas", fnt7x10);
		}
		else{
			LCD_PutStr(i, j, "Unknown", fnt7x10);
		}

		j+=verticalSpacing;
		i=10;
		i+=LCD_PutStr(i, j, "Board Versions: ", fnt7x10);
		i=otherBoardAlignment;
		LCD_PutStr(i, j, "Other Boards: ", fnt7x10);

		j+=verticalSpacing;
		i=indentAlignment;
		i+=LCD_PutStr(i, j, "SOC: ", fnt7x10);
		//Add an if/else if for the soc name and call the method
		if(ZION.SOC_BoardID==ATLAS){
			atlasSystemInfoSoc(i,j);
		}
		else{
			defaultSystemInfoSoc(i,j);
		}
		i=otherBoardAlignment+indentAlignment;
		i+=LCD_PutStr(i, j, "ZION: ", fnt7x10);
		if(ZION.zionSwitch){
			LCD_PutStr(i, j, "Not detected", fnt7x10);
		}
		else{
			LCD_PutStr(i, j, "Detected", fnt7x10);
		}

		j+=verticalSpacing;
		i=indentAlignment;
		i+=LCD_PutStr(i, j, "ASIC: ", fnt7x10);
		//Add an if/else if for your ASIC name and call the method
		if(ZION.ASIC_BoardID==ATLAS){
			atlasSystemInfoAsic(i,j);
		}
		else{
			defaultSystemInfoAsic(i,j);
		}
		//Add an if/else if if your project has additional boards. call the method
		if(ZION.SOC_BoardID==ATLAS){
			atlasSystemInfoPV(i,j);
		}
		j+=verticalSpacing;
		i=indentAlignment;
		i+=LCD_PutStr(i, j, "DISPLAY: ", fnt7x10);
		//Add an if/else if for your DISPLAY name and call the method
		if(ZION.DISPLAY_BoardID==ATLAS){
			atlasSystemInfoDisplay(i,j);
		}
		else{
			defaultSystemInfoDisplay(i,j);
		}
		//Add an if/else if for any additional boards that DEV_UI can check and call the methods
		if(ZION.SOC_BoardID==ATLAS){
			atlasSystemInfoWIFI(i,j);
			j+=verticalSpacing;
			atlasSystemInfoWIGIG(i,j);
			j+=verticalSpacing;
			atlasSystemInfoCODEC(i,j);
			j+=verticalSpacing;
			atlasSystemInfoRF(i,j);
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
		//int adjacentSpacing = 20;
		//int indentAlignment=50;


		LCD_Clear();
		getLatestADC();
		drawMenuHeader();
		i  = 135;
		j = 24;
		//horizontal divider
		LCD_FillRect(2, j, scr_width - 2, j + 3);

//add your project definition here. example is ATLAS below. ATLAS is defined as finding zion information of 1 in the device header.
		int weAreAtlas = (ZION.SOC_BoardFab == ATLAS) || (ZION.ASIC_BoardFab == ATLAS) || (ZION.DISPLAY_BoardFab == ATLAS);
		if(VSYS_ADC_VAL > VSYS_FLT){
			if(ZION.zionFinished){
				//add an additional if/else if for the new project and call your method.
				if(weAreAtlas){
					j = atlasBootMenuBootModes(indicator, previousMenu, menu, button);
				}
				else{
					j = defaultBootMenuBootModes(indicator, previousMenu, menu, button,1);
				}
			}
			else{
				j = defaultBootMenuBootModes(indicator, previousMenu, menu, button,0);
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
	int weAreAtlas = (ZION.SOC_BoardFab == ATLAS) || (ZION.ASIC_BoardFab == ATLAS) || (ZION.DISPLAY_BoardFab == ATLAS);
	if(VSYS_ADC_VAL >VSYS_FLT){
		if(ZION.SOC_EEPROM_Detected){
			//Add an if/else if for your project name and call the method
			if(weAreAtlas){
				atlasMainMenuBootModes(i,j);
			}
			else{
				defaultMainMenuBootModes(i,j);
			}
		}
		else{
			defaultMainMenuBootModes(i,j);
		}
	}
	else{
		LCD_PutStr(i, j, "OFF", fnt7x10);
		bootButtons.bootMode=0;
		checkedEEPROM=0;
	}

	i  = 135;
	j += 14;


}

void getLatestADC(){
	int i;
	float * adcValuePointer;
	 if (adcStates.adcBank1Finished && adcStates.adcBank2Finished && adcStates.adcBank3Finished){
		  adcValuePointer = getADCValues();
		  for(i=0;i<21;i++){
	  		  displayAdcValues[i]=*(adcValuePointer+i);
		  }
	 }
}
//print only the critical faults. Order is VSYS_PMI, ZION, FAULT3-9
int printFaults(int i, int j){
	int x = i;
	int y = j;
	x += LCD_PutStr(x, y, "FAULTS:", fnt7x10);
	int weAreAtlas = (ZION.SOC_BoardFab == ATLAS) || (ZION.ASIC_BoardFab == ATLAS) || (ZION.DISPLAY_BoardFab == ATLAS);
	//Add an if/else if for your project name and call the method
	if(weAreAtlas){
		x = atlasHeaderFaults(x,y);
	}
	else{
		x = defaultHeaderFaults(x,y);
	}
	return x;
}

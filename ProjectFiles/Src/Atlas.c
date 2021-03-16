/*
 * Atlas.c
 *
 *  Created on: Mar 15, 2021
 *      Author: auphilli
 */


#include "Atlas.h"

int otherBoardAlignment = 225;

extern int timeNow;

//Draws the labels for the Fault LEDs listed on the main menu display. Based off of the contents of the
//LED_FAULT_LBL defined in the project .h file.

//For new project, adjust name of #defines from Atlas to project name (i.e ATLAS_LED1 --> <ProjectName>_LED1) and add an additional
//ZION eeprom check in the drawMainMenu method in menu.c
void atlasMainMenuFaultLedLabels(){
	int i,j;
	int ledFaultAlign=scr_width-90;
	int verticalSpacing=22;
	int horizontalSpacing=20;
	i  = ledFaultAlign;
	j  = 3;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=20;
	LCD_PutStr(i, j, ATLAS_LED1_FAULT_LBL, fnt7x10);
	j+=verticalSpacing;
	i  = ledFaultAlign;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, ATLAS_LED2_FAULT_LBL, fnt7x10);
	i  = ledFaultAlign;
	j  += verticalSpacing;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, ATLAS_LED3_FAULT_LBL, fnt7x10);
	j+=verticalSpacing;
	i  = ledFaultAlign;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, ATLAS_LED4_FAULT_LBL, fnt7x10);
	i  = ledFaultAlign;
	j  += verticalSpacing;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, ATLAS_LED5_FAULT_LBL, fnt7x10);
	j+=verticalSpacing;
	i  = ledFaultAlign;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, ATLAS_LED6_FAULT_LBL, fnt7x10);
	i  = ledFaultAlign;
	j  +=verticalSpacing;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, ATLAS_LED7_FAULT_LBL, fnt7x10);
	j+=verticalSpacing;
	i  = ledFaultAlign;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, ATLAS_LED8_FAULT_LBL, fnt7x10);
	i  = ledFaultAlign;
	j  +=verticalSpacing;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, ATLAS_LED9_FAULT_LBL, fnt7x10);
}

//Draws Boot state in the main menu header. Based off of
//the ATLAS_MODEs defined in the project .h file.

//For new project, adjust name of #defines from Atlas to project name (i.e ATLAS_MAIN --> <ProjectName>_MAIN) and add an additional
//ZION eeprom check in the drawMainMenu method in menu.c
void atlasMainMenuBootModes(int i, int j){
	switch(bootButtons.bootMode){
	case UNINITIALIZED:
		LCD_PutStr(i, j, "OFF", fnt7x10);
		break;
	case STANDARD:
		LCD_PutStr(i, j, ATLAS_MAIN_STD_MODE, fnt7x10);
		break;
	case UEFI:
		LCD_PutStr(i, j, ATLAS_MAIN_UEFI_MODE, fnt7x10);
		break;
	case EDL:
		LCD_PutStr(i, j, ATLAS_MAIN_EDL_MODE, fnt7x10);
		break;
	case MASS_STORAGE:
		LCD_PutStr(i, j, ATLAS_MAIN_MASS_STORAGE_MODE, fnt7x10);
		break;
	case RECOVERY:
		LCD_PutStr(i, j, ATLAS_MAIN_RECOVERY_MODE, fnt7x10);
		break;
	}
}

//Draws Boot Menu options available if a correct project ZION eeprom is read. Also enforces that menu cannot be changed until
//boot process has been finished.
//Based off of the ATLAS_MODEs defined in the project .h file.

//For new project, adjust name of #defines from Atlas to project name (i.e ATLAS_MAIN --> <ProjectName>_MAIN) and add an additional
//ZION eeprom check in the drawMainMenu method in menu.c
int atlasBootMenuBootModes(int indicator, int previousMenu, int menu, int button){
	int j=45;
	int i=5;
	int bootModeSeparation = 20;
	int arrowLocationAdjustment = 17;
	int indentAlignment=50;
	LCD_PutStr(i,j, "ATLAS RECOGNIZED. PROVIDING ATLAS BOOT MODES:", fnt7x10);
	j+=30;
	i=42;
	LCD_PutStr(i, j, "Please Select Boot Mode:", fnt7x10);
	i=indentAlignment;
	j+=bootModeSeparation;
	LCD_PutStr(i, j, ATLAS_BOOT_STD_MODE, fnt7x10);
	j+=bootModeSeparation;
	LCD_PutStr(i, j, ATLAS_BOOT_EDL_MODE, fnt7x10);
	j+=bootModeSeparation;
	LCD_PutStr(i, j, ATLAS_BOOT_RECOVERY_MODE, fnt7x10);
	j+=bootModeSeparation;
	LCD_PutStr(i, j, ATLAS_BOOT_MASS_STORAGE_MODE, fnt7x10);
	j+=bootModeSeparation;
	LCD_PutStr(i, j, ATLAS_BOOT_UEFI_MODE, fnt7x10);
	i-= arrowLocationAdjustment;
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
				timeNow = (HAL_GetTick()/1000);
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
				timeNow = (HAL_GetTick()/1000);
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
				timeNow = (HAL_GetTick()/1000);
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
				timeNow = (HAL_GetTick()/1000);
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
				timeNow = (HAL_GetTick()/1000);
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
		i = 230;
		j = 120;
		int timeLeft=0;
		//add a small countdown so that people don't get too bored. The times below are captured experimentally based off TRIDENT
		//booting to OS is simplier/quicker as we don't need to hold other buttons until the system recognizes what happened.
		if(bootButtons.bootMode==0){
			if(setIndicator==FIRST){
				timeLeft = (timeNow+2)- (HAL_GetTick()/1000);
			}
			else{
				timeLeft = (timeNow+6)- (HAL_GetTick()/1000);
			}
		}
		else{
			if(setIndicator==FIRST){
				timeLeft = (timeNow+15)- (HAL_GetTick()/1000);
			}
			else{
				timeLeft = (timeNow+19)- (HAL_GetTick()/1000);
			}
		}
		i+=LCD_PutIntF(i, j, timeLeft, 0, fnt_dig_big);;
		LCD_PutStr(i, j, " SECS LEFT", fnt7x10);
		switch(setIndicator){
		//draw an arrow pointing at the boot option called until the modeClear flag is set by the bootButtons task.
		case FIRST:
		{
			j=95;
			i=140;
			drawUpDownArrow(i, j+5, 3, 3);
			if(bootButtons.modeClear){
				setIndicator=0;
				bootButtons.modeClear=0;
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
				bootButtons.modeClear=0;
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
				bootButtons.modeClear=0;
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
				bootButtons.modeClear=0;
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
				bootButtons.modeClear=0;
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
	return j;
}


//draw the name for the project soc board and its fab version. Used for the System Info page. Uses the ATLAS_SOC_BOARD definiton in the project
//.h file.
void atlasSystemInfoSoc(int i, int j){
	i+=LCD_PutStr(i, j, ATLAS_SOC_BOARD, fnt7x10);
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

//draw the name for the project asic board and its fab version. Used for the System Info page. Uses the ATLAS_ASIC_BOARD definition in the project
//.h file.
void atlasSystemInfoAsic(int i, int j){
	i+=LCD_PutStr(i, j, ATLAS_ASIC_BOARD, fnt7x10);
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
//draw the name for the project Display board and its fab version. Used for the System Info page. Uses the ATLAS_DISPLAY_BOARD definition in the project
//.h file.
void atlasSystemInfoDisplay(int i, int j){
	i+=LCD_PutStr(i, j, ATLAS_DISPLAY_BOARD, fnt7x10);
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
//draw the name for the project other board PV and its state of presence. Used for the System Info page. Uses the ATLAS_PV_PRSNT definition in the project
//.h file.
void atlasSystemInfoPV(int i, int j){
	int indentAlignment=20;
	i=otherBoardAlignment+indentAlignment;
	i+=LCD_PutStr(i, j, "PV: ", fnt7x10);
	if(ATLAS_PV_PRSNT){
		LCD_PutStr(i, j, "Detected", fnt7x10);
	}
	else{
		LCD_PutStr(i, j, "Not detected", fnt7x10);
	}
}
//draw the name for the project other board WIFI and its state of presence. Used for the System Info page. Uses the ATLAS_WIFI_PRSNT definition in the project
//.h file.
void atlasSystemInfoWIFI(int i, int j){
	int indentAlignment=20;
	i=otherBoardAlignment+indentAlignment;
	i+=LCD_PutStr(i, j, "WIFI: ", fnt7x10);
	if(ATLAS_WIFI_PRSNT){
		LCD_PutStr(i, j, "Detected", fnt7x10);
	}
	else{
		LCD_PutStr(i, j, "Not detected", fnt7x10);
	}
}

//draw the name for the project other board WIGIG and its state of presence. Used for the System Info page. Uses the ATLAS_WIGIG_PRSNT definition in the project
//.h file.
void atlasSystemInfoWIGIG(int i, int j){
	int indentAlignment=20;
	i=otherBoardAlignment+indentAlignment;
	i+=LCD_PutStr(i, j, "WIGIG: ", fnt7x10);
	if(ATLAS_WIGIG_PRSNT){
		LCD_PutStr(i, j, "Detected", fnt7x10);
	}
	else{
		LCD_PutStr(i, j, "Not detected", fnt7x10);
	}
}
//draw the name for the project other board CODEC and its state of presence. Used for the System Info page. Uses the ATLAS_CODEC_PRSNT definition in the project
//.h file.
void atlasSystemInfoCODEC(int i, int j){
	int indentAlignment=20;
	i=otherBoardAlignment+indentAlignment;
	i+=LCD_PutStr(i, j, "CODEC: ", fnt7x10);
	if(ATLAS_CODEC_PRSNT){
		LCD_PutStr(i, j, "Detected", fnt7x10);
	}
	else{
		LCD_PutStr(i, j, "Not detected", fnt7x10);
	}
}
//draw the name for the project other board RF and its state of presence. Used for the System Info page. Uses the ATLAS_RF_PRSNT definition in the project
//.h file.
void atlasSystemInfoRF(int i, int j){
	int indentAlignment=20;
	i=otherBoardAlignment+indentAlignment;
	i+=LCD_PutStr(i, j, "RF: ", fnt7x10);
	if(ATLAS_RF_PRSNT){
		LCD_PutStr(i, j, "Detected", fnt7x10);
	}
	else{
		LCD_PutStr(i, j, "Not detected", fnt7x10);
	}
}

//faults shown in the top margin for all menus. Needs to be relatively tiny (<12 characters)
int atlasHeaderFaults(int i, int j){
	int x = i;
	int y = j;
	if(errorLED.vsysPMIFault){
		x += LCD_PutStr(x, y, ATLAS_HEADER1_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.zionFault){
		x += LCD_PutStr(x, y, ATLAS_HEADER2_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.fault3){
		x += LCD_PutStr(x, y, ATLAS_HEADER3_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.fault4){
		x += LCD_PutStr(x, y, ATLAS_HEADER4_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.fault5){
		x += LCD_PutStr(x, y, ATLAS_HEADER5_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.fault6){
		x += LCD_PutStr(x, y, ATLAS_HEADER6_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.fault7){
		x += LCD_PutStr(x, y, ATLAS_HEADER7_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.fault8){
		x += LCD_PutStr(x, y, ATLAS_HEADER8_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.fault9){
		x += LCD_PutStr(x, y, ATLAS_HEADER9_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.ledDriver){
		x += LCD_PutStr(x, y, "LED DVR", fnt7x10);
	}
	else{
		x += LCD_PutStr(x, y, "NONE!", fnt7x10);
	}
	return x;
}

//draws the ADC and GPIO states in the status menu. Uses definitions from the project .h file to label each gpio/adc. alignment variables
//are used to adjust values if menus start looking awry
void atlasStatusADCsAndGPIOs(){
	int inputGpioAlignment=245;
	int adjacentSpacing = 20;
	int firstADCAlignment = 5;
	int secondADCAlignment = 120;
	int secondInputGpioAlignment=329;
	int i=firstADCAlignment;
	int j=95;
	int convertedFloat;
	i+= LCD_PutStr(i, j, ATLAS_AI0, fnt7x10);
	convertedFloat = 1000 * ATLAS_VSYS;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
	i=secondADCAlignment;
	i+= LCD_PutStr(i, j, ATLAS_AI9, fnt7x10);
	convertedFloat = 1000 * ATLAS_VDD_MM;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, ATLAS_IN0, fnt7x10);
	i+=LCD_PutInt(i,j,ATLAS_PV_PRSNT, fnt7x10)+adjacentSpacing;
	i=secondInputGpioAlignment;
	i+= LCD_PutStr(i, j, ATLAS_IN9, fnt7x10);
	i+=LCD_PutInt(i,j,ATLAS_IN9_GPIO, fnt7x10);


	i=firstADCAlignment;
	j+=15;
	i+= LCD_PutStr(i, j, ATLAS_AI1, fnt7x10);
	convertedFloat = 1000 * ATLAS_VREG_BOB;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
	i=secondADCAlignment;
	i+= LCD_PutStr(i, j, ATLAS_AI10, fnt7x10);
	convertedFloat = 1000 * ATLAS_SSC_CX;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, ATLAS_IN1, fnt7x10);
	i+=LCD_PutInt(i,j,ATLAS_WIFI_PRSNT, fnt7x10)+adjacentSpacing;
	i=secondInputGpioAlignment;
	i+= LCD_PutStr(i, j, ATLAS_IN10, fnt7x10);
	i+=LCD_PutInt(i,j,ATLAS_IN10_GPIO, fnt7x10);

	i=firstADCAlignment;
	j+=15;
	i+= LCD_PutStr(i, j, ATLAS_AI2, fnt7x10);
	convertedFloat = 1000 * ATLAS_S5A;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
	i=secondADCAlignment;
	i+= LCD_PutStr(i, j, ATLAS_AI11, fnt7x10);
	convertedFloat = 1000 * ATLAS_PHY_1P2;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, ATLAS_IN2, fnt7x10);
	i+=LCD_PutInt(i,j,ATLAS_WIGIG_PRSNT, fnt7x10)+adjacentSpacing;
	i=secondInputGpioAlignment;
	i+= LCD_PutStr(i, j, ATLAS_IN11, fnt7x10);
	i+=LCD_PutInt(i,j,ATLAS_IN11_GPIO, fnt7x10);

	i=firstADCAlignment;
	j+=15;
	i+= LCD_PutStr(i, j, ATLAS_AI3, fnt7x10);
	convertedFloat = 1000 * ATLAS_S6C;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
	i=secondADCAlignment;
	i+= LCD_PutStr(i, j, ATLAS_AI12, fnt7x10);
	convertedFloat = 1000 * ATLAS_CORE_PCIE;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, ATLAS_IN3, fnt7x10);
	LCD_PutInt(i,j,ATLAS_CODEC_PRSNT, fnt7x10);


	i=firstADCAlignment;
	j+=15;
	i+= LCD_PutStr(i, j, ATLAS_AI4, fnt7x10);
	convertedFloat = 1000 * ATLAS_S4E;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
	i=secondADCAlignment;
	i+= LCD_PutStr(i, j, ATLAS_AI13, fnt7x10);
	convertedFloat = 1000 * ATLAS_CORE_USB;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, ATLAS_IN4, fnt7x10);
	LCD_PutInt(i,j,ATLAS_RF_PRSNT, fnt7x10);

	i=firstADCAlignment;
	j+=15;
	i+= LCD_PutStr(i, j, ATLAS_AI5, fnt7x10);
	convertedFloat = 1000 * ATLAS_VDDMX;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
	i=secondADCAlignment;
	i+= LCD_PutStr(i, j, ATLAS_AI14, fnt7x10);
	convertedFloat = 1000 * ATLAS_S5E;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, ATLAS_IN5, fnt7x10);
	LCD_PutInt(i,j,ATLAS_IN5_GPIO, fnt7x10);

	i=firstADCAlignment;
	j+=15;
	i+= LCD_PutStr(i, j, ATLAS_AI6, fnt7x10);
	convertedFloat = 1000 * ATLAS_LPI_MX;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
	i=secondADCAlignment;
	i+= LCD_PutStr(i, j, ATLAS_AI15, fnt7x10);
	convertedFloat = 1000 * ATLAS_1P8;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, ATLAS_IN6, fnt7x10);
	LCD_PutInt(i,j,ATLAS_IN6_GPIO, fnt7x10);

	i=firstADCAlignment;
	j+=15;
	i+= LCD_PutStr(i, j, ATLAS_AI7, fnt7x10);
	convertedFloat = 1000 * ATLAS_VDDA_EBI;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, ATLAS_IN7, fnt7x10);
	LCD_PutInt(i,j,ATLAS_IN7_GPIO, fnt7x10);

	i=firstADCAlignment;
	j+=15;
	i+= LCD_PutStr(i, j, ATLAS_AI8, fnt7x10);
	convertedFloat = 1000 * ATLAS_VDD_CX;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, ATLAS_IN8, fnt7x10);
	LCD_PutInt(i,j,ATLAS_IN8_GPIO, fnt7x10);
	//horizontal divider
	j=65;
	LCD_FillRect(2, j-1, scr_width-2, j+1);
	//vertical divider
	i=230;
	LCD_FillRect(i-3, j, i+3, scr_height-2);
}

//prints the faults for the board under the status menu. Uses definitions from the Project .h file. Labels should be short (<10 char)
//and fault messages should be relatively short (<12 char)
void atlasStatusFaults(){
	int i=10;
	int j=95;
	int daughterCardAlignment=240;
	int faultVerticalSpacing = 15;
	int faultHorizontalSpacing=10;
	i+= LCD_PutStr(i, j, ATLAS_FAULT1_LBL, fnt7x10);
	if(errorLED.zionFault){
		LCD_PutStr(i, j, ATLAS_FAULT1_TRIGGER_MSG, fnt7x10);
	}
	else{
		LCD_PutStr(i, j, " Clear", fnt7x10);
	}
	i=daughterCardAlignment;
	i+= LCD_PutStr(i, j, "SOC: ", fnt7x10);
	if(ZION.SOC_EEPROM_Detected){
		LCD_PutStr(i,j,"Detected", fnt7x10);
	}
	else{
		LCD_PutStr(i,j,"Undetected", fnt7x10);
	}
	i=faultHorizontalSpacing;
	j+=faultVerticalSpacing;
	i+= LCD_PutStr(i, j, ATLAS_FAULT2_LBL, fnt7x10);
	if(errorLED.vsysPMIFault){
		LCD_PutStr(i, j, ATLAS_FAULT2_TRIGGER_MSG, fnt7x10);
	}
	else{
		LCD_PutStr(i, j, " Clear", fnt7x10);
	}
	i=daughterCardAlignment;
	i+= LCD_PutStr(i, j, "ASIC: ", fnt7x10);
	if(ZION.ASIC_EEPROM_Detected){
		LCD_PutStr(i,j,"Detected", fnt7x10);
	}
	else{
		LCD_PutStr(i,j,"Undetected", fnt7x10);
	}
	i=faultHorizontalSpacing;
	j+=faultVerticalSpacing;
	i+= LCD_PutStr(i, j, ATLAS_FAULT3_LBL, fnt7x10);
	if(errorLED.fault3){
		LCD_PutStr(i, j, ATLAS_FAULT3_TRIGGER_MSG, fnt7x10);
	}
	else{
		LCD_PutStr(i, j, " Clear", fnt7x10);
	}
	i=daughterCardAlignment;
	i+= LCD_PutStr(i, j, "Display: ", fnt7x10);
	if(ZION.DISPLAY_EEPROM_Detected){
		LCD_PutStr(i,j,"Detected", fnt7x10);
	}
	else{
		LCD_PutStr(i,j,"Undetected", fnt7x10);
	}

	i=faultHorizontalSpacing;
	j+=faultVerticalSpacing;
	i+= LCD_PutStr(i, j, ATLAS_FAULT4_LBL, fnt7x10);
	if(errorLED.fault4){
		LCD_PutStr(i, j, ATLAS_FAULT4_TRIGGER_MSG, fnt7x10);
	}
	else{
		LCD_PutStr(i, j, " Clear", fnt7x10);
	}

	i=faultHorizontalSpacing;
	j+=faultVerticalSpacing;
	i+= LCD_PutStr(i, j, ATLAS_FAULT5_LBL, fnt7x10);
	if(errorLED.fault5){
		LCD_PutStr(i, j, ATLAS_FAULT5_TRIGGER_MSG, fnt7x10);
	}
	else{
		LCD_PutStr(i, j, " Clear", fnt7x10);
	}

	i=faultHorizontalSpacing;
	j+=faultVerticalSpacing;
	i+= LCD_PutStr(i, j, ATLAS_FAULT6_LBL, fnt7x10);
	if(errorLED.fault6){
		LCD_PutStr(i, j, ATLAS_FAULT6_TRIGGER_MSG, fnt7x10);
	}
	else{
		LCD_PutStr(i, j, " Clear", fnt7x10);
	}

	i=faultHorizontalSpacing;
	j+=faultVerticalSpacing;
	i+= LCD_PutStr(i, j, ATLAS_FAULT7_LBL, fnt7x10);
	if(errorLED.fault7){
		LCD_PutStr(i, j, ATLAS_FAULT7_TRIGGER_MSG, fnt7x10);
	}
	else{
		LCD_PutStr(i, j, " Clear", fnt7x10);
	}

	i=faultHorizontalSpacing;
	j+=faultVerticalSpacing;
	i+= LCD_PutStr(i, j, ATLAS_FAULT8_LBL, fnt7x10);
	if(errorLED.fault8){
		LCD_PutStr(i, j, ATLAS_FAULT8_TRIGGER_MSG, fnt7x10);
	}
	else{
		LCD_PutStr(i, j, " Clear", fnt7x10);
	}

	i=faultHorizontalSpacing;
	j+=faultVerticalSpacing;
	i+= LCD_PutStr(i, j, ATLAS_FAULT9_LBL, fnt7x10);
	if(errorLED.fault9){
		LCD_PutStr(i, j, ATLAS_FAULT9_TRIGGER_MSG, fnt7x10);
	}
	else{
		LCD_PutStr(i, j, " Clear", fnt7x10);
	}


	//horizontal divider
	j=65;
	LCD_FillRect(2, j-1, scr_width-2, j+1);
	//vertical divider
	i=230;
	LCD_FillRect(i-3, j, i+3, scr_height-2);

}

//Boot button method for the bootButton Task. Defines what buttons/modes are available to the project board and what timings to follow.
//Two main modes -- system already turned on and in a mode, system not in a boot mode.

int atlasBootButtons(int pwrBtnReady){
	//only bother running if the power switch is enabled
	while((ATLAS_VSYS > VSYS_FLT)){
		//if we are in a boot mode
	  if(bootButtons.bootMode !=0){
		  bootButtons.modeClear=0;
		  //set the button that we need for our boot mode
		  if(bootButtons.btn1){ //DPAD UP
			  ATLAS_DPAD_UP_ON;
			  pwrBtnReady=1;
		  }
		  if(bootButtons.btn2){ //DPAD RIGHT
			  ATLAS_DPAD_RIGHT_ON;
			  pwrBtnReady=1;
		  }
		  if(bootButtons.btn3){ //DPAD LEFT
			  ATLAS_DPAD_LEFT_ON;
			  pwrBtnReady=1;
		  }
		  if(bootButtons.btn4){ //DPAD DOWN
			  ATLAS_DPAD_DOWN_ON;
			  pwrBtnReady=1;
		  }
		  if(bootButtons.btn5){
			  BTN5_ON;
			  pwrBtnReady=1;
		  }
		  if(bootButtons.edl_sw){
			  ATLAS_EDL_ON;
			  pwrBtnReady=1;
		  }
		  if(bootButtons.ex_sw){
			  EX_SW_ON;
			  pwrBtnReady=1;
		  }
		  //hold down the power button for ~13 seconds to cycle the chip
		  ATLAS_PWR_ON;
		  //setOutputGPIOState(outputGPIOs.odOut_0, OFF); //set the reset GPIO.
		  for(int x = 0; x<130;x++){
			  HAL_Delay(100);
		  }
		  //remove power
		  ATLAS_PWR_OFF;
		  //give the soc a second to fully shutdown
		  HAL_Delay(300);
		  //turn the soc back on
		  ATLAS_PWR_ON;
		  HAL_Delay(500);
		  ATLAS_PWR_OFF;
		  //setOutputGPIOState(outputGPIOs.odOut_0, ON); //turn off the reset GPIO
		  //wait four seconds while pressing the mode button for the soc to recognize the button press
		  if(pwrBtnReady){
			  for(int x = 0; x<40;x++){
				  HAL_Delay(100);
			  }
			  pwrBtnReady=0;
		  }
		  //turn off all our buttons
		  ATLAS_DPAD_UP_OFF;
		  ATLAS_DPAD_RIGHT_OFF;
		  ATLAS_DPAD_LEFT_OFF;
		  ATLAS_DPAD_DOWN_OFF;
		  BTN5_OFF;
		  ATLAS_EDL_OFF;
		  EX_SW_OFF;
		  //set our bootMode based off the button pressed
		  if(bootButtons.btn1){
			  bootButtons.bootMode= RECOVERY;
			  //errorLEDState[RECOVERY_LED]=1;
		  }
		  else if(bootButtons.btn2){
			  bootButtons.bootMode= MASS_STORAGE;
			 // errorLEDState[MASS_STORAGE_LED]=1;
		  }
		  else if(bootButtons.btn3){
			  bootButtons.bootMode= UEFI;
			  //errorLEDState[UEFI_LED]=1;
		  }
		  else if(bootButtons.edl_sw){
			  bootButtons.bootMode= EDL;
			  //errorLEDState[EDL_LED]=1;
		  }
		  else{
			  bootButtons.bootMode=STANDARD;
			  //errorLEDState[STANDARD_LED]=1;
		  }
		  //clear the bootButton variables and set the clear flag for the display menu.
		  bootButtons.btn0=0;
		  bootButtons.btn1=0;
		  bootButtons.btn2=0;
		  bootButtons.btn3=0;
		  bootButtons.btn4=0;
		  bootButtons.btn5=0;
		  bootButtons.edl_sw=0;
		  bootButtons.ex_sw=0;
		  bootButtons.modeClear=1;
		  bootButtons.bootModeSet=0;

	  }
	  else{
		  //if we aren't booted yet
		  bootButtons.modeClear=0;
		  //check if the power button or the pwrBTN flag is set.
		  if((bootButtons.btn0) || pwrBtnReady){ //power button
			  //if it is, toggle power for half a second
			  ATLAS_PWR_ON;
			  //timeTurnedOn = (HAL_GetTick());
			  //pwrBtnReady=0;
			  //pwrOn = 1;
			  HAL_Delay(500);
			  ATLAS_PWR_OFF;
			  //if pwrBtnReady is set, it means a 'alternate' mode is requested. This requires sitting on pressed button for ~4secs
			  if(pwrBtnReady){
				  for(int x = 0; x<40;x++){
						  HAL_Delay(100);
					  }
					  pwrBtnReady=0;
			  }
			  //pwrOn=0;
			  //timeTurnedOn=0;
			  //once things are finished, jot down the registered boot mode
			  if(bootButtons.btn0){
				  bootButtons.bootMode= STANDARD;
			  }
			  else if(bootButtons.btn1){
				  bootButtons.bootMode= RECOVERY;
			  }
			  else if(bootButtons.btn2){
				  bootButtons.bootMode= MASS_STORAGE;
			  }
			  else if(bootButtons.btn3){
				  bootButtons.bootMode= UEFI;
			  }
			  else if(bootButtons.edl_sw){
				  bootButtons.bootMode= EDL;
			  }
			  //and clear the btn states while setting the clear flag.
			  bootButtons.btn0=0;
			  bootButtons.btn1=0;
			  bootButtons.btn2=0;
			  bootButtons.btn3=0;
			  bootButtons.btn4=0;
			  bootButtons.btn5=0;
			  bootButtons.edl_sw=0;
			  bootButtons.ex_sw=0;
			  bootButtons.modeClear=1;
			  bootButtons.bootModeSet=0;
			  //osDelay(300);
		  }
		  //set the buttons based off the bootButton states. If boot has finished, the buttons will be turned off here.
		  if(bootButtons.btn1){ //DPAD UP
			  ATLAS_DPAD_UP_ON;
			  pwrBtnReady=1;
		  }
		  else if(!(bootButtons.btn1)){ //DPAD UP
			  ATLAS_DPAD_UP_OFF;
			  //osDelay(300);
		  }
		  if(bootButtons.btn2){ //DPAD RIGHT
			  ATLAS_DPAD_RIGHT_ON;
			  pwrBtnReady=1;
		  }
		  else if(!(bootButtons.btn2)){ //DPAD RIGHT
			  ATLAS_DPAD_RIGHT_OFF;
			  //osDelay(300);
		  }
		  if(bootButtons.btn3){ //DPAD LEFT
			  ATLAS_DPAD_LEFT_ON;
			  pwrBtnReady=1;
		  }
		  else if(!(bootButtons.btn3)){ //DPAD LEFT
			  ATLAS_DPAD_LEFT_OFF;
			  //osDelay(300);
		  }
		  if(bootButtons.btn4){
			  ATLAS_DPAD_DOWN_ON;
			  pwrBtnReady=1;
		  }
		  else if(!(bootButtons.btn4)){
			  ATLAS_DPAD_DOWN_OFF;

		  }
		  if(bootButtons.btn5){
			  BTN5_ON;
			  pwrBtnReady=1;
		  }
		  else if(!(bootButtons.btn5)){
			  BTN5_OFF;
			  //osDelay(300);
		  }
		  if(bootButtons.edl_sw){
			  ATLAS_EDL_ON;
			  pwrBtnReady=1;
		  }
		  else if(!(bootButtons.edl_sw)){
			  ATLAS_EDL_OFF;
			  //osDelay(300);
		  }
		  if(bootButtons.ex_sw){
			  EX_SW_ON;
			  pwrBtnReady=1;
		  }
		  else if(!(bootButtons.ex_sw)){
			  EX_SW_OFF;
			  //osDelay(300);
		  }
	  }
	  break;
	}
	return pwrBtnReady;
}

void atlasErrorLEDs(){
	float *presentADCValues;
	// An array of voltage rails that are monitored for faults.  Each element maps to the apporpriate ADC channel for monitoring
		// PLATFORM TEMPLATE: edit this array to include the voltages that you would like to monitor for faults.  The names are defined in main.h
		uint8_t monitor_rails[] = {VSYS, VREG_BOB, VREG_S5A, VREG_S6C};

		// An array of falling edge fault thresholds for the voltage rails that are monitored for faults.  Size of the array and index for each fault should match the voltage name in monitor_rails[].
		double monitor_fault_thresholds[] = {VSYS_FLT, VREG_BOB_FLT, VREG_S5A_FLT, VREG_S6C_FLT};

		// An array of platform gpio inputs that are monitored for faults.  Each element maps to the appropriate STM GPIO input for monitoring
		// PLATFORM TEMPLATE: edit this array to include the voltages that you would like to monitor for faults.  The names are defined in main.h
		uint8_t monitor_gpio[] = {SOC_IN6, SOC_IN7, SOC_IN8, SOC_IN11};

		// An array of logic fault thresholds for the GPIO input rails that are monitored for faults.  The fault thresholds should match the mapping used in monitor_gpio[].
		uint8_t gpio_thresholds[] = {SOC_IN6_FLT, SOC_IN7_FLT, SOC_IN8_FLT, SOC_IN11_FLT};

		uint8_t * errorLEDptr;
		  uint8_t R = false;
		  uint8_t G = false;
		  uint8_t B = false;
		  // Check that the ADC are available, and if they are, retrieve the last recorded ADC outputs.
		  if(adcStates.adcBank1Finished && adcStates.adcBank2Finished && adcStates.adcBank3Finished){
			  presentADCValues = getADCValues();
		  }

		  // Iterate through all the ADC channels that are monitored for faults
		  for (uint8_t rail = 0; rail < sizeof(monitor_rails)/sizeof(monitor_rails[0]); rail++)
		  {
			  // This switch statement maps the appropriate errorLED struct fault flag to the errorLEDptr so that we can clear or set it.
			  // To add more faults simply add more case statements.
			  // PLATFORM TEMPLATE: edit the switch statement labels to match the entries in monitor_rails[] array.
			  switch (monitor_rails[rail])
			  {
			  case VSYS:
				  errorLEDptr = &errorLED.vsysPMIFault;
				  break;
			  case VREG_BOB:
				  errorLEDptr = &errorLED.fault3;
				  break;
			  case VREG_S5A:
				  errorLEDptr = &errorLED.fault4;
				  break;
			  case VREG_S6C:
				  errorLEDptr = &errorLED.fault5;
				  break;
			  default:
				  break;
			  }
			  // If the voltage level is above the low fault threshold then clear the fault flag.
			  if (*(presentADCValues+monitor_rails[rail]) > monitor_fault_thresholds[rail])
			  {
				  *errorLEDptr = false;
			  }
			  else
			  {
				  *errorLEDptr = true;
			  }
		  }

		  // Check GPIO inputs for faults. Iterate through the inputs that are supposed to be monitored for faults.
		  for (uint8_t input = 0; input < sizeof(monitor_gpio)/sizeof(monitor_gpio[0]); input++)
		  {
			  // This switch statement maps the appropriate errorLED struct fault flag to the errorLEDptr so that we can clear or set it.
			  // To add more faults simply add more case statements.  Remember there is a maximum number of faults that can be displayed.
			  // PLATFORM TEMPLATE: edit the switch statement labels to match the entries in monitor_gpio[] array.
			  switch (monitor_gpio[input])
			  {
			  case SOC_IN0:
				  errorLEDptr = &errorLED.fault6;
				  break;
			  case SOC_IN3:
				  errorLEDptr = &errorLED.fault7;
				  break;
			  case SOC_IN8:
				  errorLEDptr = &errorLED.fault8;
				  break;
			  case SOC_IN11:
				  errorLEDptr = &errorLED.fault9;
				  break;
			  default:
				  break;
			  }
			  // If the voltage level is above the low fault threshold then clear the fault flag.
			  if (gpioInputBuf[monitor_gpio[input]] == gpio_thresholds[input])
			  {
				  *errorLEDptr = true;
			  }
			  else
			  {
				  *errorLEDptr = false;
			  }
		  }

//		  if(*(presentADCValues+Adc.adc0) > VSYS_FLT){
//			  errorLED.vsysPMIFault=false;
//		  }
//		  else{
//			  errorLED.vsysPMIFault=true;
//		  }
		  if((!ZION.SOC_EEPROM_Detected && ZION.zionFinished) || (ZION.SOC_BoardFab <0)){
			  errorLED.zionFault=true;
		  }
		  else{
			  errorLED.zionFault=false;
		  }
		  //HAL_I2C_IsDeviceReady(&hi2c1, SOC_ADDRESS, 2, 100)
		  int i2cCheck=writeI2CRegister(LED.address, 0xf0, 0x00,1,LED.i2cBank);

		  //only allow the error led write commands if the led driver responds.
		  if(i2cCheck == HAL_OK)
		  {
			  errorLED.ledDriver=false;

			  switch(bootButtons.bootMode)
			  {
				case UNINITIALIZED:
					errorLED.standard_boot=false;
					errorLED.uefi_boot=false;
					errorLED.edl_boot=false;
					errorLED.boot_fault=false;
					R = false;
					break;
				case STANDARD:
					errorLED.standard_boot=true;
					errorLED.uefi_boot=false;
					errorLED.edl_boot=false;
					errorLED.boot_fault=false;
					G = true;
					break;
				case UEFI:
					errorLED.standard_boot=false;
					errorLED.uefi_boot=true;
					errorLED.edl_boot=false;
					errorLED.boot_fault=false;
					G = true;
					B = true;
					break;
				case EDL:
					errorLED.standard_boot=false;
					errorLED.uefi_boot=false;
					errorLED.edl_boot=true;
					errorLED.boot_fault=false;
					B = true;
					break;
				case MASS_STORAGE:
					errorLED.standard_boot=true;
					errorLED.uefi_boot=false;
					errorLED.edl_boot=true;
					errorLED.boot_fault=false;
					R = true;
					B = true;
					break;
				case RECOVERY:
					errorLED.standard_boot=false;
					errorLED.uefi_boot=true;
					errorLED.edl_boot=true;
					errorLED.boot_fault=false;
					R = true;
					G = true;
					break;
			  }
			  setRGBLED(R,G,B);
			  setErrorLED(ZION_FAULT,errorLED.zionFault);
			  HAL_Delay(20);
			  setErrorLED(VSYSPMI_FAULT, errorLED.vsysPMIFault);
			  HAL_Delay(20);
			  setErrorLED(FAULT3,errorLED.fault3);
			  HAL_Delay(20);
			  setErrorLED(FAULT4,errorLED.fault4);
			  HAL_Delay(20);
			  setErrorLED(FAULT5,errorLED.fault5);
			  HAL_Delay(20);
			  setErrorLED(FAULT6,errorLED.fault6);
			  HAL_Delay(20);
			  setErrorLED(FAULT7,errorLED.fault7);
			  HAL_Delay(20);
			  setErrorLED(FAULT8,errorLED.fault8);
			  HAL_Delay(20);
			  setErrorLED(FAULT9,errorLED.fault9);
		  }
		  else{
			  errorLED.ledDriver = true;
		  }
}

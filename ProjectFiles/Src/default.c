/*
 * default.c
 *
 *  Created on: Mar 15, 2021
 *      Author: auphilli
 */

#include "default.h"

int timeNow = 0;


void defaultMainMenuFaultLedLabels(){
	int i,j;
	int ledFaultAlign=scr_width-90;
	int verticalSpacing=22;
	int horizontalSpacing=20;
	i  = ledFaultAlign;
	j  = 3;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, DEFAULT_LED1_FAULT_LBL, fnt7x10);
	j+=verticalSpacing;
	i  = ledFaultAlign;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, DEFAULT_LED2_FAULT_LBL, fnt7x10);
	i  = ledFaultAlign;
	j  += verticalSpacing;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, DEFAULT_LED3_FAULT_LBL, fnt7x10);
	j+=verticalSpacing;
	i  = ledFaultAlign;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, DEFAULT_LED4_FAULT_LBL, fnt7x10);
	i  = ledFaultAlign;
	j  += verticalSpacing;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, DEFAULT_LED5_FAULT_LBL, fnt7x10);
	j+=verticalSpacing;
	i  = ledFaultAlign;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, DEFAULT_LED6_FAULT_LBL, fnt7x10);
	i  = ledFaultAlign;
	j  +=verticalSpacing;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, DEFAULT_LED7_FAULT_LBL, fnt7x10);
	j+=verticalSpacing;
	i  = ledFaultAlign;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, DEFAULT_LED8_FAULT_LBL, fnt7x10);
	i  = ledFaultAlign;
	j  +=verticalSpacing;
	LCD_FillRect(i, j - 2, i + 12, j + 8);
	i+=horizontalSpacing;
	LCD_PutStr(i, j, DEFAULT_LED9_FAULT_LBL, fnt7x10);
}
void defaultMainMenuBootModes(int i, int j){
	switch(bootButtons.bootMode){
	case UNINITIALIZED:
		LCD_PutStr(i, j, "OFF", fnt7x10);
		break;
	case STANDARD:
		LCD_PutStr(i, j, "OS", fnt7x10);
		break;
	default:
		LCD_PutStr(i, j, "OFF", fnt7x10);
		break;
	}
}

int defaultBootMenuBootModes(int indicator, int previousMenu, int menu, int button,int eepromRead){
	int j=45;
	int i=5;
	int indentAlignment=50;
	if(eepromRead){
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
					timeNow = (HAL_GetTick()/1000);
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
					timeNow = (HAL_GetTick()/1000);
				}
				break;
			}
			}
		}
		else{
			i = 230;
			j = 120;
			int timeLeft=0;
			if(bootButtons.bootMode==0){
				timeLeft = (timeNow+2)- (HAL_GetTick()/1000);
			}
			else{
				timeLeft = (timeNow+15)- (HAL_GetTick()/1000);
			}
			i+=LCD_PutIntF(i, j, timeLeft, 0, fnt_dig_big);;
			LCD_PutStr(i, j, " SECS LEFT", fnt7x10);
			switch(setIndicator){
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
			default:
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
	return j;
}

void defaultSystemInfoSoc(int i, int j){
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
void defaultSystemInfoAsic(int i, int j){
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
void defaultSystemInfoDisplay(int i, int j){
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
int defaultHeaderFaults(int i, int j){
	int x = i;
	int y = j;
	if(errorLED.vsysPMIFault){
		x += LCD_PutStr(x, y, DEFAULT_HEADER1_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.zionFault){
		x += LCD_PutStr(x, y, DEFAULT_HEADER2_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.fault3){
		x += LCD_PutStr(x, y, DEFAULT_HEADER3_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.fault4){
		x += LCD_PutStr(x, y, DEFAULT_HEADER4_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.fault5){
		x += LCD_PutStr(x, y, DEFAULT_HEADER5_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.fault6){
		x += LCD_PutStr(x, y, DEFAULT_HEADER6_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.fault7){
		x += LCD_PutStr(x, y, DEFAULT_HEADER7_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.fault8){
		x += LCD_PutStr(x, y, DEFAULT_HEADER8_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.fault9){
		x += LCD_PutStr(x, y, DEFAULT_HEADER9_FAULT_LBL, fnt7x10);
	}
	else if(errorLED.ledDriver){
		x += LCD_PutStr(x, y, "LED DVR", fnt7x10);
	}
	else{
		x += LCD_PutStr(x, y, "NONE!", fnt7x10);
	}
	return x;
}

void defaultStatusADCsAndGPIOs(){
	int inputGpioAlignment=245;
	int adjacentSpacing = 20;
	int i=10;
	int j=95;
	int convertedFloat;
	i+= LCD_PutStr(i, j, DEFAULT_AI0, fnt7x10);
	convertedFloat = 1000 * DEFAULT_VSYS;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
	i+= LCD_PutStr(i, j, DEFAULT_AI9, fnt7x10);
	convertedFloat = 1000 * DEFAULT_AI9_VAL;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, DEFAULT_IN0, fnt7x10);
	i+=LCD_PutInt(i,j,DEFAULT_IN0_GPIO, fnt7x10)+adjacentSpacing;
	i+= LCD_PutStr(i, j, DEFAULT_IN9, fnt7x10);
	i+=LCD_PutInt(i,j,DEFAULT_IN9_GPIO, fnt7x10);


	i=10;
	j+=15;
	i+= LCD_PutStr(i, j, DEFAULT_AI1, fnt7x10);
	convertedFloat = 1000 * DEFAULT_AI1_VAL;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
	i+= LCD_PutStr(i, j, DEFAULT_AI10, fnt7x10);
	convertedFloat = 1000 * DEFAULT_AI10_VAL;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, DEFAULT_IN1, fnt7x10);
	i+=LCD_PutInt(i,j,DEFAULT_IN1_GPIO, fnt7x10)+adjacentSpacing;
	i+= LCD_PutStr(i, j, DEFAULT_IN10, fnt7x10);
	i+=LCD_PutInt(i,j,DEFAULT_IN10_GPIO, fnt7x10);

	i=10;
	j+=15;
	i+= LCD_PutStr(i, j, DEFAULT_AI2, fnt7x10);
	convertedFloat = 1000 * DEFAULT_AI2_VAL;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
	i+= LCD_PutStr(i, j, DEFAULT_AI11, fnt7x10);
	convertedFloat = 1000 * DEFAULT_AI11_VAL;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, DEFAULT_IN2, fnt7x10);
	i+=LCD_PutInt(i,j,DEFAULT_IN2_GPIO, fnt7x10)+adjacentSpacing;
	i+= LCD_PutStr(i, j, DEFAULT_IN11, fnt7x10);
	i+=LCD_PutInt(i,j,DEFAULT_IN11_GPIO, fnt7x10);

	i=10;
	j+=15;
	i+= LCD_PutStr(i, j, DEFAULT_AI3, fnt7x10);
	convertedFloat = 1000 * DEFAULT_AI3_VAL;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
	i+= LCD_PutStr(i, j, DEFAULT_AI12, fnt7x10);
	convertedFloat = 1000 * DEFAULT_AI12_VAL;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, DEFAULT_IN3, fnt7x10);
	LCD_PutInt(i,j,DEFAULT_IN3_GPIO, fnt7x10);


	i=10;
	j+=15;
	i+= LCD_PutStr(i, j, DEFAULT_AI4, fnt7x10);
	convertedFloat = 1000 * DEFAULT_AI4_VAL;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
	i+= LCD_PutStr(i, j, DEFAULT_AI13, fnt7x10);
	convertedFloat = 1000 * DEFAULT_AI13_VAL;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, DEFAULT_IN4, fnt7x10);
	LCD_PutInt(i,j,DEFAULT_IN3_GPIO, fnt7x10);

	i=10;
	j+=15;
	i+= LCD_PutStr(i, j, DEFAULT_AI5, fnt7x10);
	convertedFloat = 1000 * DEFAULT_AI5_VAL;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
	i+= LCD_PutStr(i, j, DEFAULT_AI14, fnt7x10);
	convertedFloat = 1000 * DEFAULT_AI14_VAL;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, DEFAULT_IN5, fnt7x10);
	LCD_PutInt(i,j,DEFAULT_IN5_GPIO, fnt7x10);

	i=10;
	j+=15;
	i+= LCD_PutStr(i, j, DEFAULT_AI6, fnt7x10);
	convertedFloat = 1000 * DEFAULT_AI6_VAL;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10)+adjacentSpacing;
	i+= LCD_PutStr(i, j, DEFAULT_AI15, fnt7x10);
	convertedFloat = 1000 * DEFAULT_AI15_VAL;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, DEFAULT_IN6, fnt7x10);
	LCD_PutInt(i,j,DEFAULT_IN6_GPIO, fnt7x10);

	i=10;
	j+=15;
	i+= LCD_PutStr(i, j, DEFAULT_AI7, fnt7x10);
	convertedFloat = 1000 * DEFAULT_AI7_VAL;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, DEFAULT_IN7, fnt7x10);
	LCD_PutInt(i,j,DEFAULT_IN7_GPIO, fnt7x10);

	i=10;
	j+=15;
	i+= LCD_PutStr(i, j, DEFAULT_AI8, fnt7x10);
	convertedFloat = 1000 * DEFAULT_AI8_VAL;
	i+=LCD_PutIntF(i, j, convertedFloat, 3, fnt7x10);
	i=inputGpioAlignment;
	i+= LCD_PutStr(i, j, DEFAULT_IN8, fnt7x10);
	LCD_PutInt(i,j,DEFAULT_IN8_GPIO, fnt7x10);
	//horizontal divider
	j=65;
	LCD_FillRect(2, j-1, scr_width-2, j+1);
	//vertical divider
	i=230;
	LCD_FillRect(i-3, j, i+3, scr_height-2);
}

void defaultStatusFaults(){
	int i=10;
	int j=95;
	int daughterCardAlignment=240;
	int faultVerticalSpacing = 15;
	int faultHorizontalSpacing=10;
	i+= LCD_PutStr(i, j, DEFAULT_FAULT1_LBL, fnt7x10);
	if(errorLED.zionFault){
		LCD_PutStr(i, j, DEFAULT_FAULT1_TRIGGER_MSG, fnt7x10);
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
	i+= LCD_PutStr(i, j, DEFAULT_FAULT2_LBL, fnt7x10);
	if(errorLED.vsysPMIFault){
		LCD_PutStr(i, j, DEFAULT_FAULT2_TRIGGER_MSG, fnt7x10);
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
	i+= LCD_PutStr(i, j, DEFAULT_FAULT3_LBL, fnt7x10);
	if(errorLED.fault3){
		LCD_PutStr(i, j, DEFAULT_FAULT3_TRIGGER_MSG, fnt7x10);
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
	i+= LCD_PutStr(i, j, DEFAULT_FAULT4_LBL, fnt7x10);
	if(errorLED.fault4){
		LCD_PutStr(i, j, DEFAULT_FAULT4_TRIGGER_MSG, fnt7x10);
	}
	else{
		LCD_PutStr(i, j, " Clear", fnt7x10);
	}

	i=faultHorizontalSpacing;
	j+=faultVerticalSpacing;
	i+= LCD_PutStr(i, j, DEFAULT_FAULT5_LBL, fnt7x10);
	if(errorLED.fault5){
		LCD_PutStr(i, j, DEFAULT_FAULT5_TRIGGER_MSG, fnt7x10);
	}
	else{
		LCD_PutStr(i, j, " Clear", fnt7x10);
	}

	i=faultHorizontalSpacing;
	j+=faultVerticalSpacing;
	i+= LCD_PutStr(i, j, DEFAULT_FAULT6_LBL, fnt7x10);
	if(errorLED.fault6){
		LCD_PutStr(i, j, DEFAULT_FAULT6_TRIGGER_MSG, fnt7x10);
	}
	else{
		LCD_PutStr(i, j, " Clear", fnt7x10);
	}

	i=faultHorizontalSpacing;
	j+=faultVerticalSpacing;
	i+= LCD_PutStr(i, j, DEFAULT_FAULT7_LBL, fnt7x10);
	if(errorLED.fault7){
		LCD_PutStr(i, j, DEFAULT_FAULT7_TRIGGER_MSG, fnt7x10);
	}
	else{
		LCD_PutStr(i, j, " Clear", fnt7x10);
	}

	i=faultHorizontalSpacing;
	j+=faultVerticalSpacing;
	i+= LCD_PutStr(i, j, DEFAULT_FAULT8_LBL, fnt7x10);
	if(errorLED.fault8){
		LCD_PutStr(i, j, DEFAULT_FAULT8_TRIGGER_MSG, fnt7x10);
	}
	else{
		LCD_PutStr(i, j, "Clear", fnt7x10);
	}

	i=faultHorizontalSpacing;
	j+=faultVerticalSpacing;
	i+= LCD_PutStr(i, j, DEFAULT_FAULT9_LBL, fnt7x10);
	if(errorLED.fault9){
		LCD_PutStr(i, j, DEFAULT_FAULT9_TRIGGER_MSG, fnt7x10);
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

int defaultBootButtons(int pwrBtnReady){
	while((DEFAULT_VSYS > VSYS_FLT)){
	  if(bootButtons.bootMode !=0){
		  bootButtons.modeClear=0;
		  DEFAULT_PWR_ON;
		  //using OS delay in a method leads to getting lost in the ether. Swapping to hal_delay but also breaking down the delays
		  //so that the while loop can end somewhat succiently
		  for(int x = 0; x<130;x++){
			  HAL_Delay(100);
		  }
		  DEFAULT_PWR_OFF;
		  HAL_Delay(300);
		  DEFAULT_PWR_ON;
		  HAL_Delay(500);
		  DEFAULT_PWR_OFF;

		  BTN1_OFF;
		  BTN2_OFF;
		  BTN3_OFF;
		  BTN4_OFF;
		  BTN5_OFF;
		  EDL_SW_OFF;
		  EX_SW_OFF;

		  bootButtons.bootMode=STANDARD;
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
		  bootButtons.modeClear=0;
		  if((bootButtons.btn0) || pwrBtnReady){ //power button
			  DEFAULT_PWR_ON;
			  //pwrBtnReady=0;
			  //pwrOn = 1;
			  HAL_Delay(500);
			  DEFAULT_PWR_OFF;
			  //pwrOn=0;
			  bootButtons.bootMode= STANDARD;
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
		  if(!(bootButtons.btn1)){
			  BTN1_OFF;
			  //osDelay(300);
		  }
		  if(!(bootButtons.btn2)){
			  BTN2_OFF;
			  //osDelay(300);
		  }
		  if(!(bootButtons.btn3)){
			  BTN3_OFF;
			  //osDelay(300);
		  }
		  if(!(bootButtons.btn4)){
			  BTN4_OFF;

		  }
		  if(!(bootButtons.btn5)){
			  BTN5_OFF;
			  //osDelay(300);
		  }
		  if(!(bootButtons.edl_sw)){
			  EDL_SW_OFF;
			  //osDelay(300);
		  }
		  if(!(bootButtons.ex_sw)){
			  EX_SW_OFF;
			  //osDelay(300);
		  }
	  }
	  break;
	}
	  return pwrBtnReady;
}

void defaultErrorLEDs(){
	float *presentADCValues;
	// An array of voltage rails that are monitored for faults.  Each element maps to the apporpriate ADC channel for monitoring
		// PLATFORM TEMPLATE: edit this array to include the voltages that you would like to monitor for faults.  The names are defined in main.h

		uint8_t monitor_rails[] = {VSYS};

		// An array of falling edge fault thresholds for the voltage rails that are monitored for faults.  Size of the array and index for each fault should match the voltage name in monitor_rails[].
		double monitor_fault_thresholds[] = {VSYS_FLT};

		// An array of platform gpio inputs that are monitored for faults.  Each element maps to the appropriate STM GPIO input for monitoring
		// PLATFORM TEMPLATE: edit this array to include the voltages that you would like to monitor for faults.  The names are defined in main.h
		uint8_t monitor_gpio[] = {};

		// An array of logic fault thresholds for the GPIO input rails that are monitored for faults.  The fault thresholds should match the mapping used in monitor_gpio[].
		uint8_t gpio_thresholds[] = {};

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

		  //if(DEFAULT_VSYS > VSYS_FLT){
		//	  errorLED.vsysPMIFault=false;
		  //}
		  //else{
		//	  errorLED.vsysPMIFault=true;
		 // }
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
				default:
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

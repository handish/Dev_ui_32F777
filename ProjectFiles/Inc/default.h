/*
 * default.h
 *
 *  Created on: Mar 15, 2021
 *      Author: auphilli
 */

#ifndef INC_DEFAULT_H_
#define INC_DEFAULT_H_

#include "main.h"
#include "ls027b7dh01.h"

//fault names used for the status menu.
#define DEFAULT_FAULT1_LBL			"ZION_FLT:"
#define DEFAULT_FAULT2_LBL			"VSYS_PMI:"
#define DEFAULT_FAULT3_LBL			"Fault3:"
#define DEFAULT_FAULT4_LBL			"Fault4:"
#define DEFAULT_FAULT5_LBL			"Fault5:"
#define DEFAULT_FAULT6_LBL			"Fault6:"
#define DEFAULT_FAULT7_LBL			"Fault7:"
#define DEFAULT_FAULT8_LBL			"Fault8:"
#define DEFAULT_FAULT9_LBL			"Fault9:"

//Fault descriptions used on the status menu.
#define DEFAULT_FAULT1_TRIGGER_MSG			" SOC ZION ERROR"
#define DEFAULT_FAULT2_TRIGGER_MSG			" VSYS PMI LOW"
#define DEFAULT_FAULT3_TRIGGER_MSG			" FAULT 3 Triggered"
#define DEFAULT_FAULT4_TRIGGER_MSG			" FAULT 4 Triggered"
#define DEFAULT_FAULT5_TRIGGER_MSG			" FAULT 5 Triggered"
#define DEFAULT_FAULT6_TRIGGER_MSG			" FAULT 6 Triggered"
#define DEFAULT_FAULT7_TRIGGER_MSG			" FAULT 7 Triggered"
#define DEFAULT_FAULT8_TRIGGER_MSG			" FAULT 8 Triggered"
#define DEFAULT_FAULT9_TRIGGER_MSG			" FAULT 9 Triggered"

//Labels for header Fault Descriptions.
#define DEFAULT_HEADER2_FAULT_LBL			"ZION"
#define DEFAULT_HEADER1_FAULT_LBL			"VSYS"
#define DEFAULT_HEADER3_FAULT_LBL			"FAULT3"
#define DEFAULT_HEADER4_FAULT_LBL			"FAULT4"
#define DEFAULT_HEADER5_FAULT_LBL			"FAULT5"
#define DEFAULT_HEADER6_FAULT_LBL			"FAULT6"
#define DEFAULT_HEADER7_FAULT_LBL			"FAULT7"
#define DEFAULT_HEADER8_FAULT_LBL			"FAULT8"
#define DEFAULT_HEADER9_FAULT_LBL			"FAULT9"

//Labels for main page LED Fault Descriptions.
#define DEFAULT_LED1_FAULT_LBL			"ZION_FLT"
#define DEFAULT_LED2_FAULT_LBL			"VSYS_FLT"
#define DEFAULT_LED3_FAULT_LBL			"FAULT3"
#define DEFAULT_LED4_FAULT_LBL			"FAULT4"
#define DEFAULT_LED5_FAULT_LBL			"FAULT5"
#define DEFAULT_LED6_FAULT_LBL			"FAULT6"
#define DEFAULT_LED7_FAULT_LBL			"FAULT7"
#define DEFAULT_LED8_FAULT_LBL			"FAULT8"
#define DEFAULT_LED9_FAULT_LBL			"FAULT9"

//default BOOT MODE States
#define DEFAULT_STD_MODE				"OS"
#define DEFAULT_OFF_MODE				"OFF"





// Labels for AI VOLTAGES page.
// PLATFORM TEMPLATE: Rename these defines names to voltage rails that will be monitored by the DevUI ADCs for your platform.
#define DEFAULT_AI0					"AI0:"
#define DEFAULT_AI1					"AI1:"
#define DEFAULT_AI2					"AI2:"
#define DEFAULT_AI3					"AI3:"
#define DEFAULT_AI4					"AI4:"
#define DEFAULT_AI5					"AI5:"
#define DEFAULT_AI6					"AI6:"
#define DEFAULT_AI7					"AI7:"
#define DEFAULT_AI8					"AI8:"
#define DEFAULT_AI9					"AI9:"
#define DEFAULT_AI10				"AI10:"
#define DEFAULT_AI11				"AI11:"
#define DEFAULT_AI12				"AI12:"
#define DEFAULT_AI13				"AI13:"
#define DEFAULT_AI14				"AI14:"
#define DEFAULT_AI15				"AI15:"

//external globals
extern struct adcState adcStates;
extern uint8_t gpioInputBuf[12];
extern struct zion ZION;
extern struct bootModeButtons bootButtons;
extern struct errorLEDs errorLED;
extern uint8_t errorLEDState[12];
extern int setIndicator;

//define the actual AI values.
#define DEFAULT_VSYS							displayAdcValues[Adc.adc0]
#define DEFAULT_AI1_VAL							displayAdcValues[Adc.adc1]
#define DEFAULT_AI2_VAL							displayAdcValues[Adc.adc2]
#define DEFAULT_AI3_VAL							displayAdcValues[Adc.adc3]
#define DEFAULT_AI4_VAL							displayAdcValues[Adc.adc4]
#define DEFAULT_AI5_VAL							displayAdcValues[Adc.adc5]
#define DEFAULT_AI6_VAL							displayAdcValues[Adc.adc6]
#define DEFAULT_AI7_VAL							displayAdcValues[Adc.adc7]
#define DEFAULT_AI8_VAL							displayAdcValues[Adc.adc8]
#define DEFAULT_AI9_VAL							displayAdcValues[Adc.adc9]
#define DEFAULT_AI10_VAL						displayAdcValues[Adc.adc10]
#define DEFAULT_AI11_VAL						displayAdcValues[Adc.adc11]
#define DEFAULT_AI12_VAL						displayAdcValues[Adc.adc12]
#define DEFAULT_AI13_VAL						displayAdcValues[Adc.adc13]
#define DEFAULT_AI14_VAL						displayAdcValues[Adc.adc14]
#define DEFAULT_AI15_VAL						displayAdcValues[Adc.adc15]

//define the GPIO INPUT names. Used on the Status page.
#define DEFAULT_IN0					"IN0: "
#define DEFAULT_IN1					"IN1: "
#define DEFAULT_IN2					"IN2: "
#define DEFAULT_IN3					"IN3: "
#define DEFAULT_IN4					"IN4: "
#define DEFAULT_IN5					"IN5: "
#define DEFAULT_IN6					"IN6: "
#define DEFAULT_IN7					"IN7: "
#define DEFAULT_IN8					"IN8: "
#define DEFAULT_IN9					"IN9: "
#define DEFAULT_IN10				"IN10: "
#define DEFAULT_IN11				"IN11: "


//define the GPIO inputs themselves.
#define DEFAULT_IN0_GPIO			gpioInputBuf[inputGPIOs.input0]
#define DEFAULT_IN1_GPIO			gpioInputBuf[inputGPIOs.input1]
#define DEFAULT_IN2_GPIO			gpioInputBuf[inputGPIOs.input2]
#define DEFAULT_IN3_GPIO			gpioInputBuf[inputGPIOs.input3]
#define DEFAULT_IN4_GPIO			gpioInputBuf[inputGPIOs.input4]
#define DEFAULT_IN5_GPIO			gpioInputBuf[inputGPIOs.input5]
#define DEFAULT_IN6_GPIO			gpioInputBuf[inputGPIOs.input6]
#define DEFAULT_IN7_GPIO			gpioInputBuf[inputGPIOs.input7]
#define DEFAULT_IN8_GPIO			gpioInputBuf[inputGPIOs.input8]
#define DEFAULT_IN9_GPIO			gpioInputBuf[inputGPIOs.input9]
#define DEFAULT_IN10_GPIO			gpioInputBuf[inputGPIOs.input10]
#define DEFAULT_IN11_GPIO			gpioInputBuf[inputGPIOs.input11]

//define the button usages we need for default mode.
#define DEFAULT_PWR_ON 			BTN0_ON
#define DEFAULT_PWR_OFF			BTN0_OFF

void defaultMainMenuFaultLedLabels();
void defaultMainMenuBootModes(int i, int j);
int defaultBootMenuBootModes(int indicator, int previousMenu, int menu, int button,int eepromRead);
void defaultSystemInfoSoc(int i, int j);
void defaultSystemInfoAsic(int i, int j);
void defaultSystemInfoDisplay(int i, int j);
int defaultHeaderFaults(int i, int j);
void defaultStatusADCsAndGPIOs();
void defaultStatusFaults();
int defaultBootButtons(int pwrBtnReady);
void defaultErrorLEDs();


#endif /* INC_DEFAULT_H_ */

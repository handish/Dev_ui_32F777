/*
 * menu.h
 *
 *  Created on: Feb 18, 2021
 *      Author: auphilli
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define FIRST 			1
#define SECOND 			2
#define THIRD 			3
#define FOURTH			4
#define FIFTH			5

#define ATLAS			1

// Labels for FAULT page.
// PLATFORM TEMPLATE: Rename these defines names to FAULTS that will be monitored for your platform.
#define FAULT1_LBL			"ZION_FLT:"
#define FAULT2_LBL			"VSYS_PMI:"
#define FAULT3_LBL			"Fault3:"
#define FAULT4_LBL			"Fault4:"
#define FAULT5_LBL			"Fault5:"
#define FAULT6_LBL			"Fault6:"
#define FAULT7_LBL			"Fault7:"
#define FAULT8_LBL			"Fault8:"
#define FAULT9_LBL			"Fault9:"

// Labels for AI VOLTAGES page.
// PLATFORM TEMPLATE: Rename these defines names to voltage rails that will be monitored by the DevUI ADCs for your platform.
#define AI0					"VSYS:"
#define AI1					"VREG_BOB:"
#define AI2					"S5A:"
#define AI3					"S6C:"
#define AI4					"S4E:"
#define AI5					"VDDMX:"
#define AI6					"LPI_MX:"
#define AI7					"VDDA_EBI:"
#define AI8					"VDD_CX:"
#define AI9					"VDD_MM:"
#define AI10				"SSC_CX:"
#define AI11				"PHY_1P2:"
#define AI12				"CORE_PCIE:"
#define AI13				"CORE_USB:"
#define AI14				"S5E:"
#define AI15				"1P8:"

extern uint8_t adcRestart[3];
extern uint8_t gpioInputBuf[12];
extern struct zion ZION;
extern struct bootModeButtons bootButtons;
extern struct errorLEDs errorLED;
extern uint8_t errorLEDState[12];

void initializeDisplay();
void drawMainMenu(int indicator);
void drawStatusMenu(int indicator);
void drawSystemInfoMenu(int indicator);
void drawBootMenu(int indicator, uint8_t button, int menu);
void drawMenuHeader();
void getLatestADC();
int printFaults(int i, int j);

#endif /* INC_MENU_H_ */

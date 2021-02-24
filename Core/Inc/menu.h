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

extern uint8_t adcRestart[3];
extern uint8_t gpioInputBuf[12];


void initializeDisplay();
void drawMainMenu(int indicator);
void drawStatusMenu(int indicator);
void drawSystemInfoMenu(int indicator);
void drawBootMenu(int indicator, uint8_t button, int menu);

#endif /* INC_MENU_H_ */

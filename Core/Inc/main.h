/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// External peripherals
#include "ls027b7dh01.h"
#include "menu.h"
#include "zionEeprom.h"

// Graphical resources
#include "font5x7.h"
#include "font7x10.h"
#include "digits5x9.h"
#include "digits8x16.h"
#include "bitmaps.h"
#include "font_digits.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define true	1
#define	false	0
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
int writeI2CRegister(uint8_t address, uint8_t reg, uint8_t * bytes, int numBytes, int i2CBank);
void DevUI_Error_Handler(char *msg, HAL_StatusTypeDef ErrorCode, uint8_t err_param1, uint8_t err_param2, uint8_t critical_fault);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_SS_Pin GPIO_PIN_4
#define LCD_SS_GPIO_Port GPIOE
#define UI_INPUT11_Pin GPIO_PIN_3
#define UI_INPUT11_GPIO_Port GPIOE
#define LCD_SCLK_Pin GPIO_PIN_2
#define LCD_SCLK_GPIO_Port GPIOE
#define OUT3_1V8_Pin GPIO_PIN_14
#define OUT3_1V8_GPIO_Port GPIOG
#define LED_I2C_SCL_Pin GPIO_PIN_8
#define LED_I2C_SCL_GPIO_Port GPIOB
#define SOC_UART_TX_3V3_Pin GPIO_PIN_12
#define SOC_UART_TX_3V3_GPIO_Port GPIOC
#define LCD_MISO_Pin GPIO_PIN_5
#define LCD_MISO_GPIO_Port GPIOE
#define LCD_MOSI_Pin GPIO_PIN_6
#define LCD_MOSI_GPIO_Port GPIOE
#define LED_I2C_SDA_Pin GPIO_PIN_9
#define LED_I2C_SDA_GPIO_Port GPIOB
#define UI_INPUT0_Pin GPIO_PIN_7
#define UI_INPUT0_GPIO_Port GPIOB
#define SPARE_TIM_Pin GPIO_PIN_6
#define SPARE_TIM_GPIO_Port GPIOB
#define SPARE_UART_RX_3V3_Pin GPIO_PIN_11
#define SPARE_UART_RX_3V3_GPIO_Port GPIOC
#define SPARE_UART_TX_3V3_Pin GPIO_PIN_10
#define SPARE_UART_TX_3V3_GPIO_Port GPIOC
#define OUT3_CONFIG_Pin GPIO_PIN_8
#define OUT3_CONFIG_GPIO_Port GPIOI
#define OUT1_CONFIG_Pin GPIO_PIN_4
#define OUT1_CONFIG_GPIO_Port GPIOI
#define DWN_BTN_Pin GPIO_PIN_7
#define DWN_BTN_GPIO_Port GPIOK
#define DWN_BTN_EXTI_IRQn EXTI9_5_IRQn
#define SEL_BTN_Pin GPIO_PIN_6
#define SEL_BTN_GPIO_Port GPIOK
#define SEL_BTN_EXTI_IRQn EXTI9_5_IRQn
#define UP_BTN_Pin GPIO_PIN_5
#define UP_BTN_GPIO_Port GPIOK
#define UP_BTN_EXTI_IRQn EXTI9_5_IRQn
#define UI_INPUT1_Pin GPIO_PIN_13
#define UI_INPUT1_GPIO_Port GPIOC
#define SPARE_I2C_SDA_3V3_Pin GPIO_PIN_0
#define SPARE_I2C_SDA_3V3_GPIO_Port GPIOF
#define OUT2_CONFIG_Pin GPIO_PIN_6
#define OUT2_CONFIG_GPIO_Port GPIOI
#define BACK_BTN_Pin GPIO_PIN_4
#define BACK_BTN_GPIO_Port GPIOK
#define BACK_BTN_EXTI_IRQn EXTI4_IRQn
#define UI_INPUT2_Pin GPIO_PIN_4
#define UI_INPUT2_GPIO_Port GPIOD
#define SOC_UART_RX_3V3_Pin GPIO_PIN_2
#define SOC_UART_RX_3V3_GPIO_Port GPIOD
#define SPARE_I2C_SCL_3V3_Pin GPIO_PIN_1
#define SPARE_I2C_SCL_3V3_GPIO_Port GPIOF
#define MCU_HEARTBEAT_Pin GPIO_PIN_12
#define MCU_HEARTBEAT_GPIO_Port GPIOI
#define MCU_CTRL2_Pin GPIO_PIN_1
#define MCU_CTRL2_GPIO_Port GPIOK
#define SOC_I2C_SDA_3V3_Pin GPIO_PIN_9
#define SOC_I2C_SDA_3V3_GPIO_Port GPIOC
#define SOC_I2C_SCL_3V3_Pin GPIO_PIN_8
#define SOC_I2C_SCL_3V3_GPIO_Port GPIOA
#define BTN0_Pin GPIO_PIN_2
#define BTN0_GPIO_Port GPIOF
#define MCU_CTRL1_Pin GPIO_PIN_0
#define MCU_CTRL1_GPIO_Port GPIOK
#define BTN1_Pin GPIO_PIN_3
#define BTN1_GPIO_Port GPIOF
#define BTN2_Pin GPIO_PIN_4
#define BTN2_GPIO_Port GPIOF
#define SPARE_SS_L_3V3_Pin GPIO_PIN_5
#define SPARE_SS_L_3V3_GPIO_Port GPIOH
#define ADC15_Pin GPIO_PIN_7
#define ADC15_GPIO_Port GPIOF
#define ADC14_Pin GPIO_PIN_6
#define ADC14_GPIO_Port GPIOF
#define BTN3_Pin GPIO_PIN_5
#define BTN3_GPIO_Port GPIOF
#define OUT0_CONFIG_Pin GPIO_PIN_2
#define OUT0_CONFIG_GPIO_Port GPIOH
#define UI_INPUT10_Pin GPIO_PIN_15
#define UI_INPUT10_GPIO_Port GPIOD
#define UI_INPUT5_Pin GPIO_PIN_10
#define UI_INPUT5_GPIO_Port GPIOD
#define PP_SPARE_I2C_ADC_Pin GPIO_PIN_10
#define PP_SPARE_I2C_ADC_GPIO_Port GPIOF
#define ZION_SENSE_Pin GPIO_PIN_9
#define ZION_SENSE_GPIO_Port GPIOF
#define PP_CONFIG_ADC_Pin GPIO_PIN_8
#define PP_CONFIG_ADC_GPIO_Port GPIOF
#define ADC13_Pin GPIO_PIN_3
#define ADC13_GPIO_Port GPIOC
#define UI_INPUT9_Pin GPIO_PIN_14
#define UI_INPUT9_GPIO_Port GPIOD
#define ZION_PWR_EN_Pin GPIO_PIN_12
#define ZION_PWR_EN_GPIO_Port GPIOB
#define UI_INPUT4_Pin GPIO_PIN_9
#define UI_INPUT4_GPIO_Port GPIOD
#define UI_INPUT3_Pin GPIO_PIN_8
#define UI_INPUT3_GPIO_Port GPIOD
#define ADC10_Pin GPIO_PIN_0
#define ADC10_GPIO_Port GPIOC
#define ADC11_Pin GPIO_PIN_1
#define ADC11_GPIO_Port GPIOC
#define ADC12_Pin GPIO_PIN_2
#define ADC12_GPIO_Port GPIOC
#define BTN4_Pin GPIO_PIN_12
#define BTN4_GPIO_Port GPIOF
#define OUT1_1V8_Pin GPIO_PIN_1
#define OUT1_1V8_GPIO_Port GPIOG
#define STM_ZION_I2C_SDA_Pin GPIO_PIN_15
#define STM_ZION_I2C_SDA_GPIO_Port GPIOF
#define MCU_CTRL0_Pin GPIO_PIN_4
#define MCU_CTRL0_GPIO_Port GPIOJ
#define UI_INPUT7_Pin GPIO_PIN_12
#define UI_INPUT7_GPIO_Port GPIOD
#define UI_INPUT8_Pin GPIO_PIN_13
#define UI_INPUT8_GPIO_Port GPIOD
#define OUT1_OD_Pin GPIO_PIN_3
#define OUT1_OD_GPIO_Port GPIOG
#define OUT0_OD_Pin GPIO_PIN_2
#define OUT0_OD_GPIO_Port GPIOG
#define FRONT_LED_CTRL_Pin GPIO_PIN_12
#define FRONT_LED_CTRL_GPIO_Port GPIOH
#define ADC1_Pin GPIO_PIN_1
#define ADC1_GPIO_Port GPIOA
#define ADC0_Pin GPIO_PIN_0
#define ADC0_GPIO_Port GPIOA
#define ADC4_Pin GPIO_PIN_4
#define ADC4_GPIO_Port GPIOA
#define PP_SPARE_SPI_ADC_Pin GPIO_PIN_4
#define PP_SPARE_SPI_ADC_GPIO_Port GPIOC
#define BTN5_Pin GPIO_PIN_13
#define BTN5_GPIO_Port GPIOF
#define OUT0_1V8_Pin GPIO_PIN_0
#define OUT0_1V8_GPIO_Port GPIOG
#define UART_MUX_CTRL_Pin GPIO_PIN_3
#define UART_MUX_CTRL_GPIO_Port GPIOJ
#define FTDI_UART_RX_Pin GPIO_PIN_8
#define FTDI_UART_RX_GPIO_Port GPIOE
#define UI_INPUT6_Pin GPIO_PIN_11
#define UI_INPUT6_GPIO_Port GPIOD
#define OUT2_1V8_Pin GPIO_PIN_5
#define OUT2_1V8_GPIO_Port GPIOG
#define SPARE_MISO_3V3_Pin GPIO_PIN_7
#define SPARE_MISO_3V3_GPIO_Port GPIOH
#define ADC2_Pin GPIO_PIN_2
#define ADC2_GPIO_Port GPIOA
#define ADC6_Pin GPIO_PIN_6
#define ADC6_GPIO_Port GPIOA
#define ADC5_Pin GPIO_PIN_5
#define ADC5_GPIO_Port GPIOA
#define PP_SPARE_UART_ADC_Pin GPIO_PIN_5
#define PP_SPARE_UART_ADC_GPIO_Port GPIOC
#define STM_ZION_I2C_SCL_Pin GPIO_PIN_14
#define STM_ZION_I2C_SCL_GPIO_Port GPIOF
#define SPARE_MOSI_3V3_Pin GPIO_PIN_11
#define SPARE_MOSI_3V3_GPIO_Port GPIOF
#define FTDI_UART_CTS_L_Pin GPIO_PIN_9
#define FTDI_UART_CTS_L_GPIO_Port GPIOE
#define EDL_EN_Pin GPIO_PIN_10
#define EDL_EN_GPIO_Port GPIOB
#define SPARE_SCLK_3V3_Pin GPIO_PIN_6
#define SPARE_SCLK_3V3_GPIO_Port GPIOH
#define LCD_EXTCOMM_Pin GPIO_PIN_10
#define LCD_EXTCOMM_GPIO_Port GPIOH
#define ADC3_Pin GPIO_PIN_3
#define ADC3_GPIO_Port GPIOA
#define ADC7_Pin GPIO_PIN_7
#define ADC7_GPIO_Port GPIOA
#define ADC9_Pin GPIO_PIN_1
#define ADC9_GPIO_Port GPIOB
#define ADC8_Pin GPIO_PIN_0
#define ADC8_GPIO_Port GPIOB
#define LCD_EXTMODE_Pin GPIO_PIN_0
#define LCD_EXTMODE_GPIO_Port GPIOJ
#define LCD_DISP_Pin GPIO_PIN_1
#define LCD_DISP_GPIO_Port GPIOJ
#define FTDI_UART_TX_Pin GPIO_PIN_7
#define FTDI_UART_TX_GPIO_Port GPIOE
#define FTDI_UART_RTS_L_Pin GPIO_PIN_10
#define FTDI_UART_RTS_L_GPIO_Port GPIOE
#define SPARE_SW_EN_Pin GPIO_PIN_11
#define SPARE_SW_EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/*LED defines*/
#define ON 		1
#define OFF		0

/* Button Press Decode Defines */
#define UP		1
#define	DWN		2
#define BACK	3
#define SEL		4

/* Button Flag definition (shared between Button Task and Navigation Task) */
//extern uint8_t button_press;

//Boot mode button definitions
#define BTN0_ON			HAL_GPIO_WritePin(BTN0_GPIO_Port,BTN0_Pin,ON)
#define BTN0_OFF		HAL_GPIO_WritePin(BTN0_GPIO_Port,BTN0_Pin,OFF)
#define BTN1_ON			HAL_GPIO_WritePin(BTN1_GPIO_Port,BTN1_Pin,ON)
#define BTN1_OFF		HAL_GPIO_WritePin(BTN1_GPIO_Port,BTN1_Pin,OFF)
#define BTN2_ON			HAL_GPIO_WritePin(BTN2_GPIO_Port,BTN2_Pin,ON)
#define BTN2_OFF		HAL_GPIO_WritePin(BTN2_GPIO_Port,BTN2_Pin,OFF)
#define BTN3_ON			HAL_GPIO_WritePin(BTN3_GPIO_Port,BTN3_Pin,ON)
#define BTN3_OFF		HAL_GPIO_WritePin(BTN3_GPIO_Port,BTN3_Pin,OFF)
#define BTN4_ON			HAL_GPIO_WritePin(BTN4_GPIO_Port,BTN4_Pin,ON)
#define BTN4_OFF		HAL_GPIO_WritePin(BTN4_GPIO_Port,BTN4_Pin,OFF)
#define BTN5_ON			HAL_GPIO_WritePin(BTN5_GPIO_Port,BTN5_Pin,ON)
#define BTN5_OFF		HAL_GPIO_WritePin(BTN5_GPIO_Port,BTN5_Pin,OFF)

#define EDL_SW_ON		HAL_GPIO_WritePin(EDL_EN_GPIO_Port,EDL_EN_Pin,ON)
#define EDL_SW_OFF		HAL_GPIO_WritePin(EDL_EN_GPIO_Port,EDL_EN_Pin,OFF)
#define EX_SW_ON		HAL_GPIO_WritePin(SPARE_SW_EN_GPIO_Port,SPARE_SW_EN_Pin,ON)
#define EX_SW_OFF		HAL_GPIO_WritePin(SPARE_SW_EN_GPIO_Port,SPARE_SW_EN_Pin,OFF)

/* Navigation control defines */
#define MENU_TOP							1
#define MAX_MENU_ITEMS_MAIN_MENU			3
#define MAX_MENU_ITEMS_BOOT_MENU			5
#define MAX_MENU_ITEMS_STATUS_MENU			3
#define MAX_MENU_ITEMS_SYSTEM_INFO_MENU		2

//extern uint8_t curr_highlight;

#define NOTIFY_NOCLEAR			0x00
#define	NOTIFY_CLEARALL			0xFF
#define	NOTIFY_BTN_MASK			0x0F
#define	NOTIFY_MENU_MASK		0xF0
#define NOTIFY_RUN_MENU_MASK	0xF00
#define NOTIFY_MENU_BIT			4
#define NOTIFY_MENU_RUN_BIT		8

#define BOOT_MENU			2
#define MAIN_MENU			1
#define	STATUS_MENU			3
#define	SYSTEM_INFO_MENU	4

#define UNINITIALIZED		0
#define STANDARD			1
#define UEFI				2
#define EDL					3
#define RECOVERY			4
#define MASS_STORAGE		5


#define NO_BTN_PRESS		5

#define COMA				0
#define COMB				1

//Fault LEDs
#define ZION_FAULT 			0
#define VSYSPMI_FAULT		1


//MODE LEDs
#define BOOT_FAULT_LED			7
#define STANDARD_LED			8
#define EDL_LED					9
#define UEFI_LED				9

//MODE LED / RGB LED output defines
#define RED						7
#define	GREEN					8
#define	BLUE					9


//placeholder defines for other fault LEDs
#define FAULT3				2
#define FAULT4				3
#define FAULT5				4
#define FAULT6				5
#define FAULT7				6
#define FAULT8				10
#define FAULT9				11


// Thresholds for FAULTS
#define VSYS_FLT			3.5


//spare SPI definitions
#define SPARE_SS_ON 		HAL_GPIO_WritePin(SPARE_SS_L_3V3_GPIO_Port,SPARE_SS_L_3V3_Pin,ON);
#define SPARE_SS_OFF 		HAL_GPIO_WritePin(SPARE_SS_L_3V3_GPIO_Port,SPARE_SS_L_3V3_Pin,OFF);


struct errorLEDs{
	uint8_t zionFault;
	uint8_t vsysPMIFault;
	uint8_t fault3;
	uint8_t fault4;
	uint8_t fault5;
	uint8_t fault6;
	uint8_t fault7;
	uint8_t boot_fault;
	uint8_t standard_boot;
	uint8_t uefi_boot;
	uint8_t edl_boot;
	uint8_t fault8;
	uint8_t fault9;
	uint8_t ledDriver;
};

struct LED{
	uint8_t address;
	uint8_t mode0_reg;
	uint8_t led0_reg;
	uint8_t led1_reg;
	uint8_t led2_reg;
	uint8_t led3_reg;
	uint8_t iref_reg;
	uint8_t mode0_oscon_value;
	uint8_t led7_pwm;
	uint8_t led8_pwm;
	uint8_t led9_pwm;
	uint8_t pwm;
	int i2cBank;
};
static struct LED LED = {0x60 << 1, 0x00, 0x14, 0x15, 0x16, 0x17, 0x1C,0x11,0x09,0x0A,0x0B, 0x08,1};



struct bootModeButtons{
	int bootModeSet; //tells the boot button task to get working
	int btn0; //PWR BTN
	int btn1; //DPAD UP
	int btn2; //DPAD RIGHT
	int btn3; //DPAD LEFT
	int btn4;
	int btn5;
	int edl_sw;
	int ex_sw;
	int modeClear; //Tells the display menu to keep going
	int bootMode; // tells the display method what to draw.
};

struct zion{
	int zionFinished;
	int zionSwitch;
	int SOC_EEPROM_Detected;
	int ASIC_EEPROM_Detected;
	int DISPLAY_EEPROM_Detected;
	int SOC_BoardID;
	int SOC_BoardFab;
	int SOC_Config;
	int ASIC_BoardID;
	int ASIC_BoardFab;
	int ASIC_Config;
	int DISPLAY_BoardID;
	int DISPLAY_BoardFab;
	int DISPLAY_Config;
};


struct socI2cVoltageMux{
	uint8_t address;
	uint8_t CMD_A_reg;
	uint8_t CMD_B_reg;
	uint8_t enableSW1;
	uint8_t enableSW2;
	uint8_t enableSW3;
	uint8_t enableSW4;
	uint8_t enableSW5;
	uint8_t enableSW6;
	uint8_t enableSW7;
	uint8_t enableSW8;
	uint8_t enableSW9;
	uint8_t enableSW10;
	uint8_t enableSW11;
	uint8_t enableSW12;
	uint8_t enableSW13;
	uint8_t enableSW14;
	uint8_t enableSW15;
	uint8_t enableSW16;
	uint8_t clearSwitches;
	int i2cBank;
};
static struct socI2cVoltageMux socI2cVoltageMux = {0x4C << 1, 0x14 , 0x15, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F ,0x10, 3};

struct inputGPIOs{
	int input0;
	int input1;
	int input2;
	int input3;
	int input4;
	int input5;
	int input6;
	int input7;
	int input8;
	int input9;
	int input10;
	int input11;
};
static struct inputGPIOs inputGPIOs = {0,1,2,3,4,5,6,7,8,9,10,11};

struct outputGPIOs{
	int mcu3V3_0;
	int mcu3V3_1;
	int mcu3V3_2;
	int mcu3V3_3; //also known as uart mux ctrl in the schematic
	int out1V8_0;
	int out1V8_1;
	int out1V8_2;
	int out1V8_3;
	int configOut_0;
	int configOut_1;
	int configOut_2;
	int configOut_3;
	int odOut_0;  //configure this assuming MSM_RESIN
	int odOut_1;
};
static struct outputGPIOs outputGPIOs = {0,1,2,3,4,5,6,7,8,9,10,11,12,13};


struct Adc{
	int adc0;
	int adc1;
	int adc2;
	int adc3;
	int adc4;
	int adc5;
	int adc6;
	int adc7;
	int adc8;
	int adc9;
	int adc10;
	int adc11;
	int adc12;
	int adc13;
	int adc14;
	int adc15;
	int spareSpiADC;
	int spareUartADC;
	int configADC;
	int zionADC;
	int spareI2cADC;
	float adcDivisor;
	int adcResistorDivider;
	int systemResistorDivider;
};
static struct Adc Adc = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18 ,19, 20, 3.3/4096, 3,2};
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

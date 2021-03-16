/*
 * Atlas.h
 *
 *  Created on: Mar 15, 2021
 *      Author: auphilli
 */

#ifndef INC_ATLAS_H_
#define INC_ATLAS_H_


#include "main.h"
#include "ls027b7dh01.h"

//fault names used for the status menu.
#define ATLAS_FAULT1_LBL			"ZION_FLT:"
#define ATLAS_FAULT2_LBL			"VSYS_PMI:"
#define ATLAS_FAULT3_LBL			"VREG_BOB:"
#define ATLAS_FAULT4_LBL			"VREG_S5A:"
#define ATLAS_FAULT5_LBL			"VREG_S6C:"
#define ATLAS_FAULT6_LBL			"SOC_IN6:"
#define ATLAS_FAULT7_LBL			"SOC_IN7:"
#define ATLAS_FAULT8_LBL			"SOC_IN8:"
#define ATLAS_FAULT9_LBL			"SOC_IN11:"

//Fault descriptions used on the status menu.
#define ATLAS_FAULT1_TRIGGER_MSG			" SOC ZION ERROR"
#define ATLAS_FAULT2_TRIGGER_MSG			" VSYS PMI LOW"
#define ATLAS_FAULT3_TRIGGER_MSG			" VREG_BOB LOW"
#define ATLAS_FAULT4_TRIGGER_MSG			" VREG_S5A LOW"
#define ATLAS_FAULT5_TRIGGER_MSG			" VREG_S6C LOW"
#define ATLAS_FAULT6_TRIGGER_MSG			" FAULT HIGH"
#define ATLAS_FAULT7_TRIGGER_MSG			" FAULT LOW"
#define ATLAS_FAULT8_TRIGGER_MSG			" FAULT HIGH"
#define ATLAS_FAULT9_TRIGGER_MSG			" FAULT LOW"

//Labels for header Fault Descriptions.
#define ATLAS_HEADER2_FAULT_LBL			"ZION"
#define ATLAS_HEADER1_FAULT_LBL			"VSYS"
#define ATLAS_HEADER3_FAULT_LBL			"VREG_BOB"
#define ATLAS_HEADER4_FAULT_LBL			"VREG_S5A"
#define ATLAS_HEADER5_FAULT_LBL			"VREG_S6C"
#define ATLAS_HEADER6_FAULT_LBL			"SOC_IN6"
#define ATLAS_HEADER7_FAULT_LBL			"SOC_IN7"
#define ATLAS_HEADER8_FAULT_LBL			"SOC_IN8"
#define ATLAS_HEADER9_FAULT_LBL			"SOC_IN11"

//Labels for main page LED Fault Descriptions.
#define ATLAS_LED1_FAULT_LBL			"ZION_FLT"
#define ATLAS_LED2_FAULT_LBL			"VSYS_FLT"
#define ATLAS_LED3_FAULT_LBL			"VREG_BOB"
#define ATLAS_LED4_FAULT_LBL			"VREG_S5A"
#define ATLAS_LED5_FAULT_LBL			"VREG_S6C"
#define ATLAS_LED6_FAULT_LBL			"SOC_IN6"
#define ATLAS_LED7_FAULT_LBL			"SOC_IN7"
#define ATLAS_LED8_FAULT_LBL			"SOC_IN8"
#define ATLAS_LED9_FAULT_LBL			"SOC_IN11"

//ATLAS Main Menu BOOT MODE States. Keep short as there is only about 9 characters worth of space.
#define ATLAS_MAIN_STD_MODE					"OS"
#define ATLAS_MAIN_EDL_MODE					"EDL"
#define ATLAS_MAIN_MASS_STORAGE_MODE		"MASS"
#define ATLAS_MAIN_UEFI_MODE				"UEFI"
#define ATLAS_MAIN_RECOVERY_MODE			"FFU"

//ATLAS Boot Menu BOOT MODE States
#define ATLAS_BOOT_STD_MODE					"STANDARD"
#define ATLAS_BOOT_EDL_MODE					"EMERGENCY DOWNLOAD"
#define ATLAS_BOOT_MASS_STORAGE_MODE		"MASS STORAGE"
#define ATLAS_BOOT_UEFI_MODE				"UEFI"
#define ATLAS_BOOT_RECOVERY_MODE			"RECOVERY"




// Labels for AI VOLTAGES page.
// PLATFORM TEMPLATE: Rename these defines names to voltage rails that will be monitored by the DevUI ADCs for your platform.
#define ATLAS_AI0				"VSYS:"
#define ATLAS_AI1				"VREG_BOB:"
#define ATLAS_AI2				"S5A:"
#define ATLAS_AI3				"S6C:"
#define ATLAS_AI4				"S4E:"
#define ATLAS_AI5				"VDDMX:"
#define ATLAS_AI6				"LPI_MX:"
#define ATLAS_AI7				"VDDA_EBI:"
#define ATLAS_AI8				"VDD_CX:"
#define ATLAS_AI9				"VDD_MM:"
#define ATLAS_AI10				"SSC_CX:"
#define ATLAS_AI11				"PHY_1P2:"
#define ATLAS_AI12				"CORE_PCIE:"
#define ATLAS_AI13				"CORE_USB:"
#define ATLAS_AI14				"S5E:"
#define ATLAS_AI15				"1P8:"

extern struct adcState adcStates;
extern uint8_t gpioInputBuf[12];
extern struct zion ZION;
extern struct bootModeButtons bootButtons;
extern struct errorLEDs errorLED;
extern uint8_t errorLEDState[12];
extern int setIndicator;
extern float displayAdcValues[21];
extern I2C_HandleTypeDef hi2c1;

//define the values for the ADCs
#define ATLAS_VSYS					displayAdcValues[Adc.adc0]
#define ATLAS_VREG_BOB				displayAdcValues[Adc.adc1]
#define ATLAS_S5A					displayAdcValues[Adc.adc2]
#define ATLAS_S6C					displayAdcValues[Adc.adc3]
#define ATLAS_S4E					displayAdcValues[Adc.adc4]
#define ATLAS_VDDMX					displayAdcValues[Adc.adc5]
#define ATLAS_LPI_MX				displayAdcValues[Adc.adc6]
#define ATLAS_VDDA_EBI				displayAdcValues[Adc.adc7]
#define ATLAS_VDD_CX				displayAdcValues[Adc.adc8]
#define ATLAS_VDD_MM				displayAdcValues[Adc.adc9]
#define ATLAS_SSC_CX				displayAdcValues[Adc.adc10]
#define ATLAS_PHY_1P2				displayAdcValues[Adc.adc11]
#define ATLAS_CORE_PCIE				displayAdcValues[Adc.adc12]
#define ATLAS_CORE_USB				displayAdcValues[Adc.adc13]
#define ATLAS_S5E					displayAdcValues[Adc.adc14]
#define ATLAS_1P8					displayAdcValues[Adc.adc15]

//give names for each GPIO input in the status menu
#define ATLAS_IN0				"PV: "
#define ATLAS_IN1				"WIFI: "
#define ATLAS_IN2				"WIGIG: "
#define ATLAS_IN3				"CODEC: "
#define ATLAS_IN4				"RF: "
#define ATLAS_IN5				"IN5: "
#define ATLAS_IN6				"IN6: "
#define ATLAS_IN7				"IN7: "
#define ATLAS_IN8				"IN8: "
#define ATLAS_IN9				"IN9: "
#define ATLAS_IN10				"IN10: "
#define ATLAS_IN11				"IN11: "


//define the value in each GPIO state for ATLAS
#define ATLAS_PV_PRSNT			gpioInputBuf[inputGPIOs.input0]
#define ATLAS_WIFI_PRSNT		gpioInputBuf[inputGPIOs.input1]
#define ATLAS_WIGIG_PRSNT		gpioInputBuf[inputGPIOs.input2]
#define ATLAS_CODEC_PRSNT		gpioInputBuf[inputGPIOs.input3]
#define ATLAS_RF_PRSNT			gpioInputBuf[inputGPIOs.input4]
#define ATLAS_IN5_GPIO			gpioInputBuf[inputGPIOs.input5]
#define ATLAS_IN6_GPIO			gpioInputBuf[inputGPIOs.input6]
#define ATLAS_IN7_GPIO			gpioInputBuf[inputGPIOs.input7]
#define ATLAS_IN8_GPIO			gpioInputBuf[inputGPIOs.input8]
#define ATLAS_IN9_GPIO			gpioInputBuf[inputGPIOs.input9]
#define ATLAS_IN10_GPIO			gpioInputBuf[inputGPIOs.input10]
#define ATLAS_IN11_GPIO			gpioInputBuf[inputGPIOs.input11]

//ATLAS Button Mode Definitions
#define ATLAS_PWR_ON 			BTN0_ON
#define ATLAS_PWR_OFF			BTN0_OFF
#define	ATLAS_DPAD_UP_ON		BTN1_ON
#define ATLAS_DPAD_UP_OFF		BTN1_OFF
#define ATLAS_DPAD_RIGHT_ON		BTN2_ON
#define ATLAS_DPAD_RIGHT_OFF	BTN2_OFF
#define ATLAS_DPAD_LEFT_ON		BTN3_ON
#define ATLAS_DPAD_LEFT_OFF		BTN3_OFF
#define ATLAS_DPAD_DOWN_ON		BTN4_ON
#define ATLAS_DPAD_DOWN_OFF		BTN4_OFF
#define ATLAS_EDL_ON			EDL_SW_ON
#define ATLAS_EDL_OFF			EDL_SW_OFF

// Falling Edge Thresholds for FAULTS.  Add more #defines to add additional fault thresholds for more ADC channels.
// PLATFORM TEMPLATE: Rename these defines names to voltage rails names that match your platform.  Leave the "_FLT" suffic.
// PLATFORM TEMPLATE: Choose appropriate falling edge thresholds for your platform.
#define VSYS_FLT				3.5
#define	VREG_BOB_FLT			3.3
#define VREG_S5A_FLT			1.824
#define VREG_S6C_FLT			1.264
#define VREG_S4E_FLT			0.752
#define VDDMX_FLT				0.348
#define LPI_MX_FLT				0.5
#define VDDA_EBI_FLT			0.348
#define VDD_CX_FLT				0.348
#define VDD_MM_FLT				0.348
#define SSC_CX_FLT				0.348
#define SERDES_1P2_FLT			1.2
#define CORE_UFS_PCIE_FLT		0.88
#define CORE_USB_FLT			0.912
#define VREG_S5E_FLT			1.824
#define VREG_1P8_FLT			1.8

// Input GPIO Fault Level Thresholds.
// PLATFORM TEMPLATE: Rename these defines names to logic I/O names that match your platform.  Leave the "_FLT" suffix.
// PLATFORM TEMPLATE: Choose appropriate HI/LO fault level for each I/O for your platform.
#define SOC_IN0_FLT			1
#define SOC_IN1_FLT			0
#define SOC_IN2_FLT			1
#define SOC_IN3_FLT			0
#define SOC_IN4_FLT			1
#define SOC_IN5_FLT			0
#define SOC_IN6_FLT			1
#define SOC_IN7_FLT			0
#define SOC_IN8_FLT			1
#define SOC_IN9_FLT			0
#define SOC_IN10_FLT		1
#define SOC_IN11_FLT		0

// Platform voltage to ADC channel mapping defines.
// There are currently a max of 16 ADC channels on DevUI.
// PLATFORM TEMPLATE: Rename these defines names to voltage rails names that match your platform.
#define VSYS				0
#define	VREG_BOB			1
#define VREG_S5A			2
#define VREG_S6C			3
#define VREG_S4E			4
#define VDDMX				5
#define LPI_MX				6
#define VDDA_EBI			7
#define VDD_CX				8
#define VDD_MM				9
#define SSC_CX				10
#define SERDES_1P2			11
#define CORE_UFS_PCIE		12
#define CORE_USB			13
#define VREG_S5E			14
#define VREG_1P8			15

// Platform GPIO Input to STM GPIO mapping.  There are a max of 12 GPIO inputs on the DevUI.
// PLATFORM TEMPLATE: Rename these defines names to logic I/O names that match your platform.

#define SOC_IN0				0
#define	SOC_IN1				1
#define	SOC_IN2				2
#define	SOC_IN3				3
#define	SOC_IN4				4
#define	SOC_IN5				5
#define	SOC_IN6				6
#define	SOC_IN7				7
#define	SOC_IN8				8
#define	SOC_IN9				9
#define	SOC_IN10			10
#define	SOC_IN11			11

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

#define LED_DRIVER_I2C			hi2c1


//define the names for the three main boards for the ATLAS platform
#define ATLAS_SOC_BOARD			"TRIDENT "
#define ATLAS_ASIC_BOARD		"TOGA "
#define ATLAS_DISPLAY_BOARD		"KANU "

void atlasMainMenuFaultLedLabels();
void atlasMainMenuBootModes(int i, int j);
int atlasBootMenuBootModes(int indicator, int previousMenu, int menu, int button);
void atlasSystemInfoSoc(int i, int j);
void atlasSystemInfoAsic(int i, int j);
void atlasSystemInfoDisplay(int i, int j);
void atlasSystemInfoPV(int i, int j);
void atlasSystemInfoWIFI(int i, int j);
void atlasSystemInfoWIGIG(int i, int j);
void atlasSystemInfoCODEC(int i, int j);
void atlasSystemInfoRF(int i, int j);
int atlasHeaderFaults(int i, int j);
void atlasStatusADCsAndGPIOs();
void atlasStatusFaults();
int atlasBootButtons(int pwrBtnReady);
void atlasErrorLEDs();

#endif /* INC_ATLAS_H_ */

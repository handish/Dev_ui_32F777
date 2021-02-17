/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADC_BUF_LEN 5000
#define ADC_AVG_COUNT 20
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart5_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for Heartbeat */
osThreadId_t HeartbeatHandle;
const osThreadAttr_t Heartbeat_attributes = {
  .name = "Heartbeat",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for adcRead */
osThreadId_t adcReadHandle;
const osThreadAttr_t adcRead_attributes = {
  .name = "adcRead",
  .priority = (osPriority_t) osPriorityLow1,
  .stack_size = 128 * 4
};
/* Definitions for DatScreenBlink */
osThreadId_t DatScreenBlinkHandle;
const osThreadAttr_t DatScreenBlink_attributes = {
  .name = "DatScreenBlink",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1000 * 4
};
/* Definitions for gpioInputRead */
osThreadId_t gpioInputReadHandle;
const osThreadAttr_t gpioInputRead_attributes = {
  .name = "gpioInputRead",
  .priority = (osPriority_t) osPriorityLow2,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
uint16_t adc1_buf[ADC_BUF_LEN];
uint16_t adc2_buf[ADC_BUF_LEN];
uint16_t adc3_buf[ADC_BUF_LEN];
uint8_t adcRestart[3];
//12 gpio inputs to the Dev UI
uint8_t gpioInputBuf[12];
//14 gpio outputs to the Dev UI
uint8_t gpioOutputState[14];

static uint32_t i, j, k;

uint8_t inputButtonSet = 5; //set to a higher value than any other button priority. 5 is the "unused" state
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C4_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_UART7_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void startHeartbeat(void *argument);
void startADCRead(void *argument);
void GetDaScreenBlink(void *argument);
void startGpioInputRead(void *argument);

/* USER CODE BEGIN PFP */
void uartTransmitChar(char *message,int uart);
void uartTransmitInt(uint16_t *number, int uart);
uint8_t * readI2CRegister(uint8_t address, uint8_t reg, int bytes, int i2CBank);
int writeI2CRegister(uint8_t address, uint8_t reg, uint8_t * bytes, int numBytes, int i2CBank);
void setErrorLED(int led, _Bool change);
void configureLEDDriver();
float* getADCValues();
void uartTransmitFloat(float *number, int uart);
uint8_t *getInputGPIOState(void);
void setOutputGPIOState(int gpio, int state);
void outputGPIOBufInitialization();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
	int odOut_0;
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 char spi_buf[20];
 static uint16_t LCD_Blink_White = 0b0001100000000000;
 static uint16_t LCD_Blink_Black = 0b0001000000000000;
 static uint16_t LCD_Green = 0b0100010001000100;
 static uint16_t LCD_White = 0b1110111011101110;
 static uint16_t LCD_Red = 0b1000100010001000;
 static uint16_t update_Line1 = 0b1001000010010001;
 static uint16_t update_Line2 = 0b1001000010010011;
 static uint16_t update_Line3 = 0b1001000010010010;
 static uint16_t update_dummy = 0b0000000000000000;
 static uint16_t LCD_Cleared = 0b0010000000000000;
 static _Bool ON = 1;
 static _Bool OFF = 0;
 char uartBuf[50];
 int uartBufLen;
 static const uint8_t LED_ADDR = 0x60 <<1;
 HAL_StatusTypeDef ret;
 uint8_t buf[12];
 int16_t val;


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_RTC_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  //HAL_ADC_ConfigChannel();
  outputGPIOBufInitialization();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, ADC_BUF_LEN);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, ADC_BUF_LEN);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_buf, ADC_BUF_LEN);
//  HAL_GPIO_WritePin(GPIOJ,LCD_DISP_Pin,GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOJ,LCD_EXTMODE_Pin,GPIO_PIN_RESET);
//    HAL_Delay(1);
//    HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_SET);
//    HAL_SPI_Transmit(&hspi4, (uint16_t *)&LCD_Clear, 1, 100);
//    HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_RESET);
//    HAL_Delay(1);
   int x=1;
//    uint16_t test[102];
//    test[0] = update_Line1;
//    uartTransmitInt(test[0],7);
//    while (x<101){
//  	test[x] = LCD_Green;
//  	x=x+1;
//    }
//    x=0;
//    test[61] = update_dummy;
//    uartTransmitInt(test,7);
//    HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_SET);
//    HAL_SPI_Transmit(&hspi4, (uint16_t *)test, 102, 100);
//    HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_RESET);
//    HAL_Delay(5);
//    test[0]=update_Line2;
//    uartTransmitInt(test[0],7);
//    HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_SET);
//    HAL_SPI_Transmit(&hspi4, (uint16_t *)test, 102, 100);
//    HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_RESET);
//    HAL_Delay(5);
//    test[0]=update_Line3;
//    uartTransmitInt(test[0],7);
//    HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_SET);
//    HAL_SPI_Transmit(&hspi4, (uint16_t *)test, 102, 100);
//    HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOH,FRONT_LED_CTRL_Pin,GPIO_PIN_SET);
//    HAL_Delay(5000);


  configureLEDDriver();
    setErrorLED(4,OFF);
    HAL_Delay(1000);
    setErrorLED(7,OFF);
    setErrorLED(8,ON);
    HAL_Delay(1000);
    setErrorLED(8,OFF);
    setErrorLED(9,ON);
    HAL_Delay(1000);
    setErrorLED(8,ON);
    setErrorLED(9,ON);
    HAL_Delay(1000);
    setErrorLED(9,OFF);

	SMLCD_InitGPIO();
	SMLCD_Init();
	SMLCD_Enable();
	SMLCD_Clear();
#define ORI 0
	uint8_t ori;
#if (ORI == 0)
	ori = LCD_ORIENT_NORMAL;
#elif (ORI == 1)
	ori = LCD_ORIENT_180;
#elif (ORI == 2)
	ori = LCD_ORIENT_CW;
#else
	ori = LCD_ORIENT_CCW;
#endif
	SMLCD_Orientation(ori);


	// Clear video buffer
	LCD_Clear();


	LCD_PixelMode = LCD_PSET;
//	LCD_PixelMode = LCD_PRES;
//	LCD_PixelMode = LCD_PINV;

	//SMLCD_Clear();
	SMLCD_Flush();
	if (scr_width > scr_height) {
		j = scr_height / 2;
	} else {
		j = scr_width / 2;
	}
	for (i = 4; i < j; i += 4) {
		LCD_Circle((scr_width / 2) - 1, (scr_height / 2) - 1, i);
	}
	SMLCD_Flush();
	HAL_Delay(100);

    //int check = writeI2CRegister(LED.address,LED.led0_reg,blah2,1,1);
  //  if(check){
  //	  blah = readI2CRegister(LED.address,LED.led0_reg,1,1);
  //	  uartTransmitInt(blah[0],7);
  //  }
  //  else{
  //	  uartTransmitChar("Things didn't go well.\r\n",7);
  //  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Heartbeat */
  HeartbeatHandle = osThreadNew(startHeartbeat, NULL, &Heartbeat_attributes);

  /* creation of adcRead */
  adcReadHandle = osThreadNew(startADCRead, NULL, &adcRead_attributes);

  /* creation of DatScreenBlink */
  DatScreenBlinkHandle = osThreadNew(GetDaScreenBlink, NULL, &DatScreenBlink_attributes);

  /* creation of gpioInputRead */
  gpioInputReadHandle = osThreadNew(startGpioInputRead, NULL, &gpioInputRead_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	  HAL_Delay(1000);
	  if (!x) {
		  HAL_GPIO_WritePin(GPIOI,MCU_HEARTBEAT_Pin,GPIO_PIN_SET);
		  x=1;

		  //uartTransmitChar("hello\r\n",7);
		  HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_SET);
		  HAL_SPI_Transmit(&hspi4, (uint16_t *)&LCD_Blink_White, 1, 100);
		  HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_RESET);
	  }
	  else{
		  HAL_GPIO_WritePin(GPIOI,MCU_HEARTBEAT_Pin,GPIO_PIN_RESET);
		  x=0;
		  //uartTransmitChar("here\r\n",7);
		  HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_SET);
		  HAL_SPI_Transmit(&hspi4, (uint16_t *)&LCD_Blink_Black, 1, 100);
		  HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_RESET);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_UART7
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_I2C4
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 11;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 4;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 6;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */
  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x007074AF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x007074AF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x007074AF;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x007074AF;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 7;
  hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_SS_GPIO_Port, LCD_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, OUT3_1V8_Pin|OUT1_1V8_Pin|OUT1_OD_Pin|OUT0_OD_Pin
                          |OUT0_1V8_Pin|OUT2_1V8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, OUT3_CONFIG_Pin|OUT1_CONFIG_Pin|OUT2_CONFIG_Pin|MCU_HEARTBEAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOK, MCU_CTRL2_Pin|MCU_CTRL1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, BTN0_Pin|BTN1_Pin|BTN2_Pin|BTN3_Pin
                          |BTN4_Pin|BTN5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, OUT0_CONFIG_Pin|FRONT_LED_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ZION_PWR_EN_Pin|EDL_EN_Pin|SPARE_SW_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOJ, MCU_CTRL0_Pin|UART_MUX_CTRL_Pin|LCD_EXTMODE_Pin|LCD_DISP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LCD_SS_Pin */
  GPIO_InitStruct.Pin = LCD_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UI_INPUT11_Pin */
  GPIO_InitStruct.Pin = UI_INPUT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UI_INPUT11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT3_1V8_Pin OUT1_1V8_Pin OUT1_OD_Pin OUT0_OD_Pin
                           OUT0_1V8_Pin OUT2_1V8_Pin */
  GPIO_InitStruct.Pin = OUT3_1V8_Pin|OUT1_1V8_Pin|OUT1_OD_Pin|OUT0_OD_Pin
                          |OUT0_1V8_Pin|OUT2_1V8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : UI_INPUT0_Pin */
  GPIO_InitStruct.Pin = UI_INPUT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UI_INPUT0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT3_CONFIG_Pin OUT1_CONFIG_Pin OUT2_CONFIG_Pin MCU_HEARTBEAT_Pin */
  GPIO_InitStruct.Pin = OUT3_CONFIG_Pin|OUT1_CONFIG_Pin|OUT2_CONFIG_Pin|MCU_HEARTBEAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : DWN_BTN_Pin SEL_BTN_Pin UP_BTN_Pin BACK_BTN_Pin */
  GPIO_InitStruct.Pin = DWN_BTN_Pin|SEL_BTN_Pin|UP_BTN_Pin|BACK_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pin : UI_INPUT1_Pin */
  GPIO_InitStruct.Pin = UI_INPUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UI_INPUT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UI_INPUT2_Pin UI_INPUT10_Pin UI_INPUT5_Pin UI_INPUT9_Pin
                           UI_INPUT4_Pin UI_INPUT3_Pin UI_INPUT7_Pin UI_INPUT8_Pin
                           UI_INPUT6_Pin */
  GPIO_InitStruct.Pin = UI_INPUT2_Pin|UI_INPUT10_Pin|UI_INPUT5_Pin|UI_INPUT9_Pin
                          |UI_INPUT4_Pin|UI_INPUT3_Pin|UI_INPUT7_Pin|UI_INPUT8_Pin
                          |UI_INPUT6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_CTRL2_Pin MCU_CTRL1_Pin */
  GPIO_InitStruct.Pin = MCU_CTRL2_Pin|MCU_CTRL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN0_Pin BTN1_Pin BTN2_Pin BTN3_Pin
                           BTN4_Pin BTN5_Pin */
  GPIO_InitStruct.Pin = BTN0_Pin|BTN1_Pin|BTN2_Pin|BTN3_Pin
                          |BTN4_Pin|BTN5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT0_CONFIG_Pin FRONT_LED_CTRL_Pin */
  GPIO_InitStruct.Pin = OUT0_CONFIG_Pin|FRONT_LED_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : ZION_PWR_EN_Pin EDL_EN_Pin SPARE_SW_EN_Pin */
  GPIO_InitStruct.Pin = ZION_PWR_EN_Pin|EDL_EN_Pin|SPARE_SW_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_CTRL0_Pin UART_MUX_CTRL_Pin LCD_EXTMODE_Pin LCD_DISP_Pin */
  GPIO_InitStruct.Pin = MCU_CTRL0_Pin|UART_MUX_CTRL_Pin|LCD_EXTMODE_Pin|LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void outputGPIOBufInitialization(){
	memset(gpioOutputState,0,sizeof(gpioOutputState));
}

void setOutputGPIOState(int gpio, int state){

	if(gpio == outputGPIOs.configOut_0){
		HAL_GPIO_WritePin(GPIOH,OUT0_CONFIG_Pin,state);
		gpioOutputState[outputGPIOs.configOut_0] = state;
	}

	else if(gpio == outputGPIOs.configOut_1){
		HAL_GPIO_WritePin(GPIOI,OUT1_CONFIG_Pin,state);
		gpioOutputState[outputGPIOs.configOut_1] = state;
	}

	else if(gpio == outputGPIOs.configOut_2){
		HAL_GPIO_WritePin(GPIOI,OUT2_CONFIG_Pin,state);
		gpioOutputState[outputGPIOs.configOut_2] = state;
	}

	else if(gpio == outputGPIOs.configOut_3){
		HAL_GPIO_WritePin(GPIOI,OUT3_CONFIG_Pin,state);
		gpioOutputState[outputGPIOs.configOut_3] = state;
	}

	else if(gpio == outputGPIOs.mcu3V3_0){
		HAL_GPIO_WritePin(GPIOJ,MCU_CTRL0_Pin,state);
		gpioOutputState[outputGPIOs.mcu3V3_0] = state;
	}

	else if(gpio == outputGPIOs.mcu3V3_1){
		HAL_GPIO_WritePin(GPIOK,MCU_CTRL1_Pin,state);
		gpioOutputState[outputGPIOs.mcu3V3_1] = state;
	}

	else if(gpio == outputGPIOs.mcu3V3_2){
		HAL_GPIO_WritePin(GPIOK,MCU_CTRL2_Pin,state);
		gpioOutputState[outputGPIOs.mcu3V3_2] = state;
	}

	else if(gpio == outputGPIOs.mcu3V3_3){
		HAL_GPIO_WritePin(GPIOJ,UART_MUX_CTRL_Pin,state);
		gpioOutputState[outputGPIOs.mcu3V3_3] = state;
	}

	else if(gpio == outputGPIOs.out1V8_0){
		HAL_GPIO_WritePin(GPIOG,OUT0_1V8_Pin,state);
		gpioOutputState[outputGPIOs.out1V8_0] = state;
	}

	else if(gpio == outputGPIOs.out1V8_1){
		HAL_GPIO_WritePin(GPIOG,OUT1_1V8_Pin,state);
		gpioOutputState[outputGPIOs.out1V8_1] = state;
	}

	else if(gpio == outputGPIOs.out1V8_2){
		HAL_GPIO_WritePin(GPIOG,OUT2_1V8_Pin,state);
		gpioOutputState[outputGPIOs.out1V8_2] = state;
	}

	else if(gpio == outputGPIOs.out1V8_3){
		HAL_GPIO_WritePin(GPIOG,OUT3_1V8_Pin,state);
		gpioOutputState[outputGPIOs.out1V8_3] = state;
	}

	else if(gpio == outputGPIOs.odOut_0){
		HAL_GPIO_WritePin(GPIOG,OUT0_OD_Pin,state);
		gpioOutputState[outputGPIOs.odOut_0] = state;
	}

	else if(gpio == outputGPIOs.odOut_1){
		HAL_GPIO_WritePin(GPIOG,OUT1_OD_Pin,state);
		gpioOutputState[outputGPIOs.odOut_1] = state;
	}
}

void uartTransmitChar(char *message, int uart){
	char uart_buf[200];
	int uart_buf_len;
	uart_buf_len = sprintf(uart_buf, message);
	if (uart == 7){
		HAL_UART_Transmit(&huart7, (uint8_t *)uart_buf, uart_buf_len,100);
	}
}
void uartTransmitInt(uint16_t *number, int uart){
	char uart_buf[80];
	int uart_buf_len;
	uart_buf_len = sprintf(uart_buf, "0x%x\r\n", number);
	if (uart == 7){
		HAL_UART_Transmit(&huart7, (uint8_t *)uart_buf, uart_buf_len,100);
	}

}
void uartTransmitFloat(float *number, int uart){
	char uart_buf[80];
	int uart_buf_len;
	uart_buf_len = sprintf(uart_buf, "%f\r\n", number);
	if (uart == 7){
		HAL_UART_Transmit(&huart7, (uint8_t *)uart_buf, uart_buf_len,100);
	}

}

uint8_t * readI2CRegister(uint8_t address, uint8_t reg, int bytes, int i2CBank){
	static uint8_t buf[20];
	HAL_StatusTypeDef ret;
	buf[0]=reg;
  	if(i2CBank == 1){
  		ret = HAL_I2C_Master_Transmit(&hi2c1, address, buf, 1, HAL_MAX_DELAY);
  	}
  	else if(i2CBank == 2){
  		ret = HAL_I2C_Master_Transmit(&hi2c2, address, buf, 1, HAL_MAX_DELAY);
  	}
  	else if(i2CBank == 3){
  		ret = HAL_I2C_Master_Transmit(&hi2c3, address, buf, 1, HAL_MAX_DELAY);
  	}
  	else if(i2CBank == 4){
  		ret = HAL_I2C_Master_Transmit(&hi2c4, address, buf, 1, HAL_MAX_DELAY);
  	}
	  if ( ret != HAL_OK ) {
	          return 0xfe;
	        }
	  else {
		  if(i2CBank == 1){
				ret = HAL_I2C_Master_Receive(&hi2c1, address, buf, bytes, HAL_MAX_DELAY);
			}
			else if(i2CBank == 2){
				ret = HAL_I2C_Master_Receive(&hi2c2, address, buf, bytes, HAL_MAX_DELAY);
			}
			else if(i2CBank == 3){
				ret = HAL_I2C_Master_Receive(&hi2c3, address, buf, bytes, HAL_MAX_DELAY);
			}
			else if(i2CBank == 4){
				ret = HAL_I2C_Master_Receive(&hi2c4, address, buf, bytes, HAL_MAX_DELAY);
			}
		  if ( ret != HAL_OK ) {
		          return 0xfe;
		        }
		  else{
			  uartTransmitInt(buf[0],7);
			  return buf;
		  }
}
}
int writeI2CRegister(uint8_t address, uint8_t reg, uint8_t * bytes, int numBytes, int i2CBank){
	  	uint8_t buf[20];
	  	HAL_StatusTypeDef ret;
	  	buf[0]=reg;
	  	int x = 0;
	  	for (x=0;x<(sizeof(bytes)-1);x++){
	  		buf[1+x] = bytes[x];
	  	}
	  	if(i2CBank == 1){
	  		ret = HAL_I2C_Master_Transmit(&hi2c1, address, buf, numBytes+1, HAL_MAX_DELAY);
	  	}
	  	else if(i2CBank == 2){
	  		ret = HAL_I2C_Master_Transmit(&hi2c2, address, buf, numBytes+1, HAL_MAX_DELAY);
	  	}
	  	else if(i2CBank == 3){
	  		ret = HAL_I2C_Master_Transmit(&hi2c3, address, buf, numBytes+1, HAL_MAX_DELAY);
	  	}
	  	else if(i2CBank == 4){
	  		ret = HAL_I2C_Master_Transmit(&hi2c4, address, buf, numBytes+1, HAL_MAX_DELAY);
	  	}
	  	if ( ret != HAL_OK ) {
	  	          return 0;
	  	        }
	  	else {
	  		  return 1;
	  }
}
void configureLEDDriver(){
	uint8_t currentMultiplier = 0b00000001;
	static uint8_t* clear[1];
	clear[0]=0x0;
	uint8_t * buf;
	//reduce the current multiplier to set brightness lower. See if this works. If not, we can work with PWM.
	writeI2CRegister(LED.address, LED.iref_reg, currentMultiplier,1,LED.i2cBank);
	buf = readI2CRegister(LED.address,LED.iref_reg,1,LED.i2cBank);
	uartTransmitInt(buf[0],7);
	//Turn on oscillator. Must be turned on before LED driver functions
	writeI2CRegister(LED.address,LED.mode0_reg,LED.mode0_oscon_value,1,LED.i2cBank);
	//clear the default state of the led register.
	writeI2CRegister(LED.address,LED.led0_reg,clear,1,LED.i2cBank);
	writeI2CRegister(LED.address,LED.led1_reg,clear,1,LED.i2cBank);
	writeI2CRegister(LED.address,LED.led2_reg,clear,1,LED.i2cBank);
	writeI2CRegister(LED.address,LED.led3_reg,clear,1,LED.i2cBank);
	//set the PWM for the tri-color led. Thing is bright so PWM is very low.
	writeI2CRegister(LED.address,LED.led7_pwm,LED.pwm,1,LED.i2cBank);
	writeI2CRegister(LED.address,LED.led8_pwm,LED.pwm,1,LED.i2cBank);
	writeI2CRegister(LED.address,LED.led9_pwm,LED.pwm,1,LED.i2cBank);
}

//Configures specified LED to either fully on or off.
void setErrorLED(int led,_Bool change){
	const uint8_t led0 = 0b00000001;
	const uint8_t led1 = 0b00000100;
	const uint8_t led2 = 0b00010000;
	const uint8_t led3 = 0b01000000;
	const uint8_t led0_pwm = 0b00000010;
	const uint8_t led1_pwm = 0b00001000;
	const uint8_t led2_pwm = 0b00100000;
	const uint8_t led3_pwm = 0b10000000;
  	uint8_t* ledRegisterContents;
  	int ledBitNumber;
  	uint8_t ledRegister;
	if(led <4){
		ledRegister = LED.led0_reg;
		ledBitNumber=led;
	}
	else if(led <8){
		ledRegister = LED.led1_reg;
		ledBitNumber=led-4;
	}
	else if(led <12){
		ledRegister = LED.led2_reg;
		ledBitNumber=led-8;
	}
	else{ //put this in to humor those who want to try using unconnected LEDs
		ledRegister = LED.led3_reg;
		ledBitNumber=led-12;
	}
	ledRegisterContents = readI2CRegister(LED.address,ledRegister, 1, LED.i2cBank);
	switch(ledBitNumber){

	case 0:
		if(change){
			if(led == 8){
				ledRegisterContents[0] |= led0_pwm;
			}
			else{
				ledRegisterContents[0] |= led0;
			}
		}
		else{
			if(led == 8){
				ledRegisterContents[0] &= ~led0_pwm;
			}
			else{
				ledRegisterContents[0] &= ~led0;
			}
		}
		break;

	case 1:
		if(change){
			if(led == 9){
				ledRegisterContents[0] |= led1_pwm;
			}
			else{
				ledRegisterContents[0] |= led1;
			}
		}
		else{
			if(led == 9){
				ledRegisterContents[0] &= ~led1_pwm;
			}
			else{
				ledRegisterContents[0] &= ~led1;
			}
		}
		break;

	case 2:
		if(change){
				ledRegisterContents[0] |= led2;
		}
		else{

			ledRegisterContents[0] &= ~led2;
		}
		break;

	case 3:
		if(change){
			if(led == 7){
				ledRegisterContents[0] |= led3_pwm;
			}
			else{
				ledRegisterContents[0] |= led3;
			}
		}
		else{
			if(led == 7){
				ledRegisterContents[0] &= ~led3_pwm;
			}
			else{
				ledRegisterContents[0] &= ~led3;
			}
		}
		break;
	}
	writeI2CRegister(LED.address,ledRegister,ledRegisterContents,1,LED.i2cBank);
	ledRegisterContents = readI2CRegister(LED.address,ledRegister, 1, LED.i2cBank);
}


float* getADCValues(){
	static float adcValues[20];
	int avgADCCounterValues[20];
	memset(avgADCCounterValues, 0, sizeof(avgADCCounterValues));
	int adcChannelCounter,avgCounter,adcIndex;
	int adc1DataRepeat=22;
	int adc2DataRepeat=8;
	int adc3DataRepeat=12;
	for(adcChannelCounter=0;adcChannelCounter<20;adcChannelCounter++){
		if((adcChannelCounter==Adc.adc0) || (adcChannelCounter==Adc.adc2) || (adcChannelCounter==Adc.adc3)){
			adcIndex=0;
		}
		else if((adcChannelCounter==Adc.adc1) || (adcChannelCounter==Adc.adc14) || (adcChannelCounter==Adc.adc4)){
			adcIndex=2;
		}
		else if((adcChannelCounter==Adc.spareSpiADC) || (adcChannelCounter==Adc.adc15) || (adcChannelCounter==Adc.adc5)){
			adcIndex=4;
		}
		else if((adcChannelCounter == Adc.spareUartADC) || (adcChannelCounter==Adc.configADC) || (adcChannelCounter==Adc.adc6)){
			adcIndex=6;
		}
		else if((adcChannelCounter==Adc.zionADC) || (adcChannelCounter==Adc.adc7)){
			adcIndex=8;
		}
		else if((adcChannelCounter == Adc.spareI2cADC) || (adcChannelCounter==Adc.adc8)){
			adcIndex=10;
		}
		else if(adcChannelCounter==Adc.adc9){
			adcIndex=12;
		}
		else if(adcChannelCounter==Adc.adc10){
			adcIndex=14;
		}
		else if(adcChannelCounter==Adc.adc11){
			adcIndex=16;
		}
		else if(adcChannelCounter==Adc.adc12){
			adcIndex=18;
		}
		else{
			adcIndex=20;
		}
		if((adcChannelCounter == Adc.adc0) || (adcChannelCounter == Adc.adc1) || (adcChannelCounter == Adc.spareSpiADC) || (adcChannelCounter == Adc.spareUartADC)){
			for(avgCounter=0;avgCounter<ADC_AVG_COUNT;avgCounter++){
				int shiftedIndex = adcIndex + (adc2DataRepeat*avgCounter);
				avgADCCounterValues[adcChannelCounter]+=adc2_buf[shiftedIndex];
				if (avgCounter == (ADC_AVG_COUNT-1)){
					avgADCCounterValues[adcChannelCounter] = avgADCCounterValues[adcChannelCounter]/ADC_AVG_COUNT;
				}
			}
		}
		else if((adcChannelCounter == Adc.adc2) || (adcChannelCounter == Adc.adc14) || (adcChannelCounter == Adc.adc15) || (adcChannelCounter == Adc.configADC) || (adcChannelCounter == Adc.zionADC) || (adcChannelCounter == Adc.spareI2cADC)){
			for(avgCounter=0;avgCounter<ADC_AVG_COUNT;avgCounter++){
				int shiftedIndex = adcIndex + (adc3DataRepeat*avgCounter);
				avgADCCounterValues[adcChannelCounter]+=adc3_buf[shiftedIndex];
				if (avgCounter == (ADC_AVG_COUNT-1)){
					avgADCCounterValues[adcChannelCounter] = avgADCCounterValues[adcChannelCounter]/ADC_AVG_COUNT;
				}
			}
		}
		else{
			for(avgCounter=0;avgCounter<ADC_AVG_COUNT;avgCounter++){
				int shiftedIndex = adcIndex + (adc1DataRepeat*avgCounter);
				avgADCCounterValues[adcChannelCounter]+=adc1_buf[shiftedIndex];
				if (avgCounter == (ADC_AVG_COUNT-1)){
					avgADCCounterValues[adcChannelCounter] = avgADCCounterValues[adcChannelCounter]/ADC_AVG_COUNT;
				}
			}
		}
	}
	adcValues[Adc.adc0] = (avgADCCounterValues[Adc.adc0] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.adc1] = (avgADCCounterValues[Adc.adc1] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.adc2] = (avgADCCounterValues[Adc.adc2] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.adc3] = (avgADCCounterValues[Adc.adc3] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.adc4] = (avgADCCounterValues[Adc.adc4] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.adc5] = (avgADCCounterValues[Adc.adc5] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.adc6] = (avgADCCounterValues[Adc.adc6] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.adc7] = (avgADCCounterValues[Adc.adc7] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.adc8] = (avgADCCounterValues[Adc.adc8] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.adc9] = (avgADCCounterValues[Adc.adc9] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.adc10] = (avgADCCounterValues[Adc.adc10] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.adc11] = (avgADCCounterValues[Adc.adc11] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.adc12] = (avgADCCounterValues[Adc.adc12] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.adc13] = (avgADCCounterValues[Adc.adc13] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.adc14] = (avgADCCounterValues[Adc.adc14] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.adc15] = (avgADCCounterValues[Adc.adc15] * Adc.adcDivisor) * Adc.adcResistorDivider;
	adcValues[Adc.spareSpiADC] = (avgADCCounterValues[Adc.spareSpiADC] * Adc.adcDivisor) * Adc.systemResistorDivider;
	adcValues[Adc.spareUartADC] = (avgADCCounterValues[Adc.spareUartADC] * Adc.adcDivisor) * Adc.systemResistorDivider;
	adcValues[Adc.configADC] = (avgADCCounterValues[Adc.configADC] * Adc.adcDivisor) * Adc.systemResistorDivider;
	adcValues[Adc.zionADC] = (avgADCCounterValues[Adc.zionADC] * Adc.adcDivisor) * Adc.systemResistorDivider;
	adcValues[Adc.spareI2cADC] = (avgADCCounterValues[Adc.spareI2cADC] * Adc.adcDivisor) * Adc.systemResistorDivider;
	return adcValues;
}
//// Called when first half of buffer is filled
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
//}
//
//// Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_startHeartbeat */
/**
  * @brief  Function implementing the Heartbeat thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startHeartbeat */
void startHeartbeat(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOI,MCU_HEARTBEAT_Pin);
	  osDelay(500);
  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startADCRead */
/**
* @brief Function implementing the adcRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startADCRead */
void startADCRead(void *argument)
{
  /* USER CODE BEGIN startADCRead */
  /* Infinite loop */
  for(;;)
  {
	  //empty out the data ready variables and the adc3_bufs
	memset(adcRestart,0,sizeof(adcRestart));
	memset(adc1_buf, 0, sizeof(adc1_buf));
	memset(adc2_buf, 0, sizeof(adc2_buf));
	memset(adc3_buf, 0, sizeof(adc3_buf));
	//restart the DMAs.
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, ADC_BUF_LEN);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, ADC_BUF_LEN);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_buf, ADC_BUF_LEN);
    osDelay(1000);
  }
  /* USER CODE END startADCRead */
}

/* USER CODE BEGIN Header_GetDaScreenBlink */
/**
* @brief Function implementing the DatScreenBlink thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GetDaScreenBlink */
void GetDaScreenBlink(void *argument)
{
  /* USER CODE BEGIN GetDaScreenBlink */
  /* Infinite loop */
	 static uint16_t LCD_Blink_White = 0b0001100000000000;
	 static uint16_t LCD_Blink_Black = 0b0001000000000000;
	 int x = 0;
	 float *adcValues;
  for(;;)
  {
	  	 /* setOutputGPIOState(outputGPIOs.mcu3V3_0,x);
	  	  setOutputGPIOState(outputGPIOs.mcu3V3_1,!x);
	  	  setOutputGPIOState(outputGPIOs.mcu3V3_2,x);
	  	  setOutputGPIOState(outputGPIOs.mcu3V3_3,!x);
	  	  setOutputGPIOState(outputGPIOs.configOut_0,x);
	  	  setOutputGPIOState(outputGPIOs.configOut_1,!x);
	  	  setOutputGPIOState(outputGPIOs.configOut_2,x);
	  	  setOutputGPIOState(outputGPIOs.configOut_3,!x);
	  	  setOutputGPIOState(outputGPIOs.out1V8_0,x);
	  	  setOutputGPIOState(outputGPIOs.out1V8_1,!x);
	  	  setOutputGPIOState(outputGPIOs.out1V8_2,x);
	  	  setOutputGPIOState(outputGPIOs.out1V8_3,!x);
	  	  setOutputGPIOState(outputGPIOs.odOut_0,x);
	  	  setOutputGPIOState(outputGPIOs.odOut_1,!x); */
	  	  if(inputButtonSet<5){
	  		  inputButtonSet=5;
	  	  }
	  	  if (adcRestart[0] & adcRestart[1] & adcRestart[2]){
	  		  adcValues = getADCValues();
	  		  float *adcValues1 = adcValues+1;
	  		 // uartTransmitFloat(adcValues,7);
	  	  }
	 	  if (!x) {
	 		  //HAL_GPIO_WritePin(GPIOI,MCU_HEARTBEAT_Pin,GPIO_PIN_SET);
	 		  x=1;

	 		  uartTransmitChar("hello\r\n",7);
	 		  HAL_GPIO_TogglePin(LCD_SS_GPIO_Port,LCD_SS_Pin);
	 		  //HAL_SPI_Transmit(&hspi4, (uint16_t *)&LCD_Blink_White, 1, 100);
	 		  HAL_GPIO_TogglePin(LCD_SS_GPIO_Port,LCD_SS_Pin);
	 	  }
	 	  else{
	 		  //HAL_GPIO_WritePin(GPIOI,MCU_HEARTBEAT_Pin,GPIO_PIN_RESET);
	 		  x=0;
	 		  uartTransmitChar("here\r\n",7);
	 		  HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_SET);
	 		  //HAL_SPI_Transmit(&hspi4, (uint16_t *)&LCD_Blink_Black, 1, 100);
	 		  HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_RESET);
	 	  }
	 	  osDelay(400);
  }
  /* USER CODE END GetDaScreenBlink */
}

/* USER CODE BEGIN Header_startGpioInputRead */
/**
* @brief Function implementing the gpioInputRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startGpioInputRead */
void startGpioInputRead(void *argument)
{
  /* USER CODE BEGIN startGpioInputRead */
  /* Infinite loop */
  for(;;)
  {
	  gpioInputBuf[inputGPIOs.input0] = HAL_GPIO_ReadPin(UI_INPUT0_GPIO_Port,UI_INPUT0_Pin);
	  gpioInputBuf[inputGPIOs.input1] = HAL_GPIO_ReadPin(UI_INPUT1_GPIO_Port,UI_INPUT1_Pin);
	  gpioInputBuf[inputGPIOs.input2] = HAL_GPIO_ReadPin(GPIOD,UI_INPUT2_Pin);
	  gpioInputBuf[inputGPIOs.input3] = HAL_GPIO_ReadPin(GPIOD,UI_INPUT3_Pin);
	  gpioInputBuf[inputGPIOs.input4] = HAL_GPIO_ReadPin(GPIOD,UI_INPUT4_Pin);
	  gpioInputBuf[inputGPIOs.input5] = HAL_GPIO_ReadPin(GPIOD,UI_INPUT5_Pin);
	  gpioInputBuf[inputGPIOs.input6] = HAL_GPIO_ReadPin(GPIOD,UI_INPUT6_Pin);
	  gpioInputBuf[inputGPIOs.input7] = HAL_GPIO_ReadPin(GPIOD,UI_INPUT7_Pin);
	  gpioInputBuf[inputGPIOs.input8] = HAL_GPIO_ReadPin(GPIOD,UI_INPUT8_Pin);
	  gpioInputBuf[inputGPIOs.input9] = HAL_GPIO_ReadPin(GPIOD,UI_INPUT9_Pin);
	  gpioInputBuf[inputGPIOs.input10] = HAL_GPIO_ReadPin(GPIOD,UI_INPUT10_Pin);
	  gpioInputBuf[inputGPIOs.input11] = HAL_GPIO_ReadPin(UI_INPUT11_GPIO_Port,UI_INPUT11_Pin);
	  osDelay(950);
  }
  /* USER CODE END startGpioInputRead */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

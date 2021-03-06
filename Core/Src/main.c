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
//ADC Definitions
#define ADC_BUF_LEN 			5000
#define ADC_AVG_COUNT 			20

//UART Buffer Length Definitions
#define SOC_UART_BUF_LEN		1000
#define SPARE_UART_BUF_LEN		200
#define DEBUG_UART_BUF_LEN		200



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
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart7_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for Heartbeat */
osThreadId_t HeartbeatHandle;
const osThreadAttr_t Heartbeat_attributes = {
  .name = "Heartbeat",
  .priority = (osPriority_t) osPriorityLow5,
  .stack_size = 256 * 4
};
/* Definitions for adcRead */
osThreadId_t adcReadHandle;
const osThreadAttr_t adcRead_attributes = {
  .name = "adcRead",
  .priority = (osPriority_t) osPriorityLow1,
  .stack_size = 256 * 4
};
/* Definitions for DatScreenBlink */
osThreadId_t DatScreenBlinkHandle;
const osThreadAttr_t DatScreenBlink_attributes = {
  .name = "DatScreenBlink",
  .priority = (osPriority_t) osPriorityNormal2,
  .stack_size = 2000 * 4
};
/* Definitions for gpioInputRead */
osThreadId_t gpioInputReadHandle;
const osThreadAttr_t gpioInputRead_attributes = {
  .name = "gpioInputRead",
  .priority = (osPriority_t) osPriorityLow2,
  .stack_size = 256 * 4
};
/* Definitions for navigationTask */
osThreadId_t navigationTaskHandle;
const osThreadAttr_t navigationTask_attributes = {
  .name = "navigationTask",
  .priority = (osPriority_t) osPriorityHigh2,
  .stack_size = 1000 * 4
};
/* Definitions for errorLEDs */
osThreadId_t errorLEDsHandle;
const osThreadAttr_t errorLEDs_attributes = {
  .name = "errorLEDs",
  .priority = (osPriority_t) osPriorityLow5,
  .stack_size = 1024 * 4
};
/* Definitions for zionRead */
osThreadId_t zionReadHandle;
const osThreadAttr_t zionRead_attributes = {
  .name = "zionRead",
  .priority = (osPriority_t) osPriorityHigh1,
  .stack_size = 2000 * 4
};
/* Definitions for bootButtons */
osThreadId_t bootButtonsHandle;
const osThreadAttr_t bootButtons_attributes = {
  .name = "bootButtons",
  .priority = (osPriority_t) osPriorityNormal2,
  .stack_size = 1024 * 4
};
/* Definitions for socUart */
osThreadId_t socUartHandle;
const osThreadAttr_t socUart_attributes = {
  .name = "socUart",
  .priority = (osPriority_t) osPriorityLow7,
  .stack_size = 1024 * 4
};
/* Definitions for debugUart */
osThreadId_t debugUartHandle;
const osThreadAttr_t debugUart_attributes = {
  .name = "debugUart",
  .priority = (osPriority_t) osPriorityLow7,
  .stack_size = 512 * 4
};
/* Definitions for Fault_Events */
osEventFlagsId_t Fault_EventsHandle;
const osEventFlagsAttr_t Fault_Events_attributes = {
  .name = "Fault_Events"
};
/* USER CODE BEGIN PV */
uint16_t adc1_buf[ADC_BUF_LEN];
uint16_t adc2_buf[ADC_BUF_LEN];
uint16_t adc3_buf[ADC_BUF_LEN];
uint8_t adcRestart[3];

//SOC UART Variables
#define SOC_UART			huart5
uint8_t soc_Uart_RX_Buf[SOC_UART_BUF_LEN];

//spare Uart Defintions Placed as Huart is only defined later in the file
#define SPARE_UART			huart4
uint8_t spare_Uart_RX_Buf[SPARE_UART_BUF_LEN];

//debug UART variables and Definitions
#define DEBUG_UART			huart7
uint8_t debug_Uart_RX_Buf[DEBUG_UART_BUF_LEN];

//12 gpio inputs to the Dev UI
uint8_t gpioInputBuf[12];
//14 gpio outputs to the Dev UI
uint8_t gpioOutputState[14];
//error LED state Variable
uint8_t errorLEDState[12];

static uint32_t i, j, k;

//zion status variables
struct zion ZION = {0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//boot mode variables
struct bootModeButtons bootButtons = {0,0,0,0,0,0,0,0,0,0};

struct errorLEDs errorLED = {0,0,0,0,0,0,0,0,0,0,0,0,0};




//int commandByte=1;
//int lineByte=1;
//int lineAmount=SCR_H;
//int nopBytesPerLine= 1;
//int dataBytesPerLine=SCR_W/8;
//int finalNOPByte=1;
//uint8_t transmitBuffer[48482];

uint8_t inputButtonSet = NO_BTN_PRESS; //set to a higher value than any other button priority. 5 is the "unused" state
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
void startNavigationTask(void *argument);
void startErrorLEDs(void *argument);
void startZionRead(void *argument);
void startBootButtons(void *argument);
void startSocUart(void *argument);
void startDebugUart(void *argument);

/* USER CODE BEGIN PFP */
void debugUartTransmitChar(char *message);
void debugUartTransmitStuff(char *message, int size);
void uartTransmitInt(uint16_t *number, int uart);
uint8_t * readI2CRegister(uint8_t address, uint8_t reg, int bytes, int i2CBank);
void setRGBLED(uint8_t R, uint8_t G, uint8_t B);
void setErrorLED(int led, _Bool change);
void configureLEDDriver();
float* getADCValues();
void uartTransmitFloat(float *number, int uart);
uint8_t *getInputGPIOState(void);
void setOutputGPIOState(int gpio, int state);
void outputGPIOBufInitialization();
void setVoltageMux(int comChannel, int voltageChannel, int clear);
void winbondSPIDeviceIDRead(SPI_HandleTypeDef hspi, uint8_t* data);
void spareUartTransmitRead(char *message);
uint8_t debugUartParser();
uint8_t * socUartParser();
//void LCD_DrawSomeLinesSingleLine();
//void LCD_DrawSomeLinesBatchLine();
//void LCD_BlackWhite(int color);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 //static _Bool ON = 1;
 //static _Bool OFF = 0;
  HAL_StatusTypeDef Status = HAL_OK;


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
  memset(errorLEDState,0,sizeof(errorLEDState));
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, ADC_BUF_LEN);
  if (Status != HAL_OK)
  {
  	  DevUI_Error_Handler("ADC1 Failed to start.", Status, 0, 0, true);
  }
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, ADC_BUF_LEN);
  if (Status != HAL_OK)
  {
  	  DevUI_Error_Handler("ADC2 Failed to start.", Status, 0, 0, true);
  }
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_buf, ADC_BUF_LEN);
  if (Status != HAL_OK)
  {
  	  DevUI_Error_Handler("ADC3 Failed to start.", Status, 0, 0, true);
  }
   int x=1;

   HAL_UART_Receive_DMA(&SPARE_UART, spare_Uart_RX_Buf, sizeof(spare_Uart_RX_Buf));
   HAL_UART_Receive_DMA(&SOC_UART, soc_Uart_RX_Buf, sizeof(soc_Uart_RX_Buf));
   HAL_UART_Receive_DMA(&DEBUG_UART, debug_Uart_RX_Buf, sizeof(debug_Uart_RX_Buf));
//  uint8_t data[5];
//
//  for(x=0;x<5;x++){
//	  data[x]=0xfa+x;
//  }
//  writeDataToSpareEEPROM((uint8_t*)data,SPARE_ADDRESS,0x00,sizeof(data),100);
//  for(int x=0;x<5;x++){
//	  data[x]=0;
//  }
//  readDataFromSpareEEPROM((uint8_t*)data,SPARE_ADDRESS,0x00,sizeof(data),100);
//  data[1]=0;
//  uint8_t spiDataRead[6];
//  memset(data,0x00, sizeof(data));
//  winbondSPIDeviceIDRead(hspi5,(uint8_t*)spiDataRead);
//  //char buf[30];
//  spareUartTransmitRead("Lets see what comes out!");
//  x=0;

  configureLEDDriver();
  
    setErrorLED(0,ON);
    HAL_Delay(1000);
    setErrorLED(1,OFF);
    setErrorLED(8,ON);
    HAL_Delay(1000);
    setErrorLED(8,OFF);
    setErrorLED(9,ON);
    HAL_Delay(1000);
    setErrorLED(8,ON);
    setErrorLED(9,ON);
    HAL_Delay(1000);
    setErrorLED(9,OFF);

    printf("Welcome to DevUI!\r\n");
    printf("DevUI SW Version: 1.0\r\n");
//    BTN0_ON;
//    HAL_Delay(300);
//    BTN1_ON;
//    HAL_Delay(300);
//    BTN2_ON;
//    HAL_Delay(300);
//    BTN3_ON;
//    HAL_Delay(300);
//    BTN4_ON;
//    HAL_Delay(300);
//    BTN5_ON;
//    HAL_Delay(300);
//    EDL_SW_ON;
//    HAL_Delay(300);
//    EX_SW_ON;
//    HAL_Delay(300);
//    BTN0_OFF;
//    HAL_Delay(300);
//    BTN1_OFF;
//    HAL_Delay(300);
//    BTN2_OFF;
//    HAL_Delay(300);
//    BTN3_OFF;
//    HAL_Delay(300);
//    BTN4_OFF;
//    HAL_Delay(300);
//    BTN5_OFF;
//    HAL_Delay(300);
//    EDL_SW_OFF;
//    HAL_Delay(300);
//    EX_SW_OFF;

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

  /* creation of navigationTask */
  navigationTaskHandle = osThreadNew(startNavigationTask, NULL, &navigationTask_attributes);

  /* creation of errorLEDs */
  errorLEDsHandle = osThreadNew(startErrorLEDs, NULL, &errorLEDs_attributes);

  /* creation of zionRead */
  zionReadHandle = osThreadNew(startZionRead, NULL, &zionRead_attributes);

  /* creation of bootButtons */
  bootButtonsHandle = osThreadNew(startBootButtons, NULL, &bootButtons_attributes);

  /* creation of socUart */
  socUartHandle = osThreadNew(startSocUart, NULL, &socUart_attributes);

  /* creation of debugUart */
  debugUartHandle = osThreadNew(startDebugUart, NULL, &debugUart_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of Fault_Events */
  Fault_EventsHandle = osEventFlagsNew(&Fault_Events_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* Fault Events Bit map
	   bit 0: Zion Fault
	   bit 1: vsysPMI Fault
	   bit 2: fault 3
	   bit 3: fault 4
	   bit 4: fault 5
	   bit 5: fault 6
	   bit 6: fault 7
	   bit 7: fault 8
	   bit 8: fault 9
	   bit 9: standard boot
	   bit 10: edl boot
	   bit 11: uefi boot
   */
  if (Fault_EventsHandle == NULL)
  {
	  // Event flags object not created, handle failure.
	  DevUI_Error_Handler("Faults Event Flag could not be created.\r\n",HAL_ERROR,0,0,true);
  }
  else
  {
	  // clear all event flags
	  osEventFlagsClear(Fault_EventsHandle, 0xFFFF);
  }
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
		  //HAL_SPI_Transmit(&hspi4, (uint16_t *)&LCD_Blink_White, 1, 100);
		  HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_RESET);
	  }
	  else{
		  HAL_GPIO_WritePin(GPIOI,MCU_HEARTBEAT_Pin,GPIO_PIN_RESET);
		  x=0;
		  //uartTransmitChar("here\r\n",7);
		  HAL_GPIO_WritePin(LCD_SS_GPIO_Port,LCD_SS_Pin,GPIO_PIN_SET);
		  //HAL_SPI_Transmit(&hspi4, (uint16_t *)&LCD_Blink_Black, 1, 100);
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
  hi2c3.Init.Timing = 0x100029FE;
  hi2c3.Init.OwnAddress1 = 152;
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
  /** I2C Enable Fast Mode Plus
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C3);
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
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
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

// Generic Error Handler for DevUI HAL hardware.
// char *msg is an error message that can be sent to the handler from the caller.
// err_param1 & err_param2 are additional error parameters that can be printed.
// For I2C errors, I2C device address in param1 and register address in param2.
void DevUI_Error_Handler(char *msg, HAL_StatusTypeDef ErrorCode, uint8_t err_param1, uint8_t err_param2, uint8_t critical_fault)
{
	__disable_irq();
	printf("ERROR: %s" " Code: %d Param1: 0x%x Param2: 0x%x\r\n", msg, ErrorCode, err_param1, err_param2);

	// Set error LED
	//errorLED.fault9 = true;
	// Use event group flag to indicate an error for the startErrorLED task.

	// If the fault is labeled as "critical" stay here.  Else keep running RTOS.
	if (critical_fault == true)
	{
	  while (1)
	  {
		  // HAL error occurred, sit here.  Do not continue to run OS.
	  }
	}
	else
	{
		__enable_irq();
		return;
	}
}

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart7, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

void outputGPIOBufInitialization(){
	memset(gpioOutputState,0,sizeof(gpioOutputState));
	setOutputGPIOState(outputGPIOs.odOut_0, ON); //set OD to high impediance
	setOutputGPIOState(outputGPIOs.odOut_1, ON); //set OD to high impediance
	gpioOutputState[outputGPIOs.odOut_0]=1;
	gpioOutputState[outputGPIOs.odOut_1]=1;
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

void debugUartTransmitChar(char *message){
	char uart_buf[200];
	int uart_buf_len;
	uart_buf_len = sprintf(uart_buf, message);
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)uart_buf, uart_buf_len,100);
}
void debugUartTransmitStuff(char *message, int size){
	char uart_buf[200];
	int uart_buf_len;
	uart_buf_len = sprintf(uart_buf, message);
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)uart_buf, size,100);
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
	char *err_msg;
  	if(i2CBank == 1){
  		ret = HAL_I2C_Master_Transmit(&hi2c1, address, buf, 1, HAL_MAX_DELAY);
  		err_msg = "Failed I2C Read (Transmit) bank 1.";
  	}
  	else if(i2CBank == 2){
  		ret = HAL_I2C_Master_Transmit(&hi2c2, address, buf, 1, HAL_MAX_DELAY);
  		err_msg = "Failed I2C Read (Transmit) bank 2.";
  	}
  	else if(i2CBank == 3){
  		ret = HAL_I2C_Master_Transmit(&hi2c3, address, buf, 1, HAL_MAX_DELAY);
  		err_msg = "Failed I2C Read (Transmit) bank 3.";
  	}
  	else if(i2CBank == 4){
  		ret = HAL_I2C_Master_Transmit(&hi2c4, address, buf, 1, HAL_MAX_DELAY);
  		err_msg = "Failed I2C Read (Transmit) bank 4.";
  	}
	  if ( ret != HAL_OK ) {
		  	  DevUI_Error_Handler(err_msg, ret, address, reg, false);
	          return (uint8_t*)0xfe;
	        }
	  else {
		  if(i2CBank == 1){
				ret = HAL_I2C_Master_Receive(&hi2c1, address, buf, bytes, HAL_MAX_DELAY);
				err_msg = "Failed I2C Read (Receive) bank 1.";
			}
			else if(i2CBank == 2){
				ret = HAL_I2C_Master_Receive(&hi2c2, address, buf, bytes, HAL_MAX_DELAY);
				err_msg = "Failed I2C Read (Receive) bank 2.";
			}
			else if(i2CBank == 3){
				ret = HAL_I2C_Master_Receive(&hi2c3, address, buf, bytes, HAL_MAX_DELAY);
				err_msg = "Failed I2C Read (Receive) bank 3.";
			}
			else if(i2CBank == 4){
				ret = HAL_I2C_Master_Receive(&hi2c4, address, buf, bytes, HAL_MAX_DELAY);
				err_msg = "Failed I2C Read (Receive) bank 4.";
			}
		  if ( ret != HAL_OK ) {
			  	  DevUI_Error_Handler(err_msg, ret, address, reg, false);
		          return (uint8_t*)0xfe;
		        }
		  else{
			  //uartTransmitInt(buf[0],7);
			  return buf;
		  }
}
}
int writeI2CRegister(uint8_t address, uint8_t reg, uint8_t * bytes, int numBytes, int i2CBank){
  	uint8_t buf[20];
  	HAL_StatusTypeDef ret;
  	buf[0]=reg;
  	int x = 0;
  	char *err_msg;
  	for (x=0;x<(sizeof(bytes)-1);x++){
  		buf[1+x] = bytes[x];
  	}
  	if(i2CBank == 1){
  		ret = HAL_I2C_Master_Transmit(&hi2c1, address, buf, numBytes+1, HAL_MAX_DELAY);
  		err_msg = "Failed I2C write bank 1.";
  	}
  	else if(i2CBank == 2){
  		ret = HAL_I2C_Master_Transmit(&hi2c2, address, buf, numBytes+1, HAL_MAX_DELAY);
  		err_msg = "Failed I2C write bank 2.";
  	}
  	else if(i2CBank == 3){
  		ret = HAL_I2C_Master_Transmit(&hi2c3, address, buf, numBytes+1, HAL_MAX_DELAY);
  		err_msg = "Failed I2C write bank 3.";
  	}
  	else if(i2CBank == 4){
  		ret = HAL_I2C_Master_Transmit(&hi2c4, address, buf, numBytes+1, HAL_MAX_DELAY);
  		err_msg = "Failed I2C write bank 4.";
  	}
  	if (ret != HAL_OK)
  	{
  		DevUI_Error_Handler(err_msg, ret, address, reg, false);
  		return ret;
  	}
  	else
  	{
  		return HAL_OK;
  	}
}
void configureLEDDriver(){
	uint8_t currentMultiplier = 0b00000001;
	static uint8_t* clear[1];
	clear[0]=0x0;
	uint8_t * buf;
	//reduce the current multiplier to set brightness lower. See if this works. If not, we can work with PWM.
	writeI2CRegister(LED.address, LED.iref_reg, (uint8_t*)currentMultiplier,1,LED.i2cBank);
	buf = readI2CRegister(LED.address,LED.iref_reg,1,LED.i2cBank);
	//uartTransmitInt(buf[0],7);
	//Turn on oscillator. Must be turned on before LED driver functions
	writeI2CRegister(LED.address,LED.mode0_reg,(uint8_t*)LED.mode0_oscon_value,1,LED.i2cBank);
	//clear the default state of the led register.
	writeI2CRegister(LED.address,LED.led0_reg,(uint8_t*)clear,1,LED.i2cBank);
	writeI2CRegister(LED.address,LED.led1_reg,(uint8_t*)clear,1,LED.i2cBank);
	writeI2CRegister(LED.address,LED.led2_reg,(uint8_t*)clear,1,LED.i2cBank);
	writeI2CRegister(LED.address,LED.led3_reg,(uint8_t*)clear,1,LED.i2cBank);
	//set the PWM for the tri-color led. Thing is bright so PWM is very low.
	writeI2CRegister(LED.address,LED.led7_pwm,(uint8_t*)LED.pwm,1,LED.i2cBank);
	writeI2CRegister(LED.address,LED.led8_pwm,(uint8_t*)LED.pwm,1,LED.i2cBank);
	writeI2CRegister(LED.address,LED.led9_pwm,(uint8_t*)LED.pwm,1,LED.i2cBank);
}

//Configure & set RGB LED
void setRGBLED(uint8_t R, uint8_t G, uint8_t B)
{
	setErrorLED(RED, R);
	setErrorLED(GREEN, G);
	setErrorLED(BLUE, B);

	return;
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
	//intialize a static float array to return from the method. Make static to avoid the data changing on return
	static float adcValues[21];
	//make an integer array to store the adc counts. ADC counts are out of 4096
	int avgADCCounterValues[21];
	//empty the integer array
	memset(avgADCCounterValues, 0, sizeof(avgADCCounterValues));
	int adcChannelCounter,avgCounter,adcIndex;
	//variables to denote what interval the data in the adc buffers repeats. ADC format is [data0, 0x00, data1, 0x00...]
	//the interval is (#ofchannels activated on bank) * 2
	int adc1DataRepeat=22;
	int adc2DataRepeat=8;
	int adc3DataRepeat=12;
	//iterate through all 21 adc channels...
	for(adcChannelCounter=0;adcChannelCounter<21;adcChannelCounter++){
		//for the first channels of the adc banks (ADC Bank 1's first channel is ADC3, ADC bank 2's first channel is ADC0, ADC bank 3's first channel is adc2
		if((adcChannelCounter==Adc.adc0) || (adcChannelCounter==Adc.adc2) || (adcChannelCounter==Adc.adc3)){
			//first data entry for each buffer will be the data for these adcs
			adcIndex=0;
		}
		//for the second channels of the adc banks (ADC Bank 1's second channel is ADC4, ADC bank 2's second channel is ADC1, ADC bank 3's second channel is adc14
		else if((adcChannelCounter==Adc.adc1) || (adcChannelCounter==Adc.adc14) || (adcChannelCounter==Adc.adc4)){
			adcIndex=2;
		}
		//for the third channels of the adc banks (ADC Bank 1's third channel is ADC5, ADC bank 2's third channel is spareSpiADC, ADC bank 3's third channel is adc15
		else if((adcChannelCounter==Adc.spareSpiADC) || (adcChannelCounter==Adc.adc15) || (adcChannelCounter==Adc.adc5)){
			adcIndex=4;
		}
		//for the fourth channels of the adc banks (ADC Bank 1's fourth channel is adc6, ADC bank 2's fourth channel is spareUARTADC, ADC bank 3's fourth channel is configADC
		else if((adcChannelCounter == Adc.spareUartADC) || (adcChannelCounter==Adc.configADC) || (adcChannelCounter==Adc.adc6)){
			adcIndex=6;
		}
		//for the fifth channels of the adc banks (ADC Bank 1's fifth channel is ADC7, ADC bank 3's fifth channel is zionADC
		else if((adcChannelCounter==Adc.zionADC) || (adcChannelCounter==Adc.adc7)){
			adcIndex=8;
		}
		//for the sixth channels of the adc banks (ADC Bank 1's sixth channel is ADC8, ADC bank 3's sixth channel is spareI2cADC
		else if((adcChannelCounter == Adc.spareI2cADC) || (adcChannelCounter==Adc.adc8)){
			adcIndex=10;
		}
		//for the seventh channels of the adc banks (ADC Bank 1's seventh channel is ADC9)
		else if(adcChannelCounter==Adc.adc9){
			adcIndex=12;
		}
		//for the eighth channels of the adc banks (ADC Bank 1's eighth channel is ADC10)
		else if(adcChannelCounter==Adc.adc10){
			adcIndex=14;
		}
		//for the nineth channels of the adc banks (ADC Bank 1's nineth channel is ADC11)
		else if(adcChannelCounter==Adc.adc11){
			adcIndex=16;
		}
		//for the tenth channels of the adc banks (ADC Bank 1's tenth channel is ADC12)
		else if(adcChannelCounter==Adc.adc12){
			adcIndex=18;
		}
		//for the eleventh channels of the adc banks (ADC Bank 1's eleventh channel is ADC13)
		else{
			adcIndex=20;
		}
		//for the channels that belong to the second ADC bank
		if((adcChannelCounter == Adc.adc0) || (adcChannelCounter == Adc.adc1) || (adcChannelCounter == Adc.spareSpiADC) || (adcChannelCounter == Adc.spareUartADC)){
			//parse through the buffers to grab enough values to make the asked for average amount
			for(avgCounter=0;avgCounter<ADC_AVG_COUNT;avgCounter++){
				//adjust the index to match the next data point in the buffer
				int shiftedIndex = adcIndex + (adc2DataRepeat*avgCounter);
				//add it to the rolling average count
				avgADCCounterValues[adcChannelCounter]+=adc2_buf[shiftedIndex];
				if (avgCounter == (ADC_AVG_COUNT-1)){
					//at the end, divide the total amount to get our averaged Value
					avgADCCounterValues[adcChannelCounter] = avgADCCounterValues[adcChannelCounter]/ADC_AVG_COUNT;
				}
			}
		}
		//for the channels that belong to the third ADC bank
		else if((adcChannelCounter == Adc.adc2) || (adcChannelCounter == Adc.adc14) || (adcChannelCounter == Adc.adc15) || (adcChannelCounter == Adc.configADC) || (adcChannelCounter == Adc.zionADC) || (adcChannelCounter == Adc.spareI2cADC)){
			for(avgCounter=0;avgCounter<ADC_AVG_COUNT;avgCounter++){
				int shiftedIndex = adcIndex + (adc3DataRepeat*avgCounter);
				avgADCCounterValues[adcChannelCounter]+=adc3_buf[shiftedIndex];
				if (avgCounter == (ADC_AVG_COUNT-1)){
					avgADCCounterValues[adcChannelCounter] = avgADCCounterValues[adcChannelCounter]/ADC_AVG_COUNT;
				}
			}
		}
		//for the channels that belong to the first ADC bank
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
	//for adc inputs with names ADC#, true value is found by taking the average, multiplying it by the divisor(3.3/4096), and then multiplying by the resistor divider (3)
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
	//for other adc inputs, true value is found by taking the average, multiplying it by the divisor(3.3/4096), and then multiplying by the resistor divider (2)
	adcValues[Adc.spareSpiADC] = (avgADCCounterValues[Adc.spareSpiADC] * Adc.adcDivisor) * Adc.systemResistorDivider;
	adcValues[Adc.spareUartADC] = (avgADCCounterValues[Adc.spareUartADC] * Adc.adcDivisor) * Adc.systemResistorDivider;
	adcValues[Adc.configADC] = (avgADCCounterValues[Adc.configADC] * Adc.adcDivisor) * Adc.systemResistorDivider;
	adcValues[Adc.zionADC] = (avgADCCounterValues[Adc.zionADC] * Adc.adcDivisor) * Adc.systemResistorDivider;
	adcValues[Adc.spareI2cADC] = (avgADCCounterValues[Adc.spareI2cADC] * Adc.adcDivisor) * Adc.systemResistorDivider;
	return adcValues;
}

void setVoltageMux(int comChannel, int voltageChannel, int clear){
	uint8_t dataWriteCOMA[1];
	uint8_t dataWriteCOMB[1];
	int x;
	if((comChannel) & (!clear)){
		dataWriteCOMA[0] = socI2cVoltageMux.clearSwitches;
		dataWriteCOMB[0]= voltageChannel;
	}
	else if((!comChannel) & (!clear)){
		dataWriteCOMA[0] = voltageChannel;
		dataWriteCOMB[0]= socI2cVoltageMux.clearSwitches;
	}
	else{
		dataWriteCOMA[0] = socI2cVoltageMux.clearSwitches;
		dataWriteCOMB[0]= socI2cVoltageMux.clearSwitches;
	}
	x = writeI2CRegister(socI2cVoltageMux.address, socI2cVoltageMux.CMD_A_reg, dataWriteCOMA,sizeof(dataWriteCOMA), socI2cVoltageMux.i2cBank);
	x = writeI2CRegister(socI2cVoltageMux.address, socI2cVoltageMux.CMD_B_reg, dataWriteCOMB,sizeof(dataWriteCOMB), socI2cVoltageMux.i2cBank);
}


//// Called when first half of buffer is filled
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
//}
//
//// Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
}

void winbondSPIDeviceIDRead(SPI_HandleTypeDef hspi, uint8_t* data){
	uint8_t dataSent[1] = {0x9f};
	SPARE_SS_ON;
	HAL_SPI_TransmitReceive(&hspi, (uint8_t*)dataSent, data,4,100);
	SPARE_SS_OFF;
	int x;
}

void spareUartTransmitRead(char *message){
	char uart_buf[200];
	char uart_receive_buf[200];
	int uart_buf_len;
	uart_buf_len = sprintf(uart_buf, message);
	HAL_UART_Transmit(&SPARE_UART,(uint8_t *)uart_buf, uart_buf_len,100);
	//HAL_UART_Receive(&SPARE_UART,(uint8_t*)uart_receive_buf, sizeof(uart_receive_buf),1000);
	int x;
}

uint8_t debugUartParser(){
	int x;
	uint8_t  key_uint8[4];
	uint8_t var_Seen[4];
	key_uint8[0] = (uint8_t)'G';
	key_uint8[1] = (uint8_t)'I';
	key_uint8[2] = (uint8_t)'V';
	key_uint8[3] = (uint8_t)'E';
	//memcpy(key_uint8,(const uint8_t*)key, 4);
	x=5;

	for(x=0;x<sizeof(debug_Uart_RX_Buf);x++){
		if(debug_Uart_RX_Buf[x]==key_uint8[0]){
			var_Seen[0] = 1;
		}
		else if(debug_Uart_RX_Buf[x]==key_uint8[1] && var_Seen[0]){
			var_Seen[1] = 1;
		}
		else if(debug_Uart_RX_Buf[x]==key_uint8[2] && var_Seen[1]){
			var_Seen[2] = 1;
		}
		else if(debug_Uart_RX_Buf[x]==key_uint8[3] && var_Seen[2]){
			var_Seen[3] = 1;
			break;
		}
		else{
			memset(var_Seen,0x00,sizeof(var_Seen));
		}
	}
	if(var_Seen[3]){
		memset(debug_Uart_RX_Buf,0x00,sizeof(debug_Uart_RX_Buf));
		return true;
	}
	else{
		return false;
	}
}

uint8_t * socUartParser(){
	int x;
	uint8_t  key_uint8[4];
	static uint8_t var_Seen[15];
	memset(var_Seen,0x00,sizeof(var_Seen));
	static uint8_t failure[15];
	memset(failure, 0xff,sizeof(failure));
	key_uint8[0] = (uint8_t)'F';
	key_uint8[1] = (uint8_t)'F';
	key_uint8[2] = (uint8_t)'U';
	key_uint8[3] = (uint8_t)':';
	//memcpy(key_uint8,(const uint8_t*)key, 4);
	x=5;
	int ffuBytes=4;
	for(x=0;x<sizeof(soc_Uart_RX_Buf);x++){
		if(soc_Uart_RX_Buf[x]==key_uint8[0] && var_Seen[0]==0){
			var_Seen[0] = 1;
		}
		else if(soc_Uart_RX_Buf[x]==key_uint8[1] && var_Seen[0]){
			var_Seen[1] = 1;
		}
		else if(soc_Uart_RX_Buf[x]==key_uint8[2] && var_Seen[1]){
			var_Seen[2] = 1;
		}
		else if(soc_Uart_RX_Buf[x]==key_uint8[3] && var_Seen[2]){
			var_Seen[3] = 1;
			//break;
		}
		else if(var_Seen[3]){
			if(ffuBytes<sizeof(var_Seen)){
				var_Seen[ffuBytes] = soc_Uart_RX_Buf[x];
				ffuBytes++;
			}
			else{
				break;
			}
		}
		else{
			memset(var_Seen,0x00,sizeof(var_Seen));
		}
	}
	if(var_Seen[3]){

		memset(soc_Uart_RX_Buf,0x00,sizeof(soc_Uart_RX_Buf));
		return var_Seen;
	}
	else{
		return failure;
	}
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
	  //spareUartTransmitRead("YOYOYO!\r\n");
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
  HAL_StatusTypeDef Status = HAL_OK;
  /* Infinite loop */
  for(;;)
  {
	  // Clear HAL fault LED
	  //errorLED.fault9 = false;
	  //empty out the data ready variables and the adc3_bufs
	memset(adcRestart,0,sizeof(adcRestart));
	memset(adc1_buf, 0, sizeof(adc1_buf));
	memset(adc2_buf, 0, sizeof(adc2_buf));
	memset(adc3_buf, 0, sizeof(adc3_buf));
	//restart the DMAs.
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, ADC_BUF_LEN);
	if (Status != HAL_OK)
	{
		DevUI_Error_Handler("ADC1 Failed read.", Status, 0, 0, true);
	}
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, ADC_BUF_LEN);
	if (Status != HAL_OK)
	{
		DevUI_Error_Handler("ADC2 Failed read.", Status, 0, 0, true);
	}
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_buf, ADC_BUF_LEN);
	if (Status != HAL_OK)
	{
		DevUI_Error_Handler("ADC3 Failed read.", Status, 0, 0, true);
	}
    osDelay(600);
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
//	 int x = 0;
//	 float *adcValues;
//	 HAL_StatusTypeDef ret;
	 initializeDisplay();
	 uint32_t ulNotifiedValue;
	 uint8_t button_val = 0;
	 uint8_t menu_val = 0;
	 uint8_t running_menu = 0;
//	 int *readI2c;
//	 int zionCleared=0;
	   for(;;)
	   {
	 	  ulNotifiedValue = 0;
	 	  xTaskNotifyWait(NOTIFY_NOCLEAR, NOTIFY_CLEARALL, &ulNotifiedValue, portMAX_DELAY);
	 	  // button press decode
	 	  button_val = (ulNotifiedValue & NOTIFY_BTN_MASK);
	 	  menu_val = ((ulNotifiedValue & NOTIFY_MENU_MASK) >> NOTIFY_MENU_BIT);
	 	  running_menu = ((ulNotifiedValue & NOTIFY_RUN_MENU_MASK) >> NOTIFY_MENU_RUN_BIT);
	 	  //if(!(x%400)){
	 	//	  bootButtons.btn5=1;
	 		//  x++;
	 	  //}
	 	  //setVoltageMux(COMA,socI2cVoltageMux.enableSW2,0);

	 	  //readI2c = parseZionEEPROM(SOC_ADDRESS);
	 	  //int blah = *(readI2c+4);
//		  HAL_GPIO_WritePin(ZION_PWR_EN_GPIO_Port,ZION_PWR_EN_Pin,1);
//	 	  writeZionBinaries();
//	 	  readI2c = parseZionEEPROM(ASIC_ADDRESS);
//	 	  int blah = *(readI2c);
//	 	  blah = *(readI2c+2);
//	 	  blah = *(readI2c+3);
//	 	  //readI2c = parseZionEEPROM(DISPLAY_ADDRESS);
	 	  //blah = *(readI2c+4);
//	 	  clearEEPROM(SOC_ADDRESS);
//	 	  readI2c = parseZionEEPROM(SOC_ADDRESS);
//	 	  blah = *(readI2c+5);
	 	  //clearEEPROM(SOC_ADDRESS);
	 	  //clearEEPROM(ASIC_ADDRESS);
	 	  //clearEEPROM(DISPLAY_ADDRESS);
//	 	  readDataFromEEPROM((uint8_t*)readI2c,SOC_ADDRESS,0x0,sizeof(readI2c), 500);
//	 	  printf(readI2c[0]);
//	 	  readDataFromEEPROM((uint8_t*)readI2c,ASIC_ADDRESS,0x0,sizeof(readI2c), 500);
//	 	  printf(readI2c[0]);
//	 	  readDataFromEEPROM((uint8_t*)readI2c,DISPLAY_ADDRESS,0x0,sizeof(readI2c), 500);
//	 	  printf(readI2c[0]);

	 	  //binaryToArray(readBinary,"Kanu");
//	 	  printf(*readI2c);
	 //	  printf("uNotifiedValue %d\r\n", ulNotifiedValue);
	 //	  printf("running_menu: %d\r\n", running_menu);
	 //	  printf("highlighed menu: %d\n\r", menu_val);
	 //	  printf("button_press: %d\r\n", button_val);

	 	  // If the BACK button was pressed, just run the SEL button case with the previous menu

		  switch(running_menu)
		  {
		  case BOOT_MENU:
		  {
			  //printf("BOOT_MENU\r\n");
			  drawBootMenu(menu_val, button_val, running_menu);
			  //uartTransmitChar("switch BOOT_MENU\r\n",7);
			  break;
		  }
		  case MAIN_MENU:
		  {
			  //printf("MAIN_MENU\r\n");
			  drawMainMenu(menu_val);
			  //uartTransmitChar("switch MAIN_MENU\r\n",7);
			  break;
		  }
		  case STATUS_MENU:
		  {
			  //printf("STATUS_MENU\r\n");
			  drawStatusMenu(menu_val);
			  //uartTransmitChar("switch STATUS_MENU\r\n",7);
			  break;
		  }
		  case SYSTEM_INFO_MENU:
		  {
			  //printf("SYSTEM INFO MENU\r\n");
			  //uartTransmitChar("switch SYSTEM INFO_MENU\r\n",7);
			  drawSystemInfoMenu(menu_val);
			  break;
		  }
		  }
	 	  osDelay(100);
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

/* USER CODE BEGIN Header_startNavigationTask */
/**
* @brief Function implementing the navigationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startNavigationTask */
void startNavigationTask(void *argument)
{
  /* USER CODE BEGIN startNavigationTask */
	uint8_t menu_highlight = MENU_TOP;	// variable indicates what menu item is currently being highlighted
	uint8_t	menu_run = MAIN_MENU;		// variable to track what menu is currently running
	uint8_t prev_menu = menu_run;		// variable to track what the previous menu running was, this is used for the BACK button
	uint8_t menu_Max_Items = MAX_MENU_ITEMS_MAIN_MENU;
	uint8_t prev_menu_highlight = menu_highlight; //variable to track previous menu highlight
//	int zionCleared=0;
	//uint8_t button_press = 1;
	// Clear button flags here

  /* Infinite loop */
  for(;;)
  {
	switch(inputButtonSet)
	{
	case UP:
	{
		if ((menu_highlight == MENU_TOP) || bootButtons.bootModeSet)
		{
			//do nothing
		}
		else
		{
			menu_highlight = menu_highlight - 1;
			// task notify the display task with UP and current highlighted item
			// task notification U32 bits defined as:
			// [0:3]: menu button flags [0]:UP, [1]:DWN, [2]:SEL, [3]:Reserved
			// [4:7]: menu indicator highlight flags
			// [8:11]: currently running menu flags
			xTaskNotify(DatScreenBlinkHandle, (UP | (menu_highlight << NOTIFY_MENU_BIT) | (menu_run << NOTIFY_MENU_RUN_BIT)), eSetValueWithoutOverwrite);
		}
		break;
	}
	case DWN:
	{
		if ((menu_highlight >= menu_Max_Items) || (bootButtons.bootModeSet))
		{
			//do nothing
		}
		else
		{
			menu_highlight = menu_highlight + 1;
			// task notify the display task with DWN and current highlighted item
			// task notification U32 bits defined as:
			// [0:3]: menu button flags [0]:UP, [1]:DWN, [2]:SEL, [3]:Reserved
			// [4:7]: menu selection flags
			// [8:11]: currently running menu flags
			// [12:15]: previously running menu flags
			xTaskNotify(DatScreenBlinkHandle, (DWN | (menu_highlight << NOTIFY_MENU_BIT) | (menu_run << NOTIFY_MENU_RUN_BIT)), eSetValueWithoutOverwrite);
		}
		break;
	}
	case BACK:
	{
		if(!(bootButtons.bootModeSet)){
			menu_run = prev_menu;
			menu_highlight = prev_menu_highlight; //set the highlight back to where it was for the previous menu.
		}
		// task notify the display task with SEL and what menu to run
		// task notification U32 bits defined as:
		// [0:3]: menu button flags [0]:UP, [1]:DWN, [2]:SEL, [3]:Reserved
		// [4:7]: menu selection flags
		// [8:11]: currently running menu flags
		// [12:15]: previously running menu flags
		xTaskNotify(DatScreenBlinkHandle, (BACK | (menu_highlight << NOTIFY_MENU_BIT) | (menu_run << NOTIFY_MENU_RUN_BIT)), eSetValueWithoutOverwrite);
		break;
	}
	case SEL:
	{
		if(menu_run==MAIN_MENU){
			prev_menu = menu_run;		// save currently running menu for BACK button
			menu_run = menu_highlight+1;	// update the currently running menu to what the user SELECTED. Requires +1 to match with menu values
			prev_menu_highlight = menu_highlight; //keep track of the previous menu's highlight for when back is pressed
			menu_highlight=MENU_TOP; //reset the menu highlight for the next menu
		}
//			printf("menu_run: %d\r\n", menu_run);
//			printf("prev_menu: %d\r\n", prev_menu);
//			printf("menu_run_adjust: %d\r\n", (menu_run << NOTIFY_MENU_RUN_BIT));
		// task notify the display task with SEL and what menu to run
		// task notification U32 bits defined as:
		// [0:3]: menu button flags [0]:UP, [1]:DWN, [2]:SEL, [3]:Reserved
		// [4:7]: menu selection flags
		// [8:11]: currently running menu flags
		// [12:15]: previously running menu flags
		xTaskNotify(DatScreenBlinkHandle, (SEL | (menu_highlight << NOTIFY_MENU_BIT) | (menu_run << NOTIFY_MENU_RUN_BIT)), eSetValueWithoutOverwrite);
		break;
	}
	default:
		// task notify the display task with no button press.  Just refresh the current running menu.
		xTaskNotify(DatScreenBlinkHandle, (NO_BTN_PRESS | (menu_highlight << NOTIFY_MENU_BIT) | (menu_run << NOTIFY_MENU_RUN_BIT)), eSetValueWithoutOverwrite);
		break;
	}
	//initialize the max indicator for each menu
	switch(menu_run){

	case BOOT_MENU:{
		menu_Max_Items = MAX_MENU_ITEMS_BOOT_MENU;
		break;
	}
	case MAIN_MENU:{
		menu_Max_Items = MAX_MENU_ITEMS_MAIN_MENU;
		break;
	}
	case STATUS_MENU:{
		menu_Max_Items = MAX_MENU_ITEMS_STATUS_MENU;
		break;
	}
	case SYSTEM_INFO_MENU:{
		menu_Max_Items = MAX_MENU_ITEMS_SYSTEM_INFO_MENU;
		break;
	}
	default:
		menu_Max_Items = MAX_MENU_ITEMS_BOOT_MENU;
		break;
	}
	inputButtonSet = NO_BTN_PRESS;
    osDelay(200);
  }

  /* USER CODE END startNavigationTask */
}

/* USER CODE BEGIN Header_startErrorLEDs */
/**
* @brief Function implementing the errorLEDs thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startErrorLEDs */
void startErrorLEDs(void *argument)
{
  /* USER CODE BEGIN startErrorLEDs */
  /* Infinite loop */
	int i2cCheck;
	uint8_t R;
	uint8_t G;
	uint8_t B;
	float * presentADCValues;

	// An array of voltage rails that are monitored for faults.  Each element maps to the apporpriate ADC channel for monitoring
	// PLATFORM TEMPLATE: edit this array to include the voltages that you would like to monitor for faults.  The names are defined in main.h
	uint8_t monitor_rails[] = {VSYS, V1, V2, V3};

	// An array of falling edge fault thresholds for the voltage rails that are monitored for faults.  Size of the array and index for each fault should match the voltage name in monitor_rails[].
	double monitor_fault_thresholds[] = {VSYS_FLT, V1_FLT, V2_FLT, V3_FLT};

	// An array of platform gpio inputs that are monitored for faults.  Each element maps to the appropriate STM GPIO input for monitoring
	// PLATFORM TEMPLATE: edit this array to include the voltages that you would like to monitor for faults.  The names are defined in main.h
	uint8_t monitor_gpio[] = {SOC_IN0, SOC_IN3, SOC_IN8, SOC_IN11};

	// An array of logic fault thresholds for the GPIO input rails that are monitored for faults.  The fault thresholds should match the mapping used in monitor_gpio[].
	uint8_t gpio_thresholds[] = {SOC_IN0_FLT, SOC_IN3_FLT, SOC_IN8_FLT, SOC_IN11_FLT};

	uint8_t * errorLEDptr;

  for(;;)
  {
	  R = false;
	  G = false;
	  B = false;
	  // Check that the ADC are available, and if they are, retrieve the last recorded ADC outputs.
	  if(adcRestart[0] && adcRestart[1] && adcRestart[2]){
		  presentADCValues = getADCValues();
	  }

	  // Iterate through all the ADC channels that are monitored for faults
	  for (uint8_t rail = 0; rail < sizeof(monitor_rails)/sizeof(monitor_rails[0]); rail++)
	  {
		  // This switch statement maps the appropriate errorLED struct fault flag to the errorLEDptr so that we can clear or set it.
		  // To add more faults simply add more case statements.
		  switch (monitor_rails[rail])
		  {
		  case VSYS:
			  errorLEDptr = &errorLED.vsysPMIFault;
			  break;
		  case V1:
			  errorLEDptr = &errorLED.fault3;
			  break;
		  case V2:
			  errorLEDptr = &errorLED.fault4;
			  break;
		  case V3:
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

//	  if(*(presentADCValues+Adc.adc0) > VSYS_FLT){
//		  errorLED.vsysPMIFault=false;
//	  }
//	  else{
//		  errorLED.vsysPMIFault=true;
//	  }
	  if((!ZION.SOC_EEPROM_Detected && ZION.zionFinished) || (ZION.SOC_BoardFab <0)){
		  errorLED.zionFault=true;
	  }
	  else{
		  errorLED.zionFault=false;
	  }
	  i2cCheck=writeI2CRegister(LED.address, 0xf0, 0x00,1,LED.i2cBank);

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
		  osDelay(20);
		  setErrorLED(VSYSPMI_FAULT, errorLED.vsysPMIFault);
		  osDelay(20);
		  setErrorLED(FAULT3,errorLED.fault3);
		  osDelay(20);
		  setErrorLED(FAULT4,errorLED.fault4);
		  osDelay(20);
		  setErrorLED(FAULT5,errorLED.fault5);
		  osDelay(20);
		  setErrorLED(FAULT6,errorLED.fault6);
		  osDelay(20);
		  setErrorLED(FAULT7,errorLED.fault7);
		  osDelay(20);
		  setErrorLED(FAULT8,errorLED.fault8);
		  osDelay(20);
		  setErrorLED(FAULT9,errorLED.fault9);
	  }
	  else
		  errorLED.ledDriver = true;

    osDelay(500);
  }
  /* USER CODE END startErrorLEDs */
}

/* USER CODE BEGIN Header_startZionRead */
/**
* @brief Function implementing the zionRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startZionRead */
void startZionRead(void *argument)
{
  /* USER CODE BEGIN startZionRead */
  /* Infinite loop */
	float * adcValuePointer;
	int * zionEEPROMPresent;
	int * zionHeaderData;
	int switchOn=0;
	float zionVoltage=77;

  for(;;)
  {
	  if(!ZION.zionFinished){
		  if (adcRestart[0] & adcRestart[1] & adcRestart[2]){
			  adcValuePointer = getADCValues();
			  zionVoltage = *(adcValuePointer + Adc.zionADC);
		  }
		  if(zionVoltage != 77){
			  if(zionVoltage > 3.0 && (!switchOn)){
				  int runtime = (HAL_GetTick()/1000);
				  if(runtime > 15){
					  zionEEPROMPresent= zionEEPROMPresence();
					  if(*zionEEPROMPresent){
						  ZION.SOC_EEPROM_Detected = 1;
						  zionHeaderData = parseZionEEPROM(SOC_ADDRESS);
						  ZION.SOC_BoardID = *(zionHeaderData);
						  ZION.SOC_BoardFab = *(zionHeaderData+2);
						  ZION.SOC_Config = *(zionHeaderData+3);
					  }

					  if(*(zionEEPROMPresent+1)){
						  ZION.ASIC_EEPROM_Detected = 1;
						  zionHeaderData = parseZionEEPROM(ASIC_ADDRESS);
						  ZION.ASIC_BoardID = *(zionHeaderData);
						  ZION.ASIC_BoardFab = *(zionHeaderData+2);
						  ZION.ASIC_Config = *(zionHeaderData+3);

					  }
					  if(*(zionEEPROMPresent+2)){
						  ZION.DISPLAY_EEPROM_Detected = 1;
						  zionHeaderData = parseZionEEPROM(DISPLAY_ADDRESS);
						  ZION.DISPLAY_BoardID = *(zionHeaderData);
						  ZION.DISPLAY_BoardFab = *(zionHeaderData+2);
						  ZION.DISPLAY_Config = *(zionHeaderData+3);
					  }
					  ZION.zionFinished=1;
					  osThreadExit();
				  }
			  }
			  else{
				  if(!switchOn){
					  HAL_GPIO_WritePin(ZION_PWR_EN_GPIO_Port,ZION_PWR_EN_Pin,1);
					  ZION.zionSwitch = 1;
					  switchOn=1;
				  }
				  else{
					  zionEEPROMPresent= zionEEPROMPresence();
					  if(*zionEEPROMPresent){
						  ZION.SOC_EEPROM_Detected = 1;
						  zionHeaderData = parseZionEEPROM(SOC_ADDRESS);
						  ZION.SOC_BoardID = *(zionHeaderData);
						  ZION.SOC_BoardFab = *(zionHeaderData+2);
						  ZION.SOC_Config = *(zionHeaderData+3);
					  }
					  if(*(zionEEPROMPresent+1)){
						  ZION.ASIC_EEPROM_Detected = 1;
						  zionHeaderData = parseZionEEPROM(ASIC_ADDRESS);
						  ZION.ASIC_BoardID = *(zionHeaderData);
						  ZION.ASIC_BoardFab = *(zionHeaderData+2);
						  ZION.ASIC_Config = *(zionHeaderData+3);

					  }
					  if(*(zionEEPROMPresent+2)){
						  ZION.DISPLAY_EEPROM_Detected = 1;
						  zionHeaderData = parseZionEEPROM(DISPLAY_ADDRESS);
						  ZION.DISPLAY_BoardID = *(zionHeaderData);
						  ZION.DISPLAY_BoardFab = *(zionHeaderData+2);
						  ZION.DISPLAY_Config = *(zionHeaderData+3);
					  }
					  ZION.zionFinished=1;
					  osThreadExit();
				  }
			  }
		  }
	  }
	  else{
		  //should never get here but added for completeness
		  osThreadExit();
	  }
    osDelay(400);
  }
  /* USER CODE END startZionRead */
}

/* USER CODE BEGIN Header_startBootButtons */
/**
* @brief Function implementing the bootButtons thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startBootButtons */
void startBootButtons(void *argument)
{
  /* USER CODE BEGIN startBootButtons */
  /* Infinite loop */
	int pwrBtnReady=0;
	int timeTurnedOn=0;
	int pwrOn=0;
	int presentTime=0;
  for(;;)
  {
	  if(bootButtons.bootModeSet){
		  presentTime = (HAL_GetTick());
		  if(timeTurnedOn==0){
			  timeTurnedOn=presentTime;
		  }
		  if(bootButtons.bootMode !=0){
			  if(bootButtons.btn1){ //DPAD UP
				  BTN1_ON;
				  pwrBtnReady=1;
			  }
			  if(bootButtons.btn2){ //DPAD RIGHT
				  BTN2_ON;
				  pwrBtnReady=1;
			  }
			  if(bootButtons.btn3){ //DPAD LEFT
				  BTN3_ON;
				  pwrBtnReady=1;
			  }
			  if(bootButtons.btn4){
				  BTN4_ON;
				  pwrBtnReady=1;
			  }
			  if(bootButtons.btn5){
				  BTN5_ON;
				  pwrBtnReady=1;
			  }
			  if(bootButtons.edl_sw){
				  EDL_SW_ON;
				  pwrBtnReady=1;
			  }
			  if(bootButtons.ex_sw){
				  EX_SW_ON;
				  pwrBtnReady=1;
			  }
			  BTN0_ON;
			  //setOutputGPIOState(outputGPIOs.odOut_0, OFF); //set the reset GPIO.
			  osDelay(13000);
			  BTN0_OFF;
			  osDelay(300);
			  BTN0_ON;
			  osDelay(500);
			  BTN0_OFF;
			  //setOutputGPIOState(outputGPIOs.odOut_0, ON); //turn off the reset GPIO
			  osDelay(4000);
			  BTN1_OFF;
			  BTN2_OFF;
			  BTN3_OFF;
			  BTN4_OFF;
			  BTN5_OFF;
			  EDL_SW_OFF;
			  EX_SW_OFF;
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
			  if((bootButtons.btn0) || pwrBtnReady){ //power button
				  BTN0_ON;
				  timeTurnedOn = (HAL_GetTick());
				  //pwrBtnReady=0;
				  pwrOn = 1;
				  osDelay(500);
				  BTN0_OFF;
				  if(pwrBtnReady){
					  pwrBtnReady=0;
					  osDelay(4000);
				  }
				  pwrOn=0;
				  timeTurnedOn=0;
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
			  if(bootButtons.btn1){ //DPAD UP
				  BTN1_ON;
				  pwrBtnReady=1;
			  }
			  else if(!(bootButtons.btn1)){ //DPAD UP
				  BTN1_OFF;
				  //osDelay(300);
			  }
			  if(bootButtons.btn2){ //DPAD RIGHT
				  BTN2_ON;
				  pwrBtnReady=1;
			  }
			  else if(!(bootButtons.btn2)){ //DPAD RIGHT
				  BTN2_OFF;
				  //osDelay(300);
			  }
			  if(bootButtons.btn3){ //DPAD LEFT
				  BTN3_ON;
				  pwrBtnReady=1;
			  }
			  else if(!(bootButtons.btn3)){ //DPAD LEFT
				  BTN3_OFF;
				  //osDelay(300);
			  }
			  if(bootButtons.btn4){
				  BTN4_ON;
				  pwrBtnReady=1;
			  }
			  else if(!(bootButtons.btn4)){
				  BTN4_OFF;
				  osDelay(300);
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
				  EDL_SW_ON;
				  pwrBtnReady=1;
			  }
			  else if(!(bootButtons.edl_sw)){
				  EDL_SW_OFF;
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
	  }
    osDelay(800);
  }
  /* USER CODE END startBootButtons */
}

/* USER CODE BEGIN Header_startSocUart */
/**
* @brief Function implementing the socUart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startSocUart */
void startSocUart(void *argument)
{
  /* USER CODE BEGIN startSocUart */
	uint8_t * receivedBytes;
	uint8_t values[11];
  /* Infinite loop */
  for(;;)
  {
	  receivedBytes = socUartParser();
	  if((*receivedBytes) != 0xff){
		  for(int x=4;x<15;x++){
			  values[x-4] = *(receivedBytes+x);
		  }
		  int x=0;
	  }
	  //int x = 0;
    osDelay(400);
  }
  /* USER CODE END startSocUart */
}

/* USER CODE BEGIN Header_startDebugUart */
/**
* @brief Function implementing the debugUart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startDebugUart */
void startDebugUart(void *argument)
{
  /* USER CODE BEGIN startDebugUart */
	uint8_t commandSent;
	int x;
  /* Infinite loop */
	float * presentADCValues;


  for(;;)
  {
	  if(adcRestart[0] && adcRestart[1] && adcRestart[2]){
		  presentADCValues = getADCValues();

	  }
	  commandSent = debugUartParser();
	  if(commandSent == true){
		  char buf[5];
		  debugUartTransmitChar("ADCValues:");
		  for(x=0;x<21;x++){
			  sprintf(buf, "%f", *(presentADCValues+x));
			  //snprintf(buf, 5, "%f", *(presentADCValues+x));
			  debugUartTransmitStuff(buf,5);
			  if(x<20){
				  debugUartTransmitChar(",");
			  }

		  }
		  debugUartTransmitChar("\r\n");
		  debugUartTransmitChar("INPUT GPIOs:");
		  //uint8_t gpioInputs[12];
		  //memcpy(gpioInputs,gpioInputBuf,sizeof(gpioInputBuf));
		  for(x=0;x<sizeof(gpioInputBuf);x++){
			  sprintf(buf,"%x",gpioInputBuf[x]);
			  debugUartTransmitChar(buf);
			  if(x<(sizeof(gpioInputBuf))-1){
				  debugUartTransmitChar(",");
			  }
		  }
		  //HAL_UART_Transmit(&DEBUG_UART,(uint8_t *)gpioInputs, sizeof(gpioInputs),100);
		  debugUartTransmitChar("\r\n");
		  debugUartTransmitChar("Errors:");
		  uint8_t errors[11];
		  errors[0] = errorLED.zionFault;
		  errors[1] = errorLED.vsysPMIFault;
		  errors[2] = errorLED.fault3;
		  errors[3] = errorLED.fault4;
		  errors[4] = errorLED.fault5;
		  errors[5] = errorLED.fault6;
		  errors[6] = errorLED.fault7;
		  errors[7] = errorLED.fault8;
		  errors[8] = errorLED.fault9;
		  errors[9] = errorLED.boot_fault;
		  errors[10] = errorLED.ledDriver;
		  for(x=0;x<sizeof(errors);x++){
			  sprintf(buf,"%x",errors[x]);
			  debugUartTransmitChar(buf);
			  if(x<(sizeof(errors))-1){
				  debugUartTransmitChar(",");
			  }
		  }
		  debugUartTransmitChar("\r\n");
		  debugUartTransmitChar("Boot Mode:");
		  sprintf(buf,"%d",bootButtons.bootMode);
		  debugUartTransmitChar(buf);
		  //HAL_UART_Transmit(&DEBUG_UART,(uint8_t *)bootButtons.bootMode, 1,100);
		  debugUartTransmitChar("\r\n");
	  }
    osDelay(500);
  }
  /* USER CODE END startDebugUart */
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

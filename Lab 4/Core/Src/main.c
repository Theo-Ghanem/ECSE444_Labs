/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32l4s5i_iot01.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01_psensor.h"
#include "stm32l4s5i_iot01_magneto.h"
#include "stm32l4s5i_iot01_nfctag.h"
#include "stm32l4s5i_iot01_qspi.h"
#include "stm32l4s5i_iot01_accelero.h"
#include "mx25r6435f.h"
#include "hts221.h"
#include "lis3mdl.h"
#include "lsm6dsl.h"
#include "lps22hb.h"
#include "hsensor.h"
#include "tsensor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

UART_HandleTypeDef huart1;

osThreadId taskReadSensorHandle;
osThreadId taskBtnInputHandle;
osThreadId taskTransmitHandle;
/* USER CODE BEGIN PV */
int currentSensor = 0;
int statsPrinted = 0;
int numSensors = 5;
int buttonPressed = 0;
float temperature = 0.0;
float pressure = 0.0;
int16_t acceleroData[3] = {0}; // used to store x y z values
int16_t magnetoData[3] = {0};  // used to store x y z values

int16_t temperatureSamples[100] = {0}; // type should probably be float
int16_t pressureSamples[100] = {0};    // type should probably be float
int16_t acceleroSamples_x[100] = {0};
int16_t acceleroSamples_y[100] = {0};
int16_t acceleroSamples_z[100] = {0};
int16_t magnetoSamples_x[100] = {0};
int16_t magnetoSamples_y[100] = {0};
int16_t magnetoSamples_z[100] = {0};

uint32_t flash_address_temperature =		0x00000;
uint32_t flash_address_pressure =			0x10000;
uint32_t flash_address_accelerometer_x = 	0x20000;
uint32_t flash_address_accelerometer_y = 	0x30000;
uint32_t flash_address_accelerometer_z = 	0x40000;
uint32_t flash_address_magnetometer_x =		0x50000;
uint32_t flash_address_magnetometer_y =		0x60000;
uint32_t flash_address_magnetometer_z =		0x70000;
int samples = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_OCTOSPI1_Init(void);
void StartTaskReadSensor(void const * argument);
void StartTaskBtnInput(void const * argument);
void StartTaskTransmit(void const * argument);

/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == Button_Pin)
    {
        buttonPressed = 1;
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define min(x, y) ((x) < (y) ? (x) : (y))
void computeStatistics(char* msg){
	if(BSP_QSPI_Read((uint8_t*) temperatureSamples, flash_address_temperature, sizeof(int16_t) * min(samples, 100)) != QSPI_OK)
		Error_Handler();
	if(BSP_QSPI_Read((uint8_t*) pressureSamples, flash_address_pressure, sizeof(int16_t) * min(samples, 100)) != QSPI_OK)
		Error_Handler();
	if(BSP_QSPI_Read((uint8_t*) acceleroSamples_x, flash_address_accelerometer_x, sizeof(int16_t) * min(samples, 100)) != QSPI_OK)
		Error_Handler();
	if(BSP_QSPI_Read((uint8_t*) acceleroSamples_y, flash_address_accelerometer_y, sizeof(int16_t) * min(samples, 100)) != QSPI_OK)
		Error_Handler();
	if(BSP_QSPI_Read((uint8_t*) acceleroSamples_z, flash_address_accelerometer_z, sizeof(int16_t) * min(samples, 100)) != QSPI_OK)
		Error_Handler();
	if(BSP_QSPI_Read((uint8_t*) magnetoSamples_x, flash_address_magnetometer_x, sizeof(int16_t) * min(samples, 100)) != QSPI_OK)
		Error_Handler();
	if(BSP_QSPI_Read((uint8_t*) magnetoSamples_y, flash_address_magnetometer_y, sizeof(int16_t) * min(samples, 100)) != QSPI_OK)
		Error_Handler();
	if(BSP_QSPI_Read((uint8_t*) magnetoSamples_z, flash_address_magnetometer_z, sizeof(int16_t) * min(samples, 100)) != QSPI_OK)
		Error_Handler();

	int avgTemp = 0;
	int avgPres = 0;
	int avgAccel_x = 0;
	int avgAccel_y = 0;
	int avgAccel_z = 0;
	int avgMagnet_x = 0;
	int avgMagnet_y = 0;
	int avgMagnet_z = 0;

	int varianceTemp = 0;
	int variancePres = 0;
	int varianceAccel_x = 0;
	int varianceAccel_y = 0;
	int varianceAccel_z = 0;
	int varianceMagnet_x = 0;
	int varianceMagnet_y = 0;
	int varianceMagnet_z = 0;

	for (int i = 0; i < min(samples, 100); i++) {
		avgTemp += temperatureSamples[i];
		avgPres += pressureSamples[i];
		avgAccel_x += acceleroSamples_x[i];
		avgAccel_y += acceleroSamples_y[i];
		avgAccel_z += acceleroSamples_z[i];
		avgMagnet_x += magnetoSamples_x[i];
		avgMagnet_y += magnetoSamples_y[i];
		avgMagnet_z += magnetoSamples_z[i];
	}

	avgTemp /= min(samples, 100);
	avgPres /= min(samples, 100);
	avgAccel_x /= min(samples, 100);
	avgAccel_y /= min(samples, 100);
	avgAccel_z /= min(samples, 100);
	avgMagnet_x /= min(samples, 100);
	avgMagnet_y /= min(samples, 100);
	avgMagnet_z /= min(samples, 100);

	for (int i = 0; i < min(samples, 100); i++) {
		int tempDiff = temperatureSamples[i] - avgTemp;
		int presDiff = pressureSamples[i] - avgPres;
		int accel_xDiff = acceleroSamples_x[i] - avgAccel_x;
		int accel_yDiff = acceleroSamples_y[i] - avgAccel_y;
		int accel_zDiff = acceleroSamples_z[i] - avgAccel_z;
		int magnet_xDiff = magnetoSamples_x[i] - avgMagnet_x;
		int magnet_yDiff = magnetoSamples_y[i] - avgMagnet_y;
		int magnet_zDiff = magnetoSamples_z[i] - avgMagnet_z;

		varianceTemp += tempDiff * tempDiff;
		variancePres += presDiff * presDiff;
		varianceAccel_x += accel_xDiff * accel_xDiff;
		varianceAccel_y += accel_yDiff * accel_yDiff;
		varianceAccel_z += accel_zDiff * accel_zDiff;
		varianceMagnet_x += magnet_xDiff * magnet_xDiff;
		varianceMagnet_y += magnet_yDiff * magnet_yDiff;
		varianceMagnet_z += magnet_zDiff * magnet_zDiff;
	}

	varianceTemp /= min(samples, 100);
	variancePres /= min(samples, 100);
	varianceAccel_x /= min(samples, 100);
	varianceAccel_y /= min(samples, 100);
	varianceAccel_z /= min(samples, 100);
	varianceMagnet_x /= min(samples, 100);
	varianceMagnet_y /= min(samples, 100);
	varianceMagnet_z /= min(samples, 100);

    sprintf(msg, "\r\nStatistics:\r\nSamples: %d\r\n\tTemperature:\r\n\t\tAverage: %d\r\n\t\tVariance: %d\r\n\r\n\tPressure:\r\n\t\tAverage: %d\r\n\t\tVariance: %d\r\n\r\n\tAccelerometer:\r\n\t\tAverage x: %d\r\n\t\tAverage y: %d\r\n\t\tAverage z: %d\r\n\t\tVariance x: %d\r\n\t\tVariance y: %d\r\n\t\tVariance z: %d\r\n\r\n\tMagnetometer:\r\n\t\tAverage x: %d\r\n\t\tAverage y: %d\r\n\t\tAverage z: %d\r\n\t\tVariance x: %d\r\n\t\tVariance y: %d\r\n\t\tVariance z: %d\r\n\r\n", min(samples, 100), avgTemp, varianceTemp, avgPres, variancePres, avgAccel_x, avgAccel_y, avgAccel_z, varianceAccel_x, varianceAccel_y, varianceAccel_z, avgMagnet_x, avgMagnet_y, avgMagnet_z, varianceMagnet_x, varianceMagnet_y, varianceMagnet_z);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_OCTOSPI1_Init();
  /* USER CODE BEGIN 2 */
    BSP_QSPI_Init();
    BSP_HSENSOR_Init();
    BSP_TSENSOR_Init();
    BSP_PSENSOR_Init();
    BSP_ACCELERO_Init();
    BSP_MAGNETO_Init();

  /* USER CODE END 2 */

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
  /* definition and creation of taskReadSensor */
  osThreadDef(taskReadSensor, StartTaskReadSensor, osPriorityNormal, 0, 256);
  taskReadSensorHandle = osThreadCreate(osThread(taskReadSensor), NULL);

  /* definition and creation of taskBtnInput */
  osThreadDef(taskBtnInput, StartTaskBtnInput, osPriorityNormal, 0, 256);
  taskBtnInputHandle = osThreadCreate(osThread(taskBtnInput), NULL);

  /* definition and creation of taskTransmit */
  osThreadDef(taskTransmit, StartTaskTransmit, osPriorityNormal, 0, 256);
  taskTransmitHandle = osThreadCreate(osThread(taskTransmit), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1)
    {

        //	  float humidity = BSP_HSENSOR_ReadHumidity();
        //	  float temperature = BSP_TSENSOR_ReadTemp();
        //	  char* msg = calloc(1, sizeof(char) * 100);
        //	  sprintf(msg, "The humidity is: %.2f\r\nThe temperature is: %.2f\r\n", humidity, temperature);
        //	  HAL_Delay(500);
        //	  HAL_UART_Transmit(&huart1, msg, sizeof(char) * 100, 10000);

        //	  char* msg = calloc(1, sizeof(char) * 100);
        //	  int numSensors = 4;  // Number of sensors (humidity and temperature)
        //	  float sensorValue = 0.0;
        //	  char sensorName[20];
        //	  int16_t xyzData[3] = {0};
        //	  int xyzMsg=0;
        //	  switch (currentSensor) {
        //		  case 0:
        //			  sensorValue = BSP_HSENSOR_ReadHumidity();
        //			  strcpy(sensorName, "Humidity");
        //			  break;
        //		  case 1:
        //			  sensorValue = BSP_TSENSOR_ReadTemp();
        //			  strcpy(sensorName, "Temperature");
        //			  break;
        //		  case 2:
        //			  BSP_ACCELERO_AccGetXYZ(xyzData);
        //			  strcpy(sensorName, "Accelerometer");
        //			  xyzMsg=1;
        //			  break;
        //		  case 3:
        //			  BSP_MAGNETO_AccGetXYZ(xyzData);
        //			  strcpy(sensorName, "Magnetometer");
        //			  xyzMsg=1;
        //			  break;
        //	  }
        //	  if (xyzMsg==1){
        //		  sprintf(msg, "The %s values are: X: %d, Y: %d, Z: %d\r\n", sensorName, xyzData[0], xyzData[1], xyzData[2]);
        //	  }
        //
        //	  else {
        //		  sprintf(msg, "The %s is: %.2f\r\n", sensorName, sensorValue);
        //	  }
        //
        //	  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        //
        //	  currentSensor = (currentSensor + 1) % numSensors;// Increment the currentSensor variable and wrap around
        //
        //	  HAL_Delay(500);
        //  }

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c2.Init.Timing = 0x307075B1;
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
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDError_GPIO_Port, LEDError_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LEDError_Pin */
  GPIO_InitStruct.Pin = LEDError_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDError_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskReadSensor */
/**
 * @brief  Function implementing the taskReadSensor thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskReadSensor */
void StartTaskReadSensor(void const * argument)
{
  /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for (;;)
    {
        osDelay(100);

        if(samples == 0){
        	if (BSP_QSPI_Erase_Block(flash_address_temperature) != QSPI_OK)
				Error_Handler();
			if (BSP_QSPI_Erase_Block(flash_address_pressure) != QSPI_OK)
				Error_Handler();
			if (BSP_QSPI_Erase_Block(flash_address_accelerometer_x) != QSPI_OK)
				Error_Handler();
			if (BSP_QSPI_Erase_Block(flash_address_accelerometer_y) != QSPI_OK)
				Error_Handler();
			if (BSP_QSPI_Erase_Block(flash_address_accelerometer_z) != QSPI_OK)
				Error_Handler();
			if (BSP_QSPI_Erase_Block(flash_address_magnetometer_x) != QSPI_OK)
				Error_Handler();
			if (BSP_QSPI_Erase_Block(flash_address_magnetometer_y) != QSPI_OK)
				Error_Handler();
			if (BSP_QSPI_Erase_Block(flash_address_magnetometer_z) != QSPI_OK)
				Error_Handler();
        }

        if(currentSensor == 4) continue;

        temperature = BSP_TSENSOR_ReadTemp();
        pressure = BSP_PSENSOR_ReadPressure();
        BSP_ACCELERO_AccGetXYZ(acceleroData);
        BSP_MAGNETO_GetXYZ(magnetoData);

        // =======Part 3==========
//        // erase
//		uint32_t blockAddress = 0x0;  // First address of first block of 64KB
//		if (BSP_QSPI_Erase_Block(blockAddress) != QSPI_OK) {
//			Error_Handler();
//		}
//		//write
//		uint32_t writeAddress = 0x0;  // First address of first block of 64KB
//		uint8_t data[] = { (uint8_t) temperature };  // Replace with your data
//		uint32_t dataSize = sizeof(data);
//		if (BSP_QSPI_Write(data, writeAddress, dataSize) != QSPI_OK) {
//			Error_Handler();
//		}
//
//		//read
//		uint32_t readAddress = 0x0;  // Replace with the address from where you want to read data
//		uint8_t readData[1];  // Adjust the array size according to your data size
//		if (BSP_QSPI_Read(readData, readAddress, sizeof(readData)) != QSPI_OK) {
//			Error_Handler();
//		}
//
//		// Transmit
//		char *msg = calloc(1, sizeof(char) * 100);
//		sprintf(msg, "The Temperature from FLASH is: %d\r\n", (int)readData[0]);
//		HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);


        //=======Part 4==========

        // write
        uint32_t offset = samples * sizeof(int16_t);

        int16_t temperature_int = (int16_t) (temperature);
        if (BSP_QSPI_Write((uint8_t*) &temperature_int, flash_address_temperature + offset, sizeof(int16_t)) != QSPI_OK)
            Error_Handler();

        int16_t pressure_int = (int16_t) (pressure);
		if (BSP_QSPI_Write((uint8_t*) &pressure_int, flash_address_pressure + offset, sizeof(int16_t)) != QSPI_OK)
			Error_Handler();

		if (BSP_QSPI_Write((uint8_t*) &acceleroData[0], flash_address_accelerometer_x + offset, sizeof(int16_t)) != QSPI_OK)
			Error_Handler();
		if (BSP_QSPI_Write((uint8_t*) &acceleroData[1], flash_address_accelerometer_y + offset, sizeof(int16_t)) != QSPI_OK)
			Error_Handler();
		if (BSP_QSPI_Write((uint8_t*) &acceleroData[2], flash_address_accelerometer_z + offset, sizeof(int16_t)) != QSPI_OK)
			Error_Handler();

		if (BSP_QSPI_Write((uint8_t*) &magnetoData[0], flash_address_magnetometer_x + offset, sizeof(int16_t)) != QSPI_OK)
			Error_Handler();
		if (BSP_QSPI_Write((uint8_t*) &magnetoData[1], flash_address_magnetometer_y + offset, sizeof(int16_t)) != QSPI_OK)
			Error_Handler();
		if (BSP_QSPI_Write((uint8_t*) &magnetoData[2], flash_address_magnetometer_z + offset, sizeof(int16_t)) != QSPI_OK)
			Error_Handler();


        if(samples < 999)
        	samples++;
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskBtnInput */
/**
 * @brief Function implementing the taskBtnInput thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskBtnInput */
void StartTaskBtnInput(void const * argument)
{
  /* USER CODE BEGIN StartTaskBtnInput */
    /* Infinite loop */
    for (;;)
    {
        osDelay(100);
        // task that determines when the button has been pressed,
        // and changes the mode of the application to output data
        // from the next sensor in the sequence;
        if (buttonPressed == 1)
        {
            currentSensor = (currentSensor + 1) % numSensors; // Increment the currentSensor variable and wrap around
            buttonPressed = 0;

            if(currentSensor == 0){
				samples = 0;
				statsPrinted = 0;
            }
        }
    }
  /* USER CODE END StartTaskBtnInput */
}

/* USER CODE BEGIN Header_StartTaskTransmit */
/**
 * @brief Function implementing the taskTransmit thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskTransmit */
void StartTaskTransmit(void const * argument)
{
  /* USER CODE BEGIN StartTaskTransmit */
    /* Infinite loop */
    for (;;)
    {
        osDelay(500);
        //    ======Part 2==========
        // task that transmits this data to the terminal using the virtual com port UART;
        // and,
        char *msg = calloc(1, sizeof(char) * 1000);
        switch (currentSensor)
        {
        case 0:
            sprintf(msg, "The Temperature is: %d\r\n", (int)temperature);
            break;
        case 1:
            sprintf(msg, "The Pressure is: %d\r\n", (int)pressure);
            break;
        case 2:
            sprintf(msg, "The Accelerometer values are: X: %d, Y: %d, Z: %d\r\n", acceleroData[0], acceleroData[1], acceleroData[2]);
            break;
        case 3:
            sprintf(msg, "The Magnetometer values are: X: %d, Y: %d, Z: %d\r\n", magnetoData[0], magnetoData[1], magnetoData[2]);
            break;
        // ======== PART 4 BEGIN ========
        case 4:
        	if(!statsPrinted){
				computeStatistics(msg);
				statsPrinted = 1;
        	}
        	break;
		// ========= PART 4 END =========
        default:
            sprintf(msg, "Wrong sensor");
        }

        HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }

  /* USER CODE END StartTaskTransmit */
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
        HAL_GPIO_WritePin(LEDError_GPIO_Port, LEDError_Pin, GPIO_PIN_RESET);
        __BKPT();
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

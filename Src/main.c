/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include <stdio.h> 
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void getAltitude();
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

osThreadId defaultTaskHandle;
osThreadId usrBtnHandle;
/* USER CODE BEGIN PV */
BMP280_HandleTypedef bmp280;

const float SEA_LEVEL_PRESSURE = 1013.23f;
float pressure, temperature, humidity, altitude, pressure_hpa;
float pressure_array[10],averageP=0,SumPressure,averagePinit=0;
float rawP[10],rawPinit[10];
float rawPdata,rawPdatainit;
float initFloor, refFloor = 1010.73 , currentFloor;
static char status[5];
float relative;
uint16_t size;
uint8_t Data[256];
int initPressure,countinit;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);
void TaskBtn(void const * argument);

/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

	//HAL_UART_Transmit(&huart1, Data, size, 1000);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of usrBtn */
  osThreadDef(usrBtn, TaskBtn, osPriorityNormal, 0, 128);
  usrBtnHandle = osThreadCreate(osThread(usrBtn), NULL);

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
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
//		HAL_Delay(100);
//		
//		while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
//			size = sprintf((char *)Data,
//					"Temperature/pressure reading failed\n");
//		//	HAL_UART_Transmit(&huart1, Data, size, 1000);
//			HAL_Delay(2000);
//		}

//		size = sprintf((char *)Data,"Pressure: %.2f Pa, Temperature: %.2f C",
//				pressure, temperature);
//		
//		pressure_hpa = pressure/100 ;
//		
//		// Calculate the altitude in metres 
//		
//		/*1 previous formula*/
//		//altitude = fabs((287.053*(logf((pressure)/101325))*(temperature+273.15))/9.8);
//		
//		/*2 */
//		//altitude = ((float)powf(SEA_LEVEL_PRESSURE / pressure*100, 0.190223f) - 1.0f) * (temperature + 273.15f) / 0.0065f; 
//		
//		/*3 */
//		//altitude = (44330.0 * (1.0 - powf(pressure / 101325.0,1 / 5.255)));
//		
//		/*4 */
//		//altitude = (1 - pow(pressure/(float)101325, 0.1903)) / 0.0000225577;

//		

//		//HAL_UART_Transmit(&huart1, Data, size, 1000);
//		if (bme280p) {
//			size = sprintf((char *)Data,", Humidity: %.2f\n", humidity);
//		//	HAL_UART_Transmit(&huart1, Data, size, 1000);
//		}

//		else {
//			size = sprintf((char *)Data, "\n");
//			//HAL_UART_Transmit(&huart1, Data, size, 1000);
//		}
//		HAL_Delay(2000);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	initPressure=0;
		bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;
bool bme280p = bmp280.id == BME280_CHIP_ID;
	int count=0;
	
	
	while (!bmp280_init(&bmp280, &bmp280.params)) {
		size = sprintf((char *)Data, "BMP280 initialization failed\n");
		//HAL_UART_Transmit(&huart1, Data, size, 1000);
		HAL_Delay(2000);
	}
	
	size = sprintf((char *)Data, "BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
		while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
			size = sprintf((char *)Data,
					"Temperature/pressure reading failed\n");
		//	HAL_UART_Transmit(&huart1, Data, size, 1000);
			osDelay(2000);
		}

		size = sprintf((char *)Data,"Pressure: %.2f Pa, Temperature: %.2f C",
				pressure, temperature);
	
		pressure_hpa = pressure/100 ;
		osDelay(10000);
	
  /* Infinite loop */
  for(;;)
  {
		
				//HAL_Delay(100);
		if (initPressure !=0)
		{
		while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
			size = sprintf((char *)Data,
					"Temperature/pressure reading failed\n");
		//	HAL_UART_Transmit(&huart1, Data, size, 1000);
			HAL_Delay(2000);
		}

		size = sprintf((char *)Data,"Pressure: %.2f Pa, Temperature: %.2f C",
				pressure, temperature);
	
		pressure_hpa = pressure/100 ;
		
		
//		if (count<=10)
//		{
//			
//			pressure_array[count]=pressure_hpa;
//			count++;
//			
//		}
//		else
//		{count=0;}
		
		// subtract the last reading:
			rawPdata = rawPdata - rawP[count];
		// read from the sensor:
			rawP[count] = pressure_hpa;
		// add the reading to the total:
			rawPdata = rawPdata + rawP[count];
			count++;
			
			if(count>=10)
			{
				averageP = rawPdata / 10.0;			
				count = 0;
				currentFloor = averageP;
			//	relative=currentFloor-initFloor;
			}
			
		//	currentFloor = rawP[0];
			//refFloor= 1009.70
			//
			if( abs(relative)<0.07)
			{
				sprintf(status,"zero");
				pressure_hpa = 0;
				//vTaskDelay(1); //1ms delay
			//	initPressure=0; //recalibrate
			}
			
			else if (relative >= 0.8)
			{
					//going down, perbezaan +ve
				 sprintf(status,"down");
			}
			
			else if(relative < 0.8)
			{
					//going up, prebezaan -ve
					sprintf(status,"up");
			}

		
	
		//1== 1011.10, 1nhalf ==
		
		// Calculate the altitude in metres 
		
		/*1 previous formula*/
		//altitude = fabs((287.053*(logf((pressure)/101325))*(temperature+273.15))/9.8);
		
		/*2 */
		//altitude = ((float)powf(SEA_LEVEL_PRESSURE / pressure*100, 0.190223f) - 1.0f) * (temperature + 273.15f) / 0.0065f; 
		
		/*3 */
		//altitude = (44330.0 * (1.0 - powf(pressure / 101325.0,1 / 5.255)));
		
		/*4 */
		//altitude = (1 - pow(pressure/(float)101325, 0.1903)) / 0.0000225577;

		

		//HAL_UART_Transmit(&huart1, Data, size, 1000);
		if (bme280p) {
			size = sprintf((char *)Data,", Humidity: %.2f\n", humidity);
		//	HAL_UART_Transmit(&huart1, Data, size, 1000);
		}

		else {
			size = sprintf((char *)Data, "\n");
			//HAL_UART_Transmit(&huart1, Data, size, 1000);
		}
		//HAL_Delay(2000);
		
  
		
	}
		else
		{
			while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
			size = sprintf((char *)Data,
					"Temperature/pressure reading failed\n");
		//	HAL_UART_Transmit(&huart1, Data, size, 1000);
			HAL_Delay(2000);
		}

		size = sprintf((char *)Data,"Pressure: %.2f Pa, Temperature: %.2f C",
				pressure, temperature);
	
		pressure_hpa = pressure/100 ;
		
		
		
		// subtract the last reading:
			rawPdatainit = rawPdatainit - rawPinit[countinit];
		// read from the sensor:
			rawPinit[countinit] = pressure_hpa;
		// add the reading to the total:
			rawPdatainit = rawPdatainit + rawPinit[countinit];
			countinit++;
			
			if(countinit>=10)
			{
				averagePinit = rawPdatainit / 10.0;			
				countinit = 0;
				initFloor = averagePinit;
				initPressure=1;
			}
			
		}
		relative=currentFloor-initFloor;
		  osDelay(100);
		
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_TaskBtn */
/**
* @brief Function implementing the usrBtn thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskBtn */
void TaskBtn(void const * argument)
{
  /* USER CODE BEGIN TaskBtn */
  /* Infinite loop */
  for(;;)
  {
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET)
		{
			initPressure=0;
		}
		//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
    osDelay(1);
  }
  /* USER CODE END TaskBtn */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

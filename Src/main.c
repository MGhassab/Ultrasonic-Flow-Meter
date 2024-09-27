/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdbool.h>
#include "MAX31865.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define			sample_size			2500
#define			sensor_size			5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t adc_value[sample_size];
uint8_t  tx_data[2*sample_size+sensor_size];
uint8_t  rec_mes[4]={0,0,0,0};
uint8_t  snd_cmn[4]="send";

Max31865_t		pt1000;
bool 					pt1000isOK;
float					pt1000Temp;
float					Temp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool arry_cmp(uint8_t *a,uint8_t *b,uint16_t length)
	{
		uint16_t idx_cmp;
		for(idx_cmp=0;idx_cmp<length;idx_cmp++)
		{
			if(a[idx_cmp]!= b[idx_cmp])
				return false;
		}
		return true;
	}

	void config_channel3(void)
	{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);
	HAL_ADC_ConfigChannel(&hadc3, &sConfig);
	}
	void config_channel0(void)
	{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);
	HAL_ADC_ConfigChannel(&hadc3, &sConfig);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, 0);		//Set pin F14 LOW (F103 PWM Trigger)
  /* USER CODE END 2 */
	Max31865_init(&pt1000,&hspi1,GPIOA,GPIO_PIN_4,2,50);	//Init Temperature_Sensor
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		pt1000isOK = Max31865_readTempC(&pt1000,&t);
//		pt1000Temp = Max31865_Filter(t,pt1000Temp,0.1);
//		tx_data[sample_size]									 = *((uint8_t*)(&pt1000Temp));
//		tx_data[sample_size+sensor_size+1]		 = *((uint8_t*)(&pt1000Temp)+1);
//		tx_data[sample_size+sensor_size+2]		 = *((uint8_t*)(&pt1000Temp)+2);
//		tx_data[sample_size+sensor_size+3]		 = *((uint8_t*)(&pt1000Temp)+3);
//		CDC_Transmit_FS(temp,4);
//		HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void CDC_ReceiveCallBack(uint8_t *buf, uint32_t len)
{
	// Check Received Message From PC
	uint8_t	idx_cpy_rec_buf = 0;
	for(idx_cpy_rec_buf=0;idx_cpy_rec_buf<4;idx_cpy_rec_buf++)
	 {
		rec_mes[idx_cpy_rec_buf] = *(buf+idx_cpy_rec_buf);
	 }
						
	if(arry_cmp(rec_mes,snd_cmn,4) == true)
	 {
		config_channel0();
		 
	 // Make External Trigger For F103(PWM)
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, 1);
		 uint16_t idx1_delay = 0;
		 uint16_t idx2_delay = 0;
			for(idx2_delay=0;idx2_delay<65000;idx2_delay++);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, 0);
		  
	 // Turn on ADC_Interleaved
		HAL_ADC_Start(&hadc3);
		HAL_ADC_Start(&hadc2);
		HAL_ADCEx_MultiModeStart_DMA(&hadc1,adc_value,sample_size);
		HAL_ADC_Start(&hadc3);
		HAL_ADC_Start(&hadc2);
		HAL_ADCEx_MultiModeStart_DMA(&hadc1,adc_value,sample_size);	 
	 }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{			
		// GET Temperature
			pt1000isOK = Max31865_readTempC(&pt1000,&Temp);
			pt1000Temp = Max31865_Filter(Temp,pt1000Temp,0.1);
			tx_data[2*sample_size]			 = *((uint8_t*)(&pt1000Temp));
			tx_data[2*sample_size+1]		 = *((uint8_t*)(&pt1000Temp)+1);
			tx_data[2*sample_size+2]		 = *((uint8_t*)(&pt1000Temp)+2);
			tx_data[2*sample_size+3]		 = *((uint8_t*)(&pt1000Temp)+3);
	
		// GET Pressure
			config_channel3();
			HAL_ADC_Start(&hadc3);
		  HAL_ADC_Start(&hadc2);
			tx_data[2*sample_size+sensor_size-1] = HAL_ADCEx_MultiModeGetValue(&hadc1);
	
		// Prepare Interleaved Sampled Data
			uint16_t index = 0;
			for(index=0;index<sample_size;index++)
			{
				tx_data[2*index] = adc_value[index];
				tx_data[2*index+1] =adc_value[index]>>16;
			}

		// Send Data To PC
			CDC_Transmit_FS(tx_data,sample_size*2+sensor_size);
			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
			
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
}
void CDC_transmitCallBack(uint8_t *buf, uint32_t len)
{
}

/* USER CODE END 4 */

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

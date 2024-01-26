/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_i2c.h"
#include <dtostrf.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <BMPXX80.h>
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

/* USER CODE BEGIN PV */

#define RX_BUFFER_SIZE 10
struct lcd_disp disp;

float PWM_wyp_float;
uint16_t PWM_wyp_u = 0;
char msg[32];
char msg2[32];
float wartosci[10000];
float temperature;
int32_t pressure;
float current_temp_f;
volatile float wartosc_zadana = 23.0;
char key[10];
char current_temp_ch_UART[29];
char set_temp_ch_UART[24];
char zadana[]="ZADANA: ";
char znak_linii[]=" \n\n\r";
char aktualna[] = "AKTUALNA: ";
float uchyb;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
 void MX_GPIO_Init(void);
 void MX_DMA_Init(void);
 void MX_USART3_UART_Init(void);
 void MX_USB_OTG_FS_PCD_Init(void);
 void MX_I2C1_Init(void);
 void MX_TIM1_Init(void);
 void MX_TIM3_Init(void);
 void MX_TIM4_Init(void);
 void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct Controller{
	float Kp;
	float Ki;
	float Kd;
	float Tp;
	float prev_error;
	float prev_u_I;

};


float calculate_PID(struct Controller *PID, float set_temp, float meas_temp){
	float u = 0;
	float error;
	float u_P, u_I , u_D;

	error = set_temp - meas_temp;

	// Proportional
	u_P = PID->Kp * error;

	// Integral
	u_I = PID->Ki * PID->Tp / 2.0 * (error + PID->prev_error) + PID->prev_u_I;
	PID->prev_u_I = u_I;

	// Derivative
	u_D = (error - PID->prev_error) / PID->Tp;

	PID->prev_error = error;

	// Sum of P, I and D components
	u = u_P + u_I + u_D;

	return u;
}

struct Controller PID1;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */


int main(void)
{
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
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_I2C4_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  // Initialize interrupts
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
//

  	  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  	  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  	  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  	  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  	  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  	    HAL_TIM_Base_Start_IT(&htim3);

  	  	HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3);
  	    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);

 		BMP280_Init(&hi2c4, BMP280_TEMPERATURE_16BIT, BMP280_STANDARD, BMP280_FORCEDMODE);
//
// 		htim1.Instance->CNT = 65535 / 2;
//
 		disp.addr = (0x27 << 1);
	    disp.bl = true;
	    lcd_init(&disp);

	    PID1.Kp = 1.2;
	    PID1.Ki = 0.008;
	    PID1.Kd = 0;
	    PID1.Tp = 1;
	    PID1.prev_error = 0;
	    PID1.prev_u_I = 0;

	    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)key, 10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	    while (1)
	    {

	   // Inicjalizacja wartości początkowej z pomocą przycisku USER i RESET
//	    	if(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
//	    	{
//	    		wartosc_zadana+=0.1f;
//	    		dtostrf(wartosc_zadana, 3, 1, (char *)msg);
//	    		sprintf((char *)disp.f_line,"ZADANA:   %s", (char *)msg); //LCD
//	    		HAL_Delay(50);
//	    	}

//	    	  HAL_StatusTypeDef status = HAL_UART_Receive(&huart3, key, 1, 1);
//	    	  if (status == HAL_OK){
//	    		  wartosc_zadana = atof(key);
//	    	  }

	    		if(PWM_wyp_u < 16383) {
	    			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 65535);
	    			HAL_Delay(50);
	    		}

	    		if(PWM_wyp_u > 16383 && PWM_wyp_u < 32766) {
	    			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 65535);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 65535);
					HAL_Delay(50);
				}
	    		else
	    			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);


	    		if(PWM_wyp_u > 32766 && PWM_wyp_u < 49149) {
	    			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 65535);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 65535);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 65535);
	    			HAL_Delay(50);
				}
	    		else
	    			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

	    		if(PWM_wyp_u > 49149) {
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 65535);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 65535);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 65535);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 65535);
	    			HAL_Delay(50);
				}
	    		else
	    			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

	    		if(PWM_wyp_u == 0) {
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
					HAL_Delay(50);
				}

	   	 	  dtostrf(current_temp_f, 3, 1, (char *)msg2);

	   	 	  HAL_UART_Transmit(&huart3, (uint8_t *)zadana, strlen(zadana), 1000);
	   	 	  HAL_Delay(50);
	  	  	  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 1000);
	  	  	  HAL_UART_Transmit(&huart3, (uint8_t *)znak_linii, strlen(znak_linii), 1000);

	  	  	  HAL_UART_Transmit(&huart3, (uint8_t *)aktualna, strlen(aktualna), 1000);
	  	  	  HAL_Delay(50);
	  	  	  HAL_UART_Transmit(&huart3, (uint8_t*)msg2, strlen(msg2), 1000);
	  	  	  HAL_UART_Transmit(&huart3, (uint8_t *)znak_linii, strlen(znak_linii), 1000);

			  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)key, 10);

	  	  	  if(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
	  	  		    	{
	  	  		    		wartosc_zadana+=0.1f;
	  	  		    		dtostrf(wartosc_zadana, 3, 1, (char *)msg);
	  	  		    		sprintf((char *)disp.f_line,"ZADANA:   %s", (char *)msg); //LCD
	  	  		    		HAL_Delay(50);
	  	  		    	}
	  	  	  dtostrf(wartosc_zadana, 3, 1, (char *)msg);
	  	  	  sprintf((char *)disp.s_line,"AKTUALNA: %s", (char *)msg2); //LCD
	  	  	  sprintf((char *)disp.f_line,"ZADANA:   %s", (char *)msg); //LCD

	  	  	  lcd_display(&disp);
	  	  	  HAL_Delay(50);


	  	  	  for(int i=0; i>10000; i++)
	  	  	  {
	  	  		  float NowaAktualna = current_temp_f;
	  	  		  wartosci[i] = NowaAktualna;

	  	  	  }

	  	  	  uchyb = wartosc_zadana - current_temp_f;

	  	  	  memset(key, 0, 10);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){

		BMP280_ReadTemperatureAndPressure(&current_temp_f, &pressure);
//		sprintf(current_temp_ch_UART, "Current temperature: %.2f \r",  current_temp_f);
//		HAL_UART_Transmit(&huart3, (uint8_t *)current_temp_ch_UART, sizeof(current_temp_ch_UART)-1, 1000);
//
//		sprintf((char*)set_temp_ch_UART, "Set temperature: %.2f", wartosc_zadana);
//		HAL_UART_Transmit(&huart3, (uint8_t*)set_temp_ch_UART, strlen(set_temp_ch_UART), 1000);

		PWM_wyp_float = (htim1.Init.Period * calculate_PID(&PID1, wartosc_zadana, current_temp_f));


		if(PWM_wyp_float < 0.0) PWM_wyp_u = 0;
		else if(PWM_wyp_float > htim1.Init.Period) PWM_wyp_u = htim1.Init.Period;
		else PWM_wyp_u = (uint16_t) PWM_wyp_float;

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_wyp_u);
	}
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance == USART3){
		float tmp = atof(key);
		if(tmp < 20) wartosc_zadana = 23;
		else if(tmp > 65) wartosc_zadana = 65;
		else wartosc_zadana = tmp;

		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)key, 10);

	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

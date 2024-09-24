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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	unsigned long currentMillis=0;
	unsigned long lastBlink;

//
//	void blinkLed(){
//		if (currentMillis - lastBlink > 1000){
//			lastBlink=currentMillis;
//			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
//		}
//	}

	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		if(htim->Instance == TIM1) //check if the interrupt comes from TIM1
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
			//PWM_Init(&htim1, TIM_CHANNEL_1, 50000, 100, 1000);s
		}
	}

	uint32_t PCLK1TIM(TIM_HandleTypeDef *htim) {
		uint32_t timer_clock_freq;
		uint32_t ppre1;
		uint32_t ppre2;
		uint32_t apb1_prescaler;
		uint32_t apb2_prescaler;

	  if ((htim->Instance == TIM2) || (htim->Instance == TIM3) || (htim->Instance == TIM4) ||
	      (htim->Instance == TIM5) || (htim->Instance == TIM6) || (htim->Instance == TIM7) ||
	      (htim->Instance == TIM12) || (htim->Instance == TIM13) || (htim->Instance == TIM14)) {
		   timer_clock_freq = HAL_RCC_GetPCLK1Freq();
		   uint32_t ppre1;
		   uint32_t apb1_prescaler;
		   ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
		   uint32_t apb1 = RCC_CFGR_PPRE1;
		   uint32_t wtf1 =  RCC->CFGR;
		   switch (ppre1) {
		   		       case 0b000: // Дільник 1
		   		           apb1_prescaler = 1;
		   		           break;
		   		       case 0b100: // Дільник 2
		   		           apb1_prescaler = 2;
		   		           break;
		   		       case 0b101: // Дільник 4
		   		           apb1_prescaler = 4;
		   		           break;
		   		       case 0b110: // Дільник 8
		   		           apb1_prescaler = 8;
		   		           break;
		   		       case 0b111: // Дільник 16
		   		           apb1_prescaler = 16;
		   		           break;
		   		       default:
		   		           apb1_prescaler = 1; // За замовчуванням (повинно бути недосяжно)
		   		           break;
		   		   }
		   if (apb1_prescaler>1) {
			   timer_clock_freq = 2 * timer_clock_freq;
		   }
		   return timer_clock_freq;
		   //uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
	  }
	  // Перевіримо, чи таймер знаходиться на APB2
	  else if ((htim->Instance == TIM1) || (htim->Instance == TIM8) || (htim->Instance == TIM9) ||
	           (htim->Instance == TIM10) || (htim->Instance == TIM11)) {
	       timer_clock_freq = HAL_RCC_GetPCLK2Freq();
	       // Витягуємо значення дільника APB2 з регістра RCC->CFGR
	       ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
		   switch (ppre2) {
		   	           case 0b000: // Дільник 1
		   	               apb2_prescaler = 1;
		   	               break;
		   	           case 0b100: // Дільник 2
		   	               apb2_prescaler = 2;
		   	               break;
		   	           case 0b101: // Дільник 4
		   	               apb2_prescaler = 4;
		   	               break;
		   	           case 0b110: // Дільник 8
		   	               apb2_prescaler = 8;
		   	               break;
		   	           case 0b111: // Дільник 16
		   	               apb2_prescaler = 16;
		   	               break;
		   	           default:
		   	               apb2_prescaler = 1; // За замовчуванням (повинно бути недосяжно)
		   	               break;
		   	       }
	       uint32_t apb2 = RCC_CFGR_PPRE2;
	       uint32_t wtf2 =  RCC->CFGR;
	       if (apb2_prescaler>1) {
	    	   timer_clock_freq = 2 * timer_clock_freq;
		   }
		  return timer_clock_freq;
	  }
	}

	void PWM_Init(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t target_frequency, uint8_t duty_cycle) {
		uint32_t timer_clock_freq = PCLK1TIM(htim);
		uint16_t prescaler = 1;
		uint32_t period = (timer_clock_freq / target_frequency);
		if ( period > 65535 ) {
			prescaler = (period / 65536)+2;
			period = (period/prescaler);
		}
		uint16_t pulse_length_ticks = period * duty_cycle / 100;
		__HAL_TIM_SET_PRESCALER(htim, prescaler-1);
		__HAL_TIM_SET_COMPARE(htim, channel, pulse_length_ticks);
		__HAL_TIM_SET_AUTORELOAD(htim, (uint16_t)period -1);
		HAL_TIM_Base_Start_IT(htim);
		HAL_TIM_PWM_Start(htim, channel);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim1);                                     // <<<<<<<<<< запустить таймер у режимі переривання:
  HAL_TIM_Base_Start_IT(&htim2);
  //PWM_Init(&htim1, TIM_CHANNEL_1, 1, 10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  currentMillis = HAL_GetTick();

//	  PWM_Init(&htim1, TIM_CHANNEL_1, 1000, 50);
//	  HAL_Delay(1000);
//	  PWM_Init(&htim1, TIM_CHANNEL_1, 1000, 90);
//	  HAL_Delay(1000);
	  PWM_Init(&htim1, TIM_CHANNEL_1, 50000, 25);  // PE9
	  PWM_Init(&htim2, TIM_CHANNEL_1, 2, 60);  // PA0
	  HAL_Delay(1000);
	  //blinkLed();




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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
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

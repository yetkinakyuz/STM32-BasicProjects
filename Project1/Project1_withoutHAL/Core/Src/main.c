/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @project-name	: Project 1 (with HAL Library)
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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
static void GPIO_Init(void);
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

  /* USER CODE BEGIN 2 */

  //Initialize GPIO Ports
  GPIO_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Check if the pull-up button is pressed.
	  if(!(GPIOC->IDR & (1 << 2)))
	  {
		  //50ms bounce time
		  HAL_Delay(50);

		  //Wait while the pull-up button is pressed.
		  while(!(GPIOC->IDR & (1 << 2)));

		  //50ms bounce time
		  HAL_Delay(50);

		  //Change the state of open-drain led
		  GPIOA->ODR ^= 1 << 12;
	  }

	  //Or check if the pull-down button is pressed.
	  else if((GPIOC->IDR & (1 << 8)))
	  {
		  //50ms bounce time
		  HAL_Delay(50);

		  //Wait while the pull-down button is pressed.
		  while((GPIOC->IDR & (1 << 8)));

		  //50ms bounce time
		  HAL_Delay(50);

		  //Change the state of push-pull led
		  GPIOA->ODR ^= 1 << 11;
	  }

	  else
	  {
		  continue;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  RCC->AHBENR |= (1 << 17); //GPIOA
  RCC->AHBENR |= (1 << 19); //GPIOC

  /*Configure GPIOA 11 pin : push-pull led */

  //Set GPIO pin output
  GPIOA->MODER |= (1 << 22);
  GPIOA->MODER &= ~(1 << 23);

  //Enable push-pull output mode
  GPIOA->OTYPER &= ~(1 << 11);

  //Set low output speed
  GPIOA->OSPEEDR &= ~(1 << 22);

  //Set no pull-up, no pull-down
  GPIOA->PUPDR &= ~(1 << 22);
  GPIOA->PUPDR &= ~(1 << 23);

  /*Configure GPIOA 12 pin: open-drain led */

  //Set GPIO pin output
  GPIOA->MODER |= (1 << 24);
  GPIOA->MODER &= ~(1 << 25);

  //Enable open-drain output mode
  GPIOA->OTYPER |= (1 << 12);

  //Set low output speed
  GPIOA->OSPEEDR &= ~(1 << 24);

  //Set no pull-up, no pull-down
  GPIOA->PUPDR &= ~(1 << 24);
  GPIOA->PUPDR &= ~(1 << 25);

  /*Configure GPIOC 2 pin : pull-up led */

  //Set GPIO pin input
  GPIOC->MODER &= ~(1 << 4);
  GPIOC->MODER &= ~(1 << 5);

  //Enable pull-up output mode
  GPIOC->PUPDR |= (1 << 4);
  GPIOC->PUPDR &= ~(1 << 5);

  /*Configure GPIOC 8 pin : pull-down button */

  //Set GPIO pin input
  GPIOC->MODER &= ~(1 << 16);
  GPIOC->MODER &= ~(1 << 17);

  //Enable pull-down input mode
  GPIOC->PUPDR &= ~(1 << 16);
  GPIOC->PUPDR |= (1 << 17);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

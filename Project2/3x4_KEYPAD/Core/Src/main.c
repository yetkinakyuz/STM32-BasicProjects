/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
char keypad_scanner(void);
int _write(int file, char *ptr, int len);
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
  /* USER CODE BEGIN 2 */
  char key;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  key = keypad_scanner();

  	  HAL_Delay (200);

  	  if ( key != 'n')
  	  {
		  if (key == '1' || key == '7' || key == '5' || key == '0' || key == '3' || key == '#')
			  {
				 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
			  }

		  else
		  {
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
		  }
  	  }

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, R2_Pin|R1_Pin|R3_Pin|R4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : C3_Pin */
  GPIO_InitStruct.Pin = C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(C3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R2_Pin R1_Pin R3_Pin R4_Pin */
  GPIO_InitStruct.Pin = R2_Pin|R1_Pin|R3_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : C2_Pin C1_Pin */
  GPIO_InitStruct.Pin = C2_Pin|C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
char keypad_scanner(void)
{
	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 0); //Pin voltage level is '0'
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 1); //Pin voltage level is '1'

	if (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0);
		return '1';
	}

	else if (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0);
		return '2';
	}

	else if (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0);
		return '3';
	}

	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 0); //Pin voltage level is '0'
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 1); //Pin voltage level is '1'

	if (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0);
		return '4';
	}

	else if (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0);
		return '5';
	}

	else if (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0);
		return '6';
	}

	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 0); //Pin voltage level is '0'
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 1); //Pin voltage level is '1'

	if (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0);
		return '7';
	}

	else if (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0);
		return '8';
	}

	else if (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0);
		return '9';
	}

	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 0); //Pin voltage level is '0'

	if (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0);
		return '*';
	}

	else if (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0);
		return '0';
	}

	else if (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0);
		return '#';
	}

	return 'n';
}

int _write(int file, char *ptr, int len)
{
	  /* Implement your write code here, this is used by puts and printf for example */
	  int i=0;
	  for(i=0 ; i<len ; i++)
	  ITM_SendChar((*ptr++));
	  return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

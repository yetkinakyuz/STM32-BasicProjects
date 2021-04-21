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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
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
void InitLCD(void);
void lcd_init_write(unsigned char value);
void epulse(void);
void delay_us ();
void lcd_cmd(unsigned char cmd);
void lcd_write_data (char harf);
void lcd_data(unsigned char satir, unsigned char sutun, const char *mesaj);

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
  char* key;

  int pos = 5;

  InitLCD();
  lcd_data(1, 3, "LCD & KEYPAD");
  lcd_data(2, 1, "KEY:");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  key = keypad_scanner();

	  HAL_Delay(100);

	  if (key != 'n')
	  {
		  lcd_data(2, pos, &key);

		  if(pos >= 16)
		  {
			  HAL_Delay(100);

			  pos = 5;

			  lcd_data(2, pos, "                ");
			  lcd_data(2, 1, "KEY:");
			  lcd_data(2, pos, &key);

			  pos++;
		  }

		  else
		  {
			  pos++;
		  }
	  }
    /* USER CODE END WHILE */
  }
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
  HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin|LCD_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, R2_Pin|R1_Pin|R3_Pin|R4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D4_Pin|D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_E_Pin D4_Pin D5_Pin
                           D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_E_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
void InitLCD (void)
{
  HAL_Delay (200);
  lcd_init_write(0x30); //Special Sequence:Write Function Set.
  HAL_Delay (10);
  lcd_init_write(0x30); //Special Sequence:Write Function Set.
  HAL_Delay (10);
  lcd_init_write(0x30); //Special Sequence:Write Function Set.
  HAL_Delay (10);
  lcd_init_write(0x20); //Special Sequence:Write Function Set.
  HAL_Delay (10);

  lcd_cmd(0x28);//Function set(001(DL)NFxx)-DL=0 (4bit), N=1 (2 satir),F=0 (5X8))
  lcd_cmd(0x08);//Display control (0000 1DCB)-display is off, cursor is off, blinking is off
  lcd_cmd(0x01);//Clear Display
  lcd_cmd(0x06);  // Entry mode set (0000 01(I/D)S), Automatic Increment - No Display shift.
   // Initialization ends
  lcd_cmd(0x0F);//Display control(0000 1DCB)--display is on, cursor is on, blinking is on

}
void lcd_init_write(unsigned char value)
{

	HAL_GPIO_WritePin (LCD_RS_GPIO_Port, LCD_RS_Pin,0);  //LCD_RS=0;
	GPIOB->ODR= (GPIOB->ODR & 0x0F) | (value & 0xF0) ;        //Make No Affect on other Port Pins
	epulse(); //Send Enable Signal to LCD
	HAL_GPIO_WritePin (LCD_RS_GPIO_Port, LCD_RS_Pin,1);  //LCD_RS=1;

}
void epulse()
{
	HAL_GPIO_WritePin (LCD_E_GPIO_Port, LCD_E_Pin,1);  //LCD_E=1;
	delay_us();
	HAL_GPIO_WritePin (LCD_E_GPIO_Port, LCD_E_Pin,0);  //LCD_E=0;
	delay_us();
}
void delay_us ()
{
	unsigned int time=680; //delay=1us/170x2x680=8us
	while (time--);

}
void lcd_cmd(unsigned char cmd)
{
	     HAL_GPIO_WritePin (LCD_RS_GPIO_Port, LCD_RS_Pin,0);  //LCD_RS=0;
	     GPIOB->ODR= (GPIOB->ODR & 0x0F) | (cmd & 0xF0) ;        //Make No Affect on other Port Pins
	     //Send Higher Nibble to LCDPORT
	     epulse(); //Send Enable Signal to LCD
	     GPIOB->ODR = (GPIOB->ODR & 0x0f) | (cmd<<4);//Make No Affect on other Port Pins
	     //Send Lower Nibble to LCDPORT
	     epulse(); //Send Enable Signal to LCD
	     HAL_GPIO_WritePin (LCD_RS_GPIO_Port, LCD_RS_Pin,1);  //LCD_RS=1;
	     HAL_Delay(10);


}
void lcd_write_data (char harf)
{
	HAL_GPIO_WritePin (LCD_RS_GPIO_Port, LCD_RS_Pin,1);  //LCD_RS=1;
	GPIOB->ODR = (GPIOB->ODR & 0x0f) | (harf & 0xf0);//Make No Affect on other Port Pins
     //Send Higher Nibble to LCDPORT
    epulse(); //Send Enable Signal to LCD
    GPIOB->ODR = (GPIOB->ODR & 0x0f) | (harf<<4);//Make No Affect on other Port Pins
     //Send Lower Nibble to LCDPORT
     epulse(); //Send Enable Signal to LCD
     HAL_GPIO_WritePin (LCD_RS_GPIO_Port, LCD_RS_Pin,0);  //LCD_RS=0;

     HAL_Delay(10);


}
void lcd_data(unsigned char satir, unsigned char sutun, const char *mesaj)
{
	char temp;

	switch(satir)
	{
		case 1:
			temp=sutun-1+ 0x80;
			break;
		case 2:
			temp= sutun-1+0xc0;
			break;
	}

	lcd_cmd(temp);

	do
	{
		lcd_write_data(*mesaj++);

	}while(*mesaj);

}

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
		HAL_Delay(50);

		return '1';
	}

	else if (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0);
		HAL_Delay(50);

		return '2';
	}

	else if (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0);
		HAL_Delay(50);

		return '3';
	}

	HAL_Delay(50);

	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 0); //Pin voltage level is '0'
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 1); //Pin voltage level is '1'

	if (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0);
		HAL_Delay(50);

		return '4';
	}

	else if (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0);
		HAL_Delay(50);

		return '5';
	}

	else if (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0);
		HAL_Delay(50);

		return '6';
	}

	HAL_Delay(50);

	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 0); //Pin voltage level is '0'
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 1); //Pin voltage level is '1'

	if (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0);
		HAL_Delay(50);

		return '7';
	}

	else if (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0);
		HAL_Delay(50);

		return '8';
	}

	else if (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0);
		HAL_Delay(50);

		return '9';
	}

	HAL_Delay(50);

	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 1); //Pin voltage level is '1'
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 0); //Pin voltage level is '0'

	if (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)==0);
		HAL_Delay(50);

		return '*';
	}

	else if (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)==0);
		HAL_Delay(50);

		return '0';
	}

	else if (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0)
	{
		HAL_Delay(50);
		while (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)==0);
		HAL_Delay(50);

		return '#';
	}

	HAL_Delay(50);

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

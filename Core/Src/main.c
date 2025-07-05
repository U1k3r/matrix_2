/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef enum {
	REG_NO_OP = 0x00,
	REG_DIGIT_0 = 0x01,
	REG_DIGIT_1 = 0x02,
	REG_DIGIT_2 = 0x03,
	REG_DIGIT_3 = 0x04,
	REG_DIGIT_4 = 0x05,
	REG_DIGIT_5 = 0x06,
	REG_DIGIT_6 = 0x07,
	REG_DIGIT_7 = 0x08,
	REG_DECODE_MODE = 0x09,
	REG_INTENSITY = 0x0A,
	REG_SCAN_LIMIT = 0x0B,
	REG_SHUTDOWN = 0x0C,
	REG_DISPLAY_TEST = 0x0F,
	} MAX7219_REGISTERS;
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

volatile uint8_t button_1_Count = 0;
volatile uint8_t button_2_Count = 0;
uint32_t lastButtonPressTime = 0;

const uint8_t digit_0 [8] =
{
 0b00111100,
 0b01000010,
 0b01000010,
 0b01000010,
 0b01000010,
 0b01000010,
 0b01000010,
 0b00111100
};

const uint8_t digit_1 [8] =
{
  0b00001000,
  0b00011000,
  0b00101000,
  0b00001000,
  0b00001000,
  0b00001000,
  0b00001000,
  0b00001000
};

const uint8_t digit_2 [8] =
{
  0b00011000,
  0b00100100,
  0b00001000,
  0b00010000,
  0b00100000,
  0b00100000,
  0b00100000,
  0b00111100
};

const uint8_t digit_3 [8] =
{
  0b00011000,
  0b00100100,
  0b00001000,
  0b00010000,
  0b00001000,
  0b00100100,
  0b00011000,
  0b00000000
};

const uint8_t digit_4 [8] =
{
  0b00100100,
  0b00100100,
  0b00100100,
  0b00111100,
  0b00000100,
  0b00000100,
  0b00000100,
  0b00000100
};

const uint8_t digit_5 [8] =
{
  0b00111100,
  0b00100000,
  0b00100000,
  0b00111000,
  0b00000100,
  0b00000100,
  0b00111000,
  0b00000000
};

const uint8_t digit_6 [8] =
{
  0b00011000,
  0b00100100,
  0b00100000,
  0b00111000,
  0b00100100,
  0b00100100,
  0b00100100,
  0b00011000
};

const uint8_t digit_7 [8] =
{
  0b00111100,
  0b00000100,
  0b00001000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00000000
};

const uint8_t digit_8 [8] =
{
  0b00011000,
  0b00100100,
  0b00100100,
  0b00011000,
  0b00100100,
  0b00100100,
  0b00011000,
  0b00000000
};

const uint8_t digit_9 [8] =
{
  0b00011000,
  0b00100100,
  0b00100100,
  0b00011100,
  0b00000100,
  0b00100100,
  0b00011000,
  0b00000000
};


void clear()
{
	for (int i = 0; i < 9; i++){
		  max7219_SendData(i+1, 0x00);
		  new_max_SendData(i+1, 0x00);
	}
}


void max7219_DisplayDigit(uint8_t digit)
{
	switch(digit){

	case 0:
			for (uint8_t row = 0; row < 8; row++)
				max7219_SendData(row+1, digit_0[row]);
			return;
	case 1:
			for (uint8_t row = 0; row < 8; row++)
				max7219_SendData(row+1, digit_1[row]);
			return;
	case 2:
				for (uint8_t row = 0; row < 8; row++)
					max7219_SendData(row+1, digit_2[row]);
				return;
	case 3:
			for (uint8_t row = 0; row < 8; row++)
				max7219_SendData(row+1, digit_3[row]);
			return;
	case 4:
				for (uint8_t row = 0; row < 8; row++)
					max7219_SendData(row+1, digit_4[row]);
				return;
	case 5:
				for (uint8_t row = 0; row < 8; row++)
					max7219_SendData(row+1, digit_5[row]);
				return;
	case 6:
				for (uint8_t row = 0; row < 8; row++)
					max7219_SendData(row+1, digit_6[row]);
				return;
	case 7:
				for (uint8_t row = 0; row < 8; row++)
					max7219_SendData(row+1, digit_7[row]);
				return;
	case 8:
				for (uint8_t row = 0; row < 8; row++)
					max7219_SendData(row+1, digit_8[row]);
				return;
	case 9:
				for (uint8_t row = 0; row < 8; row++)
					max7219_SendData(row+1, digit_9[row]);
				return;

	}
}

void new_max_DisplayDigit(uint8_t digit)
{
	switch(digit){

	case 0:
				for (uint8_t row = 0; row < 8; row++)
					new_max_SendData(row+1, digit_0[row]);
				return;
	case 1:
				for (uint8_t row = 0; row < 8; row++)
					new_max_SendData(row+1, digit_1[row]);
				return;
	case 2:
				for (uint8_t row = 0; row < 8; row++)
					new_max_SendData(row+1, digit_2[row]);
				return;
	case 3:
				for (uint8_t row = 0; row < 8; row++)
					new_max_SendData(row+1, digit_3[row]);
				return;
	case 4:
				for (uint8_t row = 0; row < 8; row++)
					new_max_SendData(row+1, digit_4[row]);
				return;
	case 5:
				for (uint8_t row = 0; row < 8; row++)
					new_max_SendData(row+1, digit_5[row]);
				return;
	case 6:
				for (uint8_t row = 0; row < 8; row++)
					new_max_SendData(row+1, digit_6[row]);
				return;
	case 7:
				for (uint8_t row = 0; row < 8; row++)
					new_max_SendData(row+1, digit_7[row]);
				return;
	case 8:
				for (uint8_t row = 0; row < 8; row++)
					new_max_SendData(row+1, digit_8[row]);
				return;
	case 9:
				for (uint8_t row = 0; row < 8; row++)
					new_max_SendData(row+1, digit_9[row]);
				return;

	}
}


void max7219_SendData(uint8_t addr, uint8_t data)
{
	HAL_GPIO_WritePin(MAX7219_GPIO_Port, MAX7219_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi1,&addr, 1);
	HAL_SPI_Transmit_IT(&hspi1, &data, 1);
	HAL_GPIO_WritePin(MAX7219_GPIO_Port, MAX7219_Pin, GPIO_PIN_SET);
}

void new_max_SendData(uint8_t addr, uint8_t data)
{
	HAL_GPIO_WritePin(new_max_GPIO_Port, new_max_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi1,&addr, 1);
	HAL_SPI_Transmit_IT(&hspi1, &data, 1);
	HAL_GPIO_WritePin(new_max_GPIO_Port, new_max_Pin, GPIO_PIN_SET);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t currentTime = HAL_GetTick();
	if (currentTime - lastButtonPressTime > 250){

		if (GPIO_Pin == Button1_Pin) {
        	if(HAL_GPIO_ReadPin (GPIOA, Button1_Pin)==GPIO_PIN_RESET)
        	{
        		button_1_Count = (button_1_Count + 1) % 10;
        		lastButtonPressTime = currentTime;
        	}
        }

		if (GPIO_Pin == Button2_Pin) {
        	if(HAL_GPIO_ReadPin (GPIOA, Button2_Pin)==GPIO_PIN_RESET)
        	{
        		button_2_Count = (button_2_Count + 1) % 10;
        		lastButtonPressTime = currentTime;
        	}
		}
	}
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t MAX7219_INIT()
{
	max7219_SendData(REG_SCAN_LIMIT, 0x07); // limit off
	max7219_SendData(REG_DECODE_MODE, 0x00); // decode off
	max7219_SendData(REG_SHUTDOWN, 0x01);
}

uint8_t new_max_INIT()
{
	new_max_SendData(REG_SCAN_LIMIT, 0x07); // limit off
	new_max_SendData(REG_DECODE_MODE, 0x00); // decode off
	new_max_SendData(REG_SHUTDOWN, 0x01);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  MAX7219_INIT();
  new_max_INIT();

  clear();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  max7219_DisplayDigit(1);
	  max7219_DisplayDigit(button_1_Count);

	  new_max_DisplayDigit(button_2_Count);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "SSD1331.h"
#include "matrix_lib.h"
#include "Joystick.h"
#include "MY_FLASH.h"
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
uint8_t Received;
uint8_t testinkr = 0;
DMA_HandleTypeDef hdma_spi1_tx;
joystick joy;
int x,y;
int a,b,c,d =0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 50);
	return len;
}

void drawMenu()
{
	  ssd1331_draw_line(0,49,95,49, WHITE);
	  ssd1331_display_string(7, 0, "Reczny", FONT_1206, GREEN);
	  ssd1331_display_string(7, 11, "Sekwencja", FONT_1206, GREEN);
	  ssd1331_display_string(7, 22, "Coords", FONT_1206, GREEN);
	  ssd1331_fill_rect(0,4,4,4,WHITE);
	  ssd1331_display_string(0, 51, "Anuluj", FONT_1206, GREEN);//Wyświetlenie przykładowego tekstu
	  ssd1331_display_string(83, 51, "OK", FONT_1206, GREEN);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &Received, 1);

  ssd1331_init(); //Inicjalizacja wyświetlacza OLED
  ssd1331_clear_screen(BLACK); //Ustawianie tła wyświetlacza

drawMenu();

  joystick_Start(&hadc1, &joy);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	 if(matrix_ReadKey()==4)
	 {
		 a++;
		 ssd1331_clear_screen(0);
		 ssd1331_display_string(0, 0, "S1!", FONT_1608, GREEN);

	 }
	 if(matrix_ReadKey()==8)
	 {
		 ssd1331_clear_screen(0);
		 b++;
	 	 ssd1331_display_string(0, 0, "S2", FONT_1608, GREEN);
	 }
	 if(matrix_ReadKey()==9)
	 {
		 ssd1331_clear_screen(0);
		 ssd1331_display_string(0, 0, "Hello World!", FONT_1608, GREEN);
	 }
	 if(matrix_ReadKey()==5)
	 {
		 ssd1331_draw_line(2,40,60,20, GREEN);
	 }

	 if(matrix_ReadKey()==6)
	 	 {
	 		 ssd1331_draw_line(2,40,60,20, 0);
	 	 }

	 if(matrix_ReadKey()==13)
	 	 	 {
		 ssd1331_clear_screen(0);
		// ssd1331_draw_bitmap(0,0, array_usm, 96, 64, GREEN);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	//uint8_t Data[50]; // Tablica przechowujaca wysylana wiadomosc.
	//uint16_t size = 0; // Rozmiar wysylanej wiadomosci
	int mess =  420;

	// Odebrany znak zostaje przekonwertowany na liczbe calkowita i sprawdzony
	// instrukcja warunkowa


	switch (atoi(&Received)) {

	case 0: // Jezeli odebrany zostanie znak 0
		printf("%d\r\n", mess);

		break;

	case 4: // Jezeli odebrany zostanie znak 1
		printf("%d\r\n", mess);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		break;

	default: // Jezeli odebrano nieobslugiwany znak
		printf("%d\r\n", mess);
		break;
	}

	//HAL_UART_Transmit_IT(&huart2, Data, size); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
	HAL_UART_Receive_IT(&huart2, &Received, 1); // Ponowne w��czenie nas�uchiwania
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

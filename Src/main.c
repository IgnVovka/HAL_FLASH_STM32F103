/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct
{
	uint8_t  var1;
	uint16_t var2;
	uint32_t var3;
} test_struct;//структура для записи данных  в ячейку  памяти
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define flashADDR   0x0801F800//адрес 126 страницы
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
uint8_t writeFlash(uint32_t addr);//объявление функции(прототип)
void readFlash();//объявление функции(прототип)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*Переопределение функций для printf и scanf*/
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;
}
int fgetc(FILE *f)
{
	uint8_t ch;
	HAL_UART_Receive( &huart1,(uint8_t*)&ch,1, HAL_MAX_DELAY );
	return ch;
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
  readFlash(flashADDR);
  test_struct.var1=1;//№настройки
  test_struct.var2=20;//значение настройки
  test_struct.var3=2;//размер настройки
  //uint8_t stat=writeFlash(flashADDR);//инициализация функции записи во flash
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3); // запуск таймера в режиме прерываний
  int i=0;
  while(i<=2)
  {
      writeFlash(flashADDR);
      i++;
  }
  /*HAL_FLASH_Unlock();                           //разблокировка flash 
  FLASH_EraseInitTypeDef str;                       //переменная для настроек стирания
  uint32_t pageError=0;
  str.TypeErase = FLASH_TYPEERASE_PAGES;          // стирать постранично(1кбайт)
  str.PageAddress = flashADDR;                   // адрес страницы для стирания
  str.NbPages = 1;                              //1 страницу стираем
  if(HAL_FLASHEx_Erase(&str,&pageError)==HAL_OK)
      printf("Clear str\n");
  HAL_FLASH_Lock();*/
  uint8_t *ptr;//указатель для хранения адреса
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_UART_Transmit(&huart1, (uint8_t*)"Hello\n", strlen("Hello\n"), HAL_MAX_DELAY);
     printf("Enter address: ");//ввод адреса
     scanf("%p",&ptr);//сканируем в указатель, scanf понимает по спецификатору,что это указатель
     printf("\nOn this address be kept value %p is %x\n",ptr,*ptr);//по этому адресу хранится значение %p is %x
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

/* USER CODE BEGIN 4 */
uint8_t writeFlash (uint32_t addr)
{
	HAL_StatusTypeDef status;
	uint32_t structureSize = sizeof(test_struct);          // замеряем размер структуры
	FLASH_EraseInitTypeDef FlashErase;                     // переменная для структуры, которая выполняет функцию стирания страницы
	uint32_t Error = 0;                                // переменная для записи информации об ошибках в процессе стирания

	//__disable_irq();                                       // запрещаем прерывания
	HAL_FLASH_Unlock();									     //разблокировка FLASH
    /*определил параметр для стирания*/
	FlashErase.TypeErase = FLASH_TYPEERASE_PAGES;          // стирать постранично(1кбайт)
	FlashErase.PageAddress = addr;                         // адрес страницы для стирания
	FlashErase.NbPages = structureSize / 1024 + 1;         // считаем количество страниц, чтобы наш массив поместился
	if (HAL_FLASHEx_Erase(&FlashErase, &Error) != HAL_OK)   // вызов функции стирания
	{
		HAL_FLASH_Lock();                                  // если не смог стереть, то закрыть память и вернуть ошибку
		return HAL_ERROR;
	}

    uint32_t *dataPtr = (uint32_t *)&test_struct;          // создаем указатель на нашу структуру и записываем ее кусочками по 32 бита
	for (int i = 0; i < structureSize / 4; i++)            // 4 байта = 32 бита
    {
		status += HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, dataPtr[i]);
        addr += 4;                                         // сдвигаем адрес на 4 байта
    }
	//__enable_irq();                                        // включаем прерывания обратно
	HAL_FLASH_Lock();
	return status;
}
void readFlash (uint32_t addr)
{
	uint32_t structureSize = sizeof(test_struct);
	uint32_t *dataPtr = (uint32_t *)&test_struct;
	for (int i = 0; i < structureSize / 4; i++)
	{
		dataPtr[i] = *(__IO uint32_t*)addr;
		addr += 4;
	}
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

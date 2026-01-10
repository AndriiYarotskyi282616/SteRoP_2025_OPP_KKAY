/* USER CODE BEGIN Header */
/**
  **********************************************************************
  * @file           : main.c
  * @brief          : Main program body
  **********************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **********************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"	 //obsługa pinów
#include <stdlib.h>			 //dodaje fubkcje abs()
#include <string.h>          // do memset
#include <stdio.h>           // do sprintf
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_sdram.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SDRAM_BANK_ADDR				0xD0000000

#define SDRAM_SIZE_BYTES			0x800000
#define SDRAM_SIZE_WORDS			(SDRAM_SIZE_BYTES / 2)
#define SDRAM_SIZE_DWORDS			(SDRAM_SIZE_BYTES / 4)
#define LCD_FRAME_BUFFER_LAYER0  0xD0000000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Deklaracja uchwytów zdefiniowanych w innych plikach (spi.c, fmc.c)
extern SDRAM_HandleTypeDef hsdram1;
extern SPI_HandleTypeDef hspi5; // Uchwyt do SPI ekranu

static uint32_t CzasKoncowy_Buzzera = 0;
static uint8_t aktywny_buzzer = 0;
static uint32_t CzasTrwania_Buzzera = 2000;

static uint16_t dlugosc = 2000;				//dlugość między bramkami w [mm]
volatile int32_t CzasBramkiA = 0;			//edytowalna podczas przerwań
volatile int32_t CzasBramkiB = 0;			//edytowalna podczas przerwań

static double Ograniczenie = 0.01;			//1 > 1m/s		0.1 > 10cm/s	0.01 > 1 cm/s

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void buzzer(void);
double SpeedCheck(void);
static void FillBuffer(uint32_t color);

// --- FUNKCJA NAPRAWCZA: Wymuszenie prędkości GPIO ---
void Fix_Hardware_Speed(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Włącz zegary wszystkich portów
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    // 1. SDRAM PINS - VERY HIGH SPEED
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

    // Porty SDRAM
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6; HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_0; HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15; HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_15; HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    // 2. LCD CONTROL PINS (SPI5 + CS/WRX) - VERY HIGH SPEED
    // SPI5 (PF7=SCK, PF8=MISO, PF9=MOSI)
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // LCD_NCS (PC2) & LCD_WRX (PD13) - Output Pins
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Alternate = 0;

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); // PC2 = NCS

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); // PD13 = WRX
}

// --- FUNKCJA NAPRAWCZA: ZWOLNIENIE SPI ---
// CubeMX ustawił 45Mbit/s, co zawiesza ekran (Biały Ekran).
// Zmieniamy na Prescaler 16 (ok 5.6 MHz), co jest bezpieczne dla ILI9341.
void Fix_SPI_Speed(void)
{
    hspi5.Instance = SPI5;
    hspi5.Init.Mode = SPI_MODE_MASTER;
    hspi5.Init.Direction = SPI_DIRECTION_2LINES;
    hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi5.Init.NSS = SPI_NSS_SOFT;
    // ZMIANA Z PRESCALER_2 NA PRESCALER_16
    hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi5.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&hspi5) != HAL_OK)
    {
       Error_Handler();
    }
}

// Prosty test pamięci RAM
int SDRAM_Test(void) {
    volatile uint32_t *pMem = (uint32_t*)SDRAM_BANK_ADDR;
    uint32_t patterns[] = {0x12345678, 0xAABBCCDD, 0x5555AAAA, 0x00000000, 0xFFFFFFFF};

    for (int i = 0; i < 5; i++) {
        *pMem = patterns[i];
        __DSB();
        if (*pMem != patterns[i]) return 0;

        *(pMem + 1000) = patterns[i];
        __DSB();
        if (*(pMem + 1000) != patterns[i]) return 0;
    }
    return 1;
}

// Funkcja czyszcząca
void SDRAM_Clear()
{
	for(uint32_t count = 0; count < SDRAM_SIZE_WORDS; count++)
	{
		*(__IO uint16_t*)(SDRAM_BANK_ADDR + count * 2) = (uint16_t)0x00;
	}
}

void SDRAM_WriteIncrement_bytes()
{
	for (uint32_t count = 0; count < SDRAM_SIZE_BYTES; count++)
	{
		*(__IO uint8_t*)(SDRAM_BANK_ADDR + count) = (uint8_t)count;
	}
}

void SDRAM_WriteIncrement_words()
{
    for (uint32_t count = 0; count < SDRAM_SIZE_WORDS; count++)
    {
        *(__IO uint16_t*)(SDRAM_BANK_ADDR + count * 2) = (uint16_t)count;
    }
}

void SDRAM_WriteIncrement_dwords()
{
    for (uint32_t count = 0; count < SDRAM_SIZE_DWORDS; count++)
    {
        *(__IO uint32_t*)(SDRAM_BANK_ADDR + count * 4) = count; // 4 bytes in one count (Jump 4 C pointers)
    }
}
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
  MX_CRC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

    // 1. NAPRAWA SPRZĘTOWA (Piny)
    Fix_Hardware_Speed();

    // 2. Inicjalizacja pamięci SDRAM
    if(BSP_SDRAM_Init() != SDRAM_OK) {
        Error_Handler();
    }

    // 3. Poprawka odświeżania RAM (żeby nie było pasków)
    HAL_SDRAM_ProgramRefreshRate(&hsdram1, 0x250);

    // 4. NAPRAWA SPI (Kluczowe dla Białego Ekranu!)
    // Musimy zwolnić SPI przed inicjalizacją ekranu
    Fix_SPI_Speed();

    HAL_Delay(50);

    // --- DIAGNOSTYKA RAM ---
    if (SDRAM_Test()) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET); // Czerwona OFF
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);   // Zielona ON (PG13)

        // 5. Inicjalizacja ekranu (Teraz na wolniejszym SPI)
        BSP_LCD_Init();

        BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER_LAYER0);
        BSP_LCD_SelectLayer(0);
        BSP_LCD_DisplayOn();

        // Czyścimy na CZARNO. Jeśli zadziała, biel zniknie.
        BSP_LCD_Clear(LCD_COLOR_BLACK);

        BSP_LCD_SetFont(&Font24);
        BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
        BSP_LCD_SetTextColor(LCD_COLOR_GREEN);

        BSP_LCD_DisplayStringAt(0, 140, (uint8_t *)"SPI FIXED!", CENTER_MODE);

    } else {
        // BŁĄD RAM
        while(1) {
            HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);
            HAL_Delay(100);
        }
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t blink_timer = 0;
  uint8_t blink_state = 0;
  uint32_t redraw_timer = 0;
  char msg_buffer[30];
  int counter = 0;

  while (1)
  {
	  SpeedCheck();
	  buzzer();

      // Heartbeat
      if (HAL_GetTick() - blink_timer > 500) {
          blink_timer = HAL_GetTick();
          blink_state = !blink_state;
          counter++;

          // Mrugaj kwadratem
          if(blink_state) {
              BSP_LCD_SetTextColor(LCD_COLOR_RED);
              BSP_LCD_FillRect(20, 20, 30, 30);
          } else {
              BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
              BSP_LCD_FillRect(20, 20, 30, 30);
          }
      }

      // Odświeżanie napisów
      if (HAL_GetTick() - redraw_timer > 500) {
          redraw_timer = HAL_GetTick();

          BSP_LCD_SetFont(&Font24);
          BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
          BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);

          BSP_LCD_DisplayStringAt(0, 140, (uint8_t *)"SPI FIXED!", CENTER_MODE);

          BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
          sprintf(msg_buffer, "Count: %d", counter);
          BSP_LCD_DisplayStringAt(0, 180, (uint8_t *)msg_buffer, CENTER_MODE);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
// --- Funkcja obsługi przerwań ---
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) {
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13); // LED1 (User Green)
        if (CzasBramkiA == 0) {
        	CzasBramkiA = HAL_GetTick();
        }
    }

    if (GPIO_Pin == GPIO_PIN_5) {
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14); // LED2 (User Red)
        if (CzasBramkiB == 0) {
        	CzasBramkiB = HAL_GetTick();
        }
    }
}

double SpeedCheck(void){
    double speed = 0;
    if (CzasBramkiA != 0 && !aktywny_buzzer){
    	CzasKoncowy_Buzzera = HAL_GetTick() + CzasTrwania_Buzzera;
    }
    if (CzasBramkiA != 0 && CzasBramkiB != 0){
        speed = abs(dlugosc / (CzasBramkiA - CzasBramkiB));    //wynik w [m/s]
        CzasBramkiA = 0;
        CzasBramkiB = 0;
    }
    if ((speed > Ograniczenie) && (!aktywny_buzzer)){
      CzasKoncowy_Buzzera = HAL_GetTick() + CzasTrwania_Buzzera;
      aktywny_buzzer = 1;
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);		// Włączenie PWM
    }
    return speed;
}

void buzzer(void)
{
	if (aktywny_buzzer && HAL_GetTick() >= CzasKoncowy_Buzzera){
         HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		 aktywny_buzzer = 0;
	}
}

static void FillBuffer(uint32_t color)
{
	hdma2d.Init.Mode         = DMA2D_R2M;
	hdma2d.Init.ColorMode    = DMA2D_RGB565;
	hdma2d.Init.OutputOffset = 0;
	hdma2d.Instance = DMA2D;
	if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_DMA2D_Start(&hdma2d, color, (uint32_t)0xD0000000, 240, 320) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_DMA2D_PollForTransfer(&hdma2d, 10);
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
      // W przypadku bledu migaj szybciej (dla diagnostyki)
      HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);
      HAL_Delay(50);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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

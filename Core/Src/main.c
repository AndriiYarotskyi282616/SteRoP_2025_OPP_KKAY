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
#include "dma2d.h"
#include "ltdc.h"
#include "touch.h"
#include "spi.h"
#include "fmc.h"
#include "i2c.h"
#include "gpio.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_FRAME_BUFFER_LAYER0  0xD0000000
#define LCD_FRAME_BUFFER_LAYER1  0xD0130000
#ifndef LCD_COLOR_LIGHTRED
#define LCD_COLOR_LIGHTRED      0xFFFC9F9F
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t CzasKoncowy_Buzzera = 0;
static uint8_t aktywny_buzzer = 0;
static uint8_t wlaczone_ustawienia = 0;
static uint32_t CzasTrwania_Buzzera = 2000;
static uint16_t TIMEOUT_POMIARU = 5000;

static uint8_t dlugosc = 200;              // długość między bramkami w [mm]
volatile uint32_t CzasBramkiA = 0;
volatile uint32_t CzasBramkiB = 0;

double Ograniczenie = 0.60;           // limit prędkości w [m/s]

double ostatnia_predkosc = 0.0;
double aktualna_predkosc = 0.0;
char buf[32];					//string dla prędkości
tp_state_t tp_state;			//struktura dla dotyku
char wspolrzedne[20] = {0};		//string dla współrzędnych dotyku
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void buzzer(void);
double SpeedCheck(void);
void lcd_desktop(void);
void lcd_ustawienia(void);
void lcd_przekroczenie(void);
void lcd_norma(void);
void touch_zmiana_menu(void);
void touch_ustawienia(void);

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
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_TIM2_Init();
  TP_Config();

  /* USER CODE BEGIN 2 */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_FMC_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);


  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER_LAYER0);
  BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER_LAYER1);
  BSP_LCD_SelectLayer(1);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  lcd_desktop();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
	  buzzer();	//ma zawsze czuwać by się wyłączał normalnie
	  aktualna_predkosc = SpeedCheck();	//sprawdza ograniczenie niezależnie od wyswietlacza

	  if (wlaczone_ustawienia){
		  lcd_ustawienia();
		  touch_ustawienia();
		  touch_zmiana_menu();	//sprawdza czy nie jest moment na zmiany
	  } else {	//nie włączone ustawienia => pokazywanie pomiarów
	      lcd_desktop();

	      if (aktualna_predkosc > 0){	//jeżeli nowy pomiar to aktualizuj dane
	          ostatnia_predkosc = aktualna_predkosc;
	          aktualna_predkosc = 0;	//ustawienie flagi na off
	          int calkowita = (int)ostatnia_predkosc;
	          int ulamkowa = (int)((ostatnia_predkosc - calkowita)*100);
	          sprintf(buf, "%d.%02d m/s", calkowita, ulamkowa);

	          if (ostatnia_predkosc > Ograniczenie){
	        	  lcd_przekroczenie();
	          } else {
	        	  lcd_norma();
	          }
	      }
	      touch_zmiana_menu();	//sprawdza czy nie jest moment na zmiany
	  }

	  HAL_Delay(30);	//antilag

	  //debug
	  /*TP_GetState(&tp_state);
	  if (tp_state.touchDetected){
		  sprintf(wspolrzedne, "x=%03d, y=%03d", tp_state.x, tp_state.y);
		  BSP_LCD_SetFont(&Font24);
		  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		  BSP_LCD_DisplayStringAt(0,300, (uint8_t*)wspolrzedne, CENTER_MODE);
	  }*/
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
    if(GPIO_Pin == GPIO_PIN_0){
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13); // LED1
        if (CzasBramkiA == 0){
            CzasBramkiA = HAL_GetTick();
        }
    }

    if(GPIO_Pin == GPIO_PIN_5){
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14); // LED2
        if (CzasBramkiA != 0 && CzasBramkiB == 0){
            CzasBramkiB = HAL_GetTick();
        }
    }
}

double SpeedCheck(void){
    double speed = 0;
    if (CzasBramkiA != 0 && CzasBramkiB == 0){
        if ((HAL_GetTick() - CzasBramkiA) > TIMEOUT_POMIARU){
            CzasBramkiA = 0;

            //BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
            //BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            //BSP_LCD_SetFont(&Font16);
            //BSP_LCD_DisplayStringAt(0, 130, (uint8_t*)"  RESET POMIARU  ", CENTER_MODE);
        }
    }
    if (CzasBramkiA != 0 && CzasBramkiB != 0){
        uint32_t czas;
        if (CzasBramkiB > CzasBramkiA){
             czas = CzasBramkiB - CzasBramkiA;
        }
        else{
             czas = 0;
        }
        if (czas > 0){
            speed = (double)dlugosc/(double)czas;
        }
        else{
            speed = 999.9;
        }
        CzasBramkiA = 0;
        CzasBramkiB = 0;
    }
    if (speed > Ograniczenie && speed < 999.0){
        if (aktywny_buzzer == 0){
            CzasKoncowy_Buzzera = HAL_GetTick() + CzasTrwania_Buzzera;
            aktywny_buzzer = 1;
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
        }
    }
    return speed;
}

void buzzer(void){
	if (HAL_GetTick() >= CzasKoncowy_Buzzera && aktywny_buzzer == 1){
         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		 aktywny_buzzer = 0;
	}
}

void lcd_desktop(void){
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
	BSP_LCD_DisplayStringAt(0, 10, (uint8_t*)"ODCINKOWY POMIAR", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, 30, (uint8_t*)"PREDKOSCI", CENTER_MODE);

	BSP_LCD_SetFont(&Font16);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DisplayStringAt(0, 70, (uint8_t*)"Ostatni pomiar:", CENTER_MODE);

	BSP_LCD_SetFont(&Font24);
	if (ostatnia_predkosc == 0){
		BSP_LCD_DisplayStringAt(0, 100, (uint8_t*)"0.00 m/s", CENTER_MODE);
	} else {
		BSP_LCD_DisplayStringAt(0, 100, (uint8_t*)buf, CENTER_MODE);
	}

	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_FillCircle(40, 280, 30);

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(15, 270, 51, 20);
}

void lcd_ustawienia(void){
	char tmp[32];	//tymczasowe do wyświetlenia
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
	BSP_LCD_DisplayStringAt(0, 10, (uint8_t*)"USTAWIENIA", CENTER_MODE);

	BSP_LCD_SetFont(&Font16);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DisplayStringAt(0, 50, (uint8_t*)"Ograniczenie [m/s]", CENTER_MODE);

	BSP_LCD_SetFont(&Font24);
	sprintf(tmp, "-  %d.%02d  +", (int)Ograniczenie, (int)((Ograniczenie - (int)Ograniczenie)*100));
	BSP_LCD_DisplayStringAt(0, 80, (uint8_t*)tmp, CENTER_MODE);

	BSP_LCD_SetFont(&Font16);
	BSP_LCD_DisplayStringAt(0, 120, (uint8_t*)"Czas sygnalu [s]", CENTER_MODE);

	BSP_LCD_SetFont(&Font24);
	sprintf(tmp, "-  %ld.%02ld  +", CzasTrwania_Buzzera/1000, CzasTrwania_Buzzera%1000/10);
	BSP_LCD_DisplayStringAt(0, 150, (uint8_t*)tmp, CENTER_MODE);

	BSP_LCD_SetFont(&Font16);
	BSP_LCD_DisplayStringAt(0, 190, (uint8_t*)"Czas pomiaru [s]", CENTER_MODE);

	BSP_LCD_SetFont(&Font24);
	sprintf(tmp, "-  %d.%02d  +", TIMEOUT_POMIARU/1000, TIMEOUT_POMIARU%1000/10);
	BSP_LCD_DisplayStringAt(0, 220, (uint8_t*)tmp, CENTER_MODE);

	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_FillCircle(40, 280, 30);

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(15, 270, 51, 20);
}

void lcd_przekroczenie(void){
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(0, 160, (uint8_t*)"PRZEKROCZENIE", CENTER_MODE);
}

void lcd_norma(void){
	BSP_LCD_Clear(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_SetFont(&Font24);
    BSP_LCD_DisplayStringAt(0, 160, (uint8_t*)"W NORMIE", CENTER_MODE);
}

void touch_zmiana_menu(void){
    TP_GetState(&tp_state);
    if (tp_state.touchDetected){
    	if(tp_state.x < 90 && tp_state.y > 260){
			HAL_Delay(300);
			wlaczone_ustawienia = 1 - wlaczone_ustawienia;	//zmiana stanu typu TOGGLE
			BSP_LCD_Clear(LCD_COLOR_WHITE);					//czyszczenie przed nowym menu
    	}
    }
}

void touch_ustawienia(void){
    TP_GetState(&tp_state);
    if (tp_state.touchDetected){
    	if(tp_state.x < 80 && tp_state.y > 70 && tp_state.y < 110 && Ograniczenie > 0.02){
			Ograniczenie -= 0.010;
    		HAL_Delay(100);
    	}
    	if(tp_state.x > 160 && tp_state.y > 70 && tp_state.y < 110){
			Ograniczenie += 0.010;
    		HAL_Delay(100);
    	}

    	if(tp_state.x < 80 && tp_state.y > 140 && tp_state.y < 170 && CzasTrwania_Buzzera > 99){
			CzasTrwania_Buzzera -= 100;
    		HAL_Delay(100);
    	}
    	if(tp_state.x > 160 && tp_state.y > 140 && tp_state.y < 170){
    		CzasTrwania_Buzzera += 100;
    		HAL_Delay(100);
    	}

    	if(tp_state.x < 80 && tp_state.y > 210 && tp_state.y < 240 && TIMEOUT_POMIARU > 1000){
    		TIMEOUT_POMIARU -= 100;
    		HAL_Delay(100);
    	}
    	if(tp_state.x > 160 && tp_state.y > 210 && tp_state.y < 240){
    		TIMEOUT_POMIARU += 100;
    		HAL_Delay(100);
    	}
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

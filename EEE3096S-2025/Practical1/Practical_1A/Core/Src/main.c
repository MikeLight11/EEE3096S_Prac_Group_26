/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include "lcd_stm32f0.h"
#include "lcd_stm32f0.c"
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
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
// TODO: Define input variables
int PA0_state = 0;
int PA1_state = 0;
int PA2_state = 0;
int PA3_state = 0;

int delay_mode = 0; 

int led_position = 0; 
int led_direction = 1;

int random_int = 0;
int random_on_delay = 0;
int random_off_delay = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void TIM16_IRQHandler(void);
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
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  // TODO: Start timer TIM16
  HAL_TIM_Base_Start_IT(&htim16);

  init_LCD(); // Initialize LCD display

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // TODO: Check pushbuttons to change timer delay
    PA0_state = LL_GPIO_IsInputPinSet(Button0_GPIO_Port, Button0_Pin);

    if (PA1_state == 0 && !LL_GPIO_IsInputPinSet(Button1_GPIO_Port, Button1_Pin)) {
        PA1_state = 1;
        PA2_state = 0;
        PA3_state = 0;
    } else if (PA2_state == 0 && !LL_GPIO_IsInputPinSet(Button2_GPIO_Port, Button2_Pin)) { 
        PA1_state = 0;
        PA2_state = 1;
        PA3_state = 0;
    } else if (PA3_state == 0 && !LL_GPIO_IsInputPinSet(Button3_GPIO_Port, Button3_Pin)) {
        PA1_state = 0;
        PA2_state = 0;
        PA3_state = 1;
    }

    if (PA0_state == 0 && !LL_GPIO_IsInputPinSet(Button0_GPIO_Port, Button0_Pin)) {
      // Button 0 just pressed (edge detection)
        
        if (delay_mode == 0) {
            htim16.Instance->ARR = 1000 - 1;
            delay_mode = 1;
        } else {
            htim16.Instance->ARR = 500 - 1;
            delay_mode = 0;
        }
    }
    
    PA0_state = LL_GPIO_IsInputPinSet(Button0_GPIO_Port, Button0_Pin);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */
  NVIC_EnableIRQ(TIM16_IRQn);
  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

  /**/
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED6_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED6_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TIM16_IRQHandler(void)
{
  // Acknowledge interrupt
	HAL_TIM_IRQHandler(&htim16);

	// TODO: Change LED pattern
  if (PA1_state == 1) {
    lcd_command(CLEAR);
    lcd_putstring("Button 1 pressed");

    LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
    LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
    LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
    LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
    LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
    LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
    LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
    LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
    
    // Button 1 pressed
    switch(led_position) {
      case 0: LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin); break;
      case 1: LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin); break;
      case 2: LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin); break;
      case 3: LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin); break;
      case 4: LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin); break;
      case 5: LL_GPIO_SetOutputPin(LED5_GPIO_Port, LED5_Pin); break;
      case 6: LL_GPIO_SetOutputPin(LED6_GPIO_Port, LED6_Pin); break;
      case 7: LL_GPIO_SetOutputPin(LED7_GPIO_Port, LED7_Pin); break;
    }
    
    led_position += led_direction;
    
    if (led_position >= 7) {
      led_position = 7;
      led_direction = -1; // right to left
    } else if (led_position <= 0) {
      led_position = 0;
      led_direction = 1; // left to right
    }

  } else if (PA2_state == 1) {
    // Button 2 pressed
    lcd_command(CLEAR);
    lcd_putstring("Button 2 pressed");

    LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
    LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
    LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
    LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin);
    LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin);
    LL_GPIO_SetOutputPin(LED5_GPIO_Port, LED5_Pin);
    LL_GPIO_SetOutputPin(LED6_GPIO_Port, LED6_Pin);
    LL_GPIO_SetOutputPin(LED7_GPIO_Port, LED7_Pin);
    
    switch(led_position) {
      case 0: LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin); break;
      case 1: LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin); break;
      case 2: LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin); break;
      case 3: LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin); break;
      case 4: LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin); break;
      case 5: LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin); break;
      case 6: LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin); break;
      case 7: LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin); break;
    }
    
    led_position += led_direction;
    
    if (led_position >= 7) {
      led_position = 7;
      led_direction = -1;  // right to left
    } else if (led_position <= 0) {
      led_position = 0;
      led_direction = 1;   // left to right
    }
  } else if (PA3_state == 1) {
    // Button 3 pressed
    lcd_command(CLEAR);
    lcd_putstring("Button 3 pressed");

    random_int = rand() % 256;

    GPIOB -> ODR = random_int;

    random_on_delay = rand() % 1401 + 100; // Random delay between 100ms and 1500ms
    delay(random_on_delay*300);

    while (random_int != 0) {
      int index = rand() % 8;
      while (!(random_int & (1 << index))) {
        index = rand() % 8;
      }

      switch(index) {
        case 0: LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin); break;
        case 1: LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin); break;
        case 2: LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin); break;
        case 3: LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin); break;
        case 4: LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin); break;
        case 5: LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin); break;
        case 6: LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin); break;
        case 7: LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin); break;
      }

      random_int &= ~(1 << index);

      random_off_delay = rand() % 101; // Random delay between 0ms and 100ms
      delay(random_off_delay*300);
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

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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// D15
#define TRIG_PIN GPIO_PIN_8
#define TRIG_PORT GPIOB

// D14
#define ECHO_PIN GPIO_PIN_9
#define ECHO_PORT GPIOB

// D13
#define BUTTON_1_PIN GPIO_PIN_5
#define BUTTON_1_PORT GPIOA

// D12
#define BUTTON_2_PIN GPIO_PIN_6
#define BUTTON_2_PORT GPIOA

// D11
#define BUZZER_PIN GPIO_PIN_7
#define BUZZER_PORT GPIOA

// 10 SEGMENT DISPLAY
// D10
#define SEGMENT_1_PIN GPIO_PIN_6
#define SEGMENT_1_PORT GPIOB

// D9
#define SEGMENT_2_PIN GPIO_PIN_7
#define SEGMENT_2_PORT GPIOC

// D8
#define SEGMENT_3_PIN GPIO_PIN_9
#define SEGMENT_3_PORT GPIOA

// D7
#define SEGMENT_4_PIN GPIO_PIN_8
#define SEGMENT_4_PORT GPIOA

// D6
#define SEGMENT_5_PIN GPIO_PIN_10
#define SEGMENT_5_PORT GPIOB

// D5
#define SEGMENT_6_PIN GPIO_PIN_4
#define SEGMENT_6_PORT GPIOB

// D4
#define SEGMENT_7_PIN GPIO_PIN_5
#define SEGMENT_7_PORT GPIOB

// D3
#define SEGMENT_8_PIN GPIO_PIN_3
#define SEGMENT_8_PORT GPIOB

// CN10 34
#define SEGMENT_9_PIN GPIO_PIN_13
#define SEGMENT_9_PORT GPIOB

// CN10 30
#define SEGMENT_10_PIN GPIO_PIN_4
#define SEGMENT_10_PORT GPIOC

#define PUSHUP_DOWN_DISTANCE_CM 10
#define PUSHUP_UP_DISTANCE_CM 30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
TaskHandle_t PushupCounterTaskHandle;
TaskHandle_t AreButtonsPressedTaskHandle;
TaskHandle_t LedCounterTaskHandle;

char buttonBuffer[50];
char ultraBuffer[50];

uint32_t PUSHUP_TARGET= 10;

uint32_t pushupCounter = 0;
uint8_t isButtonsPressed = 0;

GPIO_TypeDef* SEGMENT_PORTS[10] = {
	SEGMENT_1_PORT,
	SEGMENT_2_PORT,
	SEGMENT_3_PORT,
	SEGMENT_4_PORT,
	SEGMENT_5_PORT,
	SEGMENT_6_PORT,
	SEGMENT_7_PORT,
	SEGMENT_8_PORT,
	SEGMENT_9_PORT,
	SEGMENT_10_PORT
};

uint16_t SEGMENT_PINS[10] = {
	SEGMENT_1_PIN,
	SEGMENT_2_PIN,
	SEGMENT_3_PIN,
	SEGMENT_4_PIN,
	SEGMENT_5_PIN,
	SEGMENT_6_PIN,
	SEGMENT_7_PIN,
	SEGMENT_8_PIN,
	SEGMENT_9_PIN,
	SEGMENT_10_PIN
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void setPushupTarget(void);
void AreButtonsPressedTask(void *argument);
void PushupCounterTask(void *argument);
void Trigger_Ultrasonic(void);
uint32_t Get_Distance(void);
void handleWin(void);
void playSound(uint32_t frequency, uint32_t duration_ms);
void reset(void);
void LedCounterTask(void *argument);
void CheckForExitTask(void *argument);
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  setPushupTarget();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  xTaskCreate(
  PushupCounterTask,
  "PushupCounterTask",
  configMINIMAL_STACK_SIZE,
  NULL,
  1,
  &PushupCounterTaskHandle);

  xTaskCreate(AreButtonsPressedTask,
  "AreButtonsPressedTask",
  configMINIMAL_STACK_SIZE,
  NULL,
  tskIDLE_PRIORITY,
  &AreButtonsPressedTaskHandle);

  xTaskCreate(LedCounterTask,
  "LedCounterTask",
  configMINIMAL_STACK_SIZE,
  NULL,
  tskIDLE_PRIORITY,
  &LedCounterTaskHandle);

  xTaskCreate(CheckForExitTask,
  "CheckForExitTask",
  configMINIMAL_STACK_SIZE,
  NULL,
  tskIDLE_PRIORITY,
  NULL);


  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB13 PB3 PB4
                           PB5 PB6 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  /*
   * ON IOC CHANGE, UPDATE THE PA8 AND PA9 PINS TO GPIO_PULLUP
   */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void setPushupTarget(void) {
    char targetBuffer[10] = {0};
    char ch;
    int i = 0;

    HAL_UART_Transmit(&huart2, (uint8_t *)"\033[2J\033[H", 7, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"Enter pushup target: ", 21, HAL_MAX_DELAY);

    while (i < sizeof(targetBuffer) - 1) {
        HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

        if (ch == '\r') {
            break;
        }

        HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
        targetBuffer[i++] = ch;
    }

    targetBuffer[i] = '\0';
    PUSHUP_TARGET = atoi(targetBuffer);

    sprintf(ultraBuffer, "\r\nTarget Set: %lu\r\n", PUSHUP_TARGET);
    HAL_UART_Transmit(&huart2, (uint8_t*)ultraBuffer, strlen(ultraBuffer), HAL_MAX_DELAY);
}

void AreButtonsPressedTask(void *argument){

	for(;;){
		if(HAL_GPIO_ReadPin(BUTTON_1_PORT, BUTTON_1_PIN) == GPIO_PIN_RESET  && HAL_GPIO_ReadPin(BUTTON_2_PORT, BUTTON_2_PIN) == GPIO_PIN_RESET){
			isButtonsPressed = 1;
		} else {
			isButtonsPressed = 0;
		}
	}
}

void PushupCounterTask(void *argument)
{
	uint8_t downOk = 0;
	for(;;)
    {

        Trigger_Ultrasonic();

        uint32_t distance = Get_Distance();
        if(isButtonsPressed){
        	if(pushupCounter < PUSHUP_TARGET){
        		if(distance <= PUSHUP_DOWN_DISTANCE_CM && !downOk ){
        			downOk = 1;
        		}
        		else if(distance >= PUSHUP_UP_DISTANCE_CM && downOk){
        			pushupCounter++;
        			downOk = 0;
        		}
        	} else {
        		handleWin();
        		osDelay(500);
        		reset();
        	}
        } else {
        	reset();
        }

        sprintf(ultraBuffer, "Counter: %lu Distance: %lu\r\n", pushupCounter, distance);
		HAL_UART_Transmit(&huart2, (uint8_t*)ultraBuffer, strlen(ultraBuffer), HAL_MAX_DELAY);
        osDelay(500);
    }
}

void Trigger_Ultrasonic(void)
{
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    osDelay(1);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}

uint32_t Get_Distance(void)
{
	uint32_t echoStart = 0;
	uint32_t echoEnd = 0;
	uint32_t echoDuration = 0;

    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET);

    echoStart = __HAL_TIM_GET_COUNTER(&htim2);

    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET);

    echoEnd = __HAL_TIM_GET_COUNTER(&htim2);

    echoDuration = echoEnd - echoStart;

    uint32_t distance = (echoDuration * 0.0343) / 2;

    return distance;
}

void reset(void){
	pushupCounter = 0;
	for(int i = 0 ; i < PUSHUP_TARGET ; i ++){
		HAL_GPIO_WritePin(SEGMENT_PORTS[i], SEGMENT_PINS[i], GPIO_PIN_RESET);
	}
}

void handleWin(void){
//	sprintf(ultraBuffer, "WIN WIN WIN\r\n");
//	HAL_UART_Transmit(&huart4, (uint8_t*)ultraBuffer, strlen(ultraBuffer), HAL_MAX_DELAY);

	// caza
	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	osDelay(220);

	// caza
	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	osDelay(220);

	// caza caza caza
	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	osDelay(220);

	// a turma é mesmo boa
	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);


	osDelay(220);

	// é mesmo da fuzaca
	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	osDelay(220);

	// sport
	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	osDelay(220);

	// sport
	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);

	osDelay(220);

	// sport
	playSound(300, 200);
	osDelay(20);

	playSound(300, 200);
	osDelay(20);
}

void playSound(uint32_t frequency, uint32_t duration_ms){
	uint32_t period_us = 1000000 / frequency;
	uint32_t half_period_us = period_us / 2;

	uint32_t startTick = HAL_GetTick();

	while ((HAL_GetTick() - startTick) < duration_ms)
	{
		HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
		osDelay(half_period_us / 1000);

		HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
		osDelay(half_period_us / 1000);
	}
}

void LedCounterTask(void *argument){

	for(;;){
		int32_t TOTAL_LEDS = 10;
		int32_t ledCount = pushupCounter % TOTAL_LEDS;

		for (int i = 0; i < TOTAL_LEDS; i++) {
			HAL_GPIO_WritePin(SEGMENT_PORTS[i], SEGMENT_PINS[i], ledCount > i);
		}

		osDelay(300);
	}
}

void CheckForExitTask(void *argument) {
    char ch;
    for (;;) {
        HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
        if (ch == 'J' || ch == 'j') {
        	vTaskSuspend(PushupCounterTaskHandle);
			vTaskSuspend(AreButtonsPressedTaskHandle);
			vTaskSuspend(LedCounterTaskHandle);

            HAL_UART_Transmit(&huart2, (uint8_t *)"\r\nResetting...\r\n", 17, HAL_MAX_DELAY);
            reset();
            setPushupTarget();

            vTaskResume(PushupCounterTaskHandle);
            vTaskResume(AreButtonsPressedTaskHandle);
            vTaskResume(LedCounterTaskHandle);
        }
        osDelay(100);
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

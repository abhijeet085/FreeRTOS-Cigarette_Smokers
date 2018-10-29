
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId AGENTHandle[3];
osThreadId PUSHERHandle[3];
osThreadId SMOKERHandle[6];

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

char *smoker_types[3] = {"MATCHES-TOBACCO", "MATCHES-PAPER", "TOBACCO-PAPER"};
uint8_t items_on_table[3]={0,0,0};
xSemaphoreHandle agent_ready, mutex, pusher_sem[3], smoker_sem[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);
void agent(void const * argument);
void pusher(void const * argument);
void smoker(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/*void agent(void *argument);
void pusher(void *argument);
void smoker(void *argument);*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	uint8_t i=0;
	mutex = xSemaphoreCreateMutex();
	if(mutex == NULL)
	{
		Error_Handler();
	}
	vSemaphoreCreateBinary(agent_ready);
	//for(i=0; i<3; i++)
	//{
		smoker_sem[0] = xSemaphoreCreateCounting(1,0);
		smoker_sem[1] = xSemaphoreCreateCounting(1,0);
		smoker_sem[2] = xSemaphoreCreateCounting(1,0);
		
		pusher_sem[0] = xSemaphoreCreateCounting(1,0);
		pusher_sem[1] = xSemaphoreCreateCounting(1,0);
		pusher_sem[2] = xSemaphoreCreateCounting(1,0);
	//}
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of AGENT */
	uint32_t id[3] = {0,1,2};
  osThreadDef(AGENT, agent, osPriorityNormal, 3, 128);
	AGENTHandle[0] = osThreadCreate(osThread(AGENT), (void *)(id));
	AGENTHandle[1] = osThreadCreate(osThread(AGENT), (void *)(id+1));
	AGENTHandle[2] = osThreadCreate(osThread(AGENT), (void *)(id+2));
	
  /* definition and creation of PUSHER */
	osThreadDef(PUSHER, pusher, osPriorityNormal, 3, 128);
  PUSHERHandle[0] = osThreadCreate(osThread(PUSHER), (void *)(id));
	PUSHERHandle[1] = osThreadCreate(osThread(PUSHER), (void *)(id+1));
	PUSHERHandle[2] = osThreadCreate(osThread(PUSHER), (void *)(id+2));		

  /* definition and creation of SMOKER */
	uint32_t buff[6] = {0,1,2,3,4,5};
	osThreadDef(SMOKER, smoker, osPriorityNormal, 6, 128);  
  SMOKERHandle[0] = osThreadCreate(osThread(SMOKER), (void *)(buff));
	SMOKERHandle[1] = osThreadCreate(osThread(SMOKER), (void *)(buff+1));
	SMOKERHandle[2] = osThreadCreate(osThread(SMOKER), (void *)(buff+2));
	SMOKERHandle[3] = osThreadCreate(osThread(SMOKER), (void *)(buff+3));
	SMOKERHandle[4] = osThreadCreate(osThread(SMOKER), (void *)(buff+4));
	SMOKERHandle[5] = osThreadCreate(osThread(SMOKER), (void *)(buff+5));

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Red_Led_GPIO_Port, Red_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Red_Led_Pin */
  GPIO_InitStruct.Pin = Red_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Red_Led_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(GPIOD, Red_Led_Pin);
    osDelay(1000);
  }
  /* USER CODE END 5 */ 
}

/* agent function */
void agent(void const * argument)
{
  /* USER CODE BEGIN agent */
  /* Infinite loop */
	
	uint32_t agent_id = *((uint8_t *)argument);
	while(1)//a++ < 3)
	{
		if(xSemaphoreTake(agent_ready, portMAX_DELAY) == pdTRUE)
		{
			xSemaphoreGive(pusher_sem[agent_id]);
			xSemaphoreGive(pusher_sem[(agent_id+1)%3]);
			printf("Agent%u puts %s on the table for smoking\n", agent_id, smoker_types[(agent_id+2)%3]);
		}
	}
	//osThreadTerminate(AGENTHandle[agent_id]);
	//	vTaskDelete(xHandleAgent[agent_id]);
  /* USER CODE END agent */
}

/* pusher function */
void pusher(void const * argument)
{
  /* USER CODE BEGIN pusher */
  /* Infinite loop */
	uint32_t pusher_id = *((uint8_t *)argument);
	while(1)
	{
		if(xSemaphoreTake(pusher_sem[pusher_id], portMAX_DELAY) == pdTRUE)
		{
			if(xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
			{
				if(items_on_table[(pusher_id+1)%3])
				{
					items_on_table[(pusher_id+1)%3] = items_on_table[(pusher_id+1)%3] - 1;
					xSemaphoreGive(smoker_sem[(pusher_id+2)%3]);
				}
				else if(items_on_table[(pusher_id+2)%3])
				{
					items_on_table[(pusher_id+2)%3] = items_on_table[(pusher_id+2)%3] - 1;
					xSemaphoreGive(smoker_sem[(pusher_id+1)%3]);
				}
				else
				{
					items_on_table[pusher_id] = items_on_table[pusher_id] + 1;
				}
				xSemaphoreGive(mutex);
			}
		}
	}
  /* USER CODE END pusher */
}

/* smoker function */
void smoker(void const * argument)
{
  /* USER CODE BEGIN smoker */
  /* Infinite loop */
	uint32_t smoker_id = *((uint8_t *)argument);
	uint32_t type_id = smoker_id%3;

	while(1)
	{	
		osDelay(1000);
		printf("Smoker%u is waiting for %s for making the cigarette\n",smoker_id,smoker_types[type_id]);
		if(xSemaphoreTake(smoker_sem[type_id], portMAX_DELAY) == pdTRUE)
		{
			printf("Smoker%u is making the cigarette\n",smoker_id);
			//osDelay(1000);	
			xSemaphoreGive(agent_ready);
			printf("Smoker%u is smoking the cigarette\n",smoker_id);
			osDelay(1000);
			
		}
	}
  /* USER CODE END smoker */
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
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(GPIOD, Red_Led_Pin, GPIO_PIN_SET);
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

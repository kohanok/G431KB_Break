/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Break_TaskHandle;
osThreadId RainBow_TaskHandle;
osThreadId AS504X_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartBreak_Task(void const * argument);
void StartRainBow_Task(void const * argument);
void StartAS504X_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Break_Task */
  osThreadDef(Break_Task, StartBreak_Task, osPriorityNormal, 0, 128);
  Break_TaskHandle = osThreadCreate(osThread(Break_Task), NULL);

  /* definition and creation of RainBow_Task */
  osThreadDef(RainBow_Task, StartRainBow_Task, osPriorityIdle, 0, 128);
  RainBow_TaskHandle = osThreadCreate(osThread(RainBow_Task), NULL);

  /* definition and creation of AS504X_Task */
  osThreadDef(AS504X_Task, StartAS504X_Task, osPriorityIdle, 0, 128);
  AS504X_TaskHandle = osThreadCreate(osThread(AS504X_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartBreak_Task */
/**
  * @brief  Function implementing the Break_Task thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartBreak_Task */
void StartBreak_Task(void const * argument)
{
  /* USER CODE BEGIN StartBreak_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartBreak_Task */
}

/* USER CODE BEGIN Header_StartRainBow_Task */
/**
* @brief Function implementing the RainBow_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRainBow_Task */
void StartRainBow_Task(void const * argument)
{
  /* USER CODE BEGIN StartRainBow_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRainBow_Task */
}

/* USER CODE BEGIN Header_StartAS504X_Task */
/**
* @brief Function implementing the AS504X_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAS504X_Task */
void StartAS504X_Task(void const * argument)
{
  /* USER CODE BEGIN StartAS504X_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartAS504X_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

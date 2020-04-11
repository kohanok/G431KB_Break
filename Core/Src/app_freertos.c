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
#include "ws2812.h"
#include "as504x.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//extern #define B_LED_CNT 20
//extern R_LED_CNT 128

//extern #define INT_BITS 	20


_Bool g_break_flg = 0;
uint32_t g_speed = 0;

extern uint16_t zero_position;
extern float zero_position_map;
uint16_t old_angle;
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

  uint32_t pre_time = 0;
  uint32_t led_time = 200;
	uint32_t firstled = 0;
	uint32_t halfled = 0;
	uint32_t led_index = 0;
	uint32_t led_mask= 0b1110000111;


	_Bool led_flg=0;
	uint32_t led_count = 0;
  for(;;)
  {

		if(millis()-pre_time >= led_time)
		{
			pre_time = millis();
			if(g_break_flg == 0) {
				/*
				if(g_speed < 1000) {
					led_time = 200;
				}
				else {
					led_time = 50;
				}*/
				led_time = g_speed/10;
				if(led_time < 50)
					led_time = 50;
				led_count = 0;
				firstled = rotateLeft(led_mask, led_index%B_LED_CNT);
				halfled = rotateRight(led_mask, led_index%B_LED_CNT);

				led_index++;
				for(uint32_t j=0; j<12; j++) {
					if( firstled >> j & 0x01){
							ws2812SetColor(j, 255, 255, 255);
					}else {
						ws2812SetColor(j, 0, 0, 0);
					}
				}
				for(uint32_t j=20; j>11; j--) {
					if( halfled >> (j-11) & 0x01){
							ws2812SetColor(j, 255, 255, 255);
					}else {
						ws2812SetColor(j, 0, 0, 0);
					}
				}
			}
			else{ // break
				led_count++;
				if(led_count > 30) {
					led_time = 500;
				}else {
					led_time = 50;
				}
				if(led_count > 40) {
						//led_count = 0;
						g_break_flg = 0;
				}

				if(led_flg) {
					for(int i = 0; i < B_LED_CNT; i++) {
						ws2812SetColor(i, 255, 0, 0);
						led_flg = 0;
					}
				}else {
					for(int i = 0; i < B_LED_CNT; i++) {
						ws2812SetColor(i, 0, 0, 0);
						led_flg = 1;
					}
				}
			}
		}
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

	uint32_t rainbow_pre_time=0;
	uint32_t rainbow_led_time=10;
	_Bool led_flg=0;
	uint32_t led_count = 0;
	for(;;)
	{
		uint16_t i, j;
		for(j=0; j<256*5;) { // 5 cycles of all colors on wheel
			if(millis()-rainbow_pre_time >= rainbow_led_time)
			{
				rainbow_pre_time = millis();
				j++;
				rainbow_led_time = 10;
				if(g_break_flg == 0) {
					led_count = 0;
					for(i=0; i< R_LED_CNT; i++) {
						setPixelColor(i, Wheel(((i * 256 / R_LED_CNT) + j) & 255));
					}
				}
				else{ // break
					led_count++;
					if(led_count > 20) {
						rainbow_led_time = 500;
					}else {
						rainbow_led_time = 50;
					}
					if(led_count > 30) {
							//led_count = 0;
							g_break_flg = 0;
					}

					if(led_flg) {
						for(int i = 0; i < B_LED_CNT; i++) {
							setPixelColor(i, 0xff0000);
							led_flg = 0;
						}
					}else {
						for(int i = 0; i < B_LED_CNT; i++) {
							setPixelColor(i, 0);
							led_flg = 1;
						}
					}
				}
			}
		}
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

  uint32_t Task03_pre_time = 0;
  uint32_t Task03_led_time = 1000;
  //uint32_t speed = 0;

  for(;;)
  {

		if (millis()-Task03_pre_time >= Task03_led_time)
		{
			Task03_pre_time = millis();
		  uint16_t current_angle = as504x_getRawRotation();
		  float current_angle_map = as504x_read2angle(current_angle);

		  float angle = current_angle_map - zero_position_map;
		  angle = as504x_normalize(angle);
		  if( (angle - old_angle) > 10)
		  	g_break_flg = 1;
		  //else
		  	//g_break_flg = 0;
			old_angle = angle;
		}

/*
		if(angle>100) {
			g_speed = 0;
		}
		if(g_break_flg == 1) {
			//g_break_flg = 0;
		}
		else {
			g_break_flg = 0;
		}
	  //printf("Current Angle: %d\nCurrent Angle Map: %f\nAngle: %f\n\n", current_angle, current_angle_map, angle);

	  if (as504x_error()) {
		  //printf("ERROR: %d\n", as504x_getErrors());
	  }

*/
    osDelay(1);
  }

  /* USER CODE END StartAS504X_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

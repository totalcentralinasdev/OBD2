/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fdcan.h"
#include "i2c.h"
#include "TCA9535.h"
#include "FreeRTOS.h"
#include "queue.h"
//#include "iwdg.h"
#include "spi.h"

extern FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[] = { 0x10, 0x32, 0x54, 0x76, 0x98, 0x00, 0x11, 0x22, 0x33, 0x44,
		0x55, 0x66, 0x77, 0x88, 0x99, 0x00 };
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[16];
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile uint8_t g_CurrentTaskName;
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for VideoTask */
osThreadId_t VideoTaskHandle;
const osThreadAttr_t VideoTask_attributes = {
  .name = "VideoTask",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TouchGFXTask */
osThreadId_t TouchGFXTaskHandle;
const osThreadAttr_t TouchGFXTask_attributes = {
  .name = "TouchGFXTask",
  .stack_size = 8192 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Aquisition_task */
osThreadId_t Aquisition_taskHandle;
const osThreadAttr_t Aquisition_task_attributes = {
  .name = "Aquisition_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
float hallcurrent_read_current(uint16_t current_value);
uint8_t hallcurrent_check_status(uint16_t value);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void videoTaskFunc(void *argument);
extern void TouchGFX_Task(void *argument);
void StartAquisition(void *argument);

extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void) {
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	 to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
	 task. It is essential that code added to this hook function never attempts
	 to block in any way (for example, call xQueueReceive() with a block time
	 specified, or call vTaskDelay()). If the application makes use of the
	 vTaskDelete() API function (as this demo application does) then it is also
	 important that vApplicationIdleHook() is permitted to return to its calling
	 function, because it is the responsibility of the idle task to clean up
	 memory allocated by the kernel to any task that has since been deleted. */

	g_CurrentTaskName = 0;
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	 called if a stack overflow is detected. */
}
/* USER CODE END 4 */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of VideoTask */
  VideoTaskHandle = osThreadNew(videoTaskFunc, NULL, &VideoTask_attributes);

  /* creation of TouchGFXTask */
  TouchGFXTaskHandle = osThreadNew(TouchGFX_Task, NULL, &TouchGFXTask_attributes);

  /* creation of Aquisition_task */
  Aquisition_taskHandle = osThreadNew(StartAquisition, NULL, &Aquisition_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN StartDefaultTask */

	extern QueueHandle_t pin1;
	extern QueueHandle_t pin3;
	extern QueueHandle_t pin6;
	extern QueueHandle_t pin7;
	extern QueueHandle_t pin8;
	extern QueueHandle_t pin9;
	extern QueueHandle_t pin11;
	extern QueueHandle_t pin12;
	extern QueueHandle_t pin13;
	extern QueueHandle_t pin14;
	extern QueueHandle_t pin15;
	extern QueueHandle_t pin16;

	uint8_t pin1_state = 0;
	uint8_t pin3_state = 0;
	uint8_t pin6_state = 0;
	uint8_t pin7_state = 0;
	uint8_t pin8_state = 0;
	uint8_t pin9_state = 0;
	uint8_t pin11_state = 0;
	uint8_t pin12_state = 0;
	uint8_t pin13_state = 0;
	uint8_t pin14_state = 0;
	uint8_t pin15_state = 0;
	uint8_t pin16_state = 0;

	xQueueOverwrite(pin1, &pin1_state);
	xQueueOverwrite(pin3, &pin3_state);
	xQueueOverwrite(pin6, &pin6_state);
	xQueueOverwrite(pin7, &pin7_state);
	xQueueOverwrite(pin8, &pin8_state);
	xQueueOverwrite(pin9, &pin9_state);
	xQueueOverwrite(pin11, &pin11_state);
	xQueueOverwrite(pin12, &pin12_state);
	xQueueOverwrite(pin13, &pin13_state);
	xQueueOverwrite(pin14, &pin14_state);
	xQueueOverwrite(pin15, &pin15_state);
	xQueueOverwrite(pin16, &pin16_state);

	uint8_t config0 = 0x00; // 0 out - 1 IN
	uint8_t config1 = 0x00;

	uint8_t output_port0 = 0x00;
	uint8_t output_port1 = 0x00;

	TCA9535_Init(&hi2c4, config0, config1, output_port0, output_port1);

	for (;;) {
		g_CurrentTaskName = 1;
		//HAL_IWDG_Refresh(&hiwdg1);
		if (xQueuePeek(pin1, &pin1_state, 0) == pdPASS) {
			TCA9535_WritePin(&hi2c4, TCA9535_PORT0, 0, pin1_state);
		}
		if (xQueuePeek(pin3, &pin3_state, 0) == pdPASS) {
			TCA9535_WritePin(&hi2c4, TCA9535_PORT0, 2, pin3_state);
		}
		if (xQueuePeek(pin6, &pin6_state, 0) == pdPASS) {
			TCA9535_WritePin(&hi2c4, TCA9535_PORT0, 5, pin6_state);
		}
		if (xQueuePeek(pin7, &pin7_state, 0) == pdPASS) {
			TCA9535_WritePin(&hi2c4, TCA9535_PORT0, 6, pin7_state);
		}
		if (xQueuePeek(pin8, &pin8_state, 0) == pdPASS) {
			TCA9535_WritePin(&hi2c4, TCA9535_PORT0, 7, pin8_state);
		}

		if (xQueuePeek(pin9, &pin9_state, 0) == pdPASS) {
			TCA9535_WritePin(&hi2c4, TCA9535_PORT1, 0, pin9_state);
		}
		if (xQueuePeek(pin11, &pin11_state, 0) == pdPASS) {
			TCA9535_WritePin(&hi2c4, TCA9535_PORT1, 2, pin11_state);
		}
		if (xQueuePeek(pin12, &pin12_state, 0) == pdPASS) {
			TCA9535_WritePin(&hi2c4, TCA9535_PORT1, 3, pin12_state);
		}
		if (xQueuePeek(pin13, &pin13_state, 0) == pdPASS) {
			TCA9535_WritePin(&hi2c4, TCA9535_PORT1, 4, pin13_state);
		}
		if (xQueuePeek(pin14, &pin14_state, 0) == pdPASS) {
			TCA9535_WritePin(&hi2c4, TCA9535_PORT1, 5, pin14_state);
		}
		if (xQueuePeek(pin15, &pin15_state, 0) == pdPASS) {
			TCA9535_WritePin(&hi2c4, TCA9535_PORT1, 6, pin15_state);
		}
		if (xQueuePeek(pin16, &pin16_state, 0) == pdPASS) {
			TCA9535_WritePin(&hi2c4, TCA9535_PORT1, 7, pin16_state);
		}

		vTaskDelay(pdMS_TO_TICKS(100));

	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartAquisition */
/**
 * @brief Function implementing the Aquisition_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartAquisition */
void StartAquisition(void *argument)
{
  /* USER CODE BEGIN StartAquisition */
	extern ADC_HandleTypeDef hadc1;
	extern QueueHandle_t Sensor_Queue;

	struct sensor {
		float voltage;
		float current;
	};

	struct sensor sensor_values;
	int adc_digital = 0;

	uint8_t voltage_index = 0, current_index = 0;

	uint8_t N_samples = 30;

	int digital_voltage_array[N_samples];
	float current_array[N_samples];
	float result = 0;
	;
	/* Infinite loop */
	for (;;) {
		g_CurrentTaskName = 2;
		HAL_ADC_Start(&hadc1);

		while (!__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)) {
			//osDelay(1);
		}

		adc_digital = HAL_ADC_GetValue(&hadc1);

		uint16_t SPI_RX;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_SPI_Receive(&hspi2, (uint16_t*) &SPI_RX, 1, 5);
		while (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY)
			;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

		result = hallcurrent_read_current(SPI_RX);
		digital_voltage_array[voltage_index] = adc_digital;
		if (result != -80) {
			current_array[current_index] = result;
			current_index++;
		}

		voltage_index++;
		if (voltage_index > N_samples) {
			voltage_index = 0;
		}
		if (current_index > N_samples) {
			current_index = 0;
		}
		sensor_values.voltage = 0;
		sensor_values.current = 0;
		for (int i = 0; i < N_samples; i++) {
			sensor_values.voltage += digital_voltage_array[i];
			sensor_values.current += current_array[i];
		}

		sensor_values.voltage = (sensor_values.voltage * 3.3 / 65536)
				/ N_samples;
		sensor_values.current = (sensor_values.current) / N_samples;

		xQueueOverwrite(Sensor_Queue, &sensor_values);

		vTaskDelay(pdMS_TO_TICKS(5));
	}
  /* USER CODE END StartAquisition */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

uint8_t hallcurrent_check_status(uint16_t value) {
	if ((value >> 15) == 0) {
		return 0;
	} else {
		return 1;
	}
}

float hallcurrent_read_current(uint16_t current_value) {
	float result = 0;
	//uint16_t current_value = 0;

	if (!hallcurrent_check_status(current_value)) {
		current_value &= 0x1FFF;

		//current_value -= 4096;
		current_value -= 4095;

		result = (float) current_value;
		result *= 0.0125;
	} else {
		return -80;
	}

	if (result > 50) {
		result = 0.0;
	}
	return result;
}

/* USER CODE END Application */


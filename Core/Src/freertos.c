  /* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include "xv7001bb.h"

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

volatile xv7001bb_task_debug_t g_xv7001bb_task_debug;

/* USER CODE END Variables */
osThreadId task_mainHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static uint8_t xv7001bb_run_interface_checks(void);

/* USER CODE END FunctionPrototypes */

void task_main(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of task_main */
  osThreadDef(task_main, task_main, osPriorityNormal, 0, 384);
  task_mainHandle = osThreadCreate(osThread(task_main), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_task_main */
/**
  * @brief  Function implementing the task_main thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_task_main */
void task_main(void const * argument)
{
  /* USER CODE BEGIN task_main */
  uint8_t status_register = 0U;
  uint8_t status_power_state;
  uint8_t status_value_ok;
  xv_status_t status;

  (void)argument;

  /* The sensor requires a 200 ms power-up interval before the first access. */
  vTaskDelay(pdMS_TO_TICKS(XV7001BB_POWER_UP_WAIT_MS));

  g_xv7001bb_task_debug.init_status = (uint32_t)XV7001bb_Init();
  if (g_xv7001bb_task_debug.init_status == (uint32_t)XV_OK)
  {
    status = XV7001bb_ReadStatus(&status_register);
    g_xv7001bb_task_debug.read_status_status = (uint32_t)status;
    if (status == XV_OK)
    {
      g_xv7001bb_task_debug.status_raw = status_register;
    }

    status_power_state =
        status_register & XV7001BB_STATUS_STATE_MASK;
    status_value_ok = (uint8_t)(
        (status == XV_OK) &&
        ((status_register & XV7001BB_STATUS_PROCESS_OK) != 0U) &&
        ((status_power_state == XV7001BB_STATUS_POR) ||
         (status_power_state == XV7001BB_STATUS_STANDBY) ||
         (status_power_state == XV7001BB_STATUS_SLEEP) ||
         (status_power_state == XV7001BB_STATUS_SLEEP_OUT)));

    g_xv7001bb_task_debug.interface_check_complete =
        xv7001bb_run_interface_checks();
    g_xv7001bb_task_debug.interface_check_complete &=
        status_value_ok;
  }

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1000U));
  }
  /* USER CODE END task_main */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static uint8_t xv7001bb_run_interface_checks(void)
{
  uint8_t interface_ctl = 0U;
  uint8_t output_ctl = 0U;
  uint8_t filter_ctl = 0U;
  float temperature_c = 0.0f;
  float angle_deg = 0.0f;
  uint32_t all_ok = 1U;
  xv_status_t status;

  status = XV7001bb_WriteData(XV7001BB_REG_DSP_CTL2, 0x01U);
  g_xv7001bb_task_debug.write_test_status = (uint32_t)status;
  all_ok &= (uint32_t)(status == XV_OK);

  status = XV7001bb_ReadReg(XV7001BB_REG_INTERFACE_CTL,
                            &interface_ctl);
  g_xv7001bb_task_debug.read_reg_control_status = (uint32_t)status;
  g_xv7001bb_task_debug.interface_ctl = interface_ctl;
  all_ok &= (uint32_t)((status == XV_OK) && (interface_ctl == 0x00U));

  status = XV7001bb_ReadReg(XV7001BB_REG_OUTPUT_CTL, &output_ctl);
  g_xv7001bb_task_debug.read_reg_output_status = (uint32_t)status;
  g_xv7001bb_task_debug.output_ctl = output_ctl;
  all_ok &= (uint32_t)((status == XV_OK) && (output_ctl == 0x01U));

  status = XV7001bb_ReadReg(XV7001BB_REG_DSP_CTL2, &filter_ctl);
  g_xv7001bb_task_debug.read_reg_filter_status = (uint32_t)status;
  g_xv7001bb_task_debug.filter_ctl = filter_ctl;
  all_ok &= (uint32_t)((status == XV_OK) && (filter_ctl == 0x01U));

  status = XV7001bb_ReadTmp(&temperature_c);
  g_xv7001bb_task_debug.read_tmp_status = (uint32_t)status;
  g_xv7001bb_task_debug.temperature_c = temperature_c;
  all_ok &= (uint32_t)((status == XV_OK) &&
                       (temperature_c >= -40.0f) &&
                       (temperature_c <= 125.0f));

  status = XV7001bb_ReadAngle(&angle_deg);
  g_xv7001bb_task_debug.read_angle_status = (uint32_t)status;
  g_xv7001bb_task_debug.angle_deg = angle_deg;
  all_ok &= (uint32_t)((status == XV_OK) &&
                       (angle_deg >= -180.0f) &&
                       (angle_deg < 180.0f));

  return (uint8_t)(all_ok != 0U);
}

/* USER CODE END Application */


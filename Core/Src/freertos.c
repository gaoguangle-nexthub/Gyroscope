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

#define TASK_MAIN_SAMPLE_PERIOD_MS       10U
#define TASK_MAIN_SLOW_PERIOD_CYCLES     50U
#define TASK_MAIN_RETRY_CYCLES           50U

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

static void xv7001bb_run_interface_checks(void);

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
  TickType_t previous_tick;
  TickType_t last_wake_tick;
  TickType_t current_tick;
  uint32_t period_ticks;
  uint32_t slow_counter = 0U;
  uint8_t status_register;
  float temperature_c;
  float angle_deg;
  xv_status_t status;

  (void)argument;

  /*
   * The sensor and this task start together at board power-up. Keep the
   * required stabilization interval outside the driver so the SPI device
   * layer has no RTOS dependency.
   */
  vTaskDelay(pdMS_TO_TICKS(XV7001BB_POWER_UP_WAIT_MS));

  g_xv7001bb_task_debug.init_status = (uint32_t)XV7001bb_Init();
  if (g_xv7001bb_task_debug.init_status == (uint32_t)XV_OK)
  {
    xv7001bb_run_interface_checks();
  }

  previous_tick = xTaskGetTickCount();
  last_wake_tick = previous_tick;

  /* Infinite loop */
  for(;;)
  {
    vTaskDelayUntil(&last_wake_tick,
                    pdMS_TO_TICKS(TASK_MAIN_SAMPLE_PERIOD_MS));

    current_tick = xTaskGetTickCount();
    period_ticks = (uint32_t)(current_tick - previous_tick);
    previous_tick = current_tick;
    g_xv7001bb_task_debug.period_ticks = period_ticks;
    if (period_ticks > g_xv7001bb_task_debug.period_ticks_max)
    {
      g_xv7001bb_task_debug.period_ticks_max = period_ticks;
    }
    g_xv7001bb_task_debug.loop_count++;

    if (g_xv7001bb_task_debug.init_status != (uint32_t)XV_OK)
    {
      if ((g_xv7001bb_task_debug.loop_count %
           TASK_MAIN_RETRY_CYCLES) == 0U)
      {
        g_xv7001bb_task_debug.init_status =
            (uint32_t)XV7001bb_Init();
        if (g_xv7001bb_task_debug.init_status == (uint32_t)XV_OK)
        {
          xv7001bb_run_interface_checks();
        }
      }
      continue;
    }

    status = XV7001bb_ReadAngle(&angle_deg);
    g_xv7001bb_task_debug.read_angle_status = (uint32_t)status;
    if (status == XV_OK)
    {
      g_xv7001bb_task_debug.angle_deg = angle_deg;
      g_xv7001bb_task_debug.raw_rate = g_xv7001bb_debug.raw_rate;
    }
    else
    {
      g_xv7001bb_task_debug.error_count++;
    }

    slow_counter++;
    if (slow_counter >= TASK_MAIN_SLOW_PERIOD_CYCLES)
    {
      slow_counter = 0U;

      status = XV7001bb_ReadStatus(&status_register);
      g_xv7001bb_task_debug.read_status_status = (uint32_t)status;
      if (status == XV_OK)
      {
        g_xv7001bb_task_debug.status_raw = status_register;
      }

      status = XV7001bb_ReadTmp(&temperature_c);
      g_xv7001bb_task_debug.read_tmp_status = (uint32_t)status;
      if (status == XV_OK)
      {
        g_xv7001bb_task_debug.temperature_c = temperature_c;
      }
    }
  }
  /* USER CODE END task_main */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static void xv7001bb_run_interface_checks(void)
{
  uint8_t value;
  int32_t raw_rate;
  float temperature_c;
  float rate_dps;
  float angle_deg;
  uint32_t all_ok = 1U;

  /*
   * Re-write the same non-destructive LPF value so WriteData is exercised
   * explicitly from task_main, then verify every public read interface.
   */
  g_xv7001bb_task_debug.write_test_status =
      (uint32_t)XV7001bb_WriteData(XV7001BB_REG_DSP_CTL2, 0x01U);

  g_xv7001bb_task_debug.read_reg_control_status =
      (uint32_t)XV7001bb_ReadReg(XV7001BB_REG_INTERFACE_CTL, &value);
  if (g_xv7001bb_task_debug.read_reg_control_status == (uint32_t)XV_OK)
  {
    g_xv7001bb_task_debug.interface_ctl = value;
  }

  g_xv7001bb_task_debug.read_reg_output_status =
      (uint32_t)XV7001bb_ReadReg(XV7001BB_REG_OUTPUT_CTL, &value);
  if (g_xv7001bb_task_debug.read_reg_output_status == (uint32_t)XV_OK)
  {
    g_xv7001bb_task_debug.output_ctl = value;
  }

  g_xv7001bb_task_debug.read_reg_filter_status =
      (uint32_t)XV7001bb_ReadReg(XV7001BB_REG_DSP_CTL2, &value);
  if (g_xv7001bb_task_debug.read_reg_filter_status == (uint32_t)XV_OK)
  {
    g_xv7001bb_task_debug.filter_ctl = value;
  }

  g_xv7001bb_task_debug.read_status_status =
      (uint32_t)XV7001bb_ReadStatus(&value);
  if (g_xv7001bb_task_debug.read_status_status == (uint32_t)XV_OK)
  {
    g_xv7001bb_task_debug.status_raw = value;
  }

  g_xv7001bb_task_debug.read_tmp_status =
      (uint32_t)XV7001bb_ReadTmp(&temperature_c);
  if (g_xv7001bb_task_debug.read_tmp_status == (uint32_t)XV_OK)
  {
    g_xv7001bb_task_debug.temperature_c = temperature_c;
  }

  g_xv7001bb_task_debug.read_raw_rate_status =
      (uint32_t)XV7001bb_ReadRawRate(&raw_rate);
  if (g_xv7001bb_task_debug.read_raw_rate_status == (uint32_t)XV_OK)
  {
    g_xv7001bb_task_debug.raw_rate = raw_rate;
  }

  g_xv7001bb_task_debug.read_rate_status =
      (uint32_t)XV7001bb_ReadRate(&rate_dps);
  if (g_xv7001bb_task_debug.read_rate_status == (uint32_t)XV_OK)
  {
    g_xv7001bb_task_debug.rate_dps = rate_dps;
  }

  g_xv7001bb_task_debug.read_angle_status =
      (uint32_t)XV7001bb_ReadAngle(&angle_deg);
  if (g_xv7001bb_task_debug.read_angle_status == (uint32_t)XV_OK)
  {
    g_xv7001bb_task_debug.angle_deg = angle_deg;
  }

  all_ok &= (uint32_t)(g_xv7001bb_task_debug.write_test_status ==
                       (uint32_t)XV_OK);
  all_ok &= (uint32_t)(g_xv7001bb_task_debug.read_reg_control_status ==
                       (uint32_t)XV_OK);
  all_ok &= (uint32_t)(g_xv7001bb_task_debug.read_reg_output_status ==
                       (uint32_t)XV_OK);
  all_ok &= (uint32_t)(g_xv7001bb_task_debug.read_reg_filter_status ==
                       (uint32_t)XV_OK);
  all_ok &= (uint32_t)(g_xv7001bb_task_debug.read_status_status ==
                       (uint32_t)XV_OK);
  all_ok &= (uint32_t)(g_xv7001bb_task_debug.read_tmp_status ==
                       (uint32_t)XV_OK);
  all_ok &= (uint32_t)(g_xv7001bb_task_debug.read_raw_rate_status ==
                       (uint32_t)XV_OK);
  all_ok &= (uint32_t)(g_xv7001bb_task_debug.read_rate_status ==
                       (uint32_t)XV_OK);
  all_ok &= (uint32_t)(g_xv7001bb_task_debug.read_angle_status ==
                       (uint32_t)XV_OK);

  g_xv7001bb_task_debug.interface_check_complete = all_ok;
}

/* USER CODE END Application */


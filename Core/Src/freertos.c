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

#include "can.h"
#include "xv7001bb.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ANGLE_UPDATE_PERIOD_MS       10U
#define TEMPERATURE_UPDATE_PERIOD_MS 100U
#define CAN_INIT_RETRY_PERIOD_MS     100U
#define CAN_TX_PERIOD_MS             20U
#define CAN_RX_POLL_PERIOD_MS        10U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osThreadId task_can_txHandle;
osThreadId task_can_rxHandle;

/*
 * task_main 写入、task_can_tx 读取的遥测快照。
 * 写入和读取都在短临界区内完成，避免上位机读到跨周期混合字段。
 */
static int32_t s_telemetry_angle_millideg = 0;
static int16_t s_telemetry_temperature_centi_c = 0;
static uint8_t s_telemetry_flags = 0U;
static int32_t s_telemetry_rate_millidps = 0;
static uint8_t s_telemetry_rate_flags = 0U;

/* USER CODE END Variables */
osThreadId task_mainHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void xv7001bb_run_angle_loop(void);
void task_can_tx(void const * argument);
void task_can_rx(void const * argument);

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
  /* 发送任务使用独立周期，接收任务周期排空 RX FIFO0。 */
  osThreadDef(task_can_tx, task_can_tx, osPriorityBelowNormal, 0, 256);
  task_can_txHandle = osThreadCreate(osThread(task_can_tx), NULL);

  osThreadDef(task_can_rx, task_can_rx, osPriorityNormal, 0, 256);
  task_can_rxHandle = osThreadCreate(osThread(task_can_rx), NULL);

  if ((task_can_txHandle == NULL) || (task_can_rxHandle == NULL))
  {
    Error_Handler();
  }
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
  xv_status_t status;

  (void)argument;

  /* The sensor requires a 200 ms power-up interval before the first access. */
  vTaskDelay(pdMS_TO_TICKS(XV7001BB_POWER_UP_WAIT_MS));

  status = XV7001bb_Init();
  if (status == XV_OK)
  {
    vTaskDelay(pdMS_TO_TICKS(XV7001BB_BIAS_SETTLE_MS));
    (void)XV7001bb_StartBiasCalibration();
  }

  xv7001bb_run_angle_loop();
  /* USER CODE END task_main */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
 * @brief CAN 发送任务：初始化 CAN1 并以 50 Hz 发送陀螺仪遥测帧。
 * @param argument 未使用。
 * @retval None
 */
void task_can_tx(void const * argument)
{
  TickType_t next_wake;
  uint8_t sequence = 0U;

  (void)argument;

  /* 初始化只做一次；若 CAN 尚未就绪，仅阻塞本任务并周期重试。 */
  while (CAN_AppInit() != HAL_OK)
  {
    vTaskDelay(pdMS_TO_TICKS(CAN_INIT_RETRY_PERIOD_MS));
  }

  next_wake = xTaskGetTickCount();
  for (;;)
  {
    int32_t angle_millideg;
    int32_t rate_millidps;
    int16_t temperature_centi_c;
    uint8_t flags;
    uint8_t rate_flags;
    uint8_t tx_retry_count = 0U;
    HAL_StatusTypeDef angle_tx_status;
    HAL_StatusTypeDef rate_tx_status;

    taskENTER_CRITICAL();
    angle_millideg = s_telemetry_angle_millideg;
    rate_millidps = s_telemetry_rate_millidps;
    temperature_centi_c = s_telemetry_temperature_centi_c;
    flags = s_telemetry_flags;
    rate_flags = s_telemetry_rate_flags;
    taskEXIT_CRITICAL();

    do
    {
      angle_tx_status = CAN_SendGyroTelemetry(angle_millideg,
                                              temperature_centi_c,
                                              flags,
                                              sequence);
      if (angle_tx_status != HAL_BUSY)
      {
        break;
      }

      tx_retry_count++;
      if (tx_retry_count < 3U)
      {
        vTaskDelay(pdMS_TO_TICKS(1U));
      }
    } while (tx_retry_count < 3U);

    rate_tx_status = CAN_SendGyroRateTelemetry(rate_millidps,
                                               rate_flags,
                                               sequence);
    if ((rate_tx_status == HAL_BUSY) && (angle_tx_status == HAL_OK))
    {
      vTaskDelay(pdMS_TO_TICKS(1U));
      rate_tx_status = CAN_SendGyroRateTelemetry(rate_millidps,
                                                 rate_flags,
                                                 sequence);
    }

    if (angle_tx_status == HAL_OK)
    {
      sequence++;
    }

    vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(CAN_TX_PERIOD_MS));
  }
}

/**
 * @brief CAN 接收任务：周期取出 RX FIFO0 中的报文并更新调试快照。
 * @param argument 未使用。
 * @retval None
 */
void task_can_rx(void const * argument)
{
  TickType_t next_wake = xTaskGetTickCount();

  (void)argument;

  for (;;)
  {
    CAN_ReceiveTask();
    vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(CAN_RX_POLL_PERIOD_MS));
  }
}

/* 姿态角计算的稳态主循环，负责每隔 10 ms 完成一次“采样角速度 → 零偏修正 → 时间积分 → 更新角度”。 */
static void xv7001bb_run_angle_loop(void)
{
  TickType_t next_wake = xTaskGetTickCount();
  TickType_t next_temperature_wake = next_wake;
  float angle_deg = 0.0f;//保存传感器计算出来的角度，单位是度
  float temperature_c = 0.0f;
  uint8_t temperature_valid = 0U;

  for (;;)
  {
    xv_status_t angle_status;

    if (CAN_ConsumeGyroZeroRequest())
    {
      if (XV7001bb_StartBiasCalibration() == XV_OK)
      {
        angle_deg = 0.0f;
      }
    }

    angle_status = XV7001bb_ReadAngle(&angle_deg);

    if (xTaskGetTickCount() >= next_temperature_wake)
    {
      temperature_valid =
          (uint8_t)(XV7001bb_ReadTmp(&temperature_c) == XV_OK);
      next_temperature_wake +=
          pdMS_TO_TICKS(TEMPERATURE_UPDATE_PERIOD_MS);
    }

    {
      int32_t angle_millideg;
      int32_t rate_millidps = 0;
      int16_t temperature_centi_c;
      uint8_t flags =
          (uint8_t)(CAN_GYRO_PROTOCOL_VERSION << 5);
      uint8_t rate_flags =
          (uint8_t)(CAN_GYRO_PROTOCOL_VERSION << 5);

      if (angle_deg >= 0.0f)
      {
        angle_millideg = (int32_t)((angle_deg * 1000.0f) + 0.5f);
      }
      else
      {
        angle_millideg = (int32_t)((angle_deg * 1000.0f) - 0.5f);
      }

      if (temperature_c >= 0.0f)
      {
        temperature_centi_c = (int16_t)((temperature_c * 100.0f) + 0.5f);
      }
      else
      {
        temperature_centi_c = (int16_t)((temperature_c * 100.0f) - 0.5f);
      }

      if (angle_status == XV_OK)
      {
        flags |= CAN_GYRO_FLAG_ANGLE_VALID;
        if (g_xv7001bb_debug.rate_dps >= 0.0f)
        {
          rate_millidps =
              (int32_t)((g_xv7001bb_debug.rate_dps * 1000.0f) + 0.5f);
        }
        else
        {
          rate_millidps =
              (int32_t)((g_xv7001bb_debug.rate_dps * 1000.0f) - 0.5f);
        }
        rate_flags |= CAN_GYRO_RATE_FLAG_VALID;
        if ((g_xv7001bb_debug.corrected_raw_rate >=
             XV7001BB_RATE_RAW_SATURATION) ||
            (g_xv7001bb_debug.corrected_raw_rate <=
             -XV7001BB_RATE_RAW_SATURATION))
        {
          rate_flags |= CAN_GYRO_RATE_FLAG_SATURATED;
        }
      }
      if (temperature_valid != 0U)
      {
        flags |= CAN_GYRO_FLAG_TEMP_VALID;
      }
      if (g_xv7001bb_debug.bias_valid != 0U)
      {
        flags |= CAN_GYRO_FLAG_BIAS_READY;
      }
      if (g_xv7001bb_debug.bias_state == XV7001BB_BIAS_STATE_FAILED)
      {
        flags |= CAN_GYRO_FLAG_BIAS_FAILED;
      }
      if (g_xv7001bb_debug.angle_time_valid != 0U)
      {
        flags |= CAN_GYRO_FLAG_ANGLE_TIME_VALID;
      }

      taskENTER_CRITICAL();
      s_telemetry_angle_millideg = angle_millideg;
      s_telemetry_rate_millidps = rate_millidps;
      s_telemetry_temperature_centi_c = temperature_centi_c;
      s_telemetry_flags = flags;
      s_telemetry_rate_flags = rate_flags;
      taskEXIT_CRITICAL();
    }

    vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(ANGLE_UPDATE_PERIOD_MS));
  }
}

/* USER CODE END Application */


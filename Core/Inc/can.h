/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <stdint.h>

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */

/* 课程 CAN 测试帧参数。 */
#define CAN_TEST_STANDARD_ID   0x123U
#define CAN_TEST_DATA          0xAAU

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

typedef struct
{
  uint32_t id;
  bool extended;
  bool remote;
  uint8_t dlc;
  uint8_t data[8];
} CAN_Frame_t;

typedef struct
{
  uint32_t init_status;
  uint32_t can_state;
  uint32_t last_send_status;
  uint32_t send_attempts;
  uint32_t send_ok_count;
  uint32_t send_error_count;
  uint32_t free_mailboxes;
  uint32_t last_hal_error;
  uint32_t master_control_register;
  uint32_t master_status_register;
  uint32_t bit_timing_register;
  uint32_t tx_status_register;
  uint32_t error_status_register;
  uint32_t gpio_control_register;
  uint8_t can_rx_pin_level;
} CAN_DebugInfo_t;

extern volatile CAN_DebugInfo_t g_can_debug;

/* 接收调试快照：在 CAN_ReceiveTask() 断点处可直接加入 Watch。 */
extern volatile uint32_t g_can_rx_count;
extern volatile uint32_t g_can_rx_read_error_count;
extern volatile uint32_t g_can_last_rx_id;
extern volatile uint8_t g_can_last_rx_dlc;
extern volatile uint8_t g_can_last_rx_data;
extern volatile uint8_t g_can_last_rx_payload[8];
extern volatile uint8_t g_can_last_rx_extended;
extern volatile uint8_t g_can_last_rx_remote;
extern volatile uint32_t g_can_last_rx_tick;

/**
 * @brief 配置全接收过滤器并启动 CAN1。
 * @retval HAL_OK 表示 CAN1 已进入正常收发状态。
 */
HAL_StatusTypeDef CAN_AppInit(void);

/**
 * @brief 发送一个标准 CAN 数据帧。
 * @param standard_id 11 位标准标识符。
 * @param data 数据缓冲区，长度为 0 时可为 NULL。
 * @param length 数据长度，范围 0～8。
 * @retval HAL_OK 表示报文已进入硬件发送邮箱。
 */
HAL_StatusTypeDef CAN_SendStandard(uint16_t standard_id,
                                   const uint8_t *data,
                                   uint8_t length);

/**
 * @brief 使用课程测试 ID 发送一个字节。
 */
HAL_StatusTypeDef CAN_SendByte(uint8_t value);

/**
 * @brief 从 RX FIFO0 读取一帧，不等待。
 * @retval true 表示已读取一帧，false 表示当前没有待处理报文。
 */
bool CAN_PollReceive(CAN_Frame_t *frame);

/**
 * @brief 裸机周期接收任务：取出 RX FIFO0 中当前所有报文。
 *
 * 该函数由主循环周期调用。断点应设置在本函数内 CAN_PollReceive()
 * 返回 true 后的计数或快照赋值行。
 */
void CAN_ReceiveTask(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */


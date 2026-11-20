/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */

static bool can_started = false;
volatile CAN_DebugInfo_t g_can_debug;

volatile uint32_t g_can_rx_count = 0U;
volatile uint32_t g_can_rx_read_error_count = 0U;
volatile uint32_t g_can_last_rx_id = 0U;
volatile uint8_t g_can_last_rx_dlc = 0U;
volatile uint8_t g_can_last_rx_data = 0U;
volatile uint8_t g_can_last_rx_payload[8] = {0U};
volatile uint8_t g_can_last_rx_extended = 0U;
volatile uint8_t g_can_last_rx_remote = 0U;
volatile uint32_t g_can_last_rx_tick = 0U;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

HAL_StatusTypeDef CAN_AppInit(void)
{
  CAN_FilterTypeDef filter = {0};
  GPIO_InitTypeDef rx_pin = {0};
  HAL_StatusTypeDef status;

  if (can_started)
  {
    g_can_debug.init_status = (uint32_t)HAL_OK;
    return HAL_OK;
  }

  /*
   * U3 正常工作时会主动驱动 CANRX。这里增加弱上拉，便于 U3 未上电或
   * RXD 悬空时仍把 CAN 总线识别为隐性状态；正常工作时收发器会覆盖它。
   */
  rx_pin.Pin = GPIO_PIN_11;
  rx_pin.Mode = GPIO_MODE_INPUT;
  rx_pin.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &rx_pin);
  g_can_debug.can_rx_pin_level =
      (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET) ? 1U : 0U;

  if (HAL_CAN_GetState(&hcan) != HAL_CAN_STATE_READY)
  {
    g_can_debug.init_status = (uint32_t)HAL_ERROR;
    return HAL_ERROR;
  }

  /* 首轮联调接收所有标准帧和扩展帧到 FIFO0。 */
  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0U;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0U;
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

  status = HAL_CAN_ConfigFilter(&hcan, &filter);
  if (status != HAL_OK)
  {
    g_can_debug.init_status = (uint32_t)status;
    g_can_debug.last_hal_error = HAL_CAN_GetError(&hcan);
    return status;
  }

  status = HAL_CAN_Start(&hcan);
  if (status != HAL_OK)
  {
    g_can_debug.init_status = (uint32_t)status;
    g_can_debug.last_hal_error = HAL_CAN_GetError(&hcan);
    return status;
  }

  g_can_debug.init_status = (uint32_t)HAL_OK;
  g_can_debug.can_state = (uint32_t)HAL_CAN_GetState(&hcan);
  g_can_debug.last_hal_error = HAL_CAN_GetError(&hcan);
  g_can_debug.master_control_register = CAN1->MCR;
  g_can_debug.master_status_register = CAN1->MSR;
  g_can_debug.bit_timing_register = CAN1->BTR;
  g_can_debug.gpio_control_register = GPIOA->CRH;
  can_started = true;
  return HAL_OK;
}

HAL_StatusTypeDef CAN_SendStandard(uint16_t standard_id,
                                   const uint8_t *data,
                                   uint8_t length)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  HAL_CAN_StateTypeDef can_state;
  uint8_t tx_data[8] = {0U};
  uint32_t tx_mailbox;
  HAL_StatusTypeDef status;

  g_can_debug.send_attempts++;
  can_state = HAL_CAN_GetState(&hcan);
  g_can_debug.can_state = (uint32_t)can_state;

  if ((standard_id > 0x7FFU) ||
      (length > 8U) ||
      ((data == NULL) && (length != 0U)) ||
      (!can_started) ||
      ((can_state != HAL_CAN_STATE_READY) &&
       (can_state != HAL_CAN_STATE_LISTENING)))
  {
    g_can_debug.last_send_status = (uint32_t)HAL_ERROR;
    g_can_debug.send_error_count++;
    g_can_debug.last_hal_error = HAL_CAN_GetError(&hcan);
    return HAL_ERROR;
  }

  g_can_debug.free_mailboxes = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
  if (g_can_debug.free_mailboxes == 0U)
  {
    g_can_debug.last_send_status = (uint32_t)HAL_BUSY;
    g_can_debug.send_error_count++;
    return HAL_BUSY;
  }

  /*
   * STM32F1 HAL 固定拼接 8 个数据字节，即使 DLC 小于 8 也会读取
   * aData[0..7]。因此不能把单字节局部变量的地址直接交给 HAL。
   */
  if (length != 0U)
  {
    for (uint8_t index = 0U; index < length; index++)
    {
      tx_data[index] = data[index];
    }
  }

  tx_header.StdId = standard_id;
  tx_header.ExtId = 0U;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = length;
  tx_header.TransmitGlobalTime = DISABLE;

  status = HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox);
  g_can_debug.last_send_status = (uint32_t)status;
  g_can_debug.last_hal_error = HAL_CAN_GetError(&hcan);
  g_can_debug.master_control_register = CAN1->MCR;
  g_can_debug.master_status_register = CAN1->MSR;
  g_can_debug.bit_timing_register = CAN1->BTR;
  g_can_debug.tx_status_register = CAN1->TSR;
  g_can_debug.error_status_register = CAN1->ESR;
  g_can_debug.gpio_control_register = GPIOA->CRH;
  g_can_debug.can_rx_pin_level =
      (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET) ? 1U : 0U;

  if (status == HAL_OK)
  {
    g_can_debug.send_ok_count++;
  }
  else
  {
    g_can_debug.send_error_count++;
  }

  return status;
}

bool CAN_PollReceive(CAN_Frame_t *frame)
{
  CAN_RxHeaderTypeDef rx_header = {0};

  if ((frame == NULL) ||
      (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0U))
  {
    return false;
  }

  if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rx_header, frame->data) != HAL_OK)
  {
    g_can_rx_read_error_count++;
    g_can_debug.last_hal_error = HAL_CAN_GetError(&hcan);
    return false;
  }

  frame->extended = (rx_header.IDE == CAN_ID_EXT);
  frame->remote = (rx_header.RTR == CAN_RTR_REMOTE);
  frame->id = frame->extended ? rx_header.ExtId : rx_header.StdId;
  frame->dlc = (uint8_t)rx_header.DLC;
  return true;
}

void CAN_ReceiveTask(void)
{
  CAN_Frame_t frame;

  /*
   * 只有 CAN_PollReceive() 成功取出一帧后，程序才会进入本循环。
   * 联调时在 g_can_last_rx_data 赋值后的 __NOP() 行设置断点，
   * 此时本帧 ID、DLC 和 8 字节数据快照均已更新。
   */
  while (CAN_PollReceive(&frame))
  {
    g_can_rx_count++;
    g_can_last_rx_id = frame.id;
    g_can_last_rx_dlc = frame.dlc;
    g_can_last_rx_extended = frame.extended ? 1U : 0U;
    g_can_last_rx_remote = frame.remote ? 1U : 0U;
    g_can_last_rx_tick = HAL_GetTick();

    for (uint8_t index = 0U; index < 8U; index++)
    {
      if ((index < frame.dlc) && (!frame.remote))
      {
        g_can_last_rx_payload[index] = frame.data[index];
      }
      else
      {
        g_can_last_rx_payload[index] = 0U;
      }
    }

    g_can_last_rx_data = g_can_last_rx_payload[0];
  }
}

/* USER CODE END 1 */

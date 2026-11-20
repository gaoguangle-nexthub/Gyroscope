/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */

/* 原理图中 PB12 连接 XV7001BB 的 SS，低电平选中从机。 */
#define SPI_CS_LOW()   HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET)

/* 一次事务结束后释放片选，避免下一条命令被并入当前帧。 */
#define SPI_CS_HIGH()  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET)

/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
 * @brief 在一个完整 SPI 事务内发送并接收指定长度的数据。
 *
 * SPI2 使用软件 NSS，HAL 不会自动控制 PB12。函数无论成功或失败，
 * 都会在返回前恢复 PB12 为高电平，防止片选被永久拉低。
 */
HAL_StatusTypeDef SPI_TransmitReceive(const uint8_t *tx_data,
                                      uint8_t *rx_data,
                                      uint16_t length,
                                      uint32_t timeout_ms)
{
  HAL_StatusTypeDef status;

  /* 零长度请求不产生时钟，也不应产生无意义的片选脉冲。 */
  if (length == 0U)
  {
    return HAL_OK;
  }

  /* 传输前确认发送和接收缓冲区都有效，避免 HAL 访问空指针。 */
  if ((tx_data == NULL) || (rx_data == NULL))
  {
    return HAL_ERROR;
  }

  /* 拉低 PB12，建立本帧的片选有效区间。 */
  SPI_CS_LOW();

  /* 全双工传输同时驱动 MOSI 并采样 MISO。 */
  status = HAL_SPI_TransmitReceive(&hspi2, tx_data, rx_data, length, timeout_ms);

  /* 无论传输结果如何，都必须释放片选并结束本帧。 */
  SPI_CS_HIGH();

  return status;
}

/**
 * @brief 发送单字节数据。
 *
 * 主机产生时钟时必须同时接收数据，因此使用一个临时接收字节并丢弃。
 */
HAL_StatusTypeDef SPI_SendByte(uint8_t tx_data)
{
  uint8_t tx_buffer = tx_data;
  uint8_t rx_buffer = 0U;

  return SPI_TransmitReceive(&tx_buffer,
                             &rx_buffer,
                             1U,
                             SPI_TRANSFER_TIMEOUT_MS);
}

/**
 * @brief 从 SPI2 读取单字节数据，发送 0x00 作为时钟占位。
 */
HAL_StatusTypeDef SPI_ReadByte(uint8_t *rx_data)
{
  uint8_t tx_buffer = 0x00U;

  return SPI_TransmitReceive(&tx_buffer,
                             rx_data,
                             1U,
                             SPI_TRANSFER_TIMEOUT_MS);
}

/* USER CODE END 1 */

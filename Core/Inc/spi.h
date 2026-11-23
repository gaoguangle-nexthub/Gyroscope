/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN Private defines */

/*
 * 4 字节事务在 1.125 Mbit/s 下只需约 29 us。2 ms 已提供足够余量，
 * 同时保证 SPI 故障不会独占整个 10 ms 角度更新周期。
 */
#define SPI_TRANSFER_TIMEOUT_MS   2U

/* USER CODE END Private defines */

void MX_SPI2_Init(void);

/* USER CODE BEGIN Prototypes */

/**
 * @brief 在一次受控的 PB12 片选事务中完成 SPI2 全双工传输。
 * @param tx_data 发送缓冲区首地址。
 * @param rx_data 接收缓冲区首地址。
 * @param length 传输字节数。
 * @param timeout_ms HAL 等待超时时间，单位为毫秒。
 * @retval HAL_OK 表示成功，其他值表示参数错误、忙、超时或硬件错误。
 */
HAL_StatusTypeDef SPI_TransmitReceive(const uint8_t *tx_data,
                                      uint8_t *rx_data,
                                      uint16_t length,
                                      uint32_t timeout_ms);

/**
 * @brief 发送一个字节并丢弃全双工接收到的数据。
 * @param tx_data 待发送的 8 位数据。
 * @retval HAL_OK 表示发送完成，其他值表示 SPI2 传输失败。
 */
HAL_StatusTypeDef SPI_SendByte(uint8_t tx_data);

/**
 * @brief 读取一个字节，同时发送 0x00 占位数据以产生时钟。
 * @param rx_data 接收结果写入位置。
 * @retval HAL_OK 表示读取完成，其他值表示参数或传输错误。
 */
HAL_StatusTypeDef SPI_ReadByte(uint8_t *rx_data);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */


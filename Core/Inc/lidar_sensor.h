#ifndef __LIDAR_SENSOR_H
#define __LIDAR_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define DMA_BUF_SIZE 128

/* 測定値が何ms途絶えたらセンサー異常とみなすか (TSD20は200Hz=5ms周期) */
#define LIDAR_TIMEOUT_MS 100

/* lidar_nearest_mm() が「2 台とも測れていない」ときに返す値 */
#define LIDAR_DIST_NONE 0xFFFF

/* ペリフェラルハンドル (main.c で定義) */
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;

/* lidarセンサーの受信バッファ・距離値 (main.c で定義) */
extern uint8_t rx_dma_buf4[DMA_BUF_SIZE];
extern uint16_t distance4;

extern uint8_t rx_dma_buf7[DMA_BUF_SIZE];
extern uint16_t distance7;

/* 関数プロトタイプ */
void lidar(void);
int lidar_timeout(void);
uint16_t lidar_nearest_mm(void); /* 途絶えていない Lidar のうち近い方の距離 (mm) */
void lidar_start_rx(UART_HandleTypeDef *huart); /* 循環 DMA 受信を始める (エラー後のやり直しにも使う) */

#ifdef __cplusplus
}
#endif

#endif /* __LIDAR_SENSOR_H */

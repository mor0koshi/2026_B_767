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

/*
 * Lidarの取り付け位置オフセット(mm)。センサーの実測値から引いて機体基準に直す。
 * auto_mode に渡す引数は手動モードと全自動モードで必ず揃えること。
 * 揃っていないと prev_error が別条件の値で初期化され、モード切替時にD項が跳ねる。
 * (半自動モードは目標距離に現在距離を渡すので、半自動→全自動の切替では D 項が
 *  1 周期跳ねる。足回りは 1 周期 80 ずつしか変化しないので実害は小さい)
 */
#define LIDAR_OFFSET4 11
#define LIDAR_OFFSET7 38

/* 全自動モードで壁から保つ目標距離(mm) */
#define AUTO_TARGET_DIST_MM 1770

/* ペリフェラルハンドル (main.c で定義) */
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

#ifdef __cplusplus
}
#endif

#endif /* __LIDAR_SENSOR_H */

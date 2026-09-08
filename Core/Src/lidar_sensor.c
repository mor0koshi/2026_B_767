/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : lidar_sensor.c
 * @brief          : lidarセンサー(UART4/UART7)の距離データ解析処理
 ******************************************************************************
 */
/* USER CODE END Header */
#include "lidar_sensor.h"

/*
 * TSD20 (PONO) の出力フレーム : 4バイト固定
 *
 *   [0] 0x5C     フレームヘッダ
 *   [1] 距離LSB  リトルエンディアン
 *   [2] 距離MSB
 *   [3] チェックサム = ~(距離LSB + 距離MSB)  ※下位8bit
 *
 * 測定範囲外のときセンサーは 50000 を返すので、20000 超は捨てれば弾ける。
 * 通信は 460800bps / 8N1（huart4・huart7 の設定と一致）。
 */
#define LIDAR_FRAME_HEADER  0x5C
#define LIDAR_FRAME_LEN     4
#define LIDAR_DIST_MAX_MM   20000

/*
 * 最後に有効なフレームを受信した時刻。
 * distance4/7 は更新が止まっても前の値(初期値は0)が残り続けるため、
 * 「値が古いかどうか」はこの時刻でしか判断できない。
 * 起動直後は 0 なので、最初のフレームが届くまでタイムアウト扱いになる。
 */
static uint32_t last_lidar_rx4 = 0;
static uint32_t last_lidar_rx7 = 0;

/*
 * DMAリングバッファから距離フレームを取り出す。
 *
 * ・ヘッダを見つけても4バイト揃っていなければ last_index を進めずに中断し、
 *   次回の呼び出しへ持ち越す（未受信＝前周回の古いバイトを読まないため）
 * ・チェックサムが合ったフレームだけ採用し、4バイトまとめて消費する
 * ・合わなければ 0x5C はデータバイトだったとみなし、1バイト進めて再同期する
 */
static void parse_lidar_frames(const uint8_t *buf, uint16_t *last_index,
                               uint16_t current_index, uint16_t *distance_out,
                               uint32_t *last_rx_out)
{
    while (*last_index != current_index) {
        if (buf[*last_index] != LIDAR_FRAME_HEADER) {
            *last_index = (*last_index + 1) % DMA_BUF_SIZE;
            continue;
        }

        // ヘッダ位置から current_index までに何バイト受信済みか
        uint16_t avail = (uint16_t)((current_index - *last_index + DMA_BUF_SIZE) % DMA_BUF_SIZE);
        if (avail < LIDAR_FRAME_LEN) {
            break; // フレームがまだ揃っていない。次回に持ち越す
        }

        uint8_t lsb   = buf[(*last_index + 1) % DMA_BUF_SIZE];
        uint8_t msb   = buf[(*last_index + 2) % DMA_BUF_SIZE];
        uint8_t check = buf[(*last_index + 3) % DMA_BUF_SIZE];

        if ((uint8_t)(~(uint8_t)(lsb + msb)) == check) {
            uint16_t temp_dist = ((uint16_t)msb << 8) | lsb;
            if (temp_dist > 0 && temp_dist <= LIDAR_DIST_MAX_MM) {
                *distance_out = temp_dist;
                *last_rx_out  = HAL_GetTick(); // 有効な測定値が採れたときだけ更新する
            }
            *last_index = (*last_index + LIDAR_FRAME_LEN) % DMA_BUF_SIZE;
        } else {
            *last_index = (*last_index + 1) % DMA_BUF_SIZE;
        }
    }
}

void lidar(void){
    static uint16_t last_index4 = 0;
    static uint16_t last_index7 = 0;

    // UART4 (センサー1)
    uint16_t current_index4 = DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
    parse_lidar_frames(rx_dma_buf4, &last_index4, current_index4, &distance4, &last_lidar_rx4);

    // UART7 (センサー2)
    uint16_t current_index7 = DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart7_rx);
    parse_lidar_frames(rx_dma_buf7, &last_index7, current_index7, &distance7, &last_lidar_rx7);
}

/*
 * どちらか一方でも測定値が途絶えていれば 1 を返す。
 * TSD20 は 200Hz(5ms周期)なので、100ms は 20フレーム分の猶予にあたる。
 */
int lidar_timeout(void){
    uint32_t t = HAL_GetTick();
    return ((t - last_lidar_rx4) > LIDAR_TIMEOUT_MS) ||
           ((t - last_lidar_rx7) > LIDAR_TIMEOUT_MS);
}

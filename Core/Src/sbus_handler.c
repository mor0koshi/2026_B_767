/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : sbus_handler.c
 * @brief          : SBUS受信・値の加工処理
 ******************************************************************************
 */
/* USER CODE END Header */
#include "sbus_handler.h"
#include <string.h>

// Arduino の map() と同じ。入力が範囲外のときは端に丸める
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    if (x < in_min)
        x = in_min;
    if (x > in_max)
        x = in_max;

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* ===================== */
/* 初期化                */
/* ===================== */
void SBUS_Init(void) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rxbuf, SBUS_FRAME_LEN);
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

/* ===================== */
/* 受信イベント          */
/* ===================== */
/*
 * UART5 がアイドルになる (1 フレーム受信し終える) たびに呼ばれる。
 * 受信した中からヘッダ 0x0F を探して 25 バイトを 1 フレームとしてデコードし、
 * DMA を再開する (DMA は NORMAL モードなので毎回開始し直す必要がある)。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == UART5) {
        for (int i = 0; i < Size - 24; i++) {
            if (sbus_rxbuf[i] == 0x0F) {
                memcpy(sbus_frame, &sbus_rxbuf[i], 25);
                SBUS_Process();
                break;
            }
        }
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rxbuf, SBUS_FRAME_LEN);
}
/* ===================== */
/* SBUSデコード          */
/* ===================== */
// sbus_frame (25 バイト) を 11bit × 16ch にデコードする。ヘッダの確認は呼び出し元で済んでいる
void SBUS_Process(void) {
    SBUS_CH[0] = (sbus_frame[1] | sbus_frame[2] << 8) & 0x07FF;
    SBUS_CH[1] = (sbus_frame[2] >> 3 | sbus_frame[3] << 5) & 0x07FF;
    SBUS_CH[2] = (sbus_frame[3] >> 6 | sbus_frame[4] << 2 | sbus_frame[5] << 10) & 0x07FF;
    SBUS_CH[3] = (sbus_frame[5] >> 1 | sbus_frame[6] << 7) & 0x07FF;
    SBUS_CH[4] = (sbus_frame[6] >> 4 | sbus_frame[7] << 4) & 0x07FF;
    SBUS_CH[5] = (sbus_frame[7] >> 7 | sbus_frame[8] << 1 | sbus_frame[9] << 9) & 0x07FF;
    SBUS_CH[6] = (sbus_frame[9] >> 2 | sbus_frame[10] << 6) & 0x07FF;
    SBUS_CH[7] = (sbus_frame[10] >> 5 | sbus_frame[11] << 3) & 0x07FF;
    SBUS_CH[8] = (sbus_frame[12] | sbus_frame[13] << 8) & 0x07FF;
    SBUS_CH[9] = (sbus_frame[13] >> 3 | sbus_frame[14] << 5) & 0x07FF;
    SBUS_CH[10] = (sbus_frame[14] >> 6 | sbus_frame[15] << 2 | sbus_frame[16] << 10) & 0x07FF;
    SBUS_CH[11] = (sbus_frame[16] >> 1 | sbus_frame[17] << 7) & 0x07FF;
    SBUS_CH[12] = (sbus_frame[17] >> 4 | sbus_frame[18] << 4) & 0x07FF;
    SBUS_CH[13] = (sbus_frame[18] >> 7 | sbus_frame[19] << 1 | sbus_frame[20] << 9) & 0x07FF;
    SBUS_CH[14] = (sbus_frame[20] >> 2 | sbus_frame[21] << 6) & 0x07FF;
    SBUS_CH[15] = (sbus_frame[21] >> 5 | sbus_frame[22] << 3) & 0x07FF;

    SBUS_LostFrame = (sbus_frame[23] >> 2) & 0x01;
    SBUS_Failsafe = (sbus_frame[23] >> 3) & 0x01;

    // 受信が完全に途絶えたことを safety() が検出できるようにする。
    // SBUS_CH や SBUS_LostFrame はフレームが来たときしか更新されないので、
    // 信号が消えると古い値のまま固まり、それ単体では断線を判定できない。
    last_sbus_rx = HAL_GetTick();
}

// 3 段スイッチ: 上 (>1100) = -1 / 中 = 0 / 下 (<300) = 1
int get_switch_state3(int ch_value) {
    if (ch_value > 1100) {
      return -1;
    }
    if (ch_value < 300){
      return 1;
    }
    return 0;
}

// 2 段スイッチ: >1100 = 1 / それ以外 = 0
int get_switch_state2(int ch_value) {
    if (ch_value > 1100) {
      return 1;
    }
    return 0;
}

/*
 * スティック値 (368〜1680) を ±1000 に変換する。
 * 中立付近 (1000〜1050) は 1024 に丸め、変換後 ±2 以内は 0 にする (デッドバンド)。
 */
int process_stick(int ch_value) {
    if (ch_value > 1000 && ch_value < 1050){
      ch_value = 1024;
    }
    int mapped = map(ch_value, 368, 1680, -1000, 1000);
    if (mapped <= 2 && mapped >= -2){
      return 0; // デッドバンド
    }
    return mapped;
}

// スイッチとスティックを読み、手動モードのオムニ混合値 m1〜m4 を計算する
void sbus(void){
    Lmayu1 = get_switch_state2(SBUS_CH[4]);
    Lmayu2 = get_switch_state2(SBUS_CH[5]);
    Rmayu1 = get_switch_state3(SBUS_CH[6]);
    Rmayu2 = get_switch_state2(SBUS_CH[7]);
    Ltuno1 = get_switch_state3(SBUS_CH[8]);
    Rtuno2 = get_switch_state2(SBUS_CH[9]);

    rx = process_stick(SBUS_CH[0]);
    ly = process_stick(SBUS_CH[1]);
    ry = process_stick(SBUS_CH[2]);
    lx = process_stick(SBUS_CH[3]);

    // 式を変えるときは main.c の半自動 (gauto_m*) と全自動 (auto_m*) も必ず揃えること
    m1 =  -ly + lx - rx; // 左前 (Front Left)
    m2 = -ly - lx - rx; // 右前 (Front Right)
    m3 = ly - lx - rx; // 左後 (Rear Left)
    m4 =  ly + lx - rx; // 右後 (Rear Right)

    // 0.9 倍して、1 軸を倒しきったときにちょうど maxpwm (900) になるようにする
    m1 = (m1 * 9) / 10;
    m2 = (m2 * 9) / 10;
    m3 = (m3 * 9) / 10;
    m4 = (m4 * 9) / 10;
    }

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

// map関数
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
void SBUS_Process(void) {
    //@brief  The application entry point.
    // if(sbus_frame[0] != 0x0F)
    //    return;

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
}

// SBUS用のヘルパー関数
int get_switch_state3(int ch_value) {
    if (ch_value > 1100) {
      return -1;
    }
    if (ch_value < 300){
      return 1;
    }
    return 0;
}

int get_switch_state2(int ch_value) {
    if (ch_value > 1100) {
      return 1;
    }
    return 0;
}

int process_stick(int ch_value) {
    if (ch_value > 1000 && ch_value < 1050){
      ch_value = 1024;
    }
    int mapped = map(ch_value, 368, 1680, -255, 255);
    if (mapped <= 2 && mapped >= -2){
      return 0; // デッドバンド
    }
    return mapped;
}

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

    m1 =  ly - lx - rx; // 左前 (Front Left)
    m2 = -ly - lx - rx; // 右前 (Front Right)
    m3 = -ly + lx - rx; // 左後 (Rear Left)
    m4 =  ly + lx - rx; // 右後 (Rear Right)

    // モーターの値を0.7倍して減速させる
    m1 = (m1 * 7) / 10;
    m2 = (m2 * 7) / 10;
    m3 = (m3 * 7) / 10;
    m4 = (m4 * 7) / 10;
    }

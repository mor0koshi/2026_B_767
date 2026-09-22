/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : can_handler.c
 * @brief          : CAN の送受信
 ******************************************************************************
 */
/* USER CODE END Header */
#include "can_handler.h"

// CAN 送信 (固定ペイロード)。現在はどこからも呼ばれていない
void CAN_TX(uint32_t recipient) {
    // 送信用インスタンス等
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8];
    // 送信メールボックスに空きがあったら送信開始
    if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)) {
        // 送信用インスタンスの設定
        TxHeader.StdId = recipient; // 受取手のCANのID
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = 8; // データ長を8byteに設定
        TxHeader.TransmitGlobalTime = DISABLE;
        // 各データ
        TxData[0] = 1;
        TxData[1] = 0;
        TxData[2] = 0;
        TxData[3] = 0;
        TxData[4] = 0;
        TxData[5] = 0;
        TxData[6] = 0;
        TxData[7] = 0;
        // CANメッセージを送信
        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
            Error_Handler();
        }
    }
}
/*
 * CAN 受信割り込み。ID 0x001 の 8 バイトをそのまま use_data[] に写す。
 * use_data[0..3] はメインループで PV1〜PV4 (ローラーのエンコーダ値) になる。
 * last_can_rx は safety() の CAN 断判定に使う。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
    CAN_RxHeaderTypeDef RxHeader; // 受信メッセージの情報が格納されるインスタンス
    uint8_t RxData[8];            // 受信したデータを一時保存する配列
    uint32_t id;                  // CANメッセージIDを格納する変数
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        id = RxHeader.StdId;                    // RxHeaderの中に入っているidを取り出す
        if (id == 0x001 && RxHeader.DLC >= 8) { // idが0x001でデータ長が8以上の場合
            last_can_rx = HAL_GetTick();        // 受信時刻を更新
            for (int i = 0; i <= 7; i++) {
                use_data[i] = RxData[i];
            }
        }
    }
}

#ifndef __CAN_HANDLER_H
#define __CAN_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ペリフェラルハンドル (main.c で定義) */
extern CAN_HandleTypeDef hcan1;

/* 受信データ (main.c で定義)。受信割り込み HAL_CAN_RxFifo0MsgPendingCallback で更新する */
extern volatile uint8_t use_data[8]; /* ID 0x001 の最新 8 バイト。[0..3] がローラーのエンコーダ値 */
extern uint32_t last_can_rx;         /* 最後に受信した時刻。safety() の CAN 断判定に使う */

/* 関数プロトタイプ */
void CAN_TX(uint32_t recipient);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_HANDLER_H */

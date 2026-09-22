#ifndef __SBUS_HANDLER_H
#define __SBUS_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "sbus.h"
#include <stdint.h>

/*
 * 最後にSBUSフレームをデコードできた時刻から何ms経ったら受信断とみなすか。
 * SBUSは14ms(ハイスピードなら7ms)周期なので、100msは約7フレーム分の猶予。
 */
#define SBUS_TIMEOUT_MS 100

/* ペリフェラルハンドル (main.c で定義) */
extern UART_HandleTypeDef huart5;

/* SBUS受信バッファ・チャンネル値 (main.c で定義) */
extern uint8_t sbus_rxbuf[SBUS_FRAME_LEN];
extern uint8_t sbus_frame[SBUS_FRAME_LEN];
extern volatile uint16_t SBUS_CH[16];
extern uint8_t SBUS_Failsafe;
extern uint8_t SBUS_LostFrame;
extern uint32_t last_sbus_rx;

/* スティック・スイッチ加工後の値 (main.c で定義) */
extern int rx;
extern int ly;
extern int ry;
extern int lx;

extern int Lmayu1; 
extern int Lmayu2; 
extern int Rmayu1;
extern int Rmayu2; 
extern int Ltuno1;
extern int Rtuno2;


/* 関数プロトタイプ */
long map(long x, long in_min, long in_max, long out_min, long out_max);

void SBUS_Init(void);
void SBUS_Process(void);

int get_switch_state(int ch_value);
int process_stick(int ch_value);
void sbus(void);

#ifdef __cplusplus
}
#endif

#endif /* __SBUS_HANDLER_H */

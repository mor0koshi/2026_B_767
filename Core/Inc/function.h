#ifndef __FUNCTION_H
#define __FUNCTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "sbus_handler.h" /* safety() で SBUS の受信状態 (SBUS_CH, Failsafe, LostFrame, last_sbus_rx) を見るため */
#include <stdint.h>

/* ペリフェラルハンドル (main.c で定義) */
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* 共有変数 (main.c で定義) */
/* ローラーのエンコーダ値 (CAN の use_data[0..3]) */
extern volatile int16_t PV1;
extern volatile int16_t PV2;
extern volatile int16_t PV3;
extern volatile int16_t PV4;

extern int pwm1;
extern int pwm2;
extern int pwm3;
extern int pwm4;
extern int pwm5;
extern int pwm6;
extern int pwm7;
extern int pwm8;
extern int pwm9;
extern int pwm10;
extern int pwm11;
extern int pwm12;
/* rem1〜rem4, rem9〜rem12 は main.c に実体が無い (未使用) */
extern int rem1;
extern int rem2;
extern int rem3;
extern int rem4;
extern int rem5;
extern int rem6;
extern int rem7;
extern int rem8;
extern int rem9;
extern int rem10;
extern int rem11;
extern int rem12;


extern int timer_flag; /* main.c に実体が無い (未使用) */
extern int reset_flag1;
extern int reset_flag2;

extern int roller_dir1;
extern int roller_dir2;
extern int dummy;

extern int auto_ly;
extern int auto_rx;

extern uint32_t now;
extern uint32_t last_can_rx;

extern volatile uint8_t use_data[8];

/* 関数プロトタイプ */
int _write(int file, char *ptr, int len);

void CAN_TX(uint32_t recipient);

void motor_control(int SV, int PV, int maxMV, int down_pwm, int max_pwm, int *pwmm, int *dirr,int *remm);
void motor_simple_control(int SV, int step, int max_pwm, int *pwmm, int *dirr);

void roller(void);
void auto_mode(int distance1, int distance2, int reset_flag, int target_dist);
void safety(void);
/*
 * リミットスイッチ入力のノイズ除去用。
 * モーターのPWMノイズで一瞬 Low を読んだだけでフラグが立つのを防ぐ。
 * port/pin だけ初期化し、残り (stable/last/changed) は LIMIT_SW_INIT で埋めること。
 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t stable;   // ノイズ除去後の確定値
    uint8_t last;     // 前回の生の読み値
    uint32_t changed; // 生の読み値が変わった時刻
} limit_sw;

// プルアップ入力なので未押下 (High=1) を初期値にする
#define LIMIT_SW_INIT(port, pin) {(port), (pin), 1, 1, 0}

uint8_t limit_read(limit_sw *sw);

#ifdef __cplusplus
}
#endif

#endif /* __FUNCTION_H */

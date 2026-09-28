#ifndef __FUNCTION_H
#define __FUNCTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "sbus_handler.h" /* スイッチとスティックの値 (Lmayu1, ly など) を使うため */
#include <stdint.h>

/* ペリフェラルハンドル (main.c で定義) */
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
/* ローラーの motor_control 用。誤差を 1/10 するときの端数の繰り越し */
extern int rem5;
extern int rem6;
extern int rem7;
extern int rem8;

extern int reset_flag1;
extern int reset_flag2;

extern int dir1;
extern int dir2;
extern int dir3;
extern int dir4;
extern int maxpwm;

extern int souten_dir1;
extern int souten_dir2;
extern int dummy;

extern int auto_ly;
extern int auto_rx;

/* 共有変数 (function.c で定義) */
extern int roller_ready; /* ローラーが目標速度に達していれば 1 (roller() が更新) */

extern uint32_t now;

/* 関数プロトタイプ */
int _write(int file, char *ptr, int len);

/*
 * メインループから呼ぶ順番
 *   asimawari() → roller() → souten_ramp() → souten() → safety() (safety.h) → motor_outputs()
 * CAN は can_handler.h、モーター 1 個分の制御は motor_control.h、上限値などの定数は robot_limits.h
 */
void asimawari(void);       /* 足回り (20ms 周期) */
void roller(void);          /* ローラー (20ms 周期) */
void souten_ramp(void);     /* 装填モーターの pwm をランプで目標へ近づける (20ms 周期) */
void souten(void);          /* 電磁弁と装填の指令 (毎周回) */
void valves_off(void);      /* 電磁弁をすぐ閉じ、撃つスイッチが一度 OFF になるまで開かない。safety() が呼ぶ */
void motor_outputs(void);   /* PWM と DIR と電磁弁の出力。safety() の後に呼ぶ */
void outputs_all_off(void); /* 全 PWM と電磁弁を即 0。HardFault / Error_Handler から呼ぶ */

void auto_mode(int distance1, int distance2, int reset_flag, int target_dist);
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

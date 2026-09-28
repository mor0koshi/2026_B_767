#ifndef __FUNCTION_H
#define __FUNCTION_H

/*
 * 制御プログラム全体で共有する変数と関数の宣言。
 * 関数の中身は機構ごとのファイルに分けてある:
 *   asimawari.c … 足回りと Lidar PID          roller.c … ローラー
 *   hassya.c    … 発射・電磁弁・装填           limit_sw.c … リミットスイッチの読み取り
 *   output.c    … ピンへの出力と非常時の全停止
 */

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

/* 共有変数 (機構ごとのファイルで定義) */
extern int roller_ready;     /* ローラーが目標速度に達していれば 1 (roller.c の roller() が更新) */

extern uint32_t now;

/* 関数プロトタイプ */
int _write(int file, char *ptr, int len); /* printf の出力先 (main.c) */

/*
 * メインループから呼ぶ順番
 *   asimawari() → roller() → souten_ramp() → hassya() → souten() → denziben()
 *   → safety() (safety.h) → led() (led.h) → motor_outputs()
 * CAN は can_handler.h、モーター 1 個分の制御は motor_control.h、上限値などの定数は robot_limits.h
 */
/* asimawari.c */
void asimawari(void);       /* 足回り (20ms 周期) */
void auto_mode(int distance1, int distance2, int reset_flag, int target_dist); /* Lidar PID */

/* roller.c */
void roller(void);          /* ローラー (20ms 周期) */

/* hassya.c */
void hassya(void);          /* 撃ってよいか (撃つスイッチの押し直し) を決める (毎周回) */
void hassya_off(void);      /* 撃つのをすぐ全部止め、撃つスイッチを一度離すまで撃たない。safety() が通信断で呼ぶ */
void denziben(void);        /* 電磁弁の指令 (毎周回) */
uint8_t denziben_on(int n); /* 電磁弁 n (1 = lock1 / 2 = lock2) を開くなら 1。motor_outputs() が読む */
void souten(void);          /* 装填モーターの指令 (毎周回) */
void souten_ramp(void);     /* 装填モーターの pwm をランプで目標へ近づける (20ms 周期) */

/* output.c */
void motor_outputs(void);   /* PWM と DIR と電磁弁の出力。safety() の後に呼ぶ */
void outputs_all_off(void); /* 全 PWM と電磁弁を即 0。HardFault / Error_Handler から呼ぶ */

/* limit_sw.c */
/*
 * リミットスイッチ入力のノイズ除去用。
 * モーターのPWMノイズで一瞬 Low を読んだだけでフラグが立つのを防ぐ。
 * 手で初期化せず、LIMIT_SW_INIT (リミットスイッチ) か LIMIT_SW_INIT_START を使うこと。
 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t stable;   // ノイズ除去後の確定値
    uint8_t last;     // 前回の生の読み値
    uint32_t changed; // 生の読み値が変わった時刻
} limit_sw;

// 確定値 (stable) の起動時の値を start で指定する
#define LIMIT_SW_INIT_START(port, pin, start) {(port), (pin), (start), (start), 0}

// リミットスイッチ用。プルアップ入力なので未押下 (High=1) を初期値にする
#define LIMIT_SW_INIT(port, pin) LIMIT_SW_INIT_START((port), (pin), 1)

uint8_t limit_read(limit_sw *sw);

#ifdef __cplusplus
}
#endif

#endif /* __FUNCTION_H */

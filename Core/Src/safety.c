/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : safety.c
 * @brief          : 安全機能 (異常時の停止、走行中のローラー制限、状態 LED)
 ******************************************************************************
 */
/* USER CODE END Header */
#include "safety.h"
#include "can_handler.h"   /* last_can_rx */
#include "function.h"      /* pwm1〜pwm10, now */
#include "lidar_sensor.h"  /* lidar_timeout() */
#include "sbus_handler.h"  /* SBUS_CH, SBUS_Failsafe, SBUS_LostFrame, last_sbus_rx */

// 走行中のローラーの PWM 上限 (TIM1 の Period 254 に対して約 40%)
static const int ROLLER_PWM_WHILE_DRIVING = 100;

/*
 * SBUS が使えない状態なら 1 を返す。次の 4 つのどれかで判定する。
 *   1. last_sbus_rx のタイムアウト … 受信が完全に途絶えた場合。
 *      SBUS_CH も SBUS_LostFrame もフレームが来たときしか更新されないため、
 *      コネクタが抜けると古い値のまま固まる。これが無いと直前のスティック
 *      指令のまま走り続けてしまう。
 *   2. SBUS_Failsafe … 「受信機が送信機を見失った」決定的な信号。
 *      送信機の電源を切っても受信機は正常なフレームを送り続け、このビット
 *      だけを立てるので、1 でも 3 でも捕まえられない。
 *   3. SBUS_LostFrame … 単発のフレーム落ち。
 *   4. SBUS_CH[0] == 0 … 起動直後(まだ1フレームも来ていない)の保険。
 *      HAL_GetTick() がまだ SBUS_TIMEOUT_MS に満たない間は 1 が効かないため。
 */
static int sbus_lost(void) {
    return HAL_GetTick() - last_sbus_rx > SBUS_TIMEOUT_MS || SBUS_Failsafe || SBUS_LostFrame ||
           SBUS_CH[0] == 0;
}

// CAN が 100ms 以上届いていなければ 1 を返す
static int can_lost(void) {
    return HAL_GetTick() - last_can_rx > 100;
}


static void limit_pwm(int *pwm, int max) {
    if (*pwm > max) {
        *pwm = max;
    }
}

/*
 * LEDは「点灯状態を全部決めてから3本まとめて書く」。
 * 条件ごとにその場で WritePin すると、条件が変わったときに前の色を
 * 消し忘れて赤と青が同時に点く、といった消え残りが起きる。
 *
 * 青点滅 = SBUS断、赤点滅 = CAN断（両方落ちていれば紫点滅になる）、
 * 緑点滅 = Lidar断で自動モードが使えない、緑点灯 = 全て正常。
 */
static void update_status_led(int sbus_error, int can_error) {
    uint8_t blink = (now / 300) % 2;
    uint8_t green = 0;
    uint8_t blue = sbus_error ? blink : 0;
    uint8_t red = can_error ? blink : 0;

    if (!sbus_error && !can_error) {
        green = lidar_timeout() ? blink : 1;
    }

    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, green);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, blue);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, red);
}

/*
 * 安全機能。必ず PWM を出力する直前に呼ぶこと。
 *   ・SBUS か CAN が使えなければ全モーターを止める
 *   ・足回りが回っている間はローラーの PWM を頭打ちにする
 *   ・状態を LED に出す
 */
void safety(void) {
    int sbus_error = sbus_lost();
    int can_error = can_lost();

    if (sbus_error || can_error) {
        pwm1 = 0;
        pwm2 = 0;
        pwm3 = 0;
        pwm4 = 0;
        pwm5 = 0;
        pwm6 = 0;
        pwm7 = 0;
        pwm8 = 0;
        pwm9 = 0;
        pwm10 = 0;
    }

    // ローラーと足回りが同時に全力で回らないようにする（電源の取り合い対策）。
    // 速度ではなく PWM の上限。motor_simple_control は停止指令のとき必ず 0 まで
    // 落とすので、停止中の足回りを「回っている」と誤判定することはない。
    int driving = pwm1 > 0 || pwm2 > 0 || pwm3 > 0 || pwm4 > 0;
    if (driving) {
        limit_pwm(&pwm5, ROLLER_PWM_WHILE_DRIVING);
        limit_pwm(&pwm7, ROLLER_PWM_WHILE_DRIVING);
        limit_pwm(&pwm6, ROLLER_PWM_WHILE_DRIVING);
        limit_pwm(&pwm8, ROLLER_PWM_WHILE_DRIVING);
    }

    update_status_led(sbus_error, can_error);
}

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : safety.c
 * @brief          : 安全機能 (異常時の停止、走行中のローラー制限、装填の上限)
 ******************************************************************************
 */
/* USER CODE END Header */
#include "safety.h"
#include "can_handler.h"   /* last_can_rx */
#include "function.h"      /* pwm1〜pwm10, hassya_off() */
#include "sbus_handler.h"  /* SBUS_CH, SBUS_Failsafe, SBUS_LostFrame, last_sbus_rx */
#include "robot_limits.h"  /* ROLLER_PWM_WHILE_DRIVING, SOUTEN_PWM_MAX, CAN_TIMEOUT_MS */

// safety() がこの周回で判定した通信断 (1 = 断)。led() も同じ判定で表示できるよう、判定は 1 周に 1 回だけにする
static int sbus_error = 1;
static int can_error = 1;

/*
 * 非常停止の空き接点 (b 接点) を PD2 (ラベル LOCK、内蔵プルアップ) と GND の間につないでいる。
 *   通常 … 接点が閉じていて Low
 *   非常停止を押している … 接点が開いて High
 * 線が抜けても High になるので、非常停止中として止める。
 * 起動時は「非常停止中」(1) から始め、20ms 続けて Low を読んでから解除する (limit_read)。
 */
static limit_sw lock_sw = LIMIT_SW_INIT(LOCK_GPIO_Port, LOCK_Pin);

/*
 * SBUS が使えない状態なら 1 を返す。次の 1, 2, 4 のどれかで判定する (3 は使わない)。
 *   1. last_sbus_rx のタイムアウト … 受信が完全に途絶えた場合。
 *      SBUS_CH も SBUS_LostFrame もフレームが来たときしか更新されないため、
 *      コネクタが抜けると古い値のまま固まる。これが無いと直前のスティック
 *      指令のまま走り続けてしまう。
 *   2. SBUS_Failsafe … 「受信機が送信機を見失った」決定的な信号。
 *      送信機の電源を切っても受信機は正常なフレームを送り続け、このビット
 *      だけを立てるので、1 でも 3 でも捕まえられない。
 *   3. SBUS_LostFrame … 単発のフレーム落ち。ノイズで急停止するので判定に使わない。
 *   4. SBUS_CH[0] == 0 … 起動直後(まだ1フレームも来ていない)の保険。
 *      HAL_GetTick() がまだ SBUS_TIMEOUT_MS に満たない間は 1 が効かないため。
 */
static int check_sbus_lost(void) {
    return HAL_GetTick() - last_sbus_rx > SBUS_TIMEOUT_MS || SBUS_Failsafe ||
           SBUS_CH[0] == 0;
}

// CAN が CAN_TIMEOUT_MS 以上届いていなければ 1 を返す
static int check_can_lost(void) {
    return HAL_GetTick() - last_can_rx > CAN_TIMEOUT_MS;
}

// safety() がこの周回で判定した結果を返す (判定し直さない)
int sbus_lost(void) {
    return sbus_error;
}

int can_lost(void) {
    return can_error;
}


static void limit_pwm(int *pwm, int max) {
    if (*pwm > max) {
        *pwm = max;
    }
}

/*
 * 安全機能。必ず PWM を出力する直前に呼ぶこと。
 *   ・SBUS か CAN が使えないか、非常停止 (LOCK) が押されていれば、全モーターを止め、
 *     電磁弁を閉じ、撃つスイッチを離すまで撃たない
 *     (原点復帰の途中だった装填は、元に戻ると原点まで戻る)
 *   ・非常停止のときは、ローラーのスイッチも一度 OFF にするまで回さない
 *   ・足回りが回っている間はローラーの PWM を頭打ちにする
 *   ・装填 (12V 用の RS-555) の PWM を SOUTEN_PWM_MAX で頭打ちにする
 * LED の表示は led() (led.c) が行う。
 */
void safety(void) {
    sbus_error = check_sbus_lost();
    can_error = check_can_lost();
    int estop = limit_read(&lock_sw); // 1 = 非常停止中

    if (sbus_error || can_error || estop) {
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

        // 電磁弁も閉じる。通信が戻っても、撃つスイッチを一度 OFF にするまで電磁弁も装填の送りも動かさない。
        // 原点復帰は止めないので、途中だった場合は通信が戻ると原点まで戻る
        hassya_off();

        // CAN 断だとエンコーダ値 (PV) が古いまま固まり、roller() が「到達」と誤判定しうる。
        // モーターを止めている間は発射準備完了ではないので、フラグを下ろしておく
        roller_ready = 0;
    }

    // 非常停止を解除したとき、ローラーのスイッチが ON のままだと急に回り出すので、一度 OFF にさせる
    // (撃つスイッチは hassya_off() が同じようにしている)
    if (estop) {
        roller_off();
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

    // RS-555 は 12V 用なので、どこで pwm を書き換えても 12V 相当を超えさせない
    limit_pwm(&pwm9, SOUTEN_PWM_MAX);
    limit_pwm(&pwm10, SOUTEN_PWM_MAX);
}

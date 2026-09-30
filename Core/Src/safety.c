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
#include "robot_limits.h"  /* ROLLER_PWM_WHILE_DRIVING, SOUTEN_PWM_MAX, CAN_TIMEOUT_MS, ESTOP_* */

// safety() がこの周回で判定した通信断 (1 = 断)。led() も同じ判定で表示できるよう、判定は 1 周に 1 回だけにする
static int sbus_error = 1;
static int can_error = 1;

/*
 * 非常停止中なら 1 を返す。
 * 非常停止の空き接点 (b 接点) を PD2 (ラベル LOCK、内蔵プルアップ) と GND の間につないでいる。
 *   通常 … 接点が閉じていて PD2 が GND につながり Low
 *   非常停止を押している … 接点が開いて PD2 がどこにもつながらず、プルアップで High
 * 線が抜けても High になるので、非常停止中として止める。
 *
 * 止める側に倒すため、押したときと解除したときで判定を変える。
 *   ・High が ESTOP_PRESS_MS (2ms) 続いたら、すぐ非常停止にする
 *   ・Low が ESTOP_RELEASE_MS (20ms) 続くまで解除しない。途中で 1 回でも High を読んだら測り直す
 * これで配線が緩んで High / Low を行き来していても、止まったままになる。
 * 起動時は非常停止中から始める。
 */
static int check_estop(void) {
    static int estop = 1;
    static uint32_t high_since = 0; // High が続き始めた時刻
    static uint32_t low_since = 0;  // Low が続き始めた時刻
    static int prev_high = 1;
    uint32_t t = HAL_GetTick();
    int high = HAL_GPIO_ReadPin(LOCK_GPIO_Port, LOCK_Pin) == GPIO_PIN_SET;

    if (high && !prev_high) {
        high_since = t;
    }
    if (!high && prev_high) {
        low_since = t;
    }
    prev_high = high;

    if (high && t - high_since >= ESTOP_PRESS_MS) {
        estop = 1;
    }
    if (!high && t - low_since >= ESTOP_RELEASE_MS) {
        estop = 0;
    }
    return estop;
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
 *     電磁弁を閉じ、撃つスイッチとローラーのスイッチを一度 OFF にするまで動かさない
 *     (原点復帰の途中だった装填は、通信断なら元に戻ると原点まで戻る。
 *      非常停止なら撃つスイッチを押すまで戻らない)
 *   ・足回りが回っている間はローラーの PWM を頭打ちにする
 *   ・装填 (12V 用の RS-555) の PWM を SOUTEN_PWM_MAX で頭打ちにする
 * LED の表示は led() (led.c) が行う。
 */
void safety(void) {
    sbus_error = !sbus_valid; // 判定は sbus() (sbus_handler.c) がスイッチを読むときに一緒に行う
    can_error = check_can_lost();
    int estop = check_estop();

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

        // 元に戻ったとき、ローラーのスイッチが ON のままだと急に回り出すので、一度 OFF にさせる
        roller_off();

        // CAN 断だとエンコーダ値 (PV) が古いまま固まり、roller() が「到達」と誤判定しうる。
        // モーターを止めている間は発射準備完了ではないので、フラグを下ろしておく
        roller_ready = 0;
    }

    // 非常停止の間に装填へ手を入れていることがあるので、解除しても原点復帰を勝手に再開しない
    // (撃つスイッチを押すまで待つ)
    if (estop) {
        souten_hold();
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

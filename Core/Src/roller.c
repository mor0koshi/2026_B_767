/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : roller.c
 * @brief          : ローラー (上下 4 個) の目標速度の決定と速度制御
 ******************************************************************************
 */
/* USER CODE END Header */
#include "function.h"
#include "motor_control.h" /* motor_control() */
#include "robot_limits.h"  /* ローラーの目標速度・PWM 上限・ステップ */
#include <stdlib.h>        /* abs() */

/* ============================================================================
 * ローラー
 * ========================================================================== */

// 目標速度 (LOWER_ROLLER_SPEED、BAKETU1〜3_ROLLER_SPEED など) は robot_limits.h で変える
static const int ROLLER_STOP = 0;

// 0 のとき、ローラーのスイッチ (Lmayu2) を一度 OFF にするまで回さない。
// 非常停止 (LOCK) の間 roller_off() が 0 にする。解除したときスイッチが ON のままでも急に回り出さないため
static int roller_armed = 1;

// 回しているローラーが目標速度に達していれば 1。roller() が立て、led.c の color() が LED テープを点滅させる
int roller_ready = 0;

// 上ローラーの目標速度を Ltuno1 で選ぶ。
// BAKETU1 だけは RY の位置 (±1000) に比例して ±BAKETU1_RY_RANGE 変える (中央で BAKETU1_ROLLER_SPEED)
static int upper_roller_speed(void) {
    switch (Ltuno1) {
    case 1:
        return BAKETU3_ROLLER_SPEED;
    case 0:
        return BAKETU2_ROLLER_SPEED;
    case -1:
        return BAKETU1_ROLLER_SPEED + ry * BAKETU1_RY_RANGE / 1000;
    }
    return ROLLER_STOP; // Ltuno1 は -1/0/1 しか取らないので、ここには来ない
}

// ローラー 1 個分の速度制御。4 個とも同じ制御パラメータ (robot_limits.h) を使う
static void roller_motor(int speed, int PV, int *pwm, int *rem) {
    motor_control(speed, PV, ROLLER_STEP_UP, ROLLER_STEP_DOWN, ROLLER_PWM_MAX, pwm, &dummy, rem);
}

// 目標速度 speed に PV が達していれば 1。止めているローラー (speed == 0) は常に 0
static int roller_at_speed(int speed, int PV) {
    return speed != ROLLER_STOP && abs(speed - PV) <= ROLLER_READY_TOLERANCE;
}

/*
 * ローラーの目標速度を決めて速度制御する。20ms 周期で呼ぶこと。
 *
 * モーター割り当て (2026/09 のモーター載せ替え後)
 *   上ローラー : pwm5 / pwm7  (エンコーダ PV1 / PV2 付きの閉ループ)
 *   下ローラー : pwm6 / pwm8  (エンコーダ PV3 / PV4 付きの閉ループ)
 *
 * Lmayu2 == 1 のときだけ回す。上下は Lmayu1 で切り替えるので同時には回らない。
 * 止めるローラーも目標 0 で速度制御し、緩やかに減速させる。
 * 非常停止 (LOCK) の後は、Lmayu2 を一度 OFF にするまで回さない。
 */
void roller(void) {
    if (Lmayu2 != 1) {
        roller_armed = 1;
    }
    int roller_on = (Lmayu2 == 1) && roller_armed;
    int upper = (roller_on && Lmayu1 == 1) ? upper_roller_speed() : ROLLER_STOP;
    int lower = (roller_on && Lmayu1 == 0) ? LOWER_ROLLER_SPEED : ROLLER_STOP;

    roller_motor(upper, PV1, &pwm5, &rem5);
    roller_motor(upper, PV2, &pwm7, &rem7);
    roller_motor(lower, PV3, &pwm6, &rem6);
    roller_motor(lower, PV4, &pwm8, &rem8);

    // 回している方の 2 個が両方とも目標速度に達したらフラグを立てる
    roller_ready = (roller_at_speed(upper, PV1) && roller_at_speed(upper, PV2)) ||
                   (roller_at_speed(lower, PV3) && roller_at_speed(lower, PV4));
}

/*
 * ローラーのスイッチ (Lmayu2) を一度 OFF にするまで、ローラーを回さない。
 * safety() が非常停止 (LOCK) の間に呼ぶ。PWM は safety() が 0 にする。
 */
void roller_off(void) {
    roller_armed = 0;
}

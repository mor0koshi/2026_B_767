/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : asimawari.c
 * @brief          : 足回り (オムニ 4 輪)
 ******************************************************************************
 */
/* USER CODE END Header */
#include "function.h"
#include "lidar_sensor.h"  /* lidar_nearest_mm() */
#include "motor_control.h" /* motor_simple_control() */
#include "robot_limits.h"  /* DRIVE_SCALE_PERCENT, DRIVE_PWM_MAX, DRIVE_STEP, DRIVE_SLOW_* */

/*
 * 前後・左右・旋回の指令から、オムニ 4 輪それぞれの指令値を計算する。
 * 混合式を変えるときはここだけ直せばよい。
 */
static void omni_mix(int forward, int strafe, int turn, int taiya[4]) {
    taiya[0] = -forward + strafe + turn; // 左前 (pwm1)
    taiya[1] = -forward - strafe + turn; // 右前 (pwm2)
    taiya[2] = forward - strafe + turn;  // 左後 (pwm3)
    taiya[3] = forward + strafe + turn;  // 右後 (pwm4)
}

/*
 * スティック (前後 ly・左右 lx・旋回 rx) から足回り 4 輪を動かす。20ms 周期で呼ぶこと。
 * Lidar のどちらかが DRIVE_SLOW_DIST_MM 以内なら DRIVE_SLOW_PERCENT 倍に遅くする。
 * 途絶えている Lidar の古い値では判定しない (2 台とも途絶えていれば通常の速度)。
 */
void asimawari(void) {
    int taiya[4];
    int slow = lidar_nearest_mm() <= DRIVE_SLOW_DIST_MM;

    omni_mix(ly, lx, -rx, taiya);
    for (int i = 0; i < 4; i++) {
        taiya[i] = taiya[i] * DRIVE_SCALE_PERCENT / 100;
        if (slow) {
            taiya[i] = taiya[i] * DRIVE_SLOW_PERCENT / 100;
        }
    }

    motor_simple_control(taiya[0], DRIVE_STEP, DRIVE_PWM_MAX, &pwm1, &dir1);
    motor_simple_control(taiya[1], DRIVE_STEP, DRIVE_PWM_MAX, &pwm2, &dir2);
    motor_simple_control(taiya[2], DRIVE_STEP, DRIVE_PWM_MAX, &pwm3, &dir3);
    motor_simple_control(taiya[3], DRIVE_STEP, DRIVE_PWM_MAX, &pwm4, &dir4);
}

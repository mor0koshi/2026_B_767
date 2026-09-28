/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : asimawari.c
 * @brief          : 足回り (オムニ 4 輪) と、Lidar による自動走行 (PID)
 ******************************************************************************
 */
/* USER CODE END Header */
#include "function.h"
#include "lidar_sensor.h"  /* Lidar の距離、lidar_timeout() */
#include "motor_control.h" /* motor_simple_control() */

/* ============================================================================
 * 足回り
 * ========================================================================== */

// 足回りの PWM を 20ms あたり何ずつ目標値に近づけるか
static const int DRIVE_STEP = 40;

/*
 * 前後・左右・旋回の指令から、オムニ 4 輪それぞれの指令値を計算する。
 * 手動・半自動・全自動のどのモードもこの式を使うので、式を変えるのはここだけでよい。
 */
static void omni_mix(int forward, int strafe, int turn, int taiya[4]) {
    taiya[0] = -forward + strafe + turn; // 左前 (pwm1)
    taiya[1] = -forward - strafe + turn; // 右前 (pwm2)
    taiya[2] = forward - strafe + turn;  // 左後 (pwm3)
    taiya[3] = forward + strafe + turn;  // 右後 (pwm4)
}

/*
 * 走行モード (Rmayu1) に応じて前後・左右・旋回の指令を決め、足回り 4 輪を動かす。
 * 20ms 周期で呼ぶこと (auto_mode() の dt がこの周期前提)。
 *
 *   モード          前後              左右   旋回
 *   -1 手動         ly                lx     rx
 *    0 半自動       ly                lx     PID (auto_rx)
 *    1 全自動       PID (auto_ly)     lx     PID (auto_rx)
 */
void asimawari(void) {
    int d4 = distance4 - LIDAR_OFFSET4;
    int d7 = distance7 - LIDAR_OFFSET7;
    int taiya[4];

    // Lidar が途絶えていると auto_mode は「壁まで遠すぎる」と誤認して全速で走り続ける。
    // 測定値が途絶えている間は Rmayu1 によらず手動モードにする。
    if (Rmayu1 == -1 || lidar_timeout()) {
        // 手動モード。裏で PID をリセットしておく。
        // 目標距離は全自動モードと必ず揃えること (違うと切替時に D 項が跳ねる)
        auto_mode(d4, d7, 1, AUTO_TARGET_DIST_MM);
        omni_mix(ly, lx, -rx, taiya);
        for (int i = 0; i < 4; i++) {
            taiya[i] = taiya[i] * 5 / 10; // 1 軸を倒しきったとき、ちょうど maxpwm (900) になる
        }
    } else if (Rmayu1 == 0) {
        // 半自動モード。目標距離に現在距離を渡して距離の誤差を 0 にし、旋回の PID だけ効かせる
        auto_mode(d4, d7, 0, (d4 + d7) / 2);
        omni_mix(ly, lx, auto_rx, taiya);
    } else {
        // 全自動モード。auto_ly は ly と符号が逆 (PID 出力の符号は実機合わせ)
        auto_mode(d4, d7, 0, AUTO_TARGET_DIST_MM);
        omni_mix(-auto_ly, lx, auto_rx, taiya);
    }

    motor_simple_control(taiya[0], DRIVE_STEP, maxpwm, &pwm1, &dir1);
    motor_simple_control(taiya[1], DRIVE_STEP, maxpwm, &pwm2, &dir2);
    motor_simple_control(taiya[2], DRIVE_STEP, maxpwm, &pwm3, &dir3);
    motor_simple_control(taiya[3], DRIVE_STEP, maxpwm, &pwm4, &dir4);
}

/* ============================================================================
 * Lidar による自動走行 (PID)
 * ========================================================================== */

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
} PID;

static const float PID_DT = 0.02;               // 20ms 周期
static const float PID_INTEGRAL_LIMIT = 2000;   // 積分の暴走防止

// 積分をリセットする。ゲイン(Ki)ではなく積分値を消すこと。
// Ki を 0 にすると static なので電源を切るまで I 制御が復活しない。
static void pid_reset(PID *pid, float error) {
    pid->integral = 0;
    pid->prev_error = error;
}

// 誤差から PID の出力を計算する
static float pid_update(PID *pid, float error) {
    pid->integral += error * PID_DT;
    if (pid->integral > PID_INTEGRAL_LIMIT)
        pid->integral = PID_INTEGRAL_LIMIT;
    if (pid->integral < -PID_INTEGRAL_LIMIT)
        pid->integral = -PID_INTEGRAL_LIMIT;

    float derivative = (error - pid->prev_error) / PID_DT;
    pid->prev_error = error;

    return (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
}

/*
 * 2 つの Lidar の距離から、壁との距離と平行を保つ仮想スティック値を計算する。
 *   auto_ly … 距離 (平均) の PID。前後移動
 *   auto_rx … 角度 (差分) の PID。旋回
 * 20ms 周期で呼ぶこと (dt が固定)。reset_flag = 1 で積分をリセットして 0 を返す。
 *
 * ※ 出力の符号は実機の「前進/右旋回がプラスかマイナスか」に合わせて反転させること
 */
void auto_mode(int distance1, int distance2, int reset_flag, int target_dist) {
    // ゲインは実機で要調整 (Kp, Ki, Kd)
    static PID distance = {1.3, 0.008, 0.05, 0, 0};
    static PID angle = {0.6, 0.01, 0.2, 0, 0};

    float target_distance = target_dist;                // 目標距離 (mm)
    float current_dist = (distance1 + distance2) / 2.0; // 現在の距離
    float error_dist = current_dist - target_distance;  // 距離のズレ
    float error_angle = distance1 - distance2;          // 角度のズレ

    if (reset_flag == 1) {
        pid_reset(&distance, error_dist);
        pid_reset(&angle, error_angle);
        auto_ly = 0;
        auto_rx = 0;
        return;
    }

    auto_ly = (int)pid_update(&distance, error_dist);
    auto_rx = (int)pid_update(&angle, error_angle);
}

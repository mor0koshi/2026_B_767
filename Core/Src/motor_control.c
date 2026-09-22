/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : motor_control.c
 * @brief          : モーター 1 個分の制御 (ローラー用の速度制御と、足回り用のランプ制御)
 ******************************************************************************
 */
/* USER CODE END Header */
#include "motor_control.h"

/*
 * 速度制御 (積分制御)。エンコーダのあるローラーで使う。
 *
 * 積分器は pwm 自身。毎周期 pwm に MV(=誤差の1/10) を足し込むことで、
 * 誤差が 0 になるまで pwm が育っていく。
 *
 * remm は「積分項」ではなく、error/10 の整数除算で切り捨てられる端数の
 * 繰り越し(キャリー)。|rem| は必ず 10 未満に収まり、蓄積はしない。
 * これが無いと |error| < 10 の領域で MV が常に 0 になり、pwm が動かず
 * 定常偏差が残ったままになる。端数を持ち越すことでその不感帯を解消する。
 *
 * なお MV が maxMV で頭打ちになる場合、はみ出した分は繰り越さずに捨てる。
 * これは1周期あたりの変化量を制限するため(積分ワインドアップ防止)で、
 * キャリーが効くのは飽和していない = 定常付近の領域だけになる。
 */
void motor_control(int SV, int PV, int maxMV, int down_pwm, int max_pwm, int *pwmm, int *dirr, int *remm) {
    int error = 0;
    int MV = 0;
    int lastMV = 0;
    int pwm = *pwmm;
    int target_dir = 0;
    int dir = *dirr;
    int rem = *remm; // 前回の端数を復元

    // リミッター処理
    if (SV > max_pwm) {
        SV = max_pwm;
    } else if (SV < -max_pwm) {
        SV = -max_pwm;
    }

    // dir設定と絶対値化
    if (SV < 0) {
        target_dir = 0;
        SV = -SV;
    } else if (SV > 0) {
        target_dir = 1;
    }

    error = SV - PV;

    rem += error;   // ① 今回の誤差に前回の端数を足す
    MV = rem / 10;  // ② 10 で割れるぶんだけ操作量にする(Ki=0.1)
    rem -= MV * 10; // ③ 使ったぶんを引き、端数(|rem| < 10)だけ残す

    if (MV > maxMV) {
        lastMV = maxMV;
    } else if (MV < -maxMV) {
        lastMV = -maxMV;
    } else {
        lastMV = MV;
    }

    // 回転方向が目標と異なる場合,一旦pwmを0まで落としてから方向を変える
    if (SV != 0) {
        if (dir != target_dir) {
            rem = 0; // 方向転換中は端数を捨てる
            if (pwm > down_pwm) {
                pwm -= down_pwm;
            } else {
                pwm = 0;
                dir = target_dir;
            }
        } else {
            pwm += lastMV;
            if (pwm < 0) {
                pwm = 0;
            }
        }
    }

    if (pwm > max_pwm) {
        pwm = max_pwm;
    }

    // 指令値が 0 のときは緩やかにモーターを停止させる
    if (SV == 0) {
        rem = 0; // 停止指令中は端数を捨てる
        if (pwm > down_pwm) {
            pwm -= down_pwm;
        } else {
            pwm = 0;
        }
    }

    *pwmm = pwm;
    *dirr = dir;
    *remm = rem;
}
/*
 * エンコーダを使わない簡易版。足回りで使う。
 * 目標値(SV)に向けて1回の呼び出しごとにstepずつpwmを近づける。
 * SV の符号が回転方向を表し (負 = dir 0 / 正 = dir 1)、絶対値がそのまま目標 pwm になる。
 * 方向転換と停止は motor_control と同じ扱いで、どちらも step ずつ pwm を落としてから行う。
 */
void motor_simple_control(int SV, int step, int max_pwm, int *pwmm, int *dirr) {
    int pwm = *pwmm;
    int dir = *dirr;
    int target_dir = dir;

    // リミッター処理
    if (SV > max_pwm) {
        SV = max_pwm;
    } else if (SV < -max_pwm) {
        SV = -max_pwm;
    }

    // dir設定と絶対値化
    if (SV < 0) {
        target_dir = 0;
        SV = -SV;
    } else if (SV > 0) {
        target_dir = 1;
    }

    if (SV == 0) {
        // 指令値が 0 のときは緩やかにモーターを停止させる
        if (pwm > step) {
            pwm -= step;
        } else {
            pwm = 0;
        }
    } else if (dir != target_dir) {
        // 回転方向が目標と異なる場合、一旦 pwm を 0 まで落としてから方向を変える
        if (pwm > step) {
            pwm -= step;
        } else {
            pwm = 0;
            dir = target_dir;
        }
    } else if (pwm < SV) {
        pwm += step;
        if (pwm > SV) {
            pwm = SV;
        }
    } else if (pwm > SV) {
        pwm -= step;
        if (pwm < SV) {
            pwm = SV;
        }
    }

    *pwmm = pwm;
    *dirr = dir;
}

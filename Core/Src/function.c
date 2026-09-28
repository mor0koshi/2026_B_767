/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : function.c
 * @brief          : 足回り・ローラー・装填の制御と、PWM/DIR の出力
 ******************************************************************************
 */
/* USER CODE END Header */
#include "function.h"
#include "lidar_sensor.h"  /* asimawari() で Lidar の距離と lidar_timeout() を使うため */
#include "motor_control.h" /* motor_control(), motor_simple_control() */
#include "robot_limits.h"  /* 上限値・ランプ・タイムアウトの定数 */
#include "solenoid.h"      /* 電磁弁の ON 時間の制限 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// printf の出力先を USART3 にする
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, 10);
    return len;
}

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
 * ローラー
 * ========================================================================== */

// ローラーの目標速度。エンコーダ値 (PV) と同じ 0〜255 系で、TIM1 の Period 254 以下にすること
static const int ROLLER_SPEED = 245;         // 下ローラー
static const int BAKETU1_ROLLER_SPEED = 150; // 上ローラー Ltuno1 == -1　長押し
static const int BAKETU2_ROLLER_SPEED = 84; // 上ローラー Ltuno1 == 0　PS
static const int BAKETU3_ROLLER_SPEED = 76; // 上ローラー Ltuno1 == 1　旗
static const int ROLLER_STOP = 0;

// BAKETU1 は RY スティックで上げ下げできる。倒しきったときに BAKETU1_ROLLER_SPEED から変える量。
// BAKETU1_ROLLER_SPEED + BAKETU1_RY_RANGE も 254 以下にすること
static const int BAKETU1_RY_RANGE = 50;

// 目標速度との差がこれ以内なら「目標速度に達した」とみなす (PV と同じ 0〜255 系)
static const int ROLLER_READY_TOLERANCE = 5;

// 回しているローラーが目標速度に達していれば 1。roller() が立て、safety() の color() が LED を点滅させる
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
 */
void roller(void) {
    int roller_on = (Lmayu2 == 1);
    int upper = (roller_on && Lmayu1 == 1) ? upper_roller_speed() : ROLLER_STOP;
    int lower = (roller_on && Lmayu1 == 0) ? ROLLER_SPEED : ROLLER_STOP;

    roller_motor(upper, PV1, &pwm5, &rem5);
    roller_motor(upper, PV2, &pwm7, &rem7);
    roller_motor(lower, PV3, &pwm6, &rem6);
    roller_motor(lower, PV4, &pwm8, &rem8);

    // 回している方の 2 個が両方とも目標速度に達したらフラグを立てる
    roller_ready = (roller_at_speed(upper, PV1) && roller_at_speed(upper, PV2)) ||
                   (roller_at_speed(lower, PV3) && roller_at_speed(lower, PV4));
}

/* ============================================================================
 * 電磁弁と装填
 * ========================================================================== */

// 装填モーター (RS-555) の目標値。符号が向き (負 = dir 0 / 正 = dir 1) で、motor_simple_control の SV と同じ。
// souten() が毎周回決め、souten_ramp() が 20ms ごとに pwm9 / pwm10 をこの値へ近づける
static int souten1_target = 0;
static int souten2_target = 0;

// 電磁弁の ON 時間の制限 (solenoid.h)。起動時は LOCKOUT から始める
static solenoid valve1 = SOLENOID_INIT; // lock1
static solenoid valve2 = SOLENOID_INIT; // lock2

// 電磁弁を開くなら 1。souten() が決め、safety() が非常時に 0 にし、motor_outputs() が出力する
static uint8_t valve1_on = 0;
static uint8_t valve2_on = 0;

// 逆転リセットのリミットスイッチ。ノイズ除去して読む (limit_read)
static limit_sw sw_lock6 = LIMIT_SW_INIT(lock6_GPIO_Port, lock6_Pin); // 装填1 リセット開始
static limit_sw sw_lock7 = LIMIT_SW_INIT(lock7_GPIO_Port, lock7_Pin); // 装填1 原点
static limit_sw sw_lock8 = LIMIT_SW_INIT(lock8_GPIO_Port, lock8_Pin); // 装填2 リセット開始
static limit_sw sw_lock9 = LIMIT_SW_INIT(lock9_GPIO_Port, lock9_Pin); // 装填2 原点

/*
 * 原点復帰フラグを更新する。
 * lock6/lock8 で立ち、原点の lock7/lock9 で下りる (両方踏んでいれば原点側が勝つ)。
 *
 * フラグが切り替わった瞬間 (送りの端 lock6/lock8 か、原点 lock7/lock9 を踏んだとき) は、
 * 機構を端に押し付けないよう、ランプを待たずに pwm を 0 にする。その後の反転・再始動はランプで行う。
 * 「切り替わった瞬間」だけにするのは、原点で静止している間に毎周回 0 にすると、
 * 次の正転指令まで打ち消されてしまうため。
 */
static void update_homing(void) {
    int was_homing1 = reset_flag1;
    int was_homing2 = reset_flag2;

    if (limit_read(&sw_lock6) == 0) {
        reset_flag1 = 1;
    }
    if (limit_read(&sw_lock7) == 0) {
        reset_flag1 = 0;
    }
    if (limit_read(&sw_lock8) == 0) {
        reset_flag2 = 1;
    }
    if (limit_read(&sw_lock9) == 0) {
        reset_flag2 = 0;
    }

    if (reset_flag1 != was_homing1) {
        pwm9 = 0;
    }
    if (reset_flag2 != was_homing2) {
        pwm10 = 0;
    }
}

// dir の向きに SOUTEN_PWM_MAX で回すときの装填の目標値
static int souten_target(int dir) {
    return dir ? SOUTEN_PWM_MAX : -SOUTEN_PWM_MAX;
}

/*
 * 電磁弁と装填モーターの指令を決める。毎周回呼ぶ。
 * 装填モーターの pwm は souten_ramp() が、電磁弁の出力は motor_outputs() が書く。
 *
 *   Rtuno2 (撃つ)  ローラー   動作
 *   0              -          電磁弁も装填も止める
 *   1              停止中     Rmayu2 で選んだ電磁弁を開く (0=lock1 / 1=lock2)。SOLENOID_MAX_ON_MS で閉じる
 *   1              下が回転   装填1 (pwm9) を正転
 *   1              上が回転   装填2 (pwm10) を正転
 *
 * 原点復帰中は、上の結果によらず装填モーターを逆転させる。
 */
void souten(void) {
    int shoot = (Rtuno2 == 1);
    int roller_on = (Lmayu2 == 1);

    update_homing();

    // 電磁弁: ローラー停止中に撃つときだけ使う
    int use_solenoid = shoot && !roller_on;
    uint32_t t = HAL_GetTick();
    valve1_on = solenoid_update(&valve1, use_solenoid && Rmayu2 == 0, t);
    valve2_on = solenoid_update(&valve2, use_solenoid && Rmayu2 == 1, t);

    // 装填: ローラー回転中に撃つとき、回っている方のローラーへ球を送る
    int feed = shoot && roller_on;
    souten1_target = 0;
    souten2_target = 0;
    if (feed && Lmayu1 == 0) { // 下ローラー → 装填1
        souten1_target = souten_target(0);
    }
    if (feed && Lmayu1 == 1) { // 上ローラー → 装填2
        souten2_target = souten_target(1);
    }

    // 原点復帰中は優先して逆転させる
    if (reset_flag1 == 1) {
        souten1_target = souten_target(1);
    }
    if (reset_flag2 == 1) {
        souten2_target = souten_target(0);
    }
}

/*
 * 装填モーター (RS-555) の pwm を souten() の目標値へ近づける。20ms 周期で呼ぶこと。
 * 12V 用のモーターを 18V 系統で回すので、上限は SOUTEN_PWM_MAX (12V/21V ≒ 57%)。
 * 始動・停止は SOUTEN_RAMP_MS かけて変化させ、反転は一度 0 まで下げてから向きを変える。
 * リミットを踏んだときだけは update_homing() が即 0 にする。
 */
void souten_ramp(void) {
    motor_simple_control(souten1_target, SOUTEN_RAMP_STEP, SOUTEN_PWM_MAX, &pwm9, &souten_dir1);
    motor_simple_control(souten2_target, SOUTEN_RAMP_STEP, SOUTEN_PWM_MAX, &pwm10, &souten_dir2);
}

/*
 * 電磁弁を両方すぐ閉じ、撃つスイッチが一度 OFF になるまで開かないようにする。
 * safety() が通信断のときに呼ぶ。通信が戻った瞬間にスイッチが ON のままでも開かない。
 */
void valves_off(void) {
    solenoid_lockout(&valve1);
    solenoid_lockout(&valve2);
    valve1_on = 0;
    valve2_on = 0;
}

/*
 * リミットスイッチの読み取り (チャタリング/ノイズ除去)
 *
 * reset_flag は「一度 Low を読んだら立つ」「lock7/lock9 を踏むまで下りない」という
 * ラッチ構造なので、モーターのPWMノイズが 1 スキャン乗っただけで装填が回りっぱなしになる。
 * LIMIT_DEBOUNCE_MS の間 同じ値を読み続けたときだけ確定値を更新することで、
 * 単発のノイズをフラグまで通さない。
 */
#define LIMIT_DEBOUNCE_MS 20

uint8_t limit_read(limit_sw *sw) {
    uint8_t raw = (uint8_t)HAL_GPIO_ReadPin(sw->port, sw->pin);

    if (raw != sw->last) {
        // 値が動いた。ここから改めて安定時間を計り直す
        sw->last = raw;
        sw->changed = HAL_GetTick();
    } else if (raw != sw->stable && HAL_GetTick() - sw->changed >= LIMIT_DEBOUNCE_MS) {
        // 同じ値を十分な時間読み続けたので確定
        sw->stable = raw;
    }

    return sw->stable;
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

/* ============================================================================
 * 出力
 * ========================================================================== */

/*
 * PWM と DIR をまとめて出力する。
 *
 * 基板 (2026_B_main) は PWMn と DIRn が同じドライバへ行く配線なので、
 * DIR はソフトの pwmN 番号ではなく「その PWM が出ている基板ch の DIR」を書く。
 * 例: pwm1 の PWM は PD15 = 基板の PWM3 なので、DIR は d3 (PE10)。
 */
void motor_outputs(void) {
    // 足回り
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm1); // 左前 = 基板ch3 (PWM3=PD15)
    HAL_GPIO_WritePin(d3_GPIO_Port, d3_Pin, dir1);      // DIR3 = PE10
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm2); // 右前 = 基板ch4 (PWM4=PD14)
    HAL_GPIO_WritePin(d4_GPIO_Port, d4_Pin, dir2);      // DIR4 = PD11
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm3); // 左後 = 基板ch1 (PWM1=PD12)
    HAL_GPIO_WritePin(d1_GPIO_Port, d1_Pin, dir3);      // DIR1 = PB1
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm4); // 右後 = 基板ch2 (PWM2=PD13)
    HAL_GPIO_WritePin(d2_GPIO_Port, d2_Pin, dir4);      // DIR2 = PB2

    // ローラーは常に一方向なので DIR は固定値。
    // 対になる 2 個は向かい合っているので、逆の値にして互いに逆回転させる。
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm5); // 上ローラー = 基板ch7 (PWM7=PE11)
    HAL_GPIO_WritePin(d7_GPIO_Port, d7_Pin, 1);         // DIR7 = PF12
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm6); // 下ローラー = 基板ch8 (PWM8=PE9)
    HAL_GPIO_WritePin(d8_GPIO_Port, d8_Pin, 1);         // DIR8 = PF13
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm7); // 上ローラー = 基板ch5 (PWM5=PE13)
    HAL_GPIO_WritePin(d5_GPIO_Port, d5_Pin, 0);         // DIR5 = PF3
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm8); // 下ローラー = 基板ch6 (PWM6=PE14)
    HAL_GPIO_WritePin(d6_GPIO_Port, d6_Pin, 0);         // DIR6 = PF14

    // 装填は正転/逆転リセットがあるので DIR は souten_dir を出す
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm9);    // 装填1 = 基板ch10 (PWM10=PC7)
    HAL_GPIO_WritePin(d10_GPIO_Port, d10_Pin, souten_dir1); // DIR10 = PA11
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm10);   // 装填2 = 基板ch9 (PWM9=PC6)
    HAL_GPIO_WritePin(d9_GPIO_Port, d9_Pin, souten_dir2);   // DIR9 = PA12

    // 電磁弁 (GPIO High で ON)
    HAL_GPIO_WritePin(lock1_GPIO_Port, lock1_Pin, valve1_on); // lock1 = PG4
    HAL_GPIO_WritePin(lock2_GPIO_Port, lock2_Pin, valve2_on); // lock2 = PG6

    // 予備 (未使用)
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm11); // 基板ch11 (PWM11=PC8), DIR11 = PB12
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm12); // 基板ch12 (PWM12=PC9), DIR12 = PB11
}

/*
 * 全モーターと電磁弁を即座に止める。HardFault_Handler と Error_Handler から呼ぶ。
 * マイコンが止まっても、最後の PWM で回り続けたり、電磁弁 (12V 品を 18V 系統で駆動) が
 * 開きっぱなしになったりしないようにする。
 * 初期化の途中で呼ばれても安全なように、HAL のハンドル (Instance が未設定かもしれない) を
 * 使わずにレジスタへ直接書く。
 */
void outputs_all_off(void) {
    HAL_GPIO_WritePin(lock1_GPIO_Port, lock1_Pin, 0);
    HAL_GPIO_WritePin(lock2_GPIO_Port, lock2_Pin, 0);

    TIM1->CCR1 = 0; // ローラー
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCR4 = 0;
    TIM3->CCR1 = 0; // 装填・予備
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
    TIM4->CCR1 = 0; // 足回り
    TIM4->CCR2 = 0;
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 0;
}

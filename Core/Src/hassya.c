/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : hassya.c
 * @brief          : 発射 (撃つスイッチの押し直し)、電磁弁、装填モーター
 ******************************************************************************
 */
/* USER CODE END Header */
#include "function.h"
#include "motor_control.h" /* motor_simple_control() */
#include "robot_limits.h"  /* SOUTEN_PWM_MAX, SOUTEN_RAMP_STEP, SOLENOID_MAX_ON_MS */
#include "solenoid.h"      /* 電磁弁の ON 時間の制限 */

/* ============================================================================
 * 発射 (撃つスイッチの押し直し)
 * ========================================================================== */

// 今撃ってよいなら 1。hassya() が決め、souten() と denziben() が使う
static int shoot = 0;

// Rtuno2 を一度離すと 1、hassya_disarm() で 0 になる。
// 起動時は 0 なので、撃つスイッチを ON にしたまま電源を入れても撃たない
static int shoot_armed = 0;

/*
 * 撃ってよいか (shoot) を決める。毎周回、souten() と denziben() の前に呼ぶ。
 *
 * 撃つには Rtuno2 を一度離してから押す必要がある。次のときは、Rtuno2 を離すまで撃たない。
 *   ・起動時
 *   ・通信断 (safety() が hassya_off() を呼ぶ)
 *   ・電磁弁が SOLENOID_MAX_ON_MS で閉じたとき (denziben() が hassya_disarm() を呼ぶ)
 *   ・Rtuno2 を押したまま、撃つ相手を決めるスイッチ (Lmayu1 / Lmayu2 / Rmayu2) が変わったとき
 * 「押したまま」は前の周回でも Rtuno2 が ON だったこと。押し始めとスイッチの変更が
 * 同じ SBUS フレームで来た場合は、押し直しとみなして撃ってよい。
 */
void hassya(void) {
    static int prev_Rtuno2 = 0;
    static int prev_Lmayu1 = 0;
    static int prev_Lmayu2 = 0;
    static int prev_Rmayu2 = 0;
    int target_changed = Lmayu1 != prev_Lmayu1 || Lmayu2 != prev_Lmayu2 || Rmayu2 != prev_Rmayu2;

    if (Rtuno2 != 1) {
        shoot_armed = 1;
    } else if (prev_Rtuno2 == 1 && target_changed) {
        shoot_armed = 0;
    }
    shoot = (Rtuno2 == 1) && shoot_armed;

    prev_Rtuno2 = Rtuno2;
    prev_Lmayu1 = Lmayu1;
    prev_Lmayu2 = Lmayu2;
    prev_Rmayu2 = Rmayu2;
}

// Rtuno2 を一度離すまで撃たないようにする
static void hassya_disarm(void) {
    shoot_armed = 0;
}

/* ============================================================================
 * 電磁弁
 * ========================================================================== */

// 電磁弁の ON 時間の制限 (solenoid.h)。起動時は LOCKOUT から始める
static solenoid valve1 = SOLENOID_INIT; // lock1
static solenoid valve2 = SOLENOID_INIT; // lock2

// 電磁弁を開くなら 1。denziben() が決め、hassya_off() が非常時に 0 にする。
// ほかのファイルから書き換えて 800ms の制限を素通りしないよう static にし、読むのは denziben_on() だけにする
static uint8_t valve1_on = 0;
static uint8_t valve2_on = 0;

/*
 * 電磁弁の指令を決める。毎周回、hassya() の後に呼ぶ。出力は motor_outputs() (output.c) が書く。
 *
 *   撃つ (shoot)   ローラー   動作
 *   0              -          両方閉じる
 *   1              停止中     Rmayu2 で選んだ電磁弁を開く (0=lock1 / 1=lock2)。SOLENOID_MAX_ON_MS で閉じる
 *   1              回転中     両方閉じる (撃つのは装填モーター)
 */
void denziben(void) {
    int use_solenoid = shoot && (Lmayu2 != 1);
    int command1 = use_solenoid && Rmayu2 == 0;
    int command2 = use_solenoid && Rmayu2 == 1;
    uint32_t t = HAL_GetTick();
    valve1_on = solenoid_update(&valve1, command1, t);
    valve2_on = solenoid_update(&valve2, command2, t);

    // 開けたいのに開けなかった = SOLENOID_MAX_ON_MS で閉じた。
    // 押したまま Rmayu2 や Lmayu2 を切り替えて開き直さないよう、撃つスイッチを離すまで撃たない
    if ((command1 && !valve1_on) || (command2 && !valve2_on)) {
        hassya_disarm();
    }
}

// 電磁弁 n (1 = lock1 / 2 = lock2) を開くなら 1。motor_outputs() が読む
uint8_t denziben_on(int n) {
    return n == 1 ? valve1_on : valve2_on;
}

/* ============================================================================
 * 装填
 * ========================================================================== */

// 装填モーター (RS-555) の目標値。符号が向き (負 = dir 0 / 正 = dir 1) で、motor_simple_control の SV と同じ。
// souten() が毎周回決め、souten_ramp() が 20ms ごとに pwm9 / pwm10 をこの値へ近づける
static int souten1_target = 0;
static int souten2_target = 0;

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
 * 装填モーターの指令を決める。毎周回、hassya() の後に呼ぶ。pwm は souten_ramp() が書く。
 *
 *   撃つ (shoot)   ローラー   動作
 *   0              -          止める
 *   1              停止中     止める (撃つのは電磁弁)
 *   1              下が回転   装填1 (pwm9) を正転
 *   1              上が回転   装填2 (pwm10) を正転
 *
 * 原点復帰中は、上の結果によらず逆転させる。
 */
void souten(void) {
    update_homing();

    // ローラー回転中に撃つとき、回っている方のローラーへ球を送る
    int feed = shoot && (Lmayu2 == 1);
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

/* ============================================================================
 * 非常時
 * ========================================================================== */

/*
 * 撃つのをすぐ全部止める。safety() が通信断のときに呼ぶ。
 *   ・電磁弁をすぐ閉じる
 *   ・装填の目標値を 0 にする (次の周の頭で souten_ramp() が古い送りの目標へ動かさないように)
 *   ・撃つスイッチを一度離すまで、電磁弁も装填の送りも動かさない
 * 原点復帰は止めない。途中で通信が切れた場合は、通信が戻ると原点まで戻る。
 */
void hassya_off(void) {
    solenoid_lockout(&valve1);
    solenoid_lockout(&valve2);
    valve1_on = 0;
    valve2_on = 0;
    souten1_target = 0;
    souten2_target = 0;
    hassya_disarm();
}

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
#include "robot_limits.h"  /* SOUTEN_PWM_MAX, SOUTEN_RAMP_STEP, SOUTEN_TIMEOUT_MS */

/* ============================================================================
 * 発射 (撃つスイッチの押し直し)
 * ========================================================================== */

// 今撃ってよいなら 1。hassya() が決め、souten() と denziben() が使う
static int shoot = 0;

// Rtuno2 を一度離すと 1、hassya_off() や、押したままのスイッチの切り替えで 0 になる。
// 起動時は 0 なので、撃つスイッチを ON にしたまま電源を入れても撃たない
static int shoot_armed = 0;

/*
 * 撃ってよいか (shoot) を決める。毎周回、souten() と denziben() の前に呼ぶ。
 *
 * 撃つには Rtuno2 を一度離してから押す必要がある。次のときは、Rtuno2 を離すまで撃たない。
 *   ・起動時
 *   ・通信断 (safety() が hassya_off() を呼ぶ)
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

/* ============================================================================
 * 電磁弁
 * ========================================================================== */

// 電磁弁を開くなら 1。denziben() が決め、hassya_off() が非常時に 0 にする。
// ほかのファイルから書き換えて押し直しの確認を素通りしないよう static にし、読むのは denziben_on() だけにする
static uint8_t valve1_on = 0;
static uint8_t valve2_on = 0;

/*
 * 電磁弁の指令を決める。毎周回、hassya() の後に呼ぶ。出力は motor_outputs() (output.c) が書く。
 * 電磁弁は降圧回路で 12V 駆動なので、開いている時間の制限は無い (撃っている間は開いたまま)。
 *
 *   撃つ (shoot)   ローラー   動作
 *   0              -          両方閉じる
 *   1              停止中     Rmayu2 で選んだ電磁弁を開く (0=lock1 / 1=lock2)
 *   1              回転中     両方閉じる (撃つのは装填モーター)
 */
void denziben(void) {
    int use_solenoid = shoot && (Lmayu2 != 1);
    valve1_on = use_solenoid && Rmayu2 == 0;
    valve2_on = use_solenoid && Rmayu2 == 1;
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

/*
 * 装填モーターが回り続けている時間を測る。装填1 / 装填2 で 1 つずつ。
 * 目標値が変わったら測り直すので、送りと原点復帰は別々に測る。
 */
typedef struct {
    int target;     // 測り始めたときの目標値
    uint32_t since; // 目標値が target になった時刻
    int stopped;    // タイムアウトで止めたら 1。撃つスイッチを押し直すまで回さない
} souten_timer;

static souten_timer timer1 = {0, 0, 0};
static souten_timer timer2 = {0, 0, 0};

/*
 * 目標値が同じまま SOUTEN_TIMEOUT_MS 以上続いたら、原点復帰をやめてモーターを止める。
 * リミットスイッチの故障・配線外れ・機構の詰まりで、端に押し付けたまま回し続けないため。
 * pwm はリミットを踏んだときと同じくランプを待たずに 0 にする。
 *
 * 止めたあとは、撃つスイッチを押し直す (restart = 1) まで回さない。
 * 押し直しを待たないと、送りの端のスイッチを踏んだまま詰まっているとき、
 * 次の周回で原点復帰がまた始まり「4 秒回って止まる」を繰り返してしまう。
 */
static void souten_timeout(souten_timer *t, int *target, int *reset_flag, int *pwm, int restart) {
    if (restart) {
        t->stopped = 0;
    }
    if (t->stopped) {
        *target = 0;
        *reset_flag = 0;
        t->target = 0; // 押し直したとき、止める前の時刻から測らないように
        return;
    }

    if (*target != t->target) {
        t->target = *target;
        t->since = now;
    }
    if (*target != 0 && now - t->since >= SOUTEN_TIMEOUT_MS) {
        t->stopped = 1;
        *target = 0;
        *reset_flag = 0;
        *pwm = 0;
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
 * 同じ向きに SOUTEN_TIMEOUT_MS 以上回り続けたら止め、撃つスイッチを押し直すまで回さない。
 */
void souten(void) {
    static int prev_shoot = 0;
    int restart = shoot && !prev_shoot; // 撃つスイッチを押し直した
    prev_shoot = shoot;

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

    souten_timeout(&timer1, &souten1_target, &reset_flag1, &pwm9, restart);
    souten_timeout(&timer2, &souten2_target, &reset_flag2, &pwm10, restart);
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
 *   ・装填の回っている時間を測り直す (止められている間は回っていないので数えない)
 * 原点復帰は止めない。途中で通信が切れた場合は、通信が戻ると原点まで戻る。
 */
void hassya_off(void) {
    valve1_on = 0;
    valve2_on = 0;
    souten1_target = 0;
    souten2_target = 0;
    shoot_armed = 0;
    timer1.target = 0;
    timer2.target = 0;
}

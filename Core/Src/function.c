/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : function.c
 * @brief          : main.c から分離したユーザー定義関数
 ******************************************************************************
 */
/* USER CODE END Header */
#include "function.h"
#include "lidar_sensor.h" /* safety() で lidar_timeout() を使用するため */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// printf の出力先を USART3 にする
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, 10);
    return len;
}

// CAN 送信 (固定ペイロード)。現在はどこからも呼ばれていない
void CAN_TX(uint32_t recipient) {
    // 送信用インスタンス等
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8];
    // 送信メールボックスに空きがあったら送信開始
    if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)) {
        // 送信用インスタンスの設定
        TxHeader.StdId = recipient; // 受取手のCANのID
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = 8; // データ長を8byteに設定
        TxHeader.TransmitGlobalTime = DISABLE;
        // 各データ
        TxData[0] = 1;
        TxData[1] = 0;
        TxData[2] = 0;
        TxData[3] = 0;
        TxData[4] = 0;
        TxData[5] = 0;
        TxData[6] = 0;
        TxData[7] = 0;
        // CANメッセージを送信
        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
            Error_Handler();
        }
    }
}
/*
 * CAN 受信割り込み。ID 0x001 の 8 バイトをそのまま use_data[] に写す。
 * use_data[0..3] はメインループで PV1〜PV4 (ローラーのエンコーダ値) になる。
 * last_can_rx は safety() の CAN 断判定に使う。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
    CAN_RxHeaderTypeDef RxHeader; // 受信メッセージの情報が格納されるインスタンス
    uint8_t RxData[8];            // 受信したデータを一時保存する配列
    uint32_t id;                  // CANメッセージIDを格納する変数
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        id = RxHeader.StdId;                    // RxHeaderの中に入っているidを取り出す
        if (id == 0x001 && RxHeader.DLC >= 8) { // idが0x001でデータ長が8以上の場合
            last_can_rx = HAL_GetTick();        // 受信時刻を更新
            for (int i = 0; i <= 7; i++) {
                use_data[i] = RxData[i];
            }
        }
    }
}

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

// ローラーの目標速度。エンコーダ値 (PV) と同じ 0〜255 系で、TIM1 の Period 254 以下にすること
static const int ROLLER_SPEED = 245;         // 下ローラー
static const int BAKETU1_ROLLER_SPEED = 100; // 上ローラー Ltuno1 == -1
static const int BAKETU2_ROLLER_SPEED = 150; // 上ローラー Ltuno1 == 0
static const int BAKETU3_ROLLER_SPEED = 200; // 上ローラー Ltuno1 == 1
static const int ROLLER_STOP = 0;

// 未使用
uint32_t time3 = 0;
uint32_t time4 = 0;
uint32_t time5 = 0;
int set_flag1 = 0;
int set_flag2 = 0;
/*
 * ローラーの目標速度を決める。20ms 周期で呼ぶこと。
 *
 * モーター割り当て (2026/09 のモーター載せ替え後)
 *   上ローラー : pwm5 / pwm7  (エンコーダ PV1 / PV2 付きの閉ループ)
 *   下ローラー : pwm6 / pwm8  (エンコーダ PV3 / PV4 付きの閉ループ)
 *   装填       : pwm9 (装填1) / pwm10 (装填2) … 駆動は main.c。ここではローラー停止時に止めるだけ
 *
 * Lmayu2 == 1 のときだけ回す。上下は Lmayu1 で切り替えるので同時には回らない。
 * 上ローラーの速度は Ltuno1 で選ぶ。
 */
void roller(void) {
    switch (Lmayu2) {
    case 1: // ローラー回転

        if (Lmayu1 == 1) { // 上ローラー

            if (Ltuno1 == 1) { // 200
                motor_control(BAKETU3_ROLLER_SPEED, PV1, 5, 20, 245, &pwm5, &dummy, &rem5);
                motor_control(BAKETU3_ROLLER_SPEED, PV2, 5, 20, 245, &pwm7, &dummy, &rem7);

                motor_control(ROLLER_STOP,PV3, 5, 20,245, &pwm6, &dummy, &rem6);
                motor_control(ROLLER_STOP,PV4, 5, 20,245, &pwm8, &dummy, &rem8);


            }

            else if (Ltuno1 == 0) { // 150

                motor_control(BAKETU2_ROLLER_SPEED, PV1, 5, 20, 245, &pwm5, &dummy, &rem5);
                motor_control(BAKETU2_ROLLER_SPEED, PV2, 5, 20, 245, &pwm7, &dummy, &rem7);

                motor_control(ROLLER_STOP,PV3, 5, 20,245, &pwm6, &dummy, &rem6);
                motor_control(ROLLER_STOP,PV4, 5, 20,245, &pwm8, &dummy, &rem8);



            }

            else if (Ltuno1 == -1) { // 100

                motor_control(BAKETU1_ROLLER_SPEED, PV1, 5, 20, 245, &pwm5, &dummy, &rem5);
                motor_control(BAKETU1_ROLLER_SPEED, PV2, 5, 20, 245, &pwm7, &dummy, &rem7);

                motor_control(ROLLER_STOP,PV3, 5, 20,245, &pwm6, &dummy, &rem6);
                motor_control(ROLLER_STOP,PV4, 5, 20,245, &pwm8, &dummy, &rem8);


            }
        } else if (Lmayu1 == 0) { // 下ローラー

            motor_control(ROLLER_STOP, PV1, 5, 20, 245, &pwm5, &dummy, &rem5);
            motor_control(ROLLER_STOP, PV2, 5, 20, 245, &pwm7, &dummy, &rem7);

            motor_control(ROLLER_SPEED, PV3, 5, 20, 245, &pwm6, &dummy, &rem6);
            motor_control(ROLLER_SPEED, PV4, 5, 20, 245, &pwm8, &dummy, &rem8);


        }

        break;

    case 0: // ローラー停止
        // 装填
        pwm9 = 0;
        pwm10 = 0;

        motor_control(ROLLER_STOP, PV1, 5, 20, 245, &pwm5, &dummy, &rem5);
        motor_control(ROLLER_STOP, PV2, 5, 20, 245, &pwm7, &dummy, &rem7);

        motor_control(ROLLER_STOP, PV3, 5, 20, 245, &pwm6, &dummy, &rem6);
        motor_control(ROLLER_STOP, PV4, 5, 20, 245, &pwm8, &dummy, &rem8);

        break;
    }
}

/*
 * 2 つの Lidar の距離から、壁との距離と平行を保つ仮想スティック値を計算する。
 *   auto_ly … 距離 (平均) の PID。前後移動
 *   auto_rx … 角度 (差分) の PID。旋回
 * 20ms 周期で呼ぶこと (dt が固定)。reset_flag = 1 で積分をリセットして 0 を返す。
 */
void auto_mode(int distance1, int distance2, int reset_flag, int target_dist) {
    typedef struct {
        float Kp;
        float Ki;
        float Kd;
        float prev_error;
        float integral;
    } PID;
    // 距離用(横移動)と角度用(旋回)のPID実体を作成（ゲインは実機で要調整）
    static PID distance = {1.3, 0.008, 0.05, 0, 0};
    static PID angle = {0.6, 0.01, 0.2, 0, 0};
    float target_distance = target_dist; // 目標距離 (mm)
    float dt = 0.02;                     // 20ms周期

    // --- 距離（平均）と角度（差分）の計算 ---
    float current_dist = (distance1 + distance2) / 2.0; // 現在の距離
    float error_dist = current_dist - target_distance;  // 距離のズレ
    float error_angle = distance1 - distance2;          // 角度のズレ

    // --- モード切替時のリセット処理 ---
    // ゲイン(Ki)ではなく積分値(integral)を消すこと。
    // Ki を 0 にすると static なので電源を切るまで I 制御が復活しない。
    if (reset_flag == 1) {
        distance.integral = 0;
        distance.prev_error = error_dist;
        angle.integral = 0;
        angle.prev_error = error_angle;
        auto_ly = 0;
        auto_rx = 0;
        return;
    }

    // --- 1. 距離を保つためのPID（前後移動 auto_ly を計算） ---
    distance.integral += error_dist * dt;
    if (distance.integral > 2000)
        distance.integral = 2000; // 暴走防止
    if (distance.integral < -2000)
        distance.integral = -2000;

    float derivative_dist = (error_dist - distance.prev_error) / dt;

    // ※ 符号は実機の「mae移動がプラスかマイナスか」に合わせて反転させてください
    auto_ly = (int)((distance.Kp * error_dist) + (distance.Ki * distance.integral) + (distance.Kd * derivative_dist));
    distance.prev_error = error_dist;

    // --- 2. 平行にするためのPID（旋回力 rx を計算） ---
    angle.integral += error_angle * dt;
    if (angle.integral > 2000)
        angle.integral = 2000; // 暴走防止
    if (angle.integral < -2000)
        angle.integral = -2000;

    float derivative_angle = (error_angle - angle.prev_error) / dt;

    // ※ 符号は実機の「右旋回がプラスかマイナスか」に合わせて反転させてください
    auto_rx = (int)((angle.Kp * error_angle) + (angle.Ki * angle.integral) + (angle.Kd * derivative_angle));
    angle.prev_error = error_angle;
}

void safety(void) {
    int sbus_error = 0;
    int can_error = 0;
    uint8_t blink_state = (now / 300) % 2;

    /*
     * SBUS断の判定は3つを併用する。
     *   1. last_sbus_rx のタイムアウト … 受信が完全に途絶えた場合。
     *      SBUS_CH も SBUS_LostFrame もフレームが来たときしか更新されないため、
     *      コネクタが抜けると古い値のまま固まる。これが無いと直前のスティック
     *      指令のまま走り続けてしまう。
     *   2. SBUS_Failsafe … 「受信機が送信機を見失った」決定的な信号。
     *      送信機の電源を切っても受信機は正常なフレームを送り続け、このビット
     *      だけを立てるので、1 でも 3 でも捕まえられない。
     *   3. SBUS_LostFrame … 単発のフレーム落ち。
     *   SBUS_CH[0] == 0 は起動直後(まだ1フレームも来ていない)の保険。
     *   HAL_GetTick() がまだ SBUS_TIMEOUT_MS に満たない間は 1 が効かないため。
     */
    if (HAL_GetTick() - last_sbus_rx > SBUS_TIMEOUT_MS || SBUS_Failsafe || SBUS_LostFrame ||
        SBUS_CH[0] == 0) {
        sbus_error = 1;
    } else {
        sbus_error = 0;
    }
    if (HAL_GetTick() - last_can_rx > 100) {
        can_error = 1;
    } else {
        can_error = 0;
    }
    // SBUSの値とCANが来ていない場合、モーターを停止
    if (sbus_error == 1 || can_error == 1) {
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
    // ローラーと足回りが同時に全力で回らないようにする（電源の取り合い対策）
    // 足回り(pwm1〜pwm4)が1つでも回っている間は、ローラー(pwm5〜pwm8)の PWM を
    // 100 (TIM1 の Period 254 に対して約 40%) で頭打ちにする。速度ではなく PWM の上限。
    // motor_simple_control は停止指令のとき必ず 0 まで落とすので、
    // 停止中の足回りを「回っている」と誤判定することはない。
    if (pwm1 > 0 || pwm2 > 0 || pwm3 > 0 || pwm4 > 0) {
        if (pwm5 > 100) {
            pwm5 = 100;
        }
        if (pwm7 > 100) {
            pwm7 = 100;
        }
        if (pwm6 > 100) {
            pwm6 = 100;
        }
        if (pwm8 > 100) {
            pwm8 = 100;
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
    uint8_t green = 0;
    uint8_t blue = 0;
    uint8_t red = 0;

    if (sbus_error == 1) {
        blue = blink_state;
    }
    if (can_error == 1) {
        red = blink_state;
    }
    if (sbus_error == 0 && can_error == 0) {
        green = lidar_timeout() ? blink_state : 1;
    }

    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, green);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, blue);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, red);
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

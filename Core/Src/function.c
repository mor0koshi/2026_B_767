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
#include <stdlib.h>

int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, 10);
  return len;
}

// CAN
void CAN_TX(uint32_t recipient) {
    //送信用インスタンス等
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8];
    //送信メールボックスに空きがあったら送信開始
    if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)) {
        //送信用インスタンスの設定
        TxHeader.StdId = recipient;// 受取手のCANのID
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = 8;//データ長を8byteに設定
        TxHeader.TransmitGlobalTime = DISABLE;
        //各データ
        TxData[0] = 1;
        TxData[1] = 0;
        TxData[2] = 0;
        TxData[3] = 0;
        TxData[4] = 0;
        TxData[5] = 0;
        TxData[6] = 0;
        TxData[7] = 0;
        //CANメッセージを送信
        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox)
!= HAL_OK) {Error_Handler();
        }
    }
}
// RX割り込みコールバック関数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
    CAN_RxHeaderTypeDef RxHeader; // 受信メッセージの情報が格納されるインスタンス
    uint8_t RxData[8];            // 受信したデータを一時保存する配列
    uint32_t id;                  // CANメッセージIDを格納する変数
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        id = RxHeader.StdId; // RxHeaderの中に入っているidを取り出す
        if (id == 0x001 && RxHeader.DLC >= 8) { // idが0x001でデータ長が8以上の場合
            last_can_rx = HAL_GetTick();          // 受信時刻を更新
            for (int i = 0; i <= 7; i++) {
                use_data[i] = RxData[i];
            }
        }
    }
}

/*
 * 速度制御 (積分制御)
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
void motor_control(int SV, int PV, int maxMV, int down_pwm,
                   int max_pwm, int *pwmm, int *dirr, int *remm)
{
    int error = 0;
    int MV = 0;
    int lastMV = 0;
    int pwm = *pwmm;
    int target_dir = 0;
    int dir = *dirr;
    int rem = *remm;          // 前回の端数を復元

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

    rem += error;             // ① 今回の誤差に前回の端数を足す
    MV   = rem / 10;          // ② 10 で割れるぶんだけ操作量にする(Ki=0.1)
    rem -= MV * 10;           // ③ 使ったぶんを引き、端数(|rem| < 10)だけ残す

    if (MV > maxMV) {
        lastMV = maxMV;
    } else if (MV < -maxMV) {
        lastMV = -maxMV;
    } else {
        lastMV = MV;
    }

    // 回転方向が目標と異なる場合、一旦 pwm を 0 まで落としてから方向を変える
    if (SV != 0) {
        if (dir != target_dir) {
            rem = 0;                      // 方向転換中は端数を捨てる
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
        rem = 0;                          // 停止指令中は端数を捨てる
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
// 方向反転を考慮しない簡易版。目標値(SV)に向けて1回の呼び出しごとにstepずつpwmを近づける
void motor_simple_control(int SV, int step, int max_pwm, int *pwmm){
    int pwm = *pwmm;

    if (SV > max_pwm) {
        SV = max_pwm;
    } else if (SV < 0) {
        SV = 0;
    }

    if (pwm < SV) {
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
}

// マジックナンバーを意味のある定数に置き換えます
static const int ROLLER_SPEED = 960;
static const int BAKETU1_ROLLER_SPEED = 450;
static const int BAKETU2_ROLLER_SPEED = 450;
static const int ROLLER_STOP = 0;
static const int ROLLER_SPIN_NORMAL_PWM = 600;
static const int ROLLER_SPIN_REVERSE_PWM = 600;
static const int REVERCE_TIME = 2000; // リセット時に逆転させる時間(ms)

uint32_t time3 = 0;
uint32_t time4 = 0;
uint32_t time5 = 0;
int set_flag1 = 0;
int set_flag2 = 0;
void roller(void){
    switch (Lmayu) {
        case 1:

        if(Ltuno == 1){
            motor_simple_control(ROLLER_SPEED, 20, 970, &pwm5);
            motor_simple_control(ROLLER_SPEED, 20, 970, &pwm6);
            pwm7 = ROLLER_SPIN_NORMAL_PWM;
            roller_dir1 = 1; // 正転
            if(stop_flag1 == 1){
                pwm7 = 0;
                }
            }

            else if(Ltuno == 0){

            motor_control(BAKETU1_ROLLER_SPEED, PV5, 20, 20, 970, &pwm8, &dummy, &rem8);
            motor_control(BAKETU1_ROLLER_SPEED, PV6, 20, 20, 970, &pwm9, &dummy, &rem9);
            pwm10 = ROLLER_SPIN_NORMAL_PWM;
            roller_dir2 = 1; // 正転

            if(stop_flag2 == 1){
                pwm10 = 0;
            }
        }


            else if(Ltuno == -1){
            motor_control(BAKETU2_ROLLER_SPEED, PV5, 20, 20, 970, &pwm8, &dummy, &rem8);
            motor_control(BAKETU2_ROLLER_SPEED, PV6, 20, 20, 970, &pwm9, &dummy, &rem9);
            pwm10 = ROLLER_SPIN_NORMAL_PWM;
            roller_dir2 = 1; // 正転

           if(stop_flag2 == 1){
                pwm10 = 0;
            }
        }
        
        break;

      case 0:
            stop_flag1 = 0;
            stop_flag2 = 0;
            timer_flag = 0;
            pwm7 = 0;
            pwm10 = 0;
        
            if(Ltuno == 1){
            motor_control(ROLLER_STOP, PV5, 20, 20, 970, &pwm8, &dummy, &rem8);
            motor_control(ROLLER_STOP, PV6, 20, 20, 970, &pwm9, &dummy, &rem9);
            motor_simple_control(ROLLER_SPEED, 20, 970, &pwm5);
            motor_simple_control(ROLLER_SPEED, 20, 970, &pwm6);
            }

            else if(Ltuno == 0){
            motor_simple_control(ROLLER_STOP, 20, 970, &pwm5);
            motor_simple_control(ROLLER_STOP, 20, 970, &pwm6);
            motor_control(BAKETU1_ROLLER_SPEED, PV5, 20, 20, 970, &pwm8, &dummy, &rem8);
            motor_control(BAKETU1_ROLLER_SPEED, PV6, 20, 20, 970, &pwm9, &dummy, &rem9);
            }

            else if(Ltuno == -1){
            motor_simple_control(ROLLER_STOP, 20, 970, &pwm5);
            motor_simple_control(ROLLER_STOP, 20, 970, &pwm6);
            motor_control(BAKETU2_ROLLER_SPEED, PV5, 20, 20, 970, &pwm8, &dummy, &rem8);
            motor_control(BAKETU2_ROLLER_SPEED, PV6, 20, 20, 970, &pwm9, &dummy, &rem9);
        }
        break;

      case -1:

            stop_flag1 = 0;
            stop_flag2 = 0;
            timer_flag = 0;
            pwm7 = 0;
            pwm10 = 0;

            motor_simple_control(ROLLER_STOP, 20, 970, &pwm5);
            motor_simple_control(ROLLER_STOP, 20, 970, &pwm6);
            motor_control(ROLLER_STOP, PV5, 20, 20, 970, &pwm8, &dummy, &rem8);
            motor_control(ROLLER_STOP, PV6, 20, 20, 970, &pwm9, &dummy, &rem9);
            break;     
    }
    // set_flag は「今リセット逆転中か」を示す。これが無いと下のif/elseが常に成立し、
    // switch内で立てた正転指令(pwm7/pwm10)を毎周期上書きしてしまう
    if (reset_flag1 == 1 && set_flag1 == 0) {
        reset_flag1 = 0;
        set_flag1 = 1;
        time4 = now;
    }
       if(now - time4 <= REVERCE_TIME && set_flag1 == 1){
        pwm7 = ROLLER_SPIN_REVERSE_PWM;
        roller_dir1 = 0; // 逆転
       }else if(now - time4 > REVERCE_TIME && set_flag1 == 1){
        pwm7 = 0;
        set_flag1 = 0;
       }
    if (reset_flag2 == 1 && set_flag2 == 0) {
        reset_flag2 = 0;
        set_flag2 = 1;
        time5 = now;
    }
       if(now - time5 <= REVERCE_TIME && set_flag2 == 1){
        pwm10 = ROLLER_SPIN_REVERSE_PWM;
        roller_dir2 = 0; // 逆転
       }else if(now - time5 > REVERCE_TIME && set_flag2 == 1){
        pwm10 = 0;
        set_flag2 = 0;
       }



}


void auto_mode(int distance1, int distance2, int reset_flag ,int target_dist) {
  typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
} PID;
// 距離用(横移動)と角度用(旋回)のPID実体を作成（ゲインは実機で要調整）
static PID distance  = {1.3, 0.008, 0.05, 0, 0};
static PID angle = {0.6, 0.01, 0.2, 0, 0};
    float target_distance = target_dist; // 目標距離 (mm)
    float dt = 0.02; // 20ms周期

    // --- 距離（平均）と角度（差分）の計算 ---
    float current_dist = (distance1 + distance2) / 2.0;//現在の距離
    float error_dist = current_dist - target_distance; // 距離のズレ
    float error_angle = distance1 - distance2;        // 角度のズレ

    // --- モード切替時のリセット処理 ---
    // ゲイン(Ki)ではなく積分値(integral)を消すこと。
    // Ki を 0 にすると static なので電源を切るまで I 制御が復活しない。
    if (reset_flag == 1) {
        distance.integral = 0;
        distance.prev_error = error_dist;
        angle.integral = 0;
        angle.prev_error = error_angle;
        auto_ly = 0; auto_rx = 0;
        return;
    }

    // --- 1. 距離を保つためのPID（縦移動力 ly を計算） ---
    distance.integral += error_dist * dt;
    if (distance.integral > 2000) distance.integral = 2000;   // 暴走防止
    if (distance.integral < -2000) distance.integral = -2000;

    float derivative_dist = (error_dist - distance.prev_error) / dt;

    // ※ 符号は実機の「mae移動がプラスかマイナスか」に合わせて反転させてください
    auto_ly = (int)((distance.Kp * error_dist) + (distance.Ki * distance.integral) + (distance.Kd * derivative_dist));
    distance.prev_error = error_dist;

    // --- 2. 平行にするためのPID（旋回力 rx を計算） ---
    angle.integral += error_angle * dt;
    if (angle.integral > 2000) angle.integral = 2000; // 暴走防止
    if (angle.integral < -2000) angle.integral = -2000;

    float derivative_angle = (error_angle - angle.prev_error) / dt;

    // ※ 符号は実機の「右旋回がプラスかマイナスか」に合わせて反転させてください
    auto_rx = (int)((angle.Kp * error_angle) + (angle.Ki * angle.integral) + (angle.Kd * derivative_angle));
    angle.prev_error = error_angle;
}

void safety(void) {
  int sbus_error = 0;
  int can_error = 0;
  uint8_t blink_state = (now / 300) % 2;

  // Failsafe は「受信機が送信機を見失った」決定的な信号なので必ず見る
  if(SBUS_CH[0] == 0 || SBUS_LostFrame){
    sbus_error = 1;
  }else{
    sbus_error = 0;
  }
  if(HAL_GetTick() - last_can_rx > 100){
    can_error = 1;
  }else{
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
    } else if(lidar_timeout()){
        // 操縦はできるが自動モードが使えない状態。緑を点滅させて知らせる
        HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, blink_state);//green
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);//blue
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);//red
    } else {// 全て正常な場合は緑点灯
        HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);//green
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);//blue
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);//red
    }
    //ローラーと足回りが同時に動かないようにする
    if(pwm5 > 0 || pwm6 > 0 || pwm8 > 0 || pwm9 > 0){
      pwm1 = 0;
      pwm2 = 0;
      pwm3 = 0;
      pwm4 = 0;
    }
    if(pwm5 > 0 || pwm6 > 0){
      pwm8 = 0;
      pwm9 = 0;
    }
    if(pwm8 > 0 || pwm9 > 0){
      pwm5 = 0;
      pwm6 = 0;
    }

    if(sbus_error == 1 ){//SBUSが来ていない場合青点滅
      HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin,0);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,blink_state);
    }
    if(can_error == 1){//CANが来ていない場合赤点滅
      HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, blink_state);
}
}

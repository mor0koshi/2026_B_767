#ifndef __ROBOT_LIMITS_H
#define __ROBOT_LIMITS_H

/*
 * 上限値・ランプ時間・タイムアウト時間はすべてここにまとめる。
 *
 * 電源はマキタ 18V (満充電 約21V / 放電末期 約15V) で、12V 用の機器 (RS-555、電磁弁) にも
 * 降圧せずに給電している。そのため 12V 用の機器の電圧は、ここの値を使ってソフトで制限する。
 * 値を変えたら README の「調整用の定数」の表も直すこと。
 *
 * solenoid.c を PC でテストするため、このファイルは HAL に依存させないこと。
 */

/* 制御周期 (ms)。足回り・ローラー・装填のランプは、この周期で 1 段ずつ進む */
#define CONTROL_PERIOD_MS 20

/* ---- バッテリー ---- */
// ADC で電圧を測っていないので、電圧が最も高い満充電時を想定して上限を決める
#define BATTERY_FULL_MV 21000

/* ---- RS-555 (装填1 pwm9 / 装填2 pwm10、TIM3) ---- */
#define RS555_RATED_MV 12000 //12V
#define SOUTEN_PWM_FULL 1000 // TIM3 の Period 999 + 1 = duty 100%
// duty の上限 = 12V / 21V ≒ 57% → 571。装填はこの値で回す
#define SOUTEN_PWM_MAX (SOUTEN_PWM_FULL * RS555_RATED_MV / BATTERY_FULL_MV)
// 0 から SOUTEN_PWM_MAX まで上げる (下げる) のにかける時間。反転は「下げる + 上げる」でこの 2 倍かかる
#define SOUTEN_RAMP_MS 200//ms
// 1 周期あたりの変化量。切り上げて、ランプが SOUTEN_RAMP_MS より長くならないようにする
#define SOUTEN_RAMP_STEP ((SOUTEN_PWM_MAX * CONTROL_PERIOD_MS + SOUTEN_RAMP_MS - 1) / SOUTEN_RAMP_MS)

/* ---- RZ-735VA (上下ローラー pwm5〜pwm8、TIM1 Period 254) ---- */
#define ROLLER_PWM_MAX 160          // PWM の上限
#define ROLLER_STEP_UP 12           // 1 周期に PWM を上げる最大量 (motor_control の maxMV)
#define ROLLER_STEP_DOWN 20         // 停止・反転のとき 1 周期に PWM を下げる量 (motor_control の down_pwm)
#define ROLLER_PWM_WHILE_DRIVING 50 // 足回りが回っている間の PWM の上限

/* ---- 電磁弁 (lock1 / lock2、コガネイ 110 シリーズ DC12V 品) ---- */
// 12V 品を 18V 系統で駆動するので、ON にしておける時間をここまでにする。1000 未満にすること
#define SOLENOID_MAX_ON_MS 800

/* ---- 通信タイムアウト ---- */
// 最後に SBUS フレームをデコードしてから、何 ms 経ったら受信断とみなすか。
// SBUS は 14ms (ハイスピードなら 7ms) 周期なので、100ms は約 7 フレーム分の猶予
#define SBUS_TIMEOUT_MS 200
// 最後に CAN を受信してから、何 ms 経ったら CAN 断とみなすか
#define CAN_TIMEOUT_MS 100

#endif /* __ROBOT_LIMITS_H */

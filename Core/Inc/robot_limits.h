#ifndef __ROBOT_LIMITS_H
#define __ROBOT_LIMITS_H

/*
 * 上限値・ランプ時間・タイムアウト時間はすべてここにまとめる。
 *
 * 電源はマキタ 18V (満充電 約21V / 放電末期 約15V) で、12V 用の RS-555 にも降圧せずに給電している。
 * そのため RS-555 の電圧は、ここの値を使ってソフトで制限する (電磁弁は降圧回路で 12V なので制限しない)。
 * 値を変えたら README の「調整用の定数」の表も直すこと。
 *
 * 定数だけのファイルなので、HAL に依存させないこと (PC 上のテストでもそのまま使うため)。
 */

/* 制御周期 (ms)。足回り・ローラー・装填のランプは、この周期で 1 段ずつ進む */
#define CONTROL_PERIOD_MS 20

/* ---- バッテリー ---- */
// ADC で電圧を測っていないので、電圧が最も高い満充電時を想定して上限を決める
#define BATTERY_FULL_MV 21000

/* ---- 足回り (pwm1〜pwm4、TIM4 Period 999。PWM 1000 で duty 100%) ---- */
// スティックの値 (±1000) にかける倍率 (%)。1 軸を倒しきったとき 1000 × この値 / 100 になる
#define DRIVE_SCALE_PERCENT 70
// PWM の上限。DRIVE_SCALE_PERCENT を上げるときはこれも上げないと、ここで頭打ちになって速くならない
#define DRIVE_PWM_MAX 700
// 20ms ごとに PWM を目標値へ近づける量。大きいほど加速・減速が速い (40 なら 0 → 700 が 360ms)
#define DRIVE_STEP 40
#define DRIVE_SLOW_DIST_MM 800 // Lidar のどちらかがこの距離 (mm) 以下なら足回りを遅くする
#define DRIVE_SLOW_PERCENT 40  // 遅くするときの倍率 (%)。40 = 0.4 倍

/* ---- RS-555 (装填1 pwm9 / 装填2 pwm10、TIM3) ---- */
#define RS555_RATED_MV 13000 //12V
#define SOUTEN_PWM_FULL 1000 // TIM3 の Period 999 + 1 = duty 100%
// duty の上限 = 12V / 21V ≒ 57% → 571。装填はこの値で回す
#define SOUTEN_PWM_MAX (SOUTEN_PWM_FULL * RS555_RATED_MV / BATTERY_FULL_MV)
// 0 から SOUTEN_PWM_MAX まで上げる (下げる) のにかける時間。反転は「下げる + 上げる」でこの 2 倍かかる
#define SOUTEN_RAMP_MS 200//ms
// 1 周期あたりの変化量。切り上げて、ランプが SOUTEN_RAMP_MS より長くならないようにする
#define SOUTEN_RAMP_STEP ((SOUTEN_PWM_MAX * CONTROL_PERIOD_MS + SOUTEN_RAMP_MS - 1) / SOUTEN_RAMP_MS)
// 装填モーターが同じ向きにこの時間 (ms) 以上回り続けたら、リミットスイッチの故障などとみなして止める
#define SOUTEN_TIMEOUT_MS 4000

/* ---- RZ-735VA (上下ローラー pwm5〜pwm8、TIM1 Period 254) ---- */
#define ROLLER_PWM_MAX 250          // PWM の上限
#define ROLLER_STEP_UP 12           // 1 周期に PWM を上げる最大量 (motor_control の maxMV)
#define ROLLER_STEP_DOWN 20         // 停止・反転のとき 1 周期に PWM を下げる量 (motor_control の down_pwm)
#define ROLLER_PWM_WHILE_DRIVING 85 // 足回りが回っている間の PWM の上限

/* ---- ローラーの目標速度 (エンコーダ値 PV と同じ 0〜255 系。254 以下にすること) ---- */
#define LOWER_ROLLER_SPEED 254   // 下ローラー
#define BAKETU1_ROLLER_SPEED 130 // 上ローラー Ltuno1 == -1 長押し
#define BAKETU2_ROLLER_SPEED 60  // 上ローラー Ltuno1 == 0 PS
#define BAKETU3_ROLLER_SPEED 75  // 上ローラー Ltuno1 == 1 旗
// BAKETU1 を RY スティックで上げ下げできる幅。倒しきると ±この値。BAKETU1_ROLLER_SPEED + この値も 254 以下にすること
#define BAKETU1_RY_RANGE 50
// 目標速度との差がこれ以内なら「目標速度に達した」とみなす (LED テープの点滅)
#define ROLLER_READY_TOLERANCE 5

/* ---- 通信タイムアウト ---- */
// 最後に SBUS フレームをデコードしてから、何 ms 経ったら受信断とみなすか。
// SBUS は 14ms (ハイスピードなら 7ms) 周期なので、100ms は約 7 フレーム分の猶予
#define SBUS_TIMEOUT_MS 200
// 最後に CAN を受信してから、何 ms 経ったら CAN 断とみなすか
#define CAN_TIMEOUT_MS 100

#endif /* __ROBOT_LIMITS_H */

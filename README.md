# 2026_B_767

STM32F767ZI をベースにした、4輪オムニ駆動ロボットの制御ファームウェアです。プロポの SBUS 入力で走行し、上下2段のローラーと装填機構で球を撃ち出します。2基の Lidar で壁との距離と平行を保つ自動走行モードもあります。

## 概要

- **駆動**: 4輪オムニ（TIM4、エンコーダなしのランプ制御）
- **ローラー**: 上ローラー2個＋下ローラー2個（TIM1、エンコーダ付きの閉ループ速度制御）
- **装填**: 装填モーター2個（TIM3）＋リミットスイッチによる原点復帰
- **電磁弁**: 2系統（`lock1` / `lock2`）
- **操作入力**: SBUS（UART5、100000bps / 偶数パリティ / ストップビット2 / 信号反転）
- **フィードバック**: CAN1 で ID `0x001` の8バイトを受信し、ローラーのエンコーダ値として使用
- **センサー**: PONO TSD20 単点 ToF Lidar ×2（UART4・UART7、460800bps、循環DMA受信）
- **安全機構**: SBUS断線・送信機OFF・CAN断の検出による全停止、Lidar途絶時の自動モード禁止、走行中のローラー出力制限、ステータスLED

## 制御ループ

メインループ（`main.c`）は次の関数を順番に呼ぶだけです。足回りとローラーだけ20ms周期で、それ以外は毎周回実行します。

```c
PV1〜PV4 = use_data[0..3];  // CAN で受け取ったローラーのエンコーダ値
sbus();                     // スイッチとスティックを読む
lidar();                    // Lidar の距離を更新
if (20ms 経過) {
    asimawari();            // 足回り
    roller();               // ローラー
}
loader();                   // 電磁弁と装填（原点復帰を含む）
safety();                   // 異常時の停止、走行中のローラー制限、LED
motor_outputs();            // PWM と DIR を出力
```

`safety()` は PWM を出力する直前に呼ぶので、どのモードで計算された値にも必ず安全処理がかかります。

## ファイル構成（`Core/Src` / `Core/Inc`）

| ファイル | 中身 |
|---|---|
| `main.c` | 共有変数の定義、初期化、メインループ（CubeMX 生成） |
| `function.c` / `.h` | 足回り（`asimawari`）、ローラー（`roller`）、装填（`loader`）、Lidar PID（`auto_mode`）、リミットスイッチ、PWM/DIR 出力（`motor_outputs`） |
| `motor_control.c` / `.h` | モーター1個分の制御（`motor_control`、`motor_simple_control`） |
| `safety.c` / `.h` | 安全機能（`safety`）とステータス LED |
| `can_handler.c` / `.h` | CAN の送受信 |
| `sbus_handler.c` / `.h` | SBUS の受信・デコードとスイッチ/スティックの変換 |
| `lidar_sensor.c` / `.h` | Lidar のフレーム解析とタイムアウト判定 |

## SBUS チャンネル割り当て

| CH | 変数 | 種類 | 用途 |
|---|---|---|---|
| 0 | `rx` | スティック | 旋回 |
| 1 | `ly` | スティック | 前後 |
| 2 | `ry` | スティック | 未使用（値の計算のみ） |
| 3 | `lx` | スティック | 左右（横移動） |
| 4 | `Lmayu1` | 2段 | ローラー選択（1=上ローラー / 0=下ローラー） |
| 5 | `Lmayu2` | 2段 | ローラー回転（1=回す / 0=止める） |
| 6 | `Rmayu1` | 3段 | 走行モード |
| 7 | `Rmayu2` | 2段 | 電磁弁の選択 |
| 8 | `Ltuno1` | 3段 | 上ローラーの速度 |
| 9 | `Rtuno2` | 2段 | 発射（1=打つ / 0=打たない） |

- **スティック**（`process_stick`）: 368〜1680 を ±1000 に変換します。中央付近（1000〜1050）は1024に丸め、±2 以内は0にします。
- **2段スイッチ**（`get_switch_state2`）: 1100 より大きければ 1、それ以外は 0。
- **3段スイッチ**（`get_switch_state3`）: 1100 より大きければ -1、300 未満なら 1、それ以外は 0。

## 走行モード（`Rmayu1`）

| `Rmayu1` | モード | 前後 | 左右 | 旋回 |
|---|---|---|---|---|
| `-1` | 手動 | `ly` | `lx` | `rx` |
| `0` | 半自動 | `ly` | `lx` | Lidar PID の `auto_rx` |
| `1` | 全自動 | Lidar PID の `auto_ly` | `lx` | Lidar PID の `auto_rx` |

- **半自動**: 前後と左右は手で操作し、旋回だけ PID が自動で補正して壁と平行を保ちます。
- **全自動**: 壁からの距離 `AUTO_TARGET_DIST_MM`（1770mm）と平行を PID が保ちます。横移動だけ手で操作します。
- **Lidar 異常時**: どちらかの Lidar の有効な測定値が100ms以上途絶えると、`Rmayu1` の値によらず手動モードで動きます。復帰すれば元のモードに戻ります。

### オムニの混合式

`asimawari()` はモードごとに「前後・左右・旋回」の3つを決め、`omni_mix()` で4輪の指令値に変換します。混合式は `omni_mix()` の1箇所だけなので、式を変えるときもここだけ直せば全モードに反映されます。

```c
taiya[0] = -forward + strafe + turn;  // 左前 (pwm1)
taiya[1] = -forward - strafe + turn;  // 右前 (pwm2)
taiya[2] =  forward - strafe + turn;  // 左後 (pwm3)
taiya[3] =  forward + strafe + turn;  // 右後 (pwm4)
```

| モード | forward | strafe | turn |
|---|---|---|---|
| 手動 | `ly` | `lx` | `-rx`（そのあと全体を ×0.9。1軸を倒しきると `maxpwm` の 900 ちょうど） |
| 半自動 | `ly` | `lx` | `auto_rx` |
| 全自動 | `-auto_ly` | `lx` | `auto_rx` |

`auto_ly` が `ly` と逆符号なのは、PID 出力の符号を実機に合わせているためです。

足回りは `motor_simple_control` で、20ms ごとに最大 80 ずつ目標値に近づけます。上限は `maxpwm`（900、デューティ90%）です。

## ローラー（`Lmayu2` / `Lmayu1` / `Ltuno1`）

`Lmayu2 == 1` のときだけ回ります。上下は `Lmayu1` で切り替えるため、同時には回りません。

| `Lmayu1` | 回るローラー | 目標速度 |
|---|---|---|
| `1` | 上ローラー（pwm5 / pwm7） | `Ltuno1` で選択: `1`→200、`0`→150、`-1`→100 |
| `0` | 下ローラー（pwm6 / pwm8） | 245（`ROLLER_SPEED`） |

ローラーは `motor_control` で速度を閉ループ制御します。フィードバックは CAN で受け取る `PV1`〜`PV4` で、上ローラーが PV1 / PV2、下ローラーが PV3 / PV4 です。PWM の上限は 250（TIM1 の Period は 254）です。目標速度（最大245）に届かないときに、少し余裕を持って PWM を上げられるようにしています。

## 発射と装填（`Rtuno2`）

| 条件 | 動作 |
|---|---|
| `Rtuno2 == 0` | 電磁弁を両方オフにし、装填モーターを止める |
| `Rtuno2 == 1` かつ ローラー停止中 | `Rmayu2` で電磁弁を選ぶ（`0`→lock1、`1`→lock2） |
| `Rtuno2 == 1` かつ 上ローラー回転中 | 装填2（pwm10）を正転 600 |
| `Rtuno2 == 1` かつ 下ローラー回転中 | 装填1（pwm9）を正転 600 |

### 装填の原点復帰

リミットスイッチで装填モーターを逆転させ、原点まで戻します。

| 装填 | 復帰を始めるスイッチ | 原点スイッチ | 復帰中の出力 |
|---|---|---|---|
| 装填1（pwm9） | lock6 | lock7 | 逆転 600 |
| 装填2（pwm10） | lock8 | lock9 | 逆転 600 |

復帰フラグ（`reset_flag1` / `reset_flag2`）は、原点スイッチを踏むまで立ったままです。PWM のノイズでフラグが誤って立たないよう、スイッチは `limit_read()` で20ms同じ値が続いたときだけ確定させています。

## 安全機構（`safety()`）

### 全停止

次のどれかに当てはまると、pwm1〜pwm10 をすべて 0 にします。

| 異常 | 判定 |
|---|---|
| SBUS の受信が途絶えた（断線、受信機の電源断など） | 最後にフレームをデコードしてから `SBUS_TIMEOUT_MS`（100ms）経過 |
| 送信機の電源が切れた | `SBUS_Failsafe` |
| フレーム落ち | `SBUS_LostFrame` |
| 起動後まだ SBUS を受信していない | `SBUS_CH[0] == 0` |
| CAN が途絶えた | 最後の受信から100ms経過 |

`SBUS_CH` と `SBUS_LostFrame` はフレームが届いたときしか更新されません。そのため受信が完全に止まったときは、タイムアウトでしか検出できません。

### 走行中のローラー制限

足回り（pwm1〜pwm4）のどれか1つでも回っている間は、ローラー（pwm5〜pwm8）の PWM を 100 で頭打ちにします。足回りとローラーが同時に全力で電流を引かないようにするためです。

### ステータス LED

| LED | 状態 |
|---|---|
| 緑点灯 | 正常 |
| 緑点滅 | Lidar 途絶（手動操縦のみ可） |
| 青点滅 | SBUS 断 |
| 赤点滅 | CAN 断 |
| 紫点滅 | SBUS と CAN の両方が断 |

## モーターと基板チャンネルの対応

基板（2026_B_main）は PWMn と DIRn が同じドライバにつながっています。そのため DIR には、ソフト上のモーター番号ではなく、**その PWM が出ている基板チャンネルの DIR** を書きます。

| モーター | 用途 | タイマー | 基板ch | PWM ピン | DIR | DIR ピン | DIR の値 |
|---|---|---|---|---|---|---|---|
| pwm1 | 足回り 左前 | TIM4 CH4 | 3 | PD15 | d3 | PE10 | `dir1` |
| pwm2 | 足回り 右前 | TIM4 CH3 | 4 | PD14 | d4 | PD11 | `dir2` |
| pwm3 | 足回り 左後 | TIM4 CH1 | 1 | PD12 | d1 | PB1 | `dir3` |
| pwm4 | 足回り 右後 | TIM4 CH2 | 2 | PD13 | d2 | PB2 | `dir4` |
| pwm5 | 上ローラー | TIM1 CH2 | 7 | PE11 | d7 | PF12 | 0 固定 |
| pwm7 | 上ローラー | TIM1 CH3 | 5 | PE13 | d5 | PF3 | 1 固定 |
| pwm6 | 下ローラー | TIM1 CH1 | 8 | PE9 | d8 | PF13 | 0 固定 |
| pwm8 | 下ローラー | TIM1 CH4 | 6 | PE14 | d6 | PF14 | 1 固定 |
| pwm9 | 装填1 | TIM3 CH2 | 10 | PC7 | d10 | PA11 | `roller_dir1` |
| pwm10 | 装填2 | TIM3 CH1 | 9 | PC6 | d9 | PA12 | `roller_dir2` |

ローラーは上下とも、向かい合う2個が逆向きに回るよう DIR を 0 と 1 にしています。

### PWM のスケール

タイマーごとに Period が違うので、PWM に入れる値の範囲も違います。

| 系統 | タイマー | Period | 使っている上限 |
|---|---|---|---|
| 足回り | TIM4 | 999 | 900（`maxpwm`） |
| ローラー | TIM1 | 254 | 250 |
| 装填 | TIM3 | 999 | 600 |

**Period を超える値を入れると、常に 100% デューティになります。** 値やタイマー設定を変えるときは、この表と合っているか確認してください。

## 主要関数

| 関数 | ファイル | 説明 |
|---|---|---|
| `motor_control(SV, PV, maxMV, down_pwm, max_pwm, *pwm, *dir, *rem)` | motor_control.c | エンコーダ付きの速度制御（ローラー用）。誤差の1/10を毎周期 PWM に足し込む積分制御で、1周期の変化量は `maxMV` までです。`rem` は整数除算の端数の繰り越しで、誤差が小さい領域の不感帯をなくします。方向転換と停止のときは `down_pwm` ずつ PWM を落とします。 |
| `motor_simple_control(SV, step, max_pwm, *pwm, *dir)` | motor_control.c | エンコーダなしのランプ制御（足回り用）。`SV` の符号が回転方向です。PWM を `step` ずつ目標に近づけ、方向転換のときは一度 0 まで落としてから向きを変えます。 |
| `asimawari()` | function.c | 走行モードに応じて前後・左右・旋回を決め、足回り4輪を動かします。 |
| `omni_mix(forward, strafe, turn, taiya)` | function.c | オムニの混合式。全モード共通です。 |
| `roller()` | function.c | `Lmayu2` / `Lmayu1` / `Ltuno1` に応じてローラー4個の目標速度を決めます。 |
| `loader()` | function.c | 電磁弁と装填モーター。リミットスイッチによる原点復帰もここで行います。 |
| `motor_outputs()` | function.c | PWM と DIR をまとめて出力します。 |
| `auto_mode(d1, d2, reset, target)` | function.c | 2つの Lidar 距離から、距離を保つ PID（`auto_ly`）と平行を保つ PID（`auto_rx`）を計算します。`reset=1` で積分値をリセットします。 |
| `safety()` | safety.c | 異常時の全停止、走行中のローラー制限、LED 表示。 |
| `limit_read(sw)` | function.c | リミットスイッチを読みます。20ms 同じ値が続いたときだけ確定値を更新します。 |
| `HAL_CAN_RxFifo0MsgPendingCallback` | can_handler.c | CAN 受信割り込み。ID `0x001`、DLC 8以上のフレームを `use_data[]` に格納します。 |
| `CAN_TX(id)` | can_handler.c | CAN 送信。現在はどこからも呼ばれていません。 |
| `sbus()` | sbus_handler.c | スイッチとスティックを読みます。 |
| `SBUS_Process()` | sbus_handler.c | SBUS フレームを16チャンネルにデコードし、受信時刻を記録します。 |
| `lidar()` | lidar_sensor.c | DMA リングバッファから TSD20 の4バイトフレーム（`5C` / 距離下位 / 距離上位 / チェックサム）を取り出します。チェックサムが合わなければ1バイト進めて同期し直します。 |
| `lidar_timeout()` | lidar_sensor.c | どちらかの Lidar の有効な測定値が100ms以上途絶えていれば 1 を返します。 |

## 調整用の定数

| 定数 | 場所 | 値 | 内容 |
|---|---|---|---|
| `maxpwm` | main.c | 900 | 足回りの PWM 上限 |
| `ROLLER_SPEED` | function.c | 245 | 下ローラーの目標速度 |
| `BAKETU1〜3_ROLLER_SPEED` | function.c | 100 / 150 / 200 | 上ローラーの目標速度 |
| `AUTO_TARGET_DIST_MM` | lidar_sensor.h | 1770 | 全自動モードで保つ壁からの距離 (mm) |
| `LIDAR_OFFSET4` / `LIDAR_OFFSET7` | lidar_sensor.h | 11 / 38 | Lidar の取り付け位置の補正 (mm) |
| `LIDAR_TIMEOUT_MS` | lidar_sensor.h | 100 | Lidar 途絶と判定するまでの時間 |
| `SBUS_TIMEOUT_MS` | sbus_handler.h | 100 | SBUS 断と判定するまでの時間 |
| `LIMIT_DEBOUNCE_MS` | function.c | 20 | リミットスイッチのノイズ除去時間 |
| `ROLLER_PWM_WHILE_DRIVING` | safety.c | 100 | 走行中のローラーの PWM 上限 |
| PID ゲイン | function.c `auto_mode()` | 距離 1.3 / 0.008 / 0.05、角度 0.6 / 0.01 / 0.2 | Kp / Ki / Kd |

`auto_mode()` の PID 出力の符号は、実機に合わせて調整する前提です。

## ハードウェア構成（.ioc より）

- MCU: STM32F767ZITx（LQFP144）
- CAN1、UART4 / UART5 / UART7（DMA 受信）、USART2 / USART3 / UART8、TIM1・TIM3・TIM4（各4ch PWM）
- USART3 は `printf` のデバッグ出力先

## ビルド

CMake プロジェクトです（STM32CubeMX で生成）。

```sh
cmake --preset Debug
cmake --build build/Debug
```

出力は `build/Debug/2026_B_767.elf` です。

自作の `.c` ファイルを増やしたときは、`CMakeLists.txt` の `target_sources`（`# Add user sources here` の下）に追加してください。追加しないと、リンク時に関数が見つからないエラーになります。

# 2026_B_767

STM32F767ZI をベースにした、4輪オムニ駆動ロボットの制御ファームウェアです。プロポの SBUS 入力で走行し、上下2段のローラーと装填機構、または電磁弁で球を撃ち出します。2基の Lidar で壁との距離と平行を保つ自動走行モードもあります。

## 概要

- **駆動**: 4輪オムニ（TIM4、エンコーダなしのランプ制御）
- **ローラー**: 上ローラー2個＋下ローラー2個（RZ-735VA、TIM1、エンコーダ付きの閉ループ速度制御）
- **装填**: 装填モーター2個（RS-555、TIM3）＋リミットスイッチによる原点復帰
- **電磁弁**: 2系統（`lock1` / `lock2`、コガネイ 110 シリーズ DC12V 品）
- **電源**: マキタ 18V バッテリー（満充電 約21V）。12V 用の RS-555 と電磁弁も降圧せずにつなぐので、ソフトで電圧を制限する（[電源と電圧の制限](#電源と電圧の制限)）
- **操作入力**: SBUS（UART5、100000bps / 偶数パリティ / ストップビット2 / 信号反転）
- **フィードバック**: CAN1（1Mbps）で ID `0x001` の8バイトを受信し、ローラーのエンコーダ値として使う
- **センサー**: PONO TSD20 単点 ToF Lidar ×2（UART4・UART7、460800bps、循環DMA受信）
- **安全機構**: SBUS断線・送信機OFF・CAN断の検出による全停止と電磁弁の閉鎖、HardFault 時の全停止、Lidar途絶時の自動モード禁止、走行中のローラー出力制限、装填の電圧制限、ステータスLED
- **表示**: 基板の LED（LD1〜LD3）で状態、LED テープ（lock3〜lock5）でチームの色とローラーの準備完了を表示

## 名前の読み方

コードにはローマ字や略語の名前が多いので、先にまとめておきます。

| 名前 | 意味 |
|---|---|
| `asimawari` | 足回り |
| `souten` | 装填（装填モーター。`souten1` = 装填1、`souten2` = 装填2） |
| `taiya` | タイヤ（オムニ4輪それぞれの指令値） |
| `roller` | ローラー（発射用。上下2段） |
| `valve` | 電磁弁 |
| `Lmayu1` / `Lmayu2` / `Rmayu1` / `Rmayu2` / `Ltuno1` / `Rtuno2` | プロポのスイッチ（L = 左、R = 右）。割り当ては [SBUS チャンネル割り当て](#sbus-チャンネル割り当て) |
| `BAKETU1`〜`3` | 上ローラーの速度プリセット（1 = 長押し、2 = PS、3 = 旗） |
| `SV` / `PV` / `MV` | 目標値 / 実際の値（エンコーダ）/ 操作量 |
| `rem` | 整数の割り算で切り捨てた端数の繰り越し（ローラーの速度制御用） |
| `pwmN` / `dirN` | モーター N の PWM 値 / 回転方向 |
| `dN` / `mN` | 基板の DIR ピン / PWM ピンの名前 |
| `lockN` | 基板の汎用 GPIO の名前（電磁弁、LED テープ、リミットスイッチに使用） |
| `ramp` | ランプ。値をいきなり変えず、坂のように少しずつ変えること（ソフトスタート） |

## 起動の流れ

`main()` は次の順番で起動します。

1. HAL・クロック（SYSCLK 120MHz）・各ペリフェラルを初期化する。このとき全出力ピンは Low（電磁弁 OFF）から始まる
2. CAN を開始し、受信割り込みを有効にする
3. TIM4・TIM1・TIM3 の PWM を全チャンネル開始する（最初は duty 0）
4. SBUS の受信を開始する
5. 100ms 待ってから、Lidar 2台に測定開始コマンド（`5A 0A 02 02 00 F1`）を送り、循環 DMA で受信を始める
6. メインループに入る

電磁弁は起動時に LOCKOUT 状態から始まるので、撃つスイッチが ON のまま電源を入れても、一度 OFF にするまで開きません。

## 制御ループ

メインループ（`main.c`）は次の関数を順番に呼ぶだけです。足回り・ローラー・装填のランプだけ20ms周期（`CONTROL_PERIOD_MS`）で、それ以外は毎周回実行します。

```c
PV1 = use_data[0];  PV2 = use_data[1];  // CAN で受け取ったローラーのエンコーダ値
PV3 = use_data[4];  PV4 = use_data[3];  // （割り当ては「CAN 受信データ」を参照）
sbus();                     // スイッチとスティックを読む
lidar();                    // Lidar の距離を更新
if (20ms 経過) {
    asimawari();            // 足回り
    roller();               // ローラー
    souten_ramp();          // 装填モーターの PWM をランプで目標へ近づける
    printf(...);            // デバッグ出力（Lidar の距離）
}
souten();                   // 電磁弁と装填の指令（原点復帰とリミットでの即停止を含む）
safety();                   // 異常時の停止と電磁弁の閉鎖、走行中のローラー制限、装填の上限、LED
motor_outputs();            // PWM・DIR・電磁弁を出力
```

**計算する関数（`asimawari` / `roller` / `souten` など）は変数（`pwmN`、`valveN_on`）に書くだけで、ピンに出すのは最後の `motor_outputs()` だけです。** そのため、出力の直前に呼ぶ `safety()` が、どのモードで計算された値にも必ず安全処理をかけられます。`safety()` より後で `pwm` を書き換えないでください。

## ファイル構成（`Core/Src` / `Core/Inc`）

| ファイル | 中身 |
|---|---|
| `main.c` | 共有変数の定義、初期化、メインループ（CubeMX 生成） |
| `function.c` / `.h` | 足回り（`asimawari`）、ローラー（`roller`）、装填と電磁弁（`souten` / `souten_ramp`）、Lidar PID（`auto_mode`）、リミットスイッチ（`limit_read`）、出力（`motor_outputs` / `outputs_all_off`） |
| `motor_control.c` / `.h` | モーター1個分の制御（`motor_control`、`motor_simple_control`） |
| `solenoid.c` / `.h` | 電磁弁1個分の ON 時間の制限（HAL を使わないので PC でもテストできる） |
| `robot_limits.h` | 上限値・ランプ時間・タイムアウト時間の定数をまとめたヘッダ |
| `safety.c` / `.h` | 安全機能（`safety`）、ステータス LED、LED テープ |
| `can_handler.c` / `.h` | CAN の送受信 |
| `sbus_handler.c` / `.h` | SBUS の受信・デコードとスイッチ/スティックの変換 |
| `lidar_sensor.c` / `.h` | Lidar のフレーム解析とタイムアウト判定 |
| `stm32f7xx_it.c` | 割り込みハンドラ（CubeMX 生成）。`HardFault_Handler` で全出力を止める |

## SBUS チャンネル割り当て

| CH | 変数 | 種類 | 用途 |
|---|---|---|---|
| 0 | `rx` | スティック | 旋回 |
| 1 | `ly` | スティック | 前後 |
| 2 | `ry` | スティック | 上ローラーの速度調整（`Ltuno1 == -1` のときだけ） |
| 3 | `lx` | スティック | 左右（横移動） |
| 4 | `Lmayu1` | 2段 | ローラー選択（1=上ローラー / 0=下ローラー） |
| 5 | `Lmayu2` | 2段 | ローラー回転（1=回す / 0=止める） |
| 6 | `Rmayu1` | 3段 | 走行モード（-1=手動 / 0=半自動 / 1=全自動） |
| 7 | `Rmayu2` | 2段 | 電磁弁の選択（0=lock1 / 1=lock2） |
| 8 | `Ltuno1` | 3段 | 上ローラーの速度（1=BAKETU3 / 0=BAKETU2 / -1=BAKETU1） |
| 9 | `Rtuno2` | 2段 | 発射（1=撃つ / 0=撃たない） |

- **スティック**（`process_stick`）: 368〜1680 を ±1000 に変換します。中央付近（1000〜1050）は1024に丸めてちょうど 0 にし、変換後 ±2 以内も0にします（デッドバンド）。
- **2段スイッチ**（`get_switch_state2`）: 1100 より大きければ 1、それ以外は 0。
- **3段スイッチ**（`get_switch_state3`）: 1100 より大きければ -1、300 未満なら 1、それ以外は 0。

SBUS は `HAL_UARTEx_ReceiveToIdle_DMA` で受信し、ヘッダ `0x0F` から25バイトを1フレームとして16チャンネルにデコードします（`SBUS_Process`）。デコードできた時刻を `last_sbus_rx` に記録し、断線の判定に使います。

## 走行モード（`Rmayu1`）

| `Rmayu1` | モード | 前後 | 左右 | 旋回 |
|---|---|---|---|---|
| `-1` | 手動 | `ly` | `lx` | `rx` |
| `0` | 半自動 | `ly` | `lx` | Lidar PID の `auto_rx` |
| `1` | 全自動 | Lidar PID の `auto_ly` | `lx` | Lidar PID の `auto_rx` |

- **半自動**: 前後と左右は手で操作し、旋回だけ PID が自動で補正して壁と平行を保ちます。
- **全自動**: 壁からの距離 `AUTO_TARGET_DIST_MM`（1770mm）と平行を PID が保ちます。横移動だけ手で操作します。
- **Lidar 異常時**: どちらかの Lidar の有効な測定値が `LIDAR_TIMEOUT_MS`（100ms）以上途絶えると、`Rmayu1` の値によらず手動モードで動きます。復帰すれば元のモードに戻ります。途絶えたまま自動モードにすると「壁まで遠すぎる」と誤認して走り続けるためです。
- 手動モードの間も、裏で PID の積分をリセットしています。自動モードに切り替えた瞬間に、たまった積分で急に動かないようにするためです。

### Lidar PID（`auto_mode`）

2つの Lidar の距離 `d4` / `d7`（取り付け位置の補正 `LIDAR_OFFSET4` / `LIDAR_OFFSET7` を引いた値）から、仮想スティック値を2つ計算します。

| 出力 | 誤差 | 意味 | ゲイン（Kp / Ki / Kd） |
|---|---|---|---|
| `auto_ly` | 平均距離 − 目標距離 | 壁との距離を保つ（前後） | 1.3 / 0.008 / 0.05 |
| `auto_rx` | `d4` − `d7` | 壁と平行を保つ（旋回） | 0.6 / 0.01 / 0.2 |

20ms 周期（`PID_DT = 0.02`）が前提です。積分は ±2000 で頭打ちにしています。PID 出力の符号は実機に合わせて調整する前提です（`auto_ly` を `ly` と逆符号で使っているのはそのため）。

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
| 手動 | `ly` | `lx` | `-rx`（そのあと全体を ×0.5。1軸だけ倒しきると 500） |
| 半自動 | `ly` | `lx` | `auto_rx` |
| 全自動 | `-auto_ly` | `lx` | `auto_rx` |

足回りは `motor_simple_control` で、20ms ごとに `DRIVE_STEP`（40）ずつ目標値に近づけます（0 から 600 まで 300ms）。上限は `maxpwm`（600、duty 60%）で、2軸を同時に倒したときなどはここで頭打ちになります。方向転換のときは一度 0 まで落としてから向きを変えます。

## ローラー（`Lmayu2` / `Lmayu1` / `Ltuno1`）

`Lmayu2 == 1` のときだけ回ります。上下は `Lmayu1` で切り替えるため、同時には回りません。

| `Lmayu1` | 回るローラー | 目標速度 |
|---|---|---|
| `1` | 上ローラー（pwm5 / pwm7） | `Ltuno1` で選択: `1`→76（BAKETU3）、`0`→84（BAKETU2）、`-1`→150（BAKETU1、`ry` で ±50） |
| `0` | 下ローラー（pwm6 / pwm8） | 245（`ROLLER_SPEED`） |

`Ltuno1 == -1`（BAKETU1）のときだけ、`ry` スティックの位置で目標速度を変えられます。中央で 150、上に倒しきると 200、下に倒しきると 100 で、その間は比例します（幅は `BAKETU1_RY_RANGE`）。

ローラーは `motor_control` で速度を閉ループ制御します。フィードバックは CAN で受け取る回転数で、上ローラーが `PV1`（pwm5）/ `PV2`（pwm7）、下ローラーが `PV3`（pwm6）/ `PV4`（pwm8）です。

- PWM の上限は `ROLLER_PWM_MAX`（160）です。
- 1周期に上げる量は最大 `ROLLER_STEP_UP`（12）なので、始動はソフトスタートになります。
- 止めるときは、1周期に `ROLLER_STEP_DOWN`（20）ずつ下げて減速します。

回している方の2個が両方とも目標速度 ±`ROLLER_READY_TOLERANCE`（5）以内に入ると、`roller_ready` が 1 になり、LED テープが点滅します（撃ってよい合図）。

## 発射と装填（`Rtuno2`）

| 条件 | 動作 |
|---|---|
| `Rtuno2 == 0` | 電磁弁を両方閉じ、装填モーターを止める |
| `Rtuno2 == 1` かつ ローラー停止中 | `Rmayu2` で選んだ電磁弁を開く（`0`→lock1、`1`→lock2）。開くのは最大 800ms |
| `Rtuno2 == 1` かつ 上ローラー回転中 | 装填2（pwm10）を正転（上限 571） |
| `Rtuno2 == 1` かつ 下ローラー回転中 | 装填1（pwm9）を正転（上限 571） |

装填モーターは次の2段階で動かします。

1. `souten()`（毎周回）が目標値を決める（`souten1_target` / `souten2_target`）。符号が向きを表す（-571 / 0 / +571）
2. `souten_ramp()`（20ms 周期）が `motor_simple_control` で PWM を目標値へ `SOUTEN_RAMP_STEP`（58）ずつ近づける

始動・停止は `SOUTEN_RAMP_MS`（200ms）かけて変化させ、反転は一度 0 まで下げてから向きを変えます。向きの値は次のとおりです（`souten_dir1` / `souten_dir2`）。

| 装填 | 送り（正転） | 原点復帰（逆転） |
|---|---|---|
| 装填1（pwm9） | `souten_dir1 = 0` | `souten_dir1 = 1` |
| 装填2（pwm10） | `souten_dir2 = 1` | `souten_dir2 = 0` |

起動直後の向きの値は 0 なので、装填2は最初の送りだけ、向きの切り替えに1周期（20ms）かかります。

### 電磁弁の ON 時間の制限

12V 品を 18V 系統で駆動しているので、連続 ON を `SOLENOID_MAX_ON_MS`（800ms）までにしています（`solenoid.c`）。電磁弁ごとに次の3つの状態を持ちます（`valve1` / `valve2`）。

| 状態 | 弁 | 次の状態へ |
|---|---|---|
| OFF | 閉 | 撃つ指令が来たら ON（時刻を記録） |
| ON | 開 | 指令が消えたら OFF。800ms 経ったら指令によらず LOCKOUT |
| LOCKOUT | 閉 | 指令が消える（スイッチを一度 OFF にする）まで開かない |

- 撃つスイッチを押してから 800ms 経つと、押したままでも閉じます。押しっぱなしで開き直すこともありません。
- 起動時と通信断の後も LOCKOUT から始まります。スイッチが ON のまま電源を入れたり通信が戻ったりしても、勝手に撃ちません。
- 実際にピンに出す値は `valve1_on` / `valve2_on` で、`motor_outputs()` が書きます。

### 装填の原点復帰

リミットスイッチで装填モーターを逆転させ、原点まで戻します。

| 装填 | 復帰を始めるスイッチ（送りの端） | 原点スイッチ | 復帰中の出力 |
|---|---|---|---|
| 装填1（pwm9） | lock6 | lock7 | 逆転（上限 571） |
| 装填2（pwm10） | lock8 | lock9 | 逆転（上限 571） |

- 復帰フラグ（`reset_flag1` / `reset_flag2`）は、送りの端のスイッチで 1 になり、原点スイッチを踏むまで 1 のままです。復帰中は撃つ指令より優先して逆転します。
- **フラグが切り替わった瞬間（どちらかのスイッチを踏んだとき）だけは、機構を端に押し付けないよう、ランプを待たずに PWM を 0 にします。** そこからの反転・再始動はランプで行います。原点で止まっている間ずっと 0 にするわけではないので、原点からの送りは普通に始まります。
- PWM のノイズでフラグが誤って立たないよう、スイッチは `limit_read()` で `LIMIT_DEBOUNCE_MS`（20ms）同じ値が続いたときだけ確定させています。

## 安全機構（`safety()`）

### 全停止

次のどれかに当てはまると、pwm1〜pwm10 をすべて 0 にし、電磁弁を閉じます（`valves_off()`）。電磁弁は、異常が消えても撃つスイッチを一度 OFF にするまで開きません。モーターは、異常が消えると 0 からランプで動き直します。

| 異常 | 判定 |
|---|---|
| SBUS の受信が途絶えた（断線、受信機の電源断など） | 最後にフレームをデコードしてから `SBUS_TIMEOUT_MS`（200ms）経過 |
| 送信機の電源が切れた | `SBUS_Failsafe` |
| 起動後まだ SBUS を受信していない | `SBUS_CH[0] == 0` |
| CAN が途絶えた | 最後の受信から `CAN_TIMEOUT_MS`（100ms）経過 |

- `SBUS_CH` はフレームが届いたときしか更新されません。そのため受信が完全に止まったときは、タイムアウトでしか検出できません。
- 単発のフレーム落ち（`SBUS_LostFrame`）は、ノイズで急停止しないよう判定に使っていません。
- CAN 断のときはエンコーダ値が古いまま固まるので、`roller_ready` も 0 にします。

### 走行中のローラー制限

足回り（pwm1〜pwm4）のどれか1つでも回っている間は、ローラー（pwm5〜pwm8）の PWM を `ROLLER_PWM_WHILE_DRIVING`（50）で頭打ちにします。足回りとローラーが同時に全力で電流を引かないようにするためです。

### 装填の PWM 上限

装填（pwm9 / pwm10）は、どこで値を書き換えても `safety()` が最後に `SOUTEN_PWM_MAX`（571）で頭打ちにします（[電源と電圧の制限](#電源と電圧の制限)）。

### マイコンが止まったとき

`HardFault_Handler` と `Error_Handler` は、止まる前に `outputs_all_off()` で全 PWM を 0 にし、電磁弁を閉じます。最後の出力のまま回り続けたり、電磁弁が開いたままになったりしないようにするためです。初期化の途中で呼ばれても安全なように、HAL のハンドルを使わずにレジスタへ直接書いています。

### ステータス LED（基板の LD1〜LD3）

| LED | 状態 |
|---|---|
| 緑点灯 | 正常 |
| 緑点滅 | Lidar 途絶（手動操縦のみ可） |
| 青点滅 | SBUS 断 |
| 赤点滅 | CAN 断 |
| 紫点滅 | SBUS と CAN の両方が断 |

### LED テープ（lock3〜lock5）

`TEAM_COLOR`（safety.c、0 = 赤 / 1 = 青）の色で光ります。試合前に自チームの色に合わせてください。

| 状態 | 表示 |
|---|---|
| 通常 | チームの色で点灯 |
| ローラーが目標速度に到達（`roller_ready == 1`） | チームの色で点滅（300ms ごと） |
| SBUS 断 / CAN 断 | チームの色より優先して、青点滅 / 赤点滅（両方なら紫点滅） |

## 電源と電圧の制限

マキタ 18V バッテリー（満充電 約21V、放電末期 約15V）を2系統（各15Aヒューズ）で使います。12V 用の RS-555 と電磁弁にも降圧回路を入れずにつないでいるので、ソフトで電圧を制限しています。LED テープとリレーは制御系のリポから給電します。

| 機器 | 制限 | 定数 |
|---|---|---|
| RS-555（装填 pwm9 / pwm10） | duty の上限 12V / 21V ≒ 57%（TIM3 で 571/1000）。始動・停止・反転は 200ms のランプ | `SOUTEN_PWM_MAX`、`SOUTEN_RAMP_MS` |
| 電磁弁（lock1 / lock2） | 連続 ON は 800ms まで | `SOLENOID_MAX_ON_MS` |
| RZ-735VA（ローラー pwm5〜pwm8） | 1周期に上げる量を制限したソフトスタート | `ROLLER_STEP_UP` など |

- `SOUTEN_PWM_MAX` は `1000 × RS555_RATED_MV / BATTERY_FULL_MV` で自動計算されます。`RS555_RATED_MV` を上げると装填の力は増えますが、定格より高い電圧をかけることになります。
- バッテリー電圧を測る ADC は無いので、上限は電圧が最も高い満充電（`BATTERY_FULL_MV` = 21V）を想定した固定値です。放電が進むと装填は遅くなります。
- 定数はすべて `robot_limits.h` にまとめてあります。

**ハード側で必要なこと（ソフトでは防げません）**

- リセット中と書き込み中は GPIO がハイインピーダンスになり、電磁弁のトランジスタのベースが浮きます。リセット中も確実に OFF にするため、ベース–エミッタ間にプルダウン抵抗（10kΩ 程度）を入れてください。
- 電磁弁のコイルには、フライホイールダイオードを入れてください。
- 12V 品を 21V で駆動するとコイル電流が約1.75倍になります。トランジスタの定格に余裕があるか確認してください。

## モーターと基板チャンネルの対応

基板（2026_B_main）は PWMn と DIRn が同じドライバにつながっています。そのため DIR には、ソフト上のモーター番号ではなく、**その PWM が出ている基板チャンネルの DIR** を書きます。

| モーター | 用途 | タイマー | 基板ch | PWM ピン | DIR | DIR ピン | DIR の値 |
|---|---|---|---|---|---|---|---|
| pwm1 | 足回り 左前 | TIM4 CH4 | 3 | PD15 | d3 | PE10 | `dir1` |
| pwm2 | 足回り 右前 | TIM4 CH3 | 4 | PD14 | d4 | PD11 | `dir2` |
| pwm3 | 足回り 左後 | TIM4 CH1 | 1 | PD12 | d1 | PB1 | `dir3` |
| pwm4 | 足回り 右後 | TIM4 CH2 | 2 | PD13 | d2 | PB2 | `dir4` |
| pwm5 | 上ローラー | TIM1 CH2 | 7 | PE11 | d7 | PF12 | 1 固定 |
| pwm7 | 上ローラー | TIM1 CH3 | 5 | PE13 | d5 | PF3 | 0 固定 |
| pwm6 | 下ローラー | TIM1 CH1 | 8 | PE9 | d8 | PF13 | 1 固定 |
| pwm8 | 下ローラー | TIM1 CH4 | 6 | PE14 | d6 | PF14 | 0 固定 |
| pwm9 | 装填1 | TIM3 CH2 | 10 | PC7 | d10 | PA11 | `souten_dir1` |
| pwm10 | 装填2 | TIM3 CH1 | 9 | PC6 | d9 | PA12 | `souten_dir2` |
| pwm11 | 予備（未使用） | TIM3 CH3 | 11 | PC8 | d11 | PB12 | — |
| pwm12 | 予備（未使用） | TIM3 CH4 | 12 | PC9 | d12 | PB11 | — |

ローラーは上下とも、向かい合う2個が逆向きに回るよう DIR を 1 と 0 にしています。

### PWM のスケール

タイマーごとに Period が違うので、PWM に入れる値の範囲も違います。

| 系統 | タイマー | Period | PWM 周波数 | 使っている上限 |
|---|---|---|---|---|
| 足回り | TIM4 | 999 | 10kHz | 600（`maxpwm`） |
| ローラー | TIM1 | 254 | 約19.6kHz | 160（`ROLLER_PWM_MAX`） |
| 装填 | TIM3 | 999 | 10kHz | 571（`SOUTEN_PWM_MAX`） |

**Period を超える値を入れると、常に 100% デューティになります。** 値やタイマー設定を変えるときは、この表と合っているか確認してください。

## GPIO（モーター以外）

| 名前 | ピン | 向き | 用途 |
|---|---|---|---|
| `lock1` | PG4 | 出力 | 電磁弁1（High で ON） |
| `lock2` | PG6 | 出力 | 電磁弁2（High で ON） |
| `lock3` | PG7 | 出力 | LED テープ 緑 |
| `lock4` | PG5 | 出力 | LED テープ 赤 |
| `lock5` | PD10 | 出力 | LED テープ 青 |
| `lock6` | PG8 | 入力（プルアップ） | 装填1 送りの端（押すと Low） |
| `lock7` | PG14 | 入力（プルアップ） | 装填1 原点（押すと Low） |
| `lock8` | PF15 | 入力（プルアップ） | 装填2 送りの端（押すと Low） |
| `lock9` | PF11 | 入力（プルアップ） | 装填2 原点（押すと Low） |
| `LD1` | PB0 | 出力 | ステータス LED 緑 |
| `LD2` | PB7 | 出力 | ステータス LED 青 |
| `LD3` | PB14 | 出力 | ステータス LED 赤 |

## CAN 受信データ（ID `0x001`）

受信割り込み（`HAL_CAN_RxFifo0MsgPendingCallback`）が ID `0x001`・DLC 8以上のフレームを `use_data[0..7]` に写し、受信時刻を `last_can_rx` に記録します。メインループの先頭で次のように `PV` に移します。

| `use_data` | 変数 | 用途 |
|---|---|---|
| [0] | `PV1` | 上ローラー（pwm5）の回転数 |
| [1] | `PV2` | 上ローラー（pwm7）の回転数 |
| [2] | `PV5` | 未使用 |
| [3] | `PV4` | 下ローラー（pwm8）の回転数 |
| [4] | `PV3` | 下ローラー（pwm6）の回転数 |
| [5] | `PV6` | 未使用 |
| [6], [7] | — | 未使用 |

値は 0〜255 で、ローラーの目標速度と同じ単位です。

## 主要関数

| 関数 | ファイル | 説明 |
|---|---|---|
| `motor_control(SV, PV, maxMV, down_pwm, max_pwm, *pwm, *dir, *rem)` | motor_control.c | エンコーダ付きの速度制御（ローラー用）。誤差の1/10を毎周期 PWM に足し込む積分制御で、1周期の変化量は `maxMV` までです。`rem` は整数除算の端数の繰り越しで、誤差が小さい領域の不感帯をなくします。方向転換と停止のときは `down_pwm` ずつ PWM を落とします。 |
| `motor_simple_control(SV, step, max_pwm, *pwm, *dir)` | motor_control.c | エンコーダなしのランプ制御（足回り・装填用）。`SV` の符号が回転方向です（負 = dir 0 / 正 = dir 1）。PWM を `step` ずつ目標に近づけ、方向転換のときは一度 0 まで落としてから向きを変えます。 |
| `asimawari()` | function.c | 走行モードに応じて前後・左右・旋回を決め、足回り4輪を動かします（20ms 周期）。 |
| `omni_mix(forward, strafe, turn, taiya)` | function.c | オムニの混合式。全モード共通です。 |
| `roller()` | function.c | `Lmayu2` / `Lmayu1` / `Ltuno1` / `ry` に応じてローラー4個の目標速度を決め、速度制御します（20ms 周期）。 |
| `souten()` | function.c | 電磁弁と装填モーターの指令を決めます（毎周回）。リミットスイッチによる原点復帰と、リミットを踏んだ瞬間の即停止もここで行います。 |
| `souten_ramp()` | function.c | 装填モーターの PWM を、`souten()` が決めた目標値へ `motor_simple_control` で近づけます（20ms 周期）。 |
| `valves_off()` | function.c | 電磁弁をすぐ閉じ、撃つスイッチが一度 OFF になるまで開かないようにします。`safety()` が異常時に呼びます。 |
| `motor_outputs()` | function.c | PWM・DIR・電磁弁をまとめて出力します。`safety()` の後に呼びます。 |
| `outputs_all_off()` | function.c | 全 PWM と電磁弁をレジスタ直書きで即 0 にします。`HardFault_Handler` と `Error_Handler` から呼びます。 |
| `limit_read(sw)` | function.c | リミットスイッチを読みます。20ms 同じ値が続いたときだけ確定値を更新します。 |
| `auto_mode(d1, d2, reset, target)` | function.c | 2つの Lidar 距離から、距離を保つ PID（`auto_ly`）と平行を保つ PID（`auto_rx`）を計算します。`reset=1` で積分値をリセットします。 |
| `solenoid_update(s, command, now)` | solenoid.c | 電磁弁1個分の状態（OFF / ON / LOCKOUT）を進め、開くなら 1 を返します。 |
| `solenoid_lockout(s)` | solenoid.c | 電磁弁1個分をすぐ LOCKOUT にします。 |
| `safety()` | safety.c | 異常時の全停止と電磁弁の閉鎖、走行中のローラー制限、装填の上限、LED 表示。 |
| `HAL_CAN_RxFifo0MsgPendingCallback` | can_handler.c | CAN 受信割り込み。ID `0x001`、DLC 8以上のフレームを `use_data[]` に格納します。 |
| `CAN_TX(id)` | can_handler.c | CAN 送信。現在はどこからも呼ばれていません。 |
| `sbus()` | sbus_handler.c | スイッチとスティックを読みます。 |
| `SBUS_Process()` | sbus_handler.c | SBUS フレームを16チャンネルにデコードし、受信時刻を記録します。 |
| `lidar()` | lidar_sensor.c | DMA リングバッファから TSD20 の4バイトフレーム（`5C` / 距離下位 / 距離上位 / チェックサム）を取り出します。チェックサムが合わなければ1バイト進めて同期し直します。20000mm を超える値（測定範囲外）は捨てます。 |
| `lidar_timeout()` | lidar_sensor.c | どちらかの Lidar の有効な測定値が `LIDAR_TIMEOUT_MS` 以上途絶えていれば 1 を返します。 |

## 調整用の定数

### robot_limits.h（上限値・ランプ時間・タイムアウト時間）

| 定数 | 値 | 内容 |
|---|---|---|
| `CONTROL_PERIOD_MS` | 20 | 足回り・ローラー・装填のランプの周期 (ms)。PID もこの周期が前提 |
| `BATTERY_FULL_MV` | 21000 | 上限の計算に使うバッテリー電圧 (mV)。ADC が無いので満充電の固定値 |
| `RS555_RATED_MV` | 12000 | RS-555 にかけてよい電圧 (mV) |
| `SOUTEN_PWM_FULL` | 1000 | TIM3 の duty 100%（Period 999 + 1）。タイマー設定に合わせた値なので変えない |
| `SOUTEN_PWM_MAX` | 571（自動計算） | 装填の PWM 上限 = 1000 × 12V / 21V |
| `SOUTEN_RAMP_MS` | 200 | 装填を 0 から上限まで上げる（下げる）時間 (ms) |
| `SOUTEN_RAMP_STEP` | 58（自動計算） | 装填の PWM を1周期に変える量 |
| `ROLLER_PWM_MAX` | 160 | ローラーの PWM 上限 |
| `ROLLER_STEP_UP` / `ROLLER_STEP_DOWN` | 12 / 20 | ローラーの PWM を1周期に上げる最大量 / 停止時に下げる量 |
| `ROLLER_PWM_WHILE_DRIVING` | 50 | 走行中のローラーの PWM 上限 |
| `SOLENOID_MAX_ON_MS` | 800 | 電磁弁を ON にしておける最大時間 (ms)。1000 未満にすること |
| `SBUS_TIMEOUT_MS` | 200 | SBUS 断と判定するまでの時間 (ms) |
| `CAN_TIMEOUT_MS` | 100 | CAN 断と判定するまでの時間 (ms) |

### そのほか

| 定数 | 場所 | 値 | 内容 |
|---|---|---|---|
| `maxpwm` | main.c | 600 | 足回りの PWM 上限（duty 60%） |
| `DRIVE_STEP` | function.c | 40 | 足回りの PWM を1周期に変える量 |
| `ROLLER_SPEED` | function.c | 245 | 下ローラーの目標速度 |
| `BAKETU1〜3_ROLLER_SPEED` | function.c | 150 / 84 / 76 | 上ローラーの目標速度 |
| `BAKETU1_RY_RANGE` | function.c | 50 | BAKETU1 を `ry` で上げ下げできる幅。基準＋幅は 254 以下にすること |
| `ROLLER_READY_TOLERANCE` | function.c | 5 | 目標速度との差がこれ以内なら「到達」とみなす |
| `LIMIT_DEBOUNCE_MS` | function.c | 20 | リミットスイッチのノイズ除去時間 (ms) |
| PID ゲイン | function.c `auto_mode()` | 距離 1.3 / 0.008 / 0.05、角度 0.6 / 0.01 / 0.2 | Kp / Ki / Kd |
| `TEAM_COLOR` | safety.c | 1 | LED テープの色（0 = 赤 / 1 = 青）。試合前に合わせる |
| `AUTO_TARGET_DIST_MM` | lidar_sensor.h | 1770 | 全自動モードで保つ壁からの距離 (mm) |
| `LIDAR_OFFSET4` / `LIDAR_OFFSET7` | lidar_sensor.h | 0 / 0 | Lidar の取り付け位置の補正 (mm) |
| `LIDAR_TIMEOUT_MS` | lidar_sensor.h | 100 | Lidar 途絶と判定するまでの時間 (ms) |

## デバッグ出力

`printf` の出力先は USART3（115200bps、ST-LINK の仮想 COM ポート）です。今は 20ms ごとに Lidar の距離（`distance4` / `distance7`）を出力しています。送信は完了を待つ方式なので、1回ごとにループが数ms止まります。不要なときは `main.c` の `printf` をコメントアウトしてください。

## ハードウェア構成（.ioc より）

- MCU: STM32F767ZITx（LQFP144）、SYSCLK 120MHz（HSE 8MHz）
- CAN1（1Mbps）、UART4 / UART5 / UART7（DMA 受信）、USART2 / USART3 / UART8、TIM1・TIM3・TIM4（各4ch PWM）
- バッテリー電圧を測る ADC は未設定

## ビルド

CMake プロジェクトです（STM32CubeMX で生成）。

```sh
cmake --preset Debug
cmake --build build/Debug
```

出力は `build/Debug/2026_B_767.elf` です。

自作の `.c` ファイルを増やしたときは、`CMakeLists.txt` の `target_sources`（`# Add user sources here` の下）に追加してください。追加しないと、リンク時に関数が見つからないエラーになります。

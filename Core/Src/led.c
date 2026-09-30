/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : led.c
 * @brief          : ステータス LED (LD1〜LD3) と LED テープ (lock3〜lock5)、USER ボタンでの色の切り替え
 ******************************************************************************
 */
/* USER CODE END Header */
#include "led.h"
#include "function.h"     /* roller_ready, now, limit_sw, limit_read() */
#include "safety.h"       /* sbus_lost(), can_lost(), estop_on() */

// 起動時の LED テープの色 (0 = 赤 / 1 = 青)。USER ボタンで切り替えられるが、電源を切るとこの色に戻る
#define TEAM_COLOR 1

// USER ボタンを何 ms 押し続けたら色を切り替えるか。うっかり触っただけでは変わらないようにする
#define TEAM_COLOR_HOLD_MS 1000

// 今の LED テープの色 (0 = 赤 / 1 = 青)。update_team_color() が USER ボタンで切り替える
static int team_color = TEAM_COLOR;

// Nucleo の USER ボタン (PC13)。押すと High。
// 起動時は「押している」(1) 扱いから始める。こうすると、起動時に押していても一度離すまで反応しない
static limit_sw user_btn = LIMIT_SW_INIT_START(USER_Btn_GPIO_Port, USER_Btn_Pin, 1);

/*
 * USER ボタンを TEAM_COLOR_HOLD_MS 押し続けたら、LED テープの色を赤 ⇔ 青で切り替える。
 *   ・押しっぱなしでは 1 回だけ。もう一度切り替えるには一度離す
 *   ・起動時に押していた場合は、一度離すまで反応しない
 *   ・SBUS 断・CAN 断の間はテープにチームの色が出ないので、押しても無視する。
 *     エラーが消えても、一度離すまで反応しない
 * ボタンは limit_read() で読むので、20ms 未満のノイズでは押したことにならない。
 */
static void update_team_color(int link_error) {
    static uint8_t released = 0;       // 起動後・切り替え後に一度離したか
    static uint32_t pressed_since = 0; // 押し始めた時刻 (離している間は毎回更新する)
    uint8_t pressed = limit_read(&user_btn);

    if (!pressed) {
        released = 1;
        pressed_since = HAL_GetTick();
        return;
    }
    if (link_error) {
        released = 0;
        return;
    }
    if (released && HAL_GetTick() - pressed_since >= TEAM_COLOR_HOLD_MS) {
        team_color = !team_color;
        released = 0;
    }
}

/*
 * lock3〜lock5 の LED を自チームの色で光らせる (color: 0 = 赤 / 1 = 青)。
 * ローラーが目標速度に達している間 (roller_ready == 1) は点滅、それ以外は点灯。
 * SBUS 断・CAN 断のときはチームの色より優先して、update_status_led() と同じ点滅を出す
 * (青点滅 = SBUS断、赤点滅 = CAN断、両方なら紫点滅)。
 */
static void color(int color, int sbus_error, int can_error){
    uint8_t blink = (now / 300) % 2;
    uint8_t on = roller_ready ? blink : 1;
    uint8_t blue = sbus_error ? blink : 0;
    uint8_t red = can_error ? blink : 0;

    if(sbus_error || can_error){
        HAL_GPIO_WritePin(lock3_GPIO_Port, lock3_Pin, 0);//green
        HAL_GPIO_WritePin(lock4_GPIO_Port, lock4_Pin, red);//red
        HAL_GPIO_WritePin(lock5_GPIO_Port, lock5_Pin, blue);//blue
    }else if(color == 0){
        HAL_GPIO_WritePin(lock3_GPIO_Port, lock3_Pin, 0);//green
        HAL_GPIO_WritePin(lock4_GPIO_Port, lock4_Pin, on);//red
        HAL_GPIO_WritePin(lock5_GPIO_Port, lock5_Pin, 0);//blue
    }else if(color == 1){
        HAL_GPIO_WritePin(lock3_GPIO_Port, lock3_Pin, 0);//green
        HAL_GPIO_WritePin(lock4_GPIO_Port, lock4_Pin, 0);//red
        HAL_GPIO_WritePin(lock5_GPIO_Port, lock5_Pin, on);//blue
    }

}

/*
 * LEDは「点灯状態を全部決めてから3本まとめて書く」。
 * 条件ごとにその場で WritePin すると、条件が変わったときに前の色を
 * 消し忘れて赤と青が同時に点く、といった消え残りが起きる。
 *
 * 青点滅 = SBUS断、赤点滅 = CAN断（両方落ちていれば紫点滅になる）、
 * 赤点灯 = 非常停止 (LOCK)、緑点灯 = 全て正常。
 * 非常停止は通信が正常なときだけ表示する (SBUS 断・CAN 断の表示を優先する)。
 */
static void update_status_led(int sbus_error, int can_error, int estop) {
    uint8_t blink = (now / 300) % 2;
    int link_ok = !sbus_error && !can_error;
    uint8_t green = (link_ok && !estop) ? 1 : 0;
    uint8_t blue = sbus_error ? blink : 0;
    uint8_t red = can_error ? blink : (link_ok && estop);

    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, green);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, blue);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, red);

}

/*
 * ステータス LED と LED テープを光らせる。毎周回、safety() の後に呼ぶ。
 * 通信断は safety() がこの周回で判定した結果を使うので、止めた状態と表示が食い違わない。
 * roller_ready も safety() が異常時に下ろした後の値で表示する。
 */
void led(void) {
    int sbus_error = sbus_lost();
    int can_error = can_lost();

    int estop = estop_on();

    // 非常停止の間は LED テープに電源が来ないので、テープの表示は変えない (色の切り替えも受け付けない)
    update_status_led(sbus_error, can_error, estop);
    update_team_color(sbus_error || can_error || estop);
    color(team_color, sbus_error, can_error);
}

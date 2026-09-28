#ifndef __SOLENOID_H
#define __SOLENOID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 電磁弁 1 個分の ON 時間の制限。
 * 12V 品を 18V 系統で駆動しているので、連続 ON を SOLENOID_MAX_ON_MS (robot_limits.h) 未満に抑える。
 *
 *   OFF     … 閉。指令が 1 になったら ON へ (時刻を記録)
 *   ON      … 開。指令が 0 になったら OFF へ。SOLENOID_MAX_ON_MS 経ったら指令によらず LOCKOUT へ
 *   LOCKOUT … 閉。指令が 0 になるまで開かない (押しっぱなしでは開き直さない)
 *
 * 起動時と非常停止 (solenoid_lockout) の後は LOCKOUT から始めるので、
 * スイッチが ON のまま電源を入れても、通信が戻っても、スイッチを一度 OFF にするまで開かない。
 * HAL を使わないので PC でもテストできる。
 */
typedef enum {
    SOLENOID_OFF,
    SOLENOID_ON,
    SOLENOID_LOCKOUT,
} solenoid_state;

typedef struct {
    solenoid_state state;
    uint32_t on_since; // ON にした時刻 (HAL_GetTick)
} solenoid;

#define SOLENOID_INIT {SOLENOID_LOCKOUT, 0}

/* 指令 command (1 = 開けたい) と現在時刻 now から状態を進め、弁を開くなら 1 を返す */
uint8_t solenoid_update(solenoid *s, int command, uint32_t now);

/* すぐ閉じて LOCKOUT にする。非常停止で使う */
void solenoid_lockout(solenoid *s);

#ifdef __cplusplus
}
#endif

#endif /* __SOLENOID_H */

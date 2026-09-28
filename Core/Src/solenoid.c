/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : solenoid.c
 * @brief          : 電磁弁 1 個分の ON 時間の制限 (状態遷移は solenoid.h)
 ******************************************************************************
 */
/* USER CODE END Header */
#include "solenoid.h"
#include "robot_limits.h" /* SOLENOID_MAX_ON_MS */

uint8_t solenoid_update(solenoid *s, int command, uint32_t now) {
    switch (s->state) {
    case SOLENOID_OFF:
        if (command) {
            s->state = SOLENOID_ON;
            s->on_since = now;
        }
        break;
    case SOLENOID_ON:
        if (!command) {
            s->state = SOLENOID_OFF;
        } else if (now - s->on_since >= SOLENOID_MAX_ON_MS) {
            // 押しっぱなしでも閉じる。uint32_t の引き算なので HAL_GetTick() が一周しても正しい
            s->state = SOLENOID_LOCKOUT;
        }
        break;
    case SOLENOID_LOCKOUT:
        if (!command) {
            s->state = SOLENOID_OFF;
        }
        break;
    }
    return s->state == SOLENOID_ON;
}

void solenoid_lockout(solenoid *s) {
    s->state = SOLENOID_LOCKOUT;
}

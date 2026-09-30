#ifndef __SAFETY_H
#define __SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 安全機能。メインループで必ず PWM を出力する直前 (motor_outputs() の前) に呼ぶこと。
 *   ・SBUS か CAN が使えないか、非常停止 (LOCK) が押されていれば、全モーターを止め、
 *     電磁弁を閉じ、撃つスイッチとローラーのスイッチを一度 OFF にするまで動かさない
 *     (原点復帰の途中だった装填は、通信断なら元に戻ると原点まで戻る。
 *      非常停止なら撃つスイッチを押すまで戻らない)
 *   ・足回りが回っている間はローラーの PWM を頭打ちにする
 *   ・装填の PWM を SOUTEN_PWM_MAX で頭打ちにする
 */
void safety(void);

/* safety() がこの周回で判定した結果。safety() の後に呼ぶこと (led() が使う) */
int sbus_lost(void); /* SBUS が使えなければ 1 (タイムアウト / Failsafe / 未受信) */
int can_lost(void);  /* CAN が CAN_TIMEOUT_MS 以上届いていなければ 1 */

#ifdef __cplusplus
}
#endif

#endif /* __SAFETY_H */

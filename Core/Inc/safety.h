#ifndef __SAFETY_H
#define __SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 安全機能。メインループで必ず PWM を出力する直前 (motor_outputs() の前) に呼ぶこと。
 *   ・SBUS か CAN が使えなければ全モーターを止める
 *   ・足回りが回っている間はローラーの PWM を頭打ちにする
 *   ・状態を LED に出す
 */
void safety(void);

#ifdef __cplusplus
}
#endif

#endif /* __SAFETY_H */

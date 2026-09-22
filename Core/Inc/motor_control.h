#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * どちらも「今の pwm と dir」をポインタで受け取り、1 周期分だけ目標へ近づけて書き戻す。
 * SV の符号が回転方向 (負 = dir 0 / 正 = dir 1)。
 */

/* エンコーダ付きの速度制御 (ローラー用)。PV が SV に近づくよう pwm を積分で動かす */
void motor_control(int SV, int PV, int maxMV, int down_pwm, int max_pwm, int *pwmm, int *dirr, int *remm);

/* エンコーダなしのランプ制御 (足回り用)。pwm を step ずつ |SV| に近づける */
void motor_simple_control(int SV, int step, int max_pwm, int *pwmm, int *dirr);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */

#ifndef __FUNCTION_H
#define __FUNCTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "sbus_handler.h" /* SBUS_CH, SBUS_LostFrame を safety() で使用するため */
#include <stdint.h>

/* ペリフェラルハンドル (main.c で定義) */
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart3;

/* 共有変数 (main.c で定義) */
extern volatile int16_t PV5;
extern volatile int16_t PV6;

extern int pwm1;
extern int pwm2;
extern int pwm3;
extern int pwm4;
extern int pwm5;
extern int pwm6;
extern int pwm7;
extern int pwm8;
extern int pwm9;
extern int pwm10;
extern int pwm11;
extern int pwm12;
extern int rem1;
extern int rem2;
extern int rem3;
extern int rem4;
extern int rem5;
extern int rem6;
extern int rem7;
extern int rem8;
extern int rem9;
extern int rem10;
extern int rem11;
extern int rem12;


extern int stop_flag1;
extern int stop_flag2;
extern int timer_flag;
extern int reset_flag1;
extern int reset_flag2;

extern int roller_dir1;
extern int roller_dir2;
extern int dummy;

extern int auto_ly;
extern int auto_rx;

//extern uint32_t time3 = 0;
extern uint32_t now;
extern uint32_t last_can_rx;

extern volatile uint8_t use_data[8];

/* 関数プロトタイプ */
int _write(int file, char *ptr, int len);

void CAN_TX(uint32_t recipient);

void motor_control(int SV, int PV, int maxMV, int down_pwm, int max_pwm, int *pwmm, int *dirr,int *remm);
void motor_simple_control(int SV, int step, int max_pwm, int *pwmm);

void roller(void);
void auto_mode(int distance1, int distance2, int reset_flag, int target_dist);
void safety(void);

#ifdef __cplusplus
}
#endif

#endif /* __FUNCTION_H */

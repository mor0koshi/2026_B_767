/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : output.c
 * @brief          : PWM・DIR・電磁弁のピンへの出力と、非常時の全停止
 ******************************************************************************
 */
/* USER CODE END Header */
#include "function.h"

/* ============================================================================
 * 出力
 * ========================================================================== */

/*
 * PWM と DIR をまとめて出力する。
 *
 * 基板 (2026_B_main) は PWMn と DIRn が同じドライバへ行く配線なので、
 * DIR はソフトの pwmN 番号ではなく「その PWM が出ている基板ch の DIR」を書く。
 * 例: pwm1 の PWM は PD15 = 基板の PWM3 なので、DIR は d3 (PE10)。
 */
void motor_outputs(void) {
    // 足回り
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm1); // 左前 = 基板ch3 (PWM3=PD15)
    HAL_GPIO_WritePin(d3_GPIO_Port, d3_Pin, dir1);      // DIR3 = PE10
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm2); // 右前 = 基板ch4 (PWM4=PD14)
    HAL_GPIO_WritePin(d4_GPIO_Port, d4_Pin, dir2);      // DIR4 = PD11
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm3); // 左後 = 基板ch1 (PWM1=PD12)
    HAL_GPIO_WritePin(d1_GPIO_Port, d1_Pin, dir3);      // DIR1 = PB1
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm4); // 右後 = 基板ch2 (PWM2=PD13)
    HAL_GPIO_WritePin(d2_GPIO_Port, d2_Pin, dir4);      // DIR2 = PB2

    // ローラーは常に一方向なので DIR は固定値。
    // 対になる 2 個は向かい合っているので、逆の値にして互いに逆回転させる。
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm5); // 上ローラー = 基板ch7 (PWM7=PE11)
    HAL_GPIO_WritePin(d7_GPIO_Port, d7_Pin, 1);         // DIR7 = PF12
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm6); // 下ローラー = 基板ch8 (PWM8=PE9)
    HAL_GPIO_WritePin(d8_GPIO_Port, d8_Pin, 1);         // DIR8 = PF13
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm7); // 上ローラー = 基板ch5 (PWM5=PE13)
    HAL_GPIO_WritePin(d5_GPIO_Port, d5_Pin, 0);         // DIR5 = PF3
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm8); // 下ローラー = 基板ch6 (PWM6=PE14)
    HAL_GPIO_WritePin(d6_GPIO_Port, d6_Pin, 0);         // DIR6 = PF14

    // 装填は正転/逆転リセットがあるので DIR は souten_dir を出す
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm9);    // 装填1 = 基板ch10 (PWM10=PC7)
    HAL_GPIO_WritePin(d10_GPIO_Port, d10_Pin, souten_dir1); // DIR10 = PA11
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm10);   // 装填2 = 基板ch9 (PWM9=PC6)
    HAL_GPIO_WritePin(d9_GPIO_Port, d9_Pin, souten_dir2);   // DIR9 = PA12

    // 電磁弁 (GPIO High で ON)
    HAL_GPIO_WritePin(lock1_GPIO_Port, lock1_Pin, denziben_on(1)); // lock1 = PG4
    HAL_GPIO_WritePin(lock2_GPIO_Port, lock2_Pin, denziben_on(2)); // lock2 = PG6

    // 予備 (未使用)
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm11); // 基板ch11 (PWM11=PC8), DIR11 = PB12
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm12); // 基板ch12 (PWM12=PC9), DIR12 = PB11
}

/*
 * 全モーターと電磁弁を即座に止める。HardFault_Handler と Error_Handler から呼ぶ。
 * マイコンが止まっても、最後の PWM で回り続けたり、電磁弁が開きっぱなしになったりしないようにする。
 * 初期化の途中で呼ばれても安全なように、HAL のハンドル (Instance が未設定かもしれない) を
 * 使わずにレジスタへ直接書く。
 */
void outputs_all_off(void) {
    HAL_GPIO_WritePin(lock1_GPIO_Port, lock1_Pin, 0);
    HAL_GPIO_WritePin(lock2_GPIO_Port, lock2_Pin, 0);

    TIM1->CCR1 = 0; // ローラー
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCR4 = 0;
    TIM3->CCR1 = 0; // 装填・予備
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
    TIM4->CCR1 = 0; // 足回り
    TIM4->CCR2 = 0;
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 0;
}

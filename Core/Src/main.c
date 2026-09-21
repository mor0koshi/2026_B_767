/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sbus.h"
#include "function.h"
#include "sbus_handler.h"
#include "lidar_sensor.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

CAN_HandleTypeDef hcan1;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_uart8_rx;

/* USER CODE BEGIN PV */

// スティック (sbus() で ±1000 に変換済み)
int rx; // 旋回 (CH0)
int ly; // 前後 (CH1)
int ry; // 未使用 (CH2)
int lx; // 左右 (CH3)

// スイッチ (sbus() で変換済み)
int Lmayu1; // CH4 ローラー選択    1=上ローラー / 0=下ローラー
int Lmayu2; // CH5 ローラー回転    1=回す / 0=止める
int Rmayu1; // CH6 走行モード      -1=手動 / 0=半自動 / 1=全自動
int Rmayu2; // CH7 電磁弁の選択    0=lock1 / 1=lock2
int Ltuno1; // CH8 上ローラー速度  1=200 / 0=150 / -1=100
int Rtuno2; // CH9 発射            1=打つ / 0=打たない

// 手動モードのオムニ混合値 (sbus() で計算)
volatile int m1; // 左前
volatile int m2; // 右前
volatile int m3; // 左後
volatile int m4; // 右後

// CAN (ID 0x001) で受け取るローラーのエンコーダ値。0〜255
volatile int16_t PV1 = 0; // 上ローラー pwm5 (use_data[0])
volatile int16_t PV2 = 0; // 上ローラー pwm7 (use_data[1])
volatile int16_t PV3 = 0; // 下ローラー pwm6 (use_data[2])
volatile int16_t PV4 = 0; // 下ローラー pwm8 (use_data[3])
volatile int16_t PV5 = 0; // 未使用 (use_data[4])
volatile int16_t PV6 = 0; // 未使用 (use_data[5])

// 足回りの回転方向 (motor_simple_control が更新)
int dir1 = 0;
int dir2 = 0;
int dir3 = 0;
int dir4 = 0;

/*
 * 各モーターの PWM 値。タイマーごとに Period が違うので値の範囲も違う。
 *   pwm1〜pwm4  足回り   TIM4 (Period 999) … 上限 maxpwm
 *   pwm5〜pwm8  ローラー TIM1 (Period 254) … 上限 245
 *   pwm9, pwm10 装填     TIM3 (Period 999) … 600 固定
 *   pwm11, pwm12 予備    TIM3 (未使用)
 * Period を超える値を入れると常に 100% デューティになるので注意。
 */
int pwm1 = 0;
int pwm2 = 0;
int pwm3 = 0;
int pwm4 = 0;
int pwm5 = 0;
int pwm6 = 0;
int pwm7 = 0;
int pwm8 = 0;
int pwm9 = 0;
int pwm10 = 0;
int pwm11 = 0;
int pwm12 = 0;


// ローラーの motor_control 用。誤差を 1/10 するときの端数の繰り越し
int rem5 = 0;
int rem6 = 0;
int rem7 = 0;
int rem8 = 0;

int maxpwm = 1000 * 0.9; // 足回りの PWM 上限 (TIM4 の Period 999 に対して 90%)

int maxmv = 20; // 未使用

// 装填の原点復帰中フラグ。lock6/lock8 で立ち、原点の lock7/lock9 で下りる
int reset_flag1 = 0; // 装填1
int reset_flag2 = 0; // 装填2

// 逆転リセットのリミットスイッチ。ノイズ除去して読む (limit_read)
limit_sw sw_lock6 = LIMIT_SW_INIT(lock6_GPIO_Port, lock6_Pin); // 装填1 リセット開始
limit_sw sw_lock7 = LIMIT_SW_INIT(lock7_GPIO_Port, lock7_Pin); // 装填1 原点
limit_sw sw_lock8 = LIMIT_SW_INIT(lock8_GPIO_Port, lock8_Pin); // 装填2 リセット開始
limit_sw sw_lock9 = LIMIT_SW_INIT(lock9_GPIO_Port, lock9_Pin); // 装填2 原点

int roller_dir1 = 0; // 装填1(pwm9)の回転方向を保持する変数
int roller_dir2 = 0; // 装填2(pwm10)の回転方向を保持する変数

// ローラーは常に正転で方向転換しない (DIR は出力時に固定値を書く) ため、
// motor_control の dir の受け皿は共用の捨て変数でよい
int dummy = 0;

// lidar
//  UART4 用（センサー1）
uint8_t rx_dma_buf4[DMA_BUF_SIZE];
uint16_t distance4 = 0;

// UART7 用（センサー2）
uint8_t rx_dma_buf7[DMA_BUF_SIZE];
uint16_t distance7 = 0;

// auto_mode() の PID が出す仮想スティック値
int auto_ly = 0; // 前後 (壁との距離を保つ)。全自動モードで使用
int auto_rx = 0; // 旋回 (壁と平行を保つ)。半自動・全自動モードで使用

uint32_t time1 = 0; // 足回りとローラーの 20ms 周期の基準時刻
uint32_t now = 0;   // ループ先頭の HAL_GetTick()

uint32_t last_can_rx = 0; // 最後に CAN を受信した時刻 (safety() の CAN 断判定)

// SBUS

uint8_t sbus_rxbuf[SBUS_FRAME_LEN];
uint8_t sbus_frame[SBUS_FRAME_LEN];
volatile uint16_t SBUS_CH[16];
uint8_t SBUS_Failsafe = 0;
uint8_t SBUS_LostFrame = 0;
uint32_t last_sbus_rx = 0; // 最後にSBUSフレームをデコードできた時刻

// CAN 受信割り込みで書き込まれる最新の 8 バイト (ID 0x001)
volatile uint8_t use_data[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_ETH_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART7_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

    setbuf(stdout, NULL); // printf をバッファせず、その場で USART3 へ送る
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_ETH_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_TIM3_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1,
                                 CAN_IT_RX_FIFO0_MSG_PENDING); // 受信割り込みを有効化

    // 足回り (TIM4)、ローラー (TIM1)、装填 (TIM3) の PWM 出力を開始

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    SBUS_Init(); // SBUS の受信を開始

    HAL_Delay(100); // Lidar の起動待ち

    // Lidar 2 台 (UART4, UART7) に測定開始コマンドを送り、循環 DMA で受信を始める
    uint8_t startCmd[] = {0x5A, 0x0A, 0x02, 0x02, 0x00, 0xF1};
    HAL_UART_Transmit(&huart4, startCmd, sizeof(startCmd), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart7, startCmd, sizeof(startCmd), HAL_MAX_DELAY);
    HAL_Delay(20);
    HAL_UART_Receive_DMA(&huart4, rx_dma_buf4, DMA_BUF_SIZE);
    HAL_UART_Receive_DMA(&huart7, rx_dma_buf7, DMA_BUF_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
        now = HAL_GetTick();

        // CAN で受け取ったローラーのエンコーダ値
        PV1 = use_data[0];
        PV2 = use_data[1];
        PV3 = use_data[2];
        PV4 = use_data[3];
        PV5 = use_data[4];
        PV6 = use_data[5];

        sbus(); // スイッチとスティックを読み、手動用の m1〜m4 を計算

        lidar(); // Lidar の距離を更新

        // lock6/lock8 は逆転リセットの開始、lock7/lock9 は原点リミット。
        // 原点リミットは「リセットの終了」だけを担当させる。ここで pwm を直接 0 に
        // すると、原点で静止している間は通常の正転指令まで毎周回打ち消されてしまう。
        if (limit_read(&sw_lock6) == 0) {
            reset_flag1 = 1;
        }
        if (limit_read(&sw_lock7) == 0) {
            reset_flag1 = 0;
        }
        if (limit_read(&sw_lock8) == 0) {
            reset_flag2 = 1;
        }
        if (limit_read(&sw_lock9) == 0) {
            reset_flag2 = 0;
        }

        // 足回りとローラー (20ms 周期。auto_mode() の dt もこの周期が前提)
        if (now - time1 >= 20) {
            // Lidarが死んでいると auto_mode は「壁まで遠すぎる」と誤認して全速で走り続ける。
            // 測定値が途絶えている間は自動系を止め、手動モードとして扱う。
            int lidar_ng = lidar_timeout();

            if (Rmayu1 == -1 || lidar_ng) {
                // 手動モード（Lidar異常時もここに）
                // ★裏でPIDの記憶をリセットしておく。
                //   引数は全自動モードと必ず同じにすること（違うと切替時にD項が跳ねる）
                auto_mode(distance4 - LIDAR_OFFSET4, distance7 - LIDAR_OFFSET7, 1, AUTO_TARGET_DIST_MM);
                motor_simple_control(m1,80, maxpwm, &pwm1, &dir1);
                motor_simple_control(m2,80, maxpwm, &pwm2, &dir2);
                motor_simple_control(m3,80, maxpwm, &pwm3, &dir3);
                motor_simple_control(m4,80, maxpwm, &pwm4, &dir4);
            } else if (Rmayu1 == 0) {
                // 半自動モード: 前後・左右は手で操作し、旋回だけ PID で壁と平行を保つ。
                // 目標距離に現在距離を渡して距離の誤差を 0 にし、auto_ly を効かせない。
                auto_mode(distance4 - LIDAR_OFFSET4, distance7 - LIDAR_OFFSET7, 0,
                          (distance4 + distance7 - (LIDAR_OFFSET4 + LIDAR_OFFSET7)) / 2);
                // sbus() の m1〜m4 と同じ式で、rx だけ PID の auto_rx に差し替える。
                // 式を変えるときは sbus_handler.c と必ず揃えること。
                int gauto_m1 = -ly + lx + auto_rx;
                int gauto_m2 = -ly - lx + auto_rx;
                int gauto_m3 = ly - lx + auto_rx;
                int gauto_m4 = ly + lx + auto_rx;

                motor_simple_control(gauto_m1,80, maxpwm, &pwm1, &dir1);
                motor_simple_control(gauto_m2,80, maxpwm, &pwm2, &dir2);
                motor_simple_control(gauto_m3,80, maxpwm, &pwm3, &dir3);
                motor_simple_control(gauto_m4,80, maxpwm, &pwm4, &dir4);

            } else if (Rmayu1 == 1) {
                // 全自動モード: 壁からの距離と平行を PID で保ち、左右だけ手で操作する
                auto_mode(distance4 - LIDAR_OFFSET4, distance7 - LIDAR_OFFSET7, 0, AUTO_TARGET_DIST_MM);

                // sbus() の m1〜m4 の式で、ly を -auto_ly、rx を auto_rx に差し替えたもの。
                // auto_ly は ly と符号が逆 (PID 出力の符号は実機合わせ)。
                int auto_m1 = auto_ly + lx + auto_rx;
                int auto_m2 = auto_ly - lx + auto_rx;
                int auto_m3 = -auto_ly - lx + auto_rx;
                int auto_m4 = -auto_ly + lx + auto_rx;

                motor_simple_control(auto_m1, 80, maxpwm, &pwm1, &dir1);
                motor_simple_control(auto_m2, 80, maxpwm, &pwm2, &dir2);
                motor_simple_control(auto_m3, 80, maxpwm, &pwm3, &dir3);
                motor_simple_control(auto_m4, 80, maxpwm, &pwm4, &dir4);
            }

        // 足回りの pwm1〜pwm4 が決まった後に呼ぶ (safety() の頭打ちがそれを見るため)
        roller();

            time1 = now;
        }

        // 電磁弁と装填 (毎周回)
        //   ローラー停止中は電磁弁で撃ち、ローラー回転中は装填モーターで球を送る
        switch (Rtuno2) {
        case 0: // 打たない
            HAL_GPIO_WritePin(lock1_GPIO_Port, lock1_Pin, 0);
            HAL_GPIO_WritePin(lock2_GPIO_Port, lock2_Pin, 0);
            pwm9 = 0;
            pwm10 = 0;

            break;
        case 1:                // 打つ
            if (Lmayu2 == 0) { // ローラーが止まっている
                switch (Rmayu2) {
                case 0:
                    HAL_GPIO_WritePin(lock1_GPIO_Port, lock1_Pin, 1);
                    HAL_GPIO_WritePin(lock2_GPIO_Port, lock2_Pin, 0);
                    break;
                case 1:
                    HAL_GPIO_WritePin(lock1_GPIO_Port, lock1_Pin, 0);
                    HAL_GPIO_WritePin(lock2_GPIO_Port, lock2_Pin, 1);
                    break;
                }
            } else if (Lmayu2 == 1) { // ローラーが回っている
                HAL_GPIO_WritePin(lock1_GPIO_Port, lock1_Pin, 0);
                HAL_GPIO_WritePin(lock2_GPIO_Port, lock2_Pin, 0);
                if (Lmayu1 == 1) { // 上ローラー回転中 → 装填2で送る
                    pwm9 = 0;
                    pwm10 = 600;
                    roller_dir2 = 1; // 装填2 正転

                } else if (Lmayu1 == 0) { // 下ローラー回転中 → 装填1で送る
                    pwm9 = 600;
                    pwm10 = 0;
                    roller_dir1 = 1; // 装填1 正転
                }
            }

            break;
        }

        

        // 原点復帰中は上の指令より優先して装填モーターを逆転させる
        if (reset_flag1 == 1) {
            pwm9 = 600;
            roller_dir1 = 0; // 装填1 逆転リセット
        }
        if (reset_flag2 == 1) {
            pwm10 = 600;
            roller_dir2 = 0; // 装填2 逆転リセット
        }

        // 必ず PWM を出力する直前に呼ぶこと。これより後で pwm を書き換えると
        // 異常時の停止やローラーの頭打ちが効かなくなる。
         safety();

        // 基板 (2026_B_main) は PWMn と DIRn が同じドライバへ行く配線なので、
        // DIR はソフトの mN 番号ではなく「その PWM が出ている基板ch の DIR」を書く。
        // 例: m1 の PWM は PD15 = 基板の PWM3 なので、DIR は d3 (PE10)。
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm1); // m1 = 基板ch3 (PWM3=PD15)
        HAL_GPIO_WritePin(d3_GPIO_Port, d3_Pin, dir1);             // DIR3 = PE10
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm2); // m2 = 基板ch4 (PWM4=PD14)
        HAL_GPIO_WritePin(d4_GPIO_Port, d4_Pin, dir2);             // DIR4 = PD11
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm3); // m3 = 基板ch1 (PWM1=PD12)
        HAL_GPIO_WritePin(d1_GPIO_Port, d1_Pin, dir3);             // DIR1 = PB1
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm4); // m4 = 基板ch2 (PWM2=PD13)
        HAL_GPIO_WritePin(d2_GPIO_Port, d2_Pin, dir4);             // DIR2 = PB2

        // ローラーは常に一方向なので DIR は固定値。
        // 対になる 2 個は向かい合っているので、逆の値にして互いに逆回転させる。
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm5); // m5 上ローラー = 基板ch7 (PWM7=PE11)
        HAL_GPIO_WritePin(d7_GPIO_Port, d7_Pin, 0);                // DIR7 = PF12
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm6); // m6 下ローラー = 基板ch8 (PWM8=PE9)
        HAL_GPIO_WritePin(d8_GPIO_Port, d8_Pin, 0);                // DIR8 = PF13
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm7); // m7 上ローラー = 基板ch5 (PWM5=PE13)
        HAL_GPIO_WritePin(d5_GPIO_Port, d5_Pin, 1);                // DIR5 = PF3
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm8); // m8 下ローラー = 基板ch6 (PWM6=PE14)
        HAL_GPIO_WritePin(d6_GPIO_Port, d6_Pin, 1);                // DIR6 = PF14

        // 装填は正転/逆転リセットがあるので DIR は roller_dir を出す。
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm9);  // m9 装填1 = 基板ch10 (PWM10=PC7)
        HAL_GPIO_WritePin(d10_GPIO_Port, d10_Pin, roller_dir1);     // DIR10 = PA11
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm10); // m10 装填2 = 基板ch9 (PWM9=PC6)
        HAL_GPIO_WritePin(d9_GPIO_Port, d9_Pin, roller_dir2);       // DIR9 = PA12
        // 予備 (未使用)
        // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm11); // m11 = 基板ch11 (PWM11=PC8)
        // HAL_GPIO_WritePin(d11_GPIO_Port, d11_Pin, roller_dir); // DIR11 = PB12
        // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm12); // m12 = 基板ch12 (PWM12=PC9)
        // HAL_GPIO_WritePin(d12_GPIO_Port, d12_Pin, 1);         // DIR12 = PB11

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
    // 16bit の ID リストモードなので、Mask 側も含めた 4 つがそのまま受信する ID になる。
    // 受信するのは 0x001〜0x004 で、実際に使っているのは 0x001 (ローラーのエンコーダ値) だけ。
    CAN_FilterTypeDef filter;
    filter.FilterIdHigh = 0x001 << 5;               // 受信ID 1
    filter.FilterIdLow = 0x002 << 5;                // 受信ID 2
    filter.FilterMaskIdHigh = 0x003 << 5;           // 受信ID 3
    filter.FilterMaskIdLow = 0x004 << 5;            // 受信ID 4
    filter.FilterScale = CAN_FILTERSCALE_16BIT;     // 16モード
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // FIFO0へ格納
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDLIST; // IDリストモード
    filter.SlaveStartFilterBank = 14;
    filter.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(&hcan1, &filter);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 23;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 254;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 5;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 460800;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 100000;
  huart5.Init.WordLength = UART_WORDLENGTH_9B;
  huart5.Init.StopBits = UART_STOPBITS_2;
  huart5.Init.Parity = UART_PARITY_EVEN;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
  huart5.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 460800;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 100000;
  huart8.Init.WordLength = UART_WORDLENGTH_9B;
  huart8.Init.StopBits = UART_STOPBITS_2;
  huart8.Init.Parity = UART_PARITY_EVEN;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
  huart8.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, d5_Pin|d7_Pin|d8_Pin|d6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|d1_Pin|d2_Pin|d12_Pin
                          |d11_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(d3_GPIO_Port, d3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(d4_GPIO_Port, d4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, lock1_Pin|lock4_Pin|lock2_Pin|lock3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, d10_Pin|d9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : d5_Pin d7_Pin d8_Pin d6_Pin */
  GPIO_InitStruct.Pin = d5_Pin|d7_Pin|d8_Pin|d6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin d1_Pin d2_Pin d12_Pin
                           d11_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|d1_Pin|d2_Pin|d12_Pin
                          |d11_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : lock9_Pin lock8_Pin */
  GPIO_InitStruct.Pin = lock9_Pin|lock8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : d3_Pin */
  GPIO_InitStruct.Pin = d3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(d3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : d4_Pin */
  GPIO_InitStruct.Pin = d4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(d4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : lock1_Pin lock4_Pin lock2_Pin lock3_Pin */
  GPIO_InitStruct.Pin = lock1_Pin|lock4_Pin|lock2_Pin|lock3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : lock6_Pin lock7_Pin */
  GPIO_InitStruct.Pin = lock6_Pin|lock7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : d10_Pin d9_Pin */
  GPIO_InitStruct.Pin = d10_Pin|d9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

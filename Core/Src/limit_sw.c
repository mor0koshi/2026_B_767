/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : limit_sw.c
 * @brief          : リミットスイッチ・ボタンの読み取り (ノイズ除去)
 ******************************************************************************
 */
/* USER CODE END Header */
#include "function.h"

/*
 * リミットスイッチの読み取り (チャタリング/ノイズ除去)
 *
 * reset_flag は「一度 Low を読んだら立つ」「lock7/lock9 を踏むまで下りない」という
 * ラッチ構造なので、モーターのPWMノイズが 1 スキャン乗っただけで装填が回りっぱなしになる。
 * LIMIT_DEBOUNCE_MS の間 同じ値を読み続けたときだけ確定値を更新することで、
 * 単発のノイズをフラグまで通さない。
 */
#define LIMIT_DEBOUNCE_MS 20

uint8_t limit_read(limit_sw *sw) {
    uint8_t raw = (uint8_t)HAL_GPIO_ReadPin(sw->port, sw->pin);

    if (raw != sw->last) {
        // 値が動いた。ここから改めて安定時間を計り直す
        sw->last = raw;
        sw->changed = HAL_GetTick();
    } else if (raw != sw->stable && HAL_GetTick() - sw->changed >= LIMIT_DEBOUNCE_MS) {
        // 同じ値を十分な時間読み続けたので確定
        sw->stable = raw;
    }

    return sw->stable;
}

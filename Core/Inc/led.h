#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ステータス LED (LD1〜LD3) と LED テープ (lock3〜lock5) を光らせる。
 * LED テープの色 (赤 / 青) は USER ボタンの長押しで切り替える。
 * メインループで毎周回、safety() の後に呼ぶこと。
 */
void led(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */

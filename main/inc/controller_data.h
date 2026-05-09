#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// mypad構造体の定義
typedef struct {
    bool A;      // Aボタン: 1=押されている, 0=押されていない
    bool B;      // Bボタン
    bool X;      // Xボタン
    bool Y;      // Yボタン
    bool UP;     // 十字キー上
    bool DOWN;   // 十字キー下
    bool LEFT;   // 十字キー左
    bool RIGHT;  // 十字キー右
    bool L;      // Lボタン
    bool R;      // Rボタン
    bool ZL;     // ZLボタン
    bool ZR;     // ZRボタン
    bool TL;     // 左スティックの押し込み
    bool TR;     // 右スティックの押し込み
    bool MINUS; // セレクトボタン
    bool PLUS;  // スタートボタン
    bool HOME;  // システムボタン oobイベントで取得可能かも
    bool CAPTURE; // キャプチャボタン
    int16_t LX;     // 左スティックのX軸: -128～127
    int16_t LY;     // 左スティックのY軸: -128～127
    int16_t RX;     // 右スティックのX軸: -128～127
    int16_t RY;     // 右スティックのY軸: -128～127
    uint8_t battery_level; // バッテリー残量 (0-255)
    bool connected; // コントローラーが接続されているかどうか
} mypad_t;

extern mypad_t mypad;
extern const mypad_t EMPTY_MYPAD;
extern TaskHandle_t controller_task_handle;

void controller_task(void *pvParameters);
void controller_dump(mypad_t* pad);

#endif // CONTROLLER_H
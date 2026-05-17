/**
 * @file servo.h
 * @brief driver/ledcのサーボモーター制御用のオーバーラップ関数と構造体を提供
 * サーボの可動範囲は180度（0～πラジアン）と270度（0～3π/2ラジアン）
 * 関数群は外部で定義されたサーボ構造体とともに使うことを想定
 * 基本的には必要ないが，必要に応じて角度からデューティ値への変換関数も利用可能(servo_angle_to_duty)
 * サーボの角度は弧度法（ラジアン）で指定し、内部でLEDCのデューティ値に変換して制御
 * 使用例：
 * servo_t my_servo;
 * servo_init(&my_servo, SERVO_RANGE_180, LEDC_CHANNEL_0, LEDC_TIMER_0, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
 * // 90度に設定
 * my_servo.current_angle_rad = M_PI / 2;
 * servo_update_angle(&my_servo); 
 * 
 */

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <esp_err.h>
#include <driver/ledc.h>

// デフォルト値（16bitタイマー, 50Hz前提）
#define SERVO_ANGLE_MIN 1638      //16bitタイマーで0度に対応するデューティ値 (65535 * (0.5/20) = 1638)
#define SERVO_ANGLE_MAX 8192    //16bitタイマーで180度に対応するデューティ値 (65535 * (2.5/20) = 8192)
/**
 * サーボの可動範囲定義
 */
typedef enum {
    SERVO_RANGE_180,    // 0～π ラジアン (0～180度)
    SERVO_RANGE_270,    // 0～3π/2 ラジアン (0～270度)
} servo_range_t;

/**
 * サーボ制御構造体
 */
typedef struct {
    servo_range_t range;           // サーボの可動範囲
    int gpio_num;                  // サーボの制御に使用するGPIO番号
    ledc_channel_t channel;        // LEDCチャンネル設定
    ledc_timer_t timer;            // LEDCタイマー設定
    float angle_rad;               // 角度（ラジアン）
    uint32_t min_duty;             // 最小デューティ値 (0度/0rad時)default: SERVO_ANGLE_MIN
    uint32_t max_duty;             // 最大デューティ値 (180度/π or 270度/3π/2時) default: SERVO_ANGLE_MAX
} servo_t;

typedef ledc_timer_config_t servo_timer_config_t;
typedef ledc_channel_config_t servo_channel_config_t;

esp_err_t servo_timer_config(servo_timer_config_t *timer_conf);// leec_timmer_congfig()のオーバーラップ関数
esp_err_t servo_timer_config_default();// leec_timmer_congfig()のデフォルト設定用オーバーラップ関数
esp_err_t servo_channel_config(servo_channel_config_t *channel_conf);// leec_channel_congfig()のオーバーラップ関数
esp_err_t servos_channel_config(servo_t *servos, int servocount);
/**
 * @brief servo_tを初期化する
 * @returns 
 *      -ESP_OK: success
 *      -ESP_FAIL: nullpointer servo
 */
esp_err_t servo_init(servo_t *servo, servo_range_t range, int gpio_num, ledc_channel_t channel, ledc_timer_t timer, uint32_t min_duty, uint32_t max_duty);
/**
 * @brief servo_tの配列を初期化する
 * @returns 
 *      -ESP_OK: success
 *      -int i: i番目のサーボでエラー
 */
esp_err_t servos_range_and_pin_init(servo_t *servos, servo_range_t *ranges, int *servo_pins, int servocount);

/**
 * @brief サーボをservo_tのangle_radで指定した角度（弧度法）に設定する
 * @param servo: サーボ構造体ポインタ
 * @returns
 *      -ESP_OK: success
 *      -ESP_FAIL:nullpointer servo 
 *      -ESP_ERR_INVALID_ARG: servoのrangeもしくはangle_radサーボの可動範囲外
 */
esp_err_t servo_update_angle(servo_t *servo);

/**
 * @brief 複数のサーボを一括で指定角度に設定する
 * @param servos: サーボ構造体の配列
 * @param count: サーボの数
 * @returns 
 *      -ESP_OK: success
 *      -int i: i番目のサーボでエラー
 */
esp_err_t servos_update_angle(servo_t servos[], int count);

/**
 * @brief 角度（弧度法）をLEDCデューティ値に変換する
 *        範囲外の角度を指定した場合は、min_dutyまたはmax_dutyにクランプされる
 *        不正なrange指定の場合はmin_dutyが返される
 * @param servo: サーボ構造体ポインタ
 * @param angle_rad: 角度（ラジアン）
 * @param range: サーボの可動範囲
 * @param min_duty: 最小デューティ値
 * @param max_duty: 最大デューティ値
 * @return デューティ値
 */
uint32_t servo_angle_to_duty(float angle_rad, servo_range_t range, uint32_t min_duty, uint32_t max_duty);

#endif // SERVO_H
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_err.h>
#include <math.h>
#define ROBOMASTER_MAX_COUNT 8
#define ROBOMASTER_TXID_0 0x200
#define ROBOMASTER_TXID_1 0x1ff
#define MAX_CURRENT 16384 //ロボマスに遅れる最大電流値
#define MAX_SPEED_ON_MODE_ANGLE 300.0 //モーター軸基準
#define MOTOR_MIN_SPEED 3.0 //モーター軸基準,正常に動く最低rpm
#define ENCODER_RESOLUTION 8192 //モーター軸基準
#define MOTOR_MAX_RPM 500.0 //モータ軸基準
#define ROBOMAS_M3_GEAR_RATIO (3591.0 / 187.0)//M3508

//PID制御用の構造体
typedef enum{
    TARGET_MODE_NONE,
    TARGET_MODE_SPEED,
    TARGET_MODE_ANGLE
}target_mode_t;

typedef struct{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float integral_limit;
    float prev_error;
    float feedforward_current;
    float output_limit;
}pidK_t;

typedef struct {
    target_mode_t mode;
    union {
        int16_t target_current;//TARGET_MODE_NONE
        float target_speed;//TARRGET_MODE_SPEED,rpm
        float target_angle;//TARRGET_MODE_ANGLE,rad
    };
    pidK_t speed;
} pid_t;

typedef struct {
    int16_t angle;//0~8191,90度2048
    float speed;//rpm,LPFで更新するためfloat
    int16_t torque;//トルク電流
    int8_t  temperature;//℃

    int rotation;//回転数
    float gear_ratio;//1:gear_ratio＝モーター軸:出力軸のギア比

} robomaster_t;

extern robomaster_t robomas[];//can_rx_task()で受け取った値を保存する構造体
extern robomaster_t prev_robomas[];//robomasの以前の値
extern pid_t pid[]; //PID制御のパラメータと目標値，制御モードを格納するグローバル変数
extern int16_t current[];//送る電流値
extern TaskHandle_t can_rx_task_handle;//can_rx_taskのハンドラー
extern TaskHandle_t can_tx_task_handle;//can_tx_taskのハンドラー

/**
*@brief モーター軸基準のENCODER_RESOLUTIONの解像度で現在の角度を返す
 */
int32_t robomas_get_position(robomaster_t *r);

/**
*@brief 出力軸基準で現在の角度を弧度法で返す
*/
float robomas_get_position_rad(robomaster_t *r);

/**
*@brief ロボマスに送る電流値を一つ計算する．
*/
float pid_calc(pid_t *pid, robomaster_t *robomas, float dt);


/**
*@brief ロボマスター専用のTWAI設定
*       can_*x_task()を起動する前に実行
*/
esp_err_t can_driver_install_default_and_start(int can_tx_gpio,int can_rx_gpio);

/**
*@brief can送信task
*/
void can_rx_task(void *arg);

/**
*@brief current[]の電流値を送る
*/
esp_err_t can_tx(uint32_t id);

/**
*@brief can受信task
*/
void can_tx_task(void *arg);

//dump
void robomas_dump(robomaster_t *rbms);
void current_dump(int16_t cr[]);
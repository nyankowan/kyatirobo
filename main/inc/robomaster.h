#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_err.h>
#include <math.h>
#define ROBOMASTER_MAX_COUNT 8
#define ROBOMASTER_TXID_0 0x200
#define ROBOMASTER_TXID_1 0x1ff
#define MAX_CURRENT 16384
#define MAX_SPEED_ON_MODE_ANGLE 300
#define ANGLE_RANGE 8192
#define MOTOR_MAX_RPM 500

//PID制御用の構造体
typedef enum{
    TARGET_MODE_NONE=0,
    TARGET_MODE_TORQUE=1,
    TARGET_MODE_SPEED=2,
    TARGET_MODE_ANGLE=3
}target_mode_t;
typedef struct{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float integral_limit;
    float prev_error;
    float output_limit;//torqueだとMAX_CURRENT,speedだとMOTOR_MAX_RPM/gear_ratio（MAX_CURRENT）,angleだとMAX_SPEED_ON_MODE_ANGLE(v/r)
}pidK_t;

typedef struct {
    target_mode_t mode;
    union {
        //target_mode_tの値によって異なる名前を使えるようにしている
        int16_t target;//ひとまとめにtargetと捉えられるようにする
        int16_t target_current;//TARGET_MODE_NONE
        int16_t target_torque;//TARRGET_MODE_TORQUE
        int16_t target_speed;//TARRGET_MODE_SPEED,rpm
        int16_t target_angle;//TARRGET_MODE_ANGLE,rad
    };
    pidK_t torque;
    pidK_t speed;
    pidK_t angle;
    float gear_ratio;
} pid_t;

typedef struct {
    int16_t angle;//0~8191,90度2048
    int16_t speed;
    int16_t torque;
    int8_t  temperature;

    int rotation;//回転数

} robomaster_t;

extern robomaster_t robomas[];//can_rx_task()で受け取った値を保存する構造体
extern robomaster_t prev_robomas[];//robomasの以前の値
extern pid_t pid[]; //PID制御のパラメータと目標値，制御モードを格納するグローバル変数
extern int16_t current[];
extern TaskHandle_t can_rx_task_handle;
extern TaskHandle_t can_tx_task_handle;

int32_t robomas_get_position(robomaster_t *r);
// PID制御関数のプロトタイプ宣言
float pid_calc(pid_t *pid, robomaster_t *robomas, float error, float dt);

esp_err_t can_tx(uint32_t id);

/**
ロボマスター専用のTWAI設定
can_*x_task()を起動する前に実行
*/
esp_err_t can_driver_install_default_and_start(int can_tx_gpio,int can_rx_gpio);

//can通信関数のプロトタイプ宣言
void can_rx_task(void *arg);
void can_tx_task(void *arg);
void robomas_dump(robomaster_t *rbms);
void current_dump(int16_t cr[]);
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_err.h>

//PID制御用の構造体
typedef enum{
    TARGET_MODE_TORQUE=0,
    TARGET_MODE_NONE=TARGET_MODE_TORQUE,
    TARGET_MODE_SPEED,
    TARGET_MODE_ANGLE
}target_mode_t;

typedef union {
    int16_t torque;
    int16_t speed;
    int16_t angle;
}target_t;

typedef struct {
    target_mode_t mode;
    target_t target;
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
} pid_t;

typedef struct {
    pid_t pid;
    int16_t current_angle;//0~8191,90度2048
    int16_t current_speed;
    int16_t current_torque;
    int16_t  current_temperature;

} robomaster_t;

extern int16_t motor_speed[4]; //モーターの現在速度を格納するグローバル変数
extern int16_t target_speed[4]; //モーターの目標速度を格納するグローバル変数
extern int16_t current[4]; //モーターの電流値を格納するグローバル変数
extern pid_t pid[4]; //PID制御のパラメータと状態を格納するグローバル変数

extern TaskHandle_t can_rx_task_handle;
extern TaskHandle_t can_tx_task_handle;

// PID制御関数のプロトタイプ宣言
int16_t pid_calc(pid_t *pid, float target, float current, float dt);

esp_err_t can_driver_install_default_and_start(void);



//can通信関数のプロトタイプ宣言
void can_rx_task(void *arg);
void can_tx_task(void *arg);
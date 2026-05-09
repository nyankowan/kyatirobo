#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_err.h>

//PID制御用の構造体
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
} PID;

extern float motor_speed[4]; //モーターの現在速度を格納するグローバル変数
extern float target_speed[4]; //モーターの目標速度を格納するグローバル変数
extern int16_t current[4]; //モーターの電流値を格納するグローバル変数
extern PID pid[4]; //PID制御のパラメータと状態を格納するグローバル変数

extern TaskHandle_t can_rx_task_handle;
extern TaskHandle_t can_tx_task_handle;

// PID制御関数のプロトタイプ宣言
int16_t pid_calc(PID *pid, float target, float current, float dt);

esp_err_t can_driver_install_default_and_start(void);



//can通信関数のプロトタイプ宣言
void can_rx_task(void *arg);
void can_tx_task(void *arg);
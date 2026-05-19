#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_err.h>
#define ROBOMASTER_MAX_COUNT 4//一つのCAN BUSが扱えるロボマスの上限
#define BUS_NUM 2//使用するCAN BUSの数

//PID制御用の構造体
typedef enum{
    TARGET_MODE_NONE,
    TARGET_MODE_TORQUE,
    TARGET_MODE_SPEED,
    TARGET_MODE_ANGLE
}target_mode_t;

typedef struct {
    target_mode_t mode;
    union {
        //target_mode_tの値によって異なる名前を使えるようにしている
        int16_t target;//ひとまとめにtargetと捉えられるようにする
        int16_t target_current;//TARGET_MODE_NONE
        int16_t target_torque;//TARRGET_MODE_TORQUE
        int16_t target_speed;//TARRGET_MODE_SPEED
        int16_t target_angle;//TARRGET_MODE_ANGLE
    };
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
} pid_t;

typedef struct {
    int16_t angle;//0~8191,90度2048
    int16_t speed;
    int16_t torque;
    int8_t  temperature;

    int rotation;//回転数

} robomaster_t;

extern robomaster_t robomas[ROBOMASTER_MAX_COUNT*BUS_NUM];//can_rx_task()で受け取った値を保存する構造体
extern robomaster_t prev_robomas[ROBOMASTER_MAX_COUNT*BUS_NUM];//robomasの以前の値
extern pid_t pid[ROBOMASTER_MAX_COUNT*BUS_NUM]; //PID制御のパラメータと目標値，制御モードを格納するグローバル変数
extern int16_t current[ROBOMASTER_MAX_COUNT*BUS_NUM];

extern twai_message_t tx_msg;
extern twai_message_t rx_msg;

extern twai_handle_t can_bus_0;
extern twai_handle_t can_bus_1;

extern TaskHandle_t can_rx_task_handle;
extern TaskHandle_t can_tx_task_handle;

/**
*ロボマスター専用のTWAI設定
*can_*x_task()を起動する前に実行
*/
esp_err_t can_driver_install_default_and_start(int can_tx_gpio,int can_rx_gpio);
/**
*esp32の場合，物理的にtwaコントローラが一つしか実装されていないため，呼び出してはいけない．
*
*/
esp_err_t can_driver_install_default_and_start_2bus(int can_tx_gpio_0,int can_rx_gpio_0,int can_tx_gpio_1,int can_rx_gpio_1);


void set_robomas_to_rxdata(int i);
void set_txdata_to_calced_current(int num, int loop);
void can_rx();
/**
*@param loop:loop ms to use pid_calc()
*/
void can_tx(int loop);

//can通信関数のプロトタイプ宣言
void can_rx_task(void *arg);
void can_tx_task(void *arg);

void can_rx_v2(twai_handle_t can_bus);
void can_tx_v2(twai_handle_t can_bus, int loop);

void can_rx_task_2bus(void *arg);
void can_tx_task_2bus(void *arg);

/**
*@param dt:Time elapsed since this function was last called.
*
*/
int16_t pid_calc(pid_t *pid, robomaster_t *robomas, float dt);

void set_pid_target(int i, int16_t target);

void robomas_dump(robomaster_t *rbms);
void current_dump(int16_t cr[ROBOMASTER_MAX_COUNT]);
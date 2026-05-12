#include "robomaster.h"
#define ROBOMASTER_MAX_COUNT 4//Never change this value
#define CAN_TX_GPIO 5
#define CAN_RX_GPIO 4   //don't setting 6,flash/spi path interference
int16_t motor_speed[ROBOMASTER_MAX_COUNT] = {0,0,0,0};    //モーターの現在速度を格納するグローバル変数
int16_t target_speed[ROBOMASTER_MAX_COUNT] = {0,0,0,0};   //モーターの目標速度を格納するグローバル変数
int16_t current[ROBOMASTER_MAX_COUNT] = {0,0,0,0};      //モーターの電流値を格納するグローバル変数

pid_t pid[ROBOMASTER_MAX_COUNT] = {
    { .kp = 10.0, .ki = 0.0, .kd = 0.0, .integral = 0, .prev_error = 0 },
    { .kp = 15.0, .ki = 3.0, .kd = 0.001, .integral = 0, .prev_error = 0 },
    { .kp = 10.0, .ki = 20.0, .kd = 0.0, .integral = 0, .prev_error = 0 },
    { .kp = 10.0, .ki = 20.0, .kd = 0.01, .integral = 0, .prev_error = 0 }
};

TaskHandle_t can_rx_task_handle = NULL;
TaskHandle_t can_tx_task_handle = NULL;

void can_rx_task(void *arg)
{
    twai_message_t rx_msg;
    
    while (1) {
        if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {

            if (rx_msg.identifier >= 0x201 && rx_msg.identifier <= 0x204) {
                int i = rx_msg.identifier - 0x201;

                motor_speed[i] =(int16_t)((rx_msg.data[2] << 8) | rx_msg.data[3]);
            }
        }
    }
}

void can_tx_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    twai_message_t tx_msg = {
        .identifier = 0x200,
        .data_length_code = 8
    };
    int loop = 10;//制御周期10ms
    float dt = loop * 0.001f;
    while (1) {
        //PID制御計算
        for (int i = 0; i < ROBOMASTER_MAX_COUNT; i++) {
            current[i] = pid_calc(&pid[i], target_speed[i], motor_speed[i], dt);
        }

        //電流制限
        /*
        for (int i = 0; i < ROBOMASTER_MAX_COUNT; i++) {
            if (current[i] > 10000) current[i] = 10000;
            if (current[i] < -10000) current[i] = -10000;
        }
        */

        //CANメッセージのデータフィールドに電流値を格納
        for (int i = 0; i < 4; i++) {
            tx_msg.data[i*2]     = current[i] >> 8;
            tx_msg.data[i*2 + 1] = current[i] & 0xFF;
        }
        //CANメッセージの送信
        twai_transmit(&tx_msg, pdMS_TO_TICKS(1));
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(loop));
    }
}



int16_t pid_calc(pid_t *pid, float target, float current, float dt)
{
    float error = target - current;

    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;

    pid->prev_error = error;
    // アンチワインドアップ
    if (pid->integral > 10000) pid->integral = 10000;
    if (pid->integral < -10000) pid->integral = -10000;

    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    return (int16_t)output;
}

esp_err_t can_driver_install_default_and_start(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t e = twai_driver_install(&g_config, &t_config, &f_config);//could not find property 12,11
    if(e != ESP_OK) return e;
    return twai_start();
}

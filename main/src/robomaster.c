#include "robomaster.h"

robomaster_t robomas[ROBOMASTER_MAX_COUNT];
robomaster_t prev_robomas[ROBOMASTER_MAX_COUNT];
pid_t pid[ROBOMASTER_MAX_COUNT] = {
    {.mode = TARGET_MODE_NONE,      .kp = 10.0, .ki = 0.0, .kd = 0.0, .integral = 0, .prev_error = 0 },
    {.mode = TARGET_MODE_TORQUE,    .kp = 15.0, .ki = 3.0, .kd = 0.001, .integral = 0, .prev_error = 0 },
    {.mode = TARGET_MODE_SPEED,     .kp = 10.0, .ki = 20.0, .kd = 0.0, .integral = 0, .prev_error = 0 },
    {.mode = TARGET_MODE_ANGLE,     .kp = 10.0, .ki = 20.0, .kd = 0.01, .integral = 0, .prev_error = 0 }
};
int16_t current[ROBOMASTER_MAX_COUNT] = {0,0,0,0};//ロボマスに実際に送る電流値
twai_message_t tx_msg = {
    .identifier = 0x200,
    .data_length_code = 8
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
                prev_robomas[i].angle = robomas[i].angle;
                prev_robomas[i].speed = robomas[i].speed;
                prev_robomas[i].torque = robomas[i].torque;
                prev_robomas[i].temperature = robomas[i].temperature;

                robomas[i].angle = (int16_t)((rx_msg.data[0] << 8) | rx_msg.data[1]);
                robomas[i].speed = (int16_t)((rx_msg.data[2] << 8) | rx_msg.data[3]);
                robomas[i].torque = (int16_t)((rx_msg.data[4] << 8) | rx_msg.data[5]);
                robomas[i].temperature = (int8_t)rx_msg.data[6];
                if(robomas[i].angle-prev_robomas[i].angle>4095){
                    rotation--;
                }else if(robomas[i].angle-prev_robomas[i].angle<-4096){
                    rotation++;
                }
            }
        }
    }
}

void can_tx(int loop){
    //PID制御計算
        for (int i = 0; i < ROBOMASTER_MAX_COUNT; i++) {
            current[i] = pid_calc(&pid[i], &robomas[i], loop * 0.001f);
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
}

void can_tx_task(void *arg)
{
    //TickType_t last_wake = xTaskGetTickCount();
    int loop = 10;//制御周期10ms
    while (1) {
        can_tx(loop);
        vTaskDelay(loop / portTICK_PERIOD_MS);
        //vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(loop));wdt err?
    }
}

int16_t pid_calc(pid_t *pid, robomaster_t *robomas, float dt)
{
    float error;
    switch(pid->mode){
        default:
        case TARGET_MODE_NONE:
            return pid->target_current;
        case TARGET_MODE_TORQUE:
            error = pid->target_torque - robomas->torque;
            break;
        case TARGET_MODE_SPEED:
            error = pid->target_speed - robomas->speed;
            break;
        case TARGET_MODE_ANGLE:
            error = pid->target_angle - robomas->angle;
            break;
    }

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




void set_pid_target(int i, int16_t target){
    pid[i].target = target;
    pid[i].integral = 0;
    pid[i].prev_error = 0;
}

//don't setting gpio num 6,Bluepad32's flash/spi path interference
esp_err_t can_driver_install_default_and_start(int tx_gpio,int rx_gpio) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_gpio, rx_gpio, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t e = twai_driver_install(&g_config, &t_config, &f_config);//could not find property 12,11
    if(e != ESP_OK) return e;
    return twai_start();
}

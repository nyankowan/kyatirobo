#include "robomaster.h"

robomaster_t robomas[ROBOMASTER_MAX_COUNT] = {{0},{0},{0},{0},{0},{0},{0},{0}};
robomaster_t prev_robomas[ROBOMASTER_MAX_COUNT] = {{0},{0},{0},{0},{0},{0},{0},{0}};
pid_t pid[ROBOMASTER_MAX_COUNT] = {
    {.mode = TARGET_MODE_NONE,      .kp = 10.0, .ki = 0.0, .kd = 0.0, .integral = 0, .prev_error = 0 },
    {.mode = TARGET_MODE_TORQUE,    .kp = 15.0, .ki = 3.0, .kd = 0.001, .integral = 0, .prev_error = 0 },
    {.mode = TARGET_MODE_SPEED,     .kp = 5.0,  .ki = 0.0, .kd = 0.0, .integral = 0, .prev_error = 0 },
    {.mode = TARGET_MODE_ANGLE,     .kp = 0.01, .ki = 0.1, .kd = 0.02, .integral = 0, .prev_error = 0, .precurrent = 500, .threshold_current = 100},
    {.mode = TARGET_MODE_NONE,      .kp = 10.0, .ki = 0.0, .kd = 0.0, .integral = 0, .prev_error = 0 },
    {.mode = TARGET_MODE_TORQUE,    .kp = 15.0, .ki = 3.0, .kd = 0.001, .integral = 0, .prev_error = 0 },
    {.mode = TARGET_MODE_SPEED,     .kp = 5.0,  .ki = 0.0, .kd = 0.0, .integral = 0, .prev_error = 0 },
    {.mode = TARGET_MODE_ANGLE,     .kp = 5.0, .ki = 0.1, .kd = 0.01, .integral = 0, .prev_error = 0 }
    
};
int16_t current[ROBOMASTER_MAX_COUNT] = {0,0,0,0,0,0,0,0};//ロボマスに実際に送る電流値
twai_message_t tx_msg = {
    .data_length_code = 8
};

TaskHandle_t can_rx_task_handle = NULL;
TaskHandle_t can_tx_task_handle = NULL;

void can_rx_task(void *arg)
{
    twai_message_t rx_msg;
    while (1) {
        if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
            if (rx_msg.identifier >= 0x201 && rx_msg.identifier <= 0x208) {
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
                    robomas[i].rotation--;
                }else if(robomas[i].angle-prev_robomas[i].angle<-4096){
                    robomas[i].rotation++;
                }
            }
        }
    }
}

esp_err_t can_tx(uint32_t id){
        //CANメッセージのデータフィールドに電流値を格納
        tx_msg.identifier = id;
        int head = 0;
        if(id==0x1FF)head = 4;
        for (int i = 0; i < 4; i++) {
            tx_msg.data[i*2]     = current[i + head] >> 8;
            tx_msg.data[i*2 + 1] = current[i + head] & 0xFF;
        }
        //CANメッセージの送信
        return twai_transmit(&tx_msg, pdMS_TO_TICKS(1));
}

void can_tx_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    int loop = 10;//制御周期10ms
    while (1) {
        //PID制御計算
        for (int i = 0; i < ROBOMASTER_MAX_COUNT; i++) {
            current[i] = pid_calc(&pid[i], &robomas[i], loop * 0.001f);
        }
        //電流制限
        for (int i = 0; i < ROBOMASTER_MAX_COUNT; i++) {
            if (current[i] > 16384) current[i] = 16384;
            if (current[i] < -16384) current[i] = -16384;
        }

        can_tx(0x200);
        can_tx(0x1FF);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(loop));
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
            error = pid->target_angle*19 - (robomas->angle + robomas->rotation*8192);
            break;
    }

    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;

    pid->prev_error = error;
    // アンチワインドアップ
    if (pid->integral > 10000) pid->integral = 10000;
    if (pid->integral < -10000) pid->integral = -10000;

    int sign = 0;
    if(error > pid->threshold_current){
        sign = 1;
    }else if(error< -1*pid->threshold_current){
        sign = -1;
    }
    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative
                 + pid->precurrent * sign;

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
void robomas_dump(robomaster_t *rbms){
    fprintf(stderr,"angle:%5d,\tspeed:%5d,\ttorque:%5d,\ttemperature:%5d,\trotation:%d\n",rbms->angle,rbms->speed,rbms->torque,rbms->temperature,rbms->rotation);
}
void current_dump(int16_t cr[ROBOMASTER_MAX_COUNT]){
    fprintf(stderr,"0:%5d,\t1:%5d,\t2:%5d,\t3:%5d,\t4:%5d,\t5:%5d,\t6:%5d,\t7:%5d\n",cr[0],cr[1],cr[2],cr[3],cr[4],cr[5],cr[6],cr[7]);
}
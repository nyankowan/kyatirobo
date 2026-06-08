#include "robomaster.h"
#include <math.h>
#include "driver/twai.h"
#define ROBOMAS_NUM 5

robomaster_t robomas[ROBOMAS_NUM] = {
    {.gear_ratio = ROBOMAS_M3_GEAR_RATIO, .init_angle = 0},
    {.gear_ratio = ROBOMAS_M2_GEAR_RATIO, .init_angle = INIT_ANGLE_R},
    {.gear_ratio = ROBOMAS_M3_GEAR_RATIO, .init_angle = 0},
    {.gear_ratio = ROBOMAS_M2_GEAR_RATIO, .init_angle = 0},
    {.gear_ratio = ROBOMAS_M3_GEAR_RATIO, .init_angle = 0}
};
robomaster_t prev_robomas[ROBOMAS_NUM] = {{0},{0},{0},{0},{0}};
pid_t pid[ROBOMAS_NUM] = {
//  {.mode= TARGET_MODE_ANGLE,  .speed = {.Kp = 10, .Ki = 0.1, .Kd = 0, .integral = 0, .integral_limit = 10000, .prev_error = 0, .output_limit = MAX_CURRENT}},
    {.mode= TARGET_MODE_ANGLE,  .speed = {.Kp = 10, .Ki = 0.1, .Kd = 0, .integral = 0, .integral_limit = 10000, .prev_error = 0, .feedforward_current = 1500, .output_limit = MAX_CURRENT}},
    {.mode= TARGET_MODE_ANGLE,  .speed = {.Kp = 10, .Ki = 0.1, .Kd = 0, .integral = 0, .integral_limit = 10000, .prev_error = 0, .feedforward_current = 1500, .output_limit = MAX_CURRENT}},
    {.mode= TARGET_MODE_SPEED,  .speed = {.Kp = 10, .Ki = 0.1, .Kd = 0, .integral = 0, .integral_limit = 10000, .prev_error = 0, .output_limit = MAX_CURRENT}},
    {.mode= TARGET_MODE_ANGLE,  .speed = {.Kp = 10, .Ki = 0.1, .Kd = 0, .integral = 0, .integral_limit = 10000, .prev_error = 0, .feedforward_current = 1500, .output_limit = MAX_CURRENT}},
    {.mode= TARGET_MODE_NONE,  .speed = {.Kp = 0, .Ki = 0, .Kd = 0, .integral = 0, .integral_limit = 0, .prev_error = 0, .output_limit = 0}}
};
int init_angle[ROBOMAS_NUM] = {0,0,0,0,0};
int16_t current[ROBOMAS_NUM] = {0,0,0,0,0};
twai_message_t tx_msg = {
    .data_length_code = 8
};

TaskHandle_t can_rx_task_handle = NULL;
TaskHandle_t can_tx_task_handle = NULL;

int32_t robomas_get_position(robomaster_t *r)
{
    return r->angle + r->rotation * ENCODER_RESOLUTION;
}

float robomas_get_position_rad(robomaster_t *r)
{
    return robomas_get_position(r)
        * (2.0f * M_PI / ENCODER_RESOLUTION)
        / r->gear_ratio;
}

/**
*@brief スピード用のpid計算
*/
float calc_speed(pid_t *pid, float error, float dt){
    pidK_t *pidK = &pid->speed; 
    float derivative = (error - pidK->prev_error) / dt;
    pidK->prev_error = error;
    // 仮出力
    float result =
        pidK->Kp * error
        + pidK->Ki * pidK->integral
        + pidK->Kd * derivative;
    // 条件付き積分
    if (
        fabs(result) < pidK->output_limit ||
        (result > 0 && error < 0) ||
        (result < 0 && error > 0)
    ) {
        pidK->integral += error * dt;
    }
    // integral clamp
    if (pidK->integral > pidK->integral_limit)pidK->integral = pidK->integral_limit;
    if (pidK->integral < -pidK->integral_limit)pidK->integral = -pidK->integral_limit;
    // 最終出力
    result =
        pidK->Kp * error
        + pidK->Ki * pidK->integral
        + pidK->Kd * derivative;
        //feedforward
        //静止摩擦打ち消し
    float target_speed = error + robomas->speed;
    if(fabs(target_speed) > 1.0) result += pidK->feedforward_current * tanhf(target_speed / 50.0);
    //if(pid->mode == TARGET_MODE_ANGLE)
    //result += mgrsinθ;


    // 出力制限
    if (result > pidK->output_limit)result = pidK->output_limit;
    if (result < -pidK->output_limit)result = -pidK->output_limit;

    return result;
}

float pid_calc(pid_t *pid, robomaster_t *robomas, float dt)
{   
    float error;
    float current = 0;
    switch(pid->mode){
        default:
        case TARGET_MODE_NONE:
            current = pid->target_current;
            break;
        case TARGET_MODE_SPEED:
            error = pid->target_speed * robomas->gear_ratio - robomas->speed;
            current = calc_speed(pid, error, dt);
            break;
        case TARGET_MODE_ANGLE:
            error = pid->target_angle  - (robomas_get_position_rad(robomas) - robomas->init_angle);
            float speed_cmd = MAX_SPEED_ON_MODE_ANGLE * tanhf(error/0.1);
            ////動き出し最低保証rpm
            if(fabs(speed_cmd) > 0.1 ){
                speed_cmd += copysignf(MOTOR_MIN_SPEED, speed_cmd);
            }
            current = calc_speed(pid, speed_cmd - robomas->speed, dt);
            break;
    }
    //ロボマスに送れる最大電流
    if(current > MAX_CURRENT) current = MAX_CURRENT;
    else if(current < -MAX_CURRENT) current = -MAX_CURRENT;
    return (int16_t)current;
}

void can_rx_task(void *arg)
{
    twai_message_t rx_msg;
    while (1) {
        if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
            if (rx_msg.identifier >= 0x201 && rx_msg.identifier <= 0x208) {
                int i = rx_msg.identifier - 0x201;
                prev_robomas[i] = robomas[i];

                robomas[i].angle = (int16_t)((rx_msg.data[0] << 8) | rx_msg.data[1]);
                robomas[i].speed = robomas[i].speed * 0.1 + ((int16_t)((rx_msg.data[2] << 8) | rx_msg.data[3])) * 0.9;
                // robomas[i].speed = (int16_t)((rx_msg.data[2] << 8) | rx_msg.data[3]);
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
        if(id==ROBOMASTER_TXID_1)head = 4;
        for (int i = 0; i < 4; i++) {
            int idx = i + head;

            int16_t out = 0;
            if(idx < ROBOMAS_NUM){
                out = current[idx];
            }

            tx_msg.data[i*2]     = out >> 8;
            tx_msg.data[i*2 + 1] = out & 0xFF;
        }
        //CANメッセージの送信
        return twai_transmit(&tx_msg, pdMS_TO_TICKS(1));
}

void can_tx_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    int loop = 2;//制御周期ms sdkconfigでCONFIG_FREERTOS_HZ=1000にしておく
    while (1) {
        twai_status_info_t s;
        twai_get_status_info(&s);

        if(s.state == TWAI_STATE_BUS_OFF){
            printf("BUS OFF\n");
            twai_initiate_recovery();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        //PID制御計算
        for (int i = 0; i < ROBOMAS_NUM; i++) {
            current[i] = pid_calc(&pid[i], &robomas[i], loop * 0.001f);
        }
        can_tx(ROBOMASTER_TXID_0);
        can_tx(ROBOMASTER_TXID_1);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(loop));
    }
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
    fprintf(stderr,"angle:%5d,\tspeed:%5f,\ttorque:%5d,\ttemperature:%5d,\trotation:%d\n",rbms->angle,rbms->speed,rbms->torque,rbms->temperature,rbms->rotation);
}
void current_dump(int16_t cr[]){
    fprintf(stderr,"currents: ");
    for(int i = 0; i< ROBOMAS_NUM;i++){
        fprintf(stderr,"%d: %6d | ",i,cr[i]);
    }
    fprintf(stderr,"\n");
}
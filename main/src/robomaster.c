#include "robomaster.h"
#define ROBOMAS_NUM 5
robomaster_t robomas[ROBOMAS_NUM] = {{0},{0},{0},{0},{0}};
robomaster_t prev_robomas[ROBOMAS_NUM] = {{0},{0},{0},{0},{0}};
pid_t pid[ROBOMAS_NUM] = {
    {.mode = TARGET_MODE_NONE, .torque = {.Kp = 0, .Ki = 0, .Kd = 0, .integral = 0, .prev_error = 0}, .speed = {.Kp = 0, .Ki = 0, .Kd = 0, .integral = 0, .prev_error = 0}, .angle = {.Kp = 0, .Ki = 0, .Kd = 0, .integral = 0, .prev_error = 0}},
    {TARGET_MODE_TORQUE}
    {}
    {}
    {}
};
int16_t current[ROBOMAS_NUM] = {0,0,0,0,0};//ロボマスに実際に送る電流値
twai_message_t tx_msg = {
    .data_length_code = 8
};

TaskHandle_t can_rx_task_handle = NULL;
TaskHandle_t can_tx_task_handle = NULL;

int32_t robomas_get_position(robomaster_t *r)
{
    return r->angle + r->rotation * 8192;
}

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
        if(id==ROBOMASTER_TXID_1)head = 4;
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
    int loop = 1;//制御周期10ms
    while (1) {
        //PID制御計算
        for (int i = 0; i < ROBOMAS_NUM; i++) {
            current[i] = pid_calc(&pid[i], &robomas[i], 0, loop * 0.001f);
        }
        can_tx(ROBOMASTER_TXID_0);
        can_tx(ROBOMASTER_TXID_1);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(loop));
    }
}

float pid_calc(pid_t *pid, robomaster_t *robomas, float error, float dt)
{
    float calc(target_mode_t mode, float error){
         pidK_t *pidK = &pid->torque + (mode - 1);

    float derivative =
        (error - pidK->prev_error) / dt;

    pidK->prev_error = error;

    //--------------------------------
    // 仮出力
    //--------------------------------

    float result =
          pidK->Kp * error
        + pidK->Ki * pidK->integral
        + pidK->Kd * derivative;

    //--------------------------------
    // 条件付き積分
    //--------------------------------

    if (
        fabs(result) < pidK->output_limit ||
        (result > 0 && error < 0) ||
        (result < 0 && error > 0)
    ) {
        pidK->integral += error * dt;
    }

    //--------------------------------
    // integral clamp
    //--------------------------------

    if (pidK->integral > pidK->integral_limit)
        pidK->integral = pidK->integral_limit;

    if (pidK->integral < -pidK->integral_limit)
        pidK->integral = -pidK->integral_limit;

    //--------------------------------
    // 最終出力
    //--------------------------------

    result =
          pidK->Kp * error
        + pidK->Ki * pidK->integral
        + pidK->Kd * derivative;

    //--------------------------------
    // 出力制限
    //--------------------------------

    if (result > pidK->output_limit)
        result = pidK->output_limit;

    if (result < -pidK->output_limit)
        result = -pidK->output_limit;

    return result;
    }
    int current = 0;
    switch(pid->mode){
        default:
        case TARGET_MODE_NONE:
            return pid->target_current;
        case TARGET_MODE_TORQUE:
            error = pid->target_torque - robomas->torque;
            current = calc(TARGET_MODE_TORQUE, error);
            break;
        case TARGET_MODE_SPEED:
            error = pid->target_speed * pid->gear_ratio - robomas->speed;
            current = calc(TARGET_MODE_TORQUE, calc(TARGET_MODE_SPEED, error));
            break;
        case TARGET_MODE_ANGLE:
            error = pid->target_angle * pid->gear_ratio * ANGLE_RANGE/(2.0 * M_PI) - robomas_get_position(robomas);
            current = calc(TARGET_MODE_TORQUE, calc(TARGET_MODE_SPEED, calc(TARGET_MODE_ANGLE,error)));
            break;
    }
    //フォードフォワード
    //current += mgrsinθ
    if(current > MAX_CURRENT) current = MAX_CURRENT;
    else if(current < -MAX_CURRENT) current = -MAX_CURRENT;
    return (int16_t)current;
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
void current_dump(int16_t cr[]){
    fprintf(stderr,"currents: ");
    for(int i = 0; i< ROBOMAS_NUM;i++){
        fprintf(stderr,"%1d: %d\t",i,cr[i]);
    }
    fprintf(stderr,"\n");
}
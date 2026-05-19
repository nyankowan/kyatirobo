#include "robomaster.h"

robomaster_t robomas[ROBOMASTER_MAX_COUNT*BUS_NUM] = {{0},{0},{0},{0},
                                                      {0},{0},{0},{0}};
robomaster_t prev_robomas[ROBOMASTER_MAX_COUNT*BUS_NUM] = {{0},{0},{0},{0},
                                                           {0},{0},{0},{0}};
pid_t pid[ROBOMASTER_MAX_COUNT*BUS_NUM] = {
    {.mode = TARGET_MODE_NONE,      .kp = 10.0, .ki = 0.0, .kd = 0.0, .integral = 0, .prev_error = 0 },
    {.mode = TARGET_MODE_TORQUE,    .kp = 15.0, .ki = 3.0, .kd = 0.001, .integral = 0, .prev_error = 0 },
    {.mode = TARGET_MODE_SPEED,     .kp = 5.0 , .ki = 0.0, .kd = 0.0, .integral = 0, .prev_error = 0 },
    {.mode = TARGET_MODE_ANGLE,     .kp = 10.0, .ki = 3.0, .kd = 0.01, .integral = 0, .prev_error = 0 },

    {.mode = TARGET_MODE_SPEED,     .kp = 5.0 , .ki = 0.0, .kd = 0.0, .integral = 0, .prev_error = 0 },
    {0},
    {0},
    {0},
};
int16_t current[ROBOMASTER_MAX_COUNT*BUS_NUM] = {0,0,0,0,
                                                 0,0,0,0};//ロボマスに実際に送る電流値
twai_message_t tx_msg = {
    .identifier = 0x200,
    .data_length_code = 8
};
twai_message_t rx_msg;

twai_handle_t can_bus_0;
twai_handle_t can_bus_1;

TaskHandle_t can_rx_task_handle = NULL;
TaskHandle_t can_tx_task_handle = NULL;


//don't setting gpio num 6,Bluepad32's flash/spi path interference
esp_err_t can_driver_install_default_and_start(int tx_gpio,int rx_gpio) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_gpio, rx_gpio, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t e = twai_driver_install(&g_config, &t_config, &f_config);//could not find property 12,11
    if(e != ESP_OK) return e;
    return twai_start();
}

esp_err_t can_driver_install_default_and_start_2bus(int tx_gpio_0,int rx_gpio_0,int tx_gpio_1,int rx_gpio_1) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_gpio_0, rx_gpio_0, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    //bus 0
    g_config.controller_id = 0;
    esp_err_t e = twai_driver_install_v2(&g_config, &t_config, &f_config, &can_bus_0);
    if(e != ESP_OK){
        fprintf(stderr,"Failed to install can driver 0\n");
        return e;
    } 
    e = twai_start_v2(can_bus_0);
    if(e != ESP_OK){
        fprintf(stderr,"Failed to start can driver 0\n");
    }

    //bus 1
    g_config.controller_id = 1;
    g_config.tx_io = tx_gpio_1;
    g_config.rx_io = rx_gpio_1;
    e = twai_driver_install_v2(&g_config, &t_config, &f_config, &can_bus_1);
    if(e != ESP_OK){
        fprintf(stderr,"Failed to install can driver 1\n");
        return e;
    } 
    e = twai_start_v2(can_bus_1);
    if(e != ESP_OK){
        fprintf(stderr,"Failed to start can driver 1\n");
        return e;
    }
   return ESP_OK;

}

void set_robomas_to_rxdata(int i){
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
    //fprintf(stderr,"%d:",i);
    //robomas_dump(&robomas[i]);
}

void set_txdata_to_calced_current(int num, int loop){
    int head = ROBOMASTER_MAX_COUNT*num;
    //PID制御計算
    for (int i = head; i < ROBOMASTER_MAX_COUNT + head; i++) {
        current[i] = pid_calc(&pid[i], &robomas[i], loop * 0.001f);
    }
    //電流制限
    /*
    for (int i = head; i < ROBOMASTER_MAX_COUNT + head; i++) {
        if (current[i] > 10000) current[i] = 10000;
        if (current[i] < -10000) current[i] = -10000;
    }
    */
    //CANメッセージのデータフィールドに電流値を格納
    for (int i = 0; i < ROBOMASTER_MAX_COUNT; i++) {
        tx_msg.data[i*2]     = current[i + head] >> 8;
        tx_msg.data[i*2 + 1] = current[i + head] & 0xFF;
    }
}

void can_rx(){
    if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
        if (rx_msg.identifier >= 0x201 && rx_msg.identifier <= 0x204) {
            set_robomas_to_rxdata(rx_msg.identifier - 0x201);
        }
    }
}
void can_tx(int loop){
    set_txdata_to_calced_current(0,loop);
    //CANメッセージの送信
    twai_transmit(&tx_msg, pdMS_TO_TICKS(1));
}

void can_rx_task(void *arg)
{
    while (1) {
        can_rx();
    }
}

void can_tx_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    int loop = 10;//制御周期10ms
    while (1) {
        can_tx(loop);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(loop));
    }
}

void can_rx_v2(twai_handle_t can_bus){
    if (twai_receive_v2(can_bus,&rx_msg, portMAX_DELAY) == ESP_OK) {
        if (rx_msg.identifier >= 0x201 && rx_msg.identifier <= 0x204) {
            int i = rx_msg.identifier - 0x201;
            if(can_bus==can_bus_1){
                i += ROBOMASTER_MAX_COUNT;
            }
            set_robomas_to_rxdata(i);
        }
    }
}
void can_tx_v2(twai_handle_t can_bus, int loop){
    int num = 0;
    if(can_bus==can_bus_1){
        num = 1;
    }
    set_txdata_to_calced_current(num, loop);
    
    //CANメッセージの送信
    twai_transmit_v2(can_bus,&tx_msg, pdMS_TO_TICKS(1));
}

void can_rx_task_2bus(void *arg){
    while(1){
        can_rx_v2(can_bus_0);
        can_rx_v2(can_bus_1);
    }
}

void can_tx_task_2bus(void *arg){
    TickType_t last_wake = xTaskGetTickCount();
    int loop = 10;
    while(1){
        can_tx_v2(can_bus_0, loop);
        can_tx_v2(can_bus_1, loop);
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

void robomas_dump(robomaster_t *rbms){
    fprintf(stderr,"angle:%d,\tspeed:%d,\ttorque:%d,\ttemperature:%d,\trotation:%d\n",rbms->angle,rbms->speed,rbms->torque,rbms->temperature,rbms->rotation);
}
void current_dump(int16_t cr[ROBOMASTER_MAX_COUNT]){
    fprintf(stderr,"0:%d,\t1:%d,\t2:%d,\t3:%d\n",cr[0],cr[1],cr[2],cr[3]);
}

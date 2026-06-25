#include "robomaster.h"
#include <math.h>
#include "esp_log.h"
#include "driver/twai.h"
#include "freertos/projdefs.h"
#define ROBOMAS_NUM 5
#define ROBOMAS_TAG "robomaster"


mit_t mit[ROBOMAS_NUM] = {
    {.position = 0, .velocity = 0, .kp = 0, .kd = 0, .torque = 0},
    {.position = 0, .velocity = 0, .kp = 0, .kd = 0, .torque = 0},
    {.position = 0, .velocity = 0, .kp = 0, .kd = 0, .torque = 0},
    {.position = 0, .velocity = 0, .kp = 0, .kd = 0, .torque = 0},
    {.position = 0, .velocity = 0, .kp = 0, .kd = 0, .torque = 0},
};

robomaster_t robomas[ROBOMAS_NUM] = {
    {.gear_ratio = ROBOMAS_M2_GEAR_RATIO, .init_angle = 0,              .mit = &mit[0]},
    {.gear_ratio = ROBOMAS_M2_GEAR_RATIO, .init_angle = INIT_ANGLE_R,   .mit = &mit[1]},
    {.gear_ratio = ROBOMAS_M3_GEAR_RATIO, .init_angle = 0,              .mit = &mit[2]},
    {.gear_ratio = ROBOMAS_M2_GEAR_RATIO, .init_angle = 0,              .mit = &mit[3]},
    {.gear_ratio = ROBOMAS_M3_GEAR_RATIO, .init_angle = 0,              .mit = &mit[4]},
};
robomaster_t prev_robomas[ROBOMAS_NUM] = {{0},{0},{0},{0},{0}};
int32_t current[ROBOMAS_NUM] = {0,0,0,0,0};


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

void robomas_angle_init(robomaster_t *robomas){
    robomas->init_angle = robomas_get_position_rad(robomas);
}

void robomas_angle_init_and_stop(robomaster_t *robomas){
    set_mit_t(robomas->mit, 0, 0, 0, robomas->mit->kd, 0);
    robomas_angle_init(robomas);
}

mit_t set_mit_t(mit_t *mit, float position, float velocity, float kp, float kd, float torque){
    if(mit != NULL && sizeof(*mit) == sizeof(mit_t)){
        mit->position = position;
        mit->velocity = velocity;
        mit->kp = kp;
        mit->kd = kd;
        mit->torque = torque;
        return *mit;
    }else{
        ESP_LOGW(ROBOMAS_TAG, "mit_t pointer is NULL or else pointer");
        return (mit_t){
            .position = position,
            .velocity = velocity,
            .kp = kp,
            .kd = kd,
            .torque = torque,
        };
    }
    
}

float mit_calc(robomaster_t *robomas)
{
    float speed_rad_s = (robomas->speed * (2.0f * M_PI / 60.0f)) / robomas->gear_ratio;
    return  robomas->mit->kp * (robomas->mit->position - (robomas_get_position_rad(robomas) - robomas->init_angle)) +
            robomas->mit->kd * (robomas->mit->velocity - speed_rad_s) +
            robomas->mit->torque;
}

void can_rx_task(void *arg)
{
    twai_message_t rx_msg;
    while (1) {
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(1)) == ESP_OK) {
            if (rx_msg.identifier >= 0x201 && rx_msg.identifier <= 0x208) {
                int i = rx_msg.identifier - 0x201;
                if (i >= 0 && i < ROBOMAS_NUM) {
                    prev_robomas[i] = robomas[i];

                    robomas[i].angle = (int16_t)((rx_msg.data[0] << 8) | rx_msg.data[1]);
                    robomas[i].speed = (int16_t)((rx_msg.data[2] << 8) | rx_msg.data[3]);
                    robomas[i].torque = (int16_t)((rx_msg.data[4] << 8) | rx_msg.data[5]);
                    robomas[i].temperature = (int8_t)rx_msg.data[6];
                    if (robomas[i].angle - prev_robomas[i].angle > 4095) {
                        robomas[i].rotation--;
                    } else if (robomas[i].angle - prev_robomas[i].angle < -4096) {
                        robomas[i].rotation++;
                    }
                }
            }
        }
    }
}

esp_err_t can_tx(uint32_t id){
        //CANメッセージのデータフィールドに電流値を格納
        twai_message_t tx_msg = {
            .data_length_code = 8,
            .identifier = id,
            .flags = TWAI_MSG_FLAG_NONE,
        };
        int head = 0;
        if(id == ROBOMASTER_TXID_1) {
            head = 4;
        } else if(id != ROBOMASTER_TXID_0) {
            ESP_LOGE(ROBOMAS_TAG, "can_tx invalid id=0x%03lx\n", id);
            return ESP_ERR_INVALID_ARG;
        }
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
            ESP_LOGE(ROBOMAS_TAG, "BUS OFF");
            if(twai_initiate_recovery() != ESP_OK){
                ESP_LOGE(ROBOMAS_TAG, "cant recover");
            }else{
                ESP_LOGI(ROBOMAS_TAG, "recovered");
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }else if(s.state == TWAI_STATE_STOPPED){
            ESP_LOGE(ROBOMAS_TAG, "TWAI STOPPED");
            if(twai_start() != ESP_OK){
                ESP_LOGE(ROBOMAS_TAG, "cant start");
            }else{
                ESP_LOGI(ROBOMAS_TAG, "started");
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }else if(s.state == TWAI_STATE_RECOVERING){
            ESP_LOGI(ROBOMAS_TAG, "TWAI RECOVERING");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        //MIT制御計算
        for (int i = 0; i < ROBOMAS_NUM; i++) {
            current[i] = mit_calc(&robomas[i]);
            if(current[i] > MAX_CURRENT)current[i] = MAX_CURRENT;
            if(current[i] < -MAX_CURRENT)current[i] = -MAX_CURRENT;

        }
        can_tx(ROBOMASTER_TXID_0);
        can_tx(ROBOMASTER_TXID_1);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(loop));
    }
}

esp_err_t can_driver_install_default_and_start(int tx_gpio,int rx_gpio) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_gpio, rx_gpio, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t e = twai_driver_install(&g_config, &t_config, &f_config);
    if(e != ESP_OK) return e;
    return twai_start();
}

void robomas_dump(robomaster_t *rbms){
    ESP_LOGI(ROBOMAS_TAG,"angle:%5d,\toutput angle rad: %3.2fspeed:%5d,\ttorque:%5d,\ttemperature:%5d,\trotation:%d\n",rbms->angle, robomas_get_position_rad(rbms), rbms->speed,rbms->torque,rbms->temperature,rbms->rotation);
}

void current_dump(){
    fprintf(stderr,"currents: ");
    for(int i = 0; i< ROBOMAS_NUM;i++){
        fprintf(stderr,"%d: %6ld | ",i,current[i]);
    }
    fprintf(stderr,"\n");
}
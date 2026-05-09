#include "servo.h"
#include <math.h>

esp_err_t servo_timer_config(servo_timer_config_t *timer_conf) {
    return ledc_timer_config(timer_conf);
}
esp_err_t servo_channel_config(servo_channel_config_t *channel_conf) {
    return ledc_channel_config(channel_conf);
}
esp_err_t servo_timer_config_default() {
    servo_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    return ledc_timer_config(&timer_conf);
}

esp_err_t servos_channel_config(servo_t  *servos, int servocount) {
    servo_channel_config_t ch;
    for (int i = 0; i < servocount; i++) {
        ch = (servo_channel_config_t){
            .gpio_num = servos[i].gpio_num,
            .channel = servos[i].channel,
            .timer_sel = servos[i].timer,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty = 0
        };
        if(ledc_channel_config(&ch) != ESP_OK) {
            return i+1; // i番目のサーボでエラーが発生
        }
    }
    return ESP_OK;
}

esp_err_t servo_init(servo_t *servo, servo_range_t range, int gpio_num, ledc_channel_t channel, 
                    ledc_timer_t timer, uint32_t min_duty, uint32_t max_duty)
{
    if (servo == NULL)return ESP_FAIL;
    
    servo->range = range;
    servo->gpio_num = gpio_num;
    servo->channel = channel;
    servo->timer = timer;
    servo->angle_rad = 0.0f;
    servo->min_duty = min_duty;
    servo->max_duty = max_duty;

    return ESP_OK;
}

esp_err_t servos_range_and_pin_init(servo_t *servos, servo_range_t *ranges, int *servo_pins, int servocount){
    for (int i = 0; i < servocount  ; i++) {
        if
        (
            servo_init
            (
                &servos[i], 
                ranges[i],
                servo_pins[i],
                LEDC_CHANNEL_0 + i,
                LEDC_TIMER_0,
                SERVO_ANGLE_MIN,
                SERVO_ANGLE_MAX
            ) != ESP_OK
        ){
            return i+1; // i番目のサーボでエラーが発生
        }
    }
    return ESP_OK;
}

esp_err_t servo_update_angle(servo_t *servo)
{
    if (servo == NULL) {
        return ESP_FAIL; // NULLポインタエラー
    }

    // 角度の有効性チェック
    float max_rad;
    if (servo->range == SERVO_RANGE_180) {
        max_rad = M_PI;
    } else if(servo->range == SERVO_RANGE_270) {
        max_rad = 3.0f * M_PI / 2.0f;
    }else{
        return ESP_ERR_INVALID_ARG;
    }

    //範囲外の角度を設定した場合servo_angle_to_dutyによりクランプされるが，
    //サーボの角度を更新しないほうが安全なため，エラーを返す
    if (servo->angle_rad < 0.0f || servo->angle_rad > max_rad) {
        return ESP_ERR_INVALID_ARG;  // 範囲外
    }

    // デューティ値を計算
    uint32_t duty = servo_angle_to_duty(servo-> angle_rad, servo->range, servo->min_duty, servo->max_duty);

    // LEDCにデューティ値を設定
    ledc_set_duty(LEDC_LOW_SPEED_MODE, servo->channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, servo->channel);

    return ESP_OK;
}
esp_err_t servos_update_angle(servo_t servos[], int count) {
    for (int i = 0; i < count; i++) {
        if (servo_update_angle(&servos[i]) != ESP_OK) {
            return i+1; // i番目のサーボでエラーが発生
        }
    }
    return ESP_OK; // 全てのサーボが正常に設定された
}

uint32_t servo_angle_to_duty(float angle_rad, servo_range_t range, uint32_t min_duty, uint32_t max_duty)
{
    if(angle_rad<0)return min_duty;

    float max_rad;
    
    if (range == SERVO_RANGE_180) {
        max_rad = M_PI;          // π ラジアン = 180度
        if(angle_rad>max_rad)return max_duty;
    } else if(range == SERVO_RANGE_270) {
        max_rad = 3.0f * M_PI / 2.0f;  // 3π/2 ラジアン = 270度
        if(angle_rad>max_rad)return max_duty;
    }else {
        return min_duty; // 不正な範囲指定の場合は最小デューティ値を返す
    }

    return (min_duty + (uint32_t)((angle_rad / max_rad) * (max_duty - min_duty)));

}
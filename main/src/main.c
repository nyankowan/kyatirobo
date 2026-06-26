#include <math.h>
#include <stdlib.h>

#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <hci_dump.h>
#include <hci_dump_embedded_stdout.h>
#include <uni.h>
#include "sdkconfig.h"

#include "controller_data.h"
#include "coordinate.h"
#include "servo.h"
#include "robomaster.h"
#include "limitswitch.h"

#include <driver/twai.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

#define DEBUG 1
#define MAIN_TAG "main"

// Defined in controller.c
struct uni_platform* get_my_platform(void);

void controll_task(void *pvParameters);
void exec_command();
void calibration();
#if DEBUG
void debug_task(void *arg);
#endif

#define pushed(button,prev_button) (button == true && prev_button == false)
#define depushed(button,prev_button) (button == false && prev_button == true)

#define DIRECT_MOVE_SPEED 0.1
#define POLAR_RATIO (8.0/3.0)
#define LOOP_MS 30

#define DIRECT_INIT (direct_t){.x = INIT_ANGLE_R, .y = 0.0}
#define SERVO_COUNT 6//サーボの数
#define LIMITSWITCH_COUNT 4

//don't use 6-11.these pins for flash memorry
//0 for boot/reset(pull up)
//1,3 for UART0
//2 for boot strap
//34-39 input only
//safe to use 4,5,12-15,18-23,25-32(depends on use)
#define CAN_TX_GPIO 5
#define CAN_RX_GPIO 4
gpio_num_t servo_pins[SERVO_COUNT] = {18, 19, 21, 22, 23, 25};//サーボの制御に使用するGPIO番号
gpio_num_t limitswitch_pins[LIMITSWITCH_COUNT] = {12, 13, 14, 15}; 

servo_range_t servo_ranges[SERVO_COUNT] = {
    SERVO_RANGE_270, 
    SERVO_RANGE_180, 
    SERVO_RANGE_180, 
    SERVO_RANGE_180, 
    SERVO_RANGE_270, 
    SERVO_RANGE_270
};//サーボの可動範囲
limitswitch_t limitswitches[LIMITSWITCH_COUNT];
servo_t servos[SERVO_COUNT];
mypad_t mypad;
mypad_t prev_mypad;

//coordinate
direct_t xy = DIRECT_INIT;

int app_main(void)
{
#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    btstack_stdio_init();
#endif  // CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
#endif  // CONFIG_ESP_CONSOLE_UART

    // Configure BTstack for ESP32 VHCI Controller
    btstack_init();

    // Must be called before uni_init()
    uni_platform_set_custom(get_my_platform());
 
    // Init Bluepad32.
    uni_init(0 /* argc */, NULL /* argv */);

    // Initialize GPIO for limitswitch
    limitswitches_init(limitswitches, limitswitch_pins, LIMITSWITCH_COUNT);

    // Initialize LEDC for servo control
    servo_timer_config_default();//デューティ値の範囲は0～65535の16bitタイマー, 周波数は50Hz前提の設定
    servos_range_and_pin_init(servos, servo_ranges, servo_pins, SERVO_COUNT);//サーボ構造体の初期化
    servos_channel_config(servos, SERVO_COUNT);

    if (can_driver_install_default_and_start(CAN_TX_GPIO,CAN_RX_GPIO) != ESP_OK) {
        loge("Error: CAN driver install failed\n");
        return -1;
    }
    xTaskCreatePinnedToCore(robomas_can_rx_task, "can_rx_task", 2048, NULL, 10, &can_rx_task_handle, APP_CPU_NUM);
    xTaskCreatePinnedToCore(robomas_can_tx_task, "can_tx_task", 3072, NULL, 5, &can_tx_task_handle, APP_CPU_NUM);
    xTaskCreatePinnedToCore(controll_task, "controll_task", 3072, NULL, 1, NULL, APP_CPU_NUM);
#if DEBUG
    xTaskCreate(debug_task, "debug", 4096, NULL, 1, NULL);
#endif
    // Does not return.
    btstack_run_loop_execute();

    return 0;
}
    
// Controll task
void controll_task(void *pvParameters) {
    calibration();
    for(;;
    //get datas
    prev_mypad = mypad, 
    get_gpdata(&mypad),
    get_limitswitches_level(limitswitches, LIMITSWITCH_COUNT))
    {
        if(pushed(mypad.HOME,prev_mypad.HOME)){
            calibration();
        }else{
            exec_command();
        }
        vTaskDelay(LOOP_MS / portTICK_PERIOD_MS);// vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_MS));//wdt err
    }
}

int sign(float f){
    if(f>0)return 1;
    if(f<0)return -1;
    return 0;
}
void exec_command(){
    if (mypad.RIGHT) xy.x += DIRECT_MOVE_SPEED;
    if (to_polar(xy).r < INIT_ANGLE_R) xy.x -= DIRECT_MOVE_SPEED;
    if (xy.y < 0 && xy.x > 0) xy.x = 0;
    if (mypad.LEFT) xy.x -= DIRECT_MOVE_SPEED;
    if (to_polar(xy).r < INIT_ANGLE_R) xy.x += DIRECT_MOVE_SPEED;
    if (mypad.UP) xy.y += DIRECT_MOVE_SPEED;
    if (mypad.DOWN) xy.y -= DIRECT_MOVE_SPEED;
    if (to_polar(xy).r < INIT_ANGLE_R) xy.y += DIRECT_MOVE_SPEED;
    if (xy.y < 0 && xy.x > 0) xy.y = 0;

    set_mit_t(  robomas[0].mit, 
                to_polar(xy).theta * POLAR_RATIO, 
                0.0,
                20.0,
                5.0, 
                robomas[0].precurrent * sign(to_polar(xy).theta));

    set_mit_t(  robomas[1].mit,
                -to_polar(xy).r,
                0.0,
                20.0,
                5.0,
                -robomas[1].precurrent * sign(to_polar(xy).r));
    servos[0].angle_rad = to_polar(xy).theta;
    servos_update_angle(servos, SERVO_COUNT);
}

//極座標アームを初期化する
void calibration(){
    servos[0].angle_rad = 0;
    servos_update_angle(servos, SERVO_COUNT);
#define calib_robomas_num  2
    int calib_done_num = 0;
    bool calib_done_robomas[calib_robomas_num] = {false,false,/*false,false*/};
    for(int i = 0; i < calib_robomas_num ;i++){
        set_mit_t(  robomas[i].mit,
                0,
                -1 * pow(-1,i),
                0,
                20,
                -robomas[i].precurrent * pow(-1,i));
    }
    
    for(;calib_done_num < calib_robomas_num;get_limitswitches_level(limitswitches, LIMITSWITCH_COUNT)){
        for(int i = 0; i < calib_robomas_num; i++){
            if(!calib_done_robomas[i] && limitswitches[i].pressed){
                robomas_angle_init_and_stop(&robomas[i]);
                calib_done_robomas[i] = true;
                calib_done_num++;
            }
        }
        vTaskDelay(LOOP_MS / portTICK_PERIOD_MS); // Delay to prevent spamming the console
    }
    xy = DIRECT_INIT;
    fprintf(stderr,"================ calib_done ================\n");
}

#if DEBUG
void debug_task(void *arg)
{
    while(1){
        printf("=====================================================\n");

        // printf("tx stack: %u\n",
        //     uxTaskGetStackHighWaterMark(can_tx_task_handle));

        // printf("rx stack: %u\n",
        //     uxTaskGetStackHighWaterMark(can_rx_task_handle));

        twai_status_info_t status;
        twai_get_status_info(&status);

        printf(
            "state=%d txerr=%lu rxerr=%lu txfail=%lu\n",
            status.state,
            (unsigned long)status.tx_error_counter,
            (unsigned long)status.rx_error_counter,
            (unsigned long)status.tx_failed_count
        );
        current_dump(current);
        for(int i = 0; i < 2; i++){
            mit_dump(robomas[i].mit);
            robomas_dump(&robomas[i]);
            fprintf(stderr, "\n");
        }
        // controller_dump(&mypad);
        coordinate_dump(&xy);
        limitswitches_dump(limitswitches , LIMITSWITCH_COUNT);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

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

#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

#define MAIN_TAG "main"

// Defined in controller.c
struct uni_platform* get_my_platform(void);

#define LOOP_MS 30

#define SERVO_COUNT 7//サーボの数7

//don't use 6-11.these pins for flash memorry
//0 for boot/reset(pull up)
//1,3 for UART0
//2 for boot strap
//34-39 input only
//safe to use 4,5,12-15,18-23,25-32(depends on use)
#define CAN_TX_GPIO_0 4
#define CAN_RX_GPIO_0 5

int servo_pins[SERVO_COUNT] = {18, 19, 21, 22, 23, 25, 26};//サーボの制御に使用するGPIO番号
servo_range_t servo_ranges[SERVO_COUNT] = {
    SERVO_RANGE_180, 
    SERVO_RANGE_180, 
    SERVO_RANGE_180, 
    SERVO_RANGE_180, 
    SERVO_RANGE_270, 
    SERVO_RANGE_270,
    SERVO_RANGE_270
};//サーボの可動範囲

servo_t servos[SERVO_COUNT];


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

    // Initialize LEDC for servo control
    servo_timer_config_default();//デューティ値の範囲は0～65535の16bitタイマー, 周波数は50Hz前提の設定
    servos_range_and_pin_init(servos, servo_ranges, servo_pins, SERVO_COUNT);//サーボ構造体の初期化
    servos_channel_config(servos, SERVO_COUNT);

#if 1
    if (can_driver_install_default_and_start(CAN_TX_GPIO_0,CAN_RX_GPIO_0) != ESP_OK) {
        loge("Error: CAN driver install failed\n");
        return -1;
    }

    xTaskCreatePinnedToCore(can_rx_task, "can_rx_task", 2024, NULL, 10, &can_rx_task_handle, APP_CPU_NUM);
    xTaskCreatePinnedToCore(can_tx_task, "can_tx_task", 2024, NULL, 5, &can_tx_task_handle, APP_CPU_NUM);
#else//error
    esp_err_t e = can_driver_install_default_and_start_2bus(CAN_TX_GPIO_0,CAN_RX_GPIO_0,CAN_TX_GPIO_1,CAN_RX_GPIO_1);
    if (e != ESP_OK) {
        loge("Error: CAN driver install failed\n");
        char s[100];
        loge("%s\n",esp_err_to_name_r(e,s,100));
        return -1;
    }

    xTaskCreatePinnedToCore(can_rx_task_2bus, "can_rx_task", 2024, NULL, 10, &can_rx_task_handle, APP_CPU_NUM);
    xTaskCreatePinnedToCore(can_tx_task_2bus, "can_tx_task", 2024, NULL, 5, &can_tx_task_handle, APP_CPU_NUM);
#endif
    xTaskCreatePinnedToCore(controller_task, "controller_task", 4096, NULL, 1, &controller_task_handle, APP_CPU_NUM);

    // Does not return.
    btstack_run_loop_execute();

    return 0;
}
    
// Controller task function
void controller_task(void *pvParameters) {
    //TickType_t last_wake = xTaskGetTickCount();
    //coordinate
    direct_t xy = {3.0, 1.0};

    while (1) {
        if(!mypad.connected){
            vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to prevent spamming the console
            continue;
        }
        if(mypad.LEFT){
            xy.x -= 0.1;
            if(xy.x < 1.0 && xy.x > -1.0 && xy.y < 1.0 && xy.y > -1.0)xy.x = 1.0;//原点付近への進入禁止
        }
        if(mypad.RIGHT){
            xy.x += 0.1;
            if(xy.x > -1.0 && xy.x < 1.0 && xy.y < 1.0 && xy.y > -1.0)xy.x = -1.0;//原点付近への進入禁止
            if(xy.x > 0 && xy.y<0)xy.x = 0.0;//270度以上への進入禁止
        }
        if(mypad.UP){
            xy.y += 0.1;
            if(xy.y > -1.0 && xy.y < 1.0 && xy.x < 1.0 && xy.x > -1.0)xy.y = -1.0;//原点付近への進入禁止
        }
        if(mypad.DOWN){
            xy.y -= 0.1;
            if(xy.y > -1.0 && xy.y < 1.0 && xy.x < 1.0 && xy.x > -1.0)xy.y = 1.0;//原点付近への進入禁止
            if(xy.x > 0 && xy.y < 0)xy.y = 0.0;//0度以下への進入禁止
        }
        if(mypad.A == true && prev_mypad.A == false){//少し取りこぼす.get_mypad関数などを作って，mypadから値を取得するようにすると良さそう（タスク間のqueueみたいな役割？），prev_mypadは消して良い．
            printf("pressed A\n");
        }

        for(int i = 0;i<4;i++){
            pid[i].target_speed = mypad.RY*2;
        }
        servos[0].angle_rad = to_polar(xy).theta;
        //fprintf(stderr,"%d\n",current[2]);

        
        // Update servo angles based on controller input
        servos_update_angle(servos, SERVO_COUNT);
        robomas_dump(&robomas[2]);
        current_dump(current);
        //controller_dump(&mypad);
        //coordinate_dump(&xy);
        //can_tx(LOOP_MS); //don't call this in controller_task.it's unstable.create tx_task.
        //vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_MS));//wdt err?
        vTaskDelay(LOOP_MS / portTICK_PERIOD_MS); // Delay to prevent spamming the console
    }
}


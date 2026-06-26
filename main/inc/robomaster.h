
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/twai_types_deprecated.h"
#include <esp_err.h>
#define ROBOMASTER_MAX_COUNT 8
#define ROBOMASTER_TXID_0 0x200
#define ROBOMASTER_TXID_1 0x1ff
#define MAX_CURRENT 16384 //ロボマスに遅れる最大電流値
#define MOTOR_MIN_SPEED 3.0 //モーター軸基準,正常に動く最低rpm
#define ENCODER_RESOLUTION 8192 //モーター軸基準
#define MOTOR_MAX_RPM 500.0 //モータ軸基準
#define ROBOMAS_M3_GEAR_RATIO (3591.0 / 187.0)//M3508
#define ROBOMAS_M2_GEAR_RATIO 36//M2006
#define INIT_ANGLE_R 157.0/15//157(mm)/30PI(mm) * 2PI(rad)
#define ROBOMAS_NUM 4

typedef struct{
    float position;
    float velocity;
    float kp;
    float kd;
    float torque;
}mit_t;

//ロボマスの状態を保存
typedef struct {
    int16_t angle;//モーター軸 0~8191,90度=2048
    int16_t speed;//モーター軸rpm
    int16_t torque;//トルク電流
    int8_t  temperature;//℃

    int rotation;//モーター軸回転数
    float gear_ratio;//1:gear_ratio＝モーター軸:出力軸のギア比
    float init_angle;//出力軸rad
    float home_angle;//出力軸rad
    float precurrent;

    mit_t *mit;
} robomaster_t;

extern robomaster_t robomas[];//can_rx_task()で受け取った値を保存する構造体
extern robomaster_t prev_robomas[];//robomasの以前の値
extern int32_t current[];//送る電流値
extern TaskHandle_t can_rx_task_handle;//can_rx_taskのハンドラー
extern TaskHandle_t can_tx_task_handle;//can_tx_taskのハンドラー

/**
*@brief モーター軸基準のENCODER_RESOLUTIONの解像度で現在の角度を返す
 */
int32_t robomas_get_position(robomaster_t *r);

/**
*@brief 出力軸基準で現在の角度を弧度法で返す
*/
float robomas_get_position_rad(robomaster_t *r);

/**
 *@brief 現在の位置を初期位置にする
 */
void robomas_angle_init(robomaster_t *robomas);
/**
 *@brief 現在の位置を初期位置にし，モータの速度を0にする
 */
void robomas_angle_init_and_stop(robomaster_t *robomas);

/**
 *@brief mit_tポインタの先に値を代入する
 *@param mit NULLを渡しても，戻り値を返す
 */
mit_t set_mit_t(mit_t *mit, float position, float velocity, float kp, float kd, float torque);
/**
*@brief ロボマスに送る電流値を一つ計算する．
*/
float mit_calc(robomaster_t *robomas);


/**
*@brief ロボマスター専用のTWAI設定
*       can_*x_task()を起動する前に実行
*/
esp_err_t can_driver_install_default_and_start(int can_tx_gpio,int can_rx_gpio);

/**
 *@brief can受信キューからデータを取り出し，ロボマス用のデータをrobomas[]に格納，同時にprev_robomas[]に以前の値を保存．
 */
esp_err_t robomas_can_rx(twai_message_t *rx_msg);
/**
*@brief current[]の電流値を送る
*/
esp_err_t robomas_can_tx(uint32_t id);


/**
*@brief can受信task
*/
void robomas_can_rx_task(void *arg);

/**
*@brief can送信task
*/
void robomas_can_tx_task(void *arg);

twai_state_t can_error_handling();

//dump
void robomas_dump(robomaster_t *rbms);
void mit_dump(mit_t *mit);
void current_dump();
// Example file - Public Domain
// Need help? https://tinyurl.com/bluepad32-help
#include <string.h>
#include <uni.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "controller_data.h"
#include "robomaster.h"

const mypad_t EMPTY_MYPAD = {
    .A = false,
    .B = false,
    .X = false,
    .Y = false,
    .UP = false,
    .DOWN = false,
    .LEFT = false,
    .RIGHT = false,
    .L = false,
    .R = false,
    .ZL = false,
    .ZR = false,
    .TL = false,
    .TR = false,
    .MINUS = false,
    .PLUS = false,
    .HOME = false,
    .CAPTURE = false,
    .LX = 0,
    .LY = 0,
    .RX = 0,
    .RY = 0,
    .battery_level = 0,
    .connected = false
};

mypad_t mypad = EMPTY_MYPAD;
mypad_t prev_mypad = EMPTY_MYPAD;
TaskHandle_t controller_task_handle = NULL;

#define PRO_CONTROLLER_COD 0b0010010100001000//cod=0x00002508

// Custom "instance"
typedef struct my_platform_instance_s {
    uni_gamepad_seat_t gamepad_seat;  // which "seat" is being used
}my_platform_instance_t;

// Declarations
static void trigger_event_on_gamepad(uni_hid_device_t* d);
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d);

//
// Platform Overrides
//
static void my_platform_init(int argc, const char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    logi("custom: init()\n");

    uni_gamepad_set_mappings_type(UNI_GAMEPAD_MAPPINGS_TYPE_SWITCH);
}

static void my_platform_on_init_complete(void) {
    logi("custom: on_init_complete()\n");
    // Safe to call "unsafe" functions since they are called from BT thread

    // Start scanning
    uni_bt_start_scanning_and_autoconnect_unsafe();
    uni_bt_allow_incoming_connections(true);

    // Based on runtime condition, you can delete or list the stored BT keys.
    if (1)
        uni_bt_del_keys_unsafe();
    else
        uni_bt_list_keys_unsafe();
}

static uni_error_t my_platform_on_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
    // You can filter discovered devices here.
    // Just return any value different from UNI_ERROR_SUCCESS;
    // @param addr: the Bluetooth address
    // @param name: could be NULL, could be zero-length, or might contain the name.
    // @param cod: Class of Device. See "uni_bt_defines.h" for possible values.
    // @param rssi: Received Signal Strength Indicator (RSSI) measured in dBms. The higher (255) the better.
    if(cod != PRO_CONTROLLER_COD) {
        logi("Ignoring non-Pro Controller device\n");
        return UNI_ERROR_IGNORE_DEVICE;
    }
    if(!(addr[0] == 0x98 &&
        addr[1] == 0xB6 &&
        addr[2] == 0xE9 &&
        addr[3] == 0x2A &&
        addr[4] == 0xE0 && 
        addr[5] == 0xEE)) {
        logi("Ignoring other Pro Controller device\n");
        return UNI_ERROR_IGNORE_DEVICE;
    }

    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_device_connected(uni_hid_device_t* d) {
    logi("custom: device connected: %p\n", d);
    mypad = EMPTY_MYPAD;
    mypad.battery_level = d->controller.battery;
    mypad.connected = true;

    prev_mypad = EMPTY_MYPAD;
}

static void my_platform_on_device_disconnected(uni_hid_device_t* d) {
    logi("custom: device disconnected: %p\n", d);
    mypad = EMPTY_MYPAD;
    mypad.connected = false;
}

static uni_error_t my_platform_on_device_ready(uni_hid_device_t* d) {
    logi("custom: device ready: %p\n", d);
    my_platform_instance_t* ins = get_my_platform_instance(d);
    ins->gamepad_seat = GAMEPAD_SEAT_A;

    trigger_event_on_gamepad(d);

    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    static uni_controller_t prev_ctl = {0};
    uni_gamepad_t *prev_gp = &prev_ctl.gamepad;
    uni_gamepad_t *gp = &ctl->gamepad;
    uni_gamepad_remap(gp);
    uni_gamepad_remap(prev_gp);
    // Update mypad
    convert_gp(gp, &mypad);  
    mypad.battery_level = ctl->battery;
    convert_gp(prev_gp, &prev_mypad);
    prev_mypad.battery_level = prev_ctl.battery;

    if (memcmp(&prev_ctl, ctl, sizeof(*ctl)) == 0) {
        return;
    }
    prev_ctl = *ctl;
}

static const uni_property_t* my_platform_get_property(uni_property_idx_t idx) {
    ARG_UNUSED(idx);
    return NULL;
}

/**
 * Handle Out-of-Band events sent by Bluepad32. 
 * For example, you can use this to detect when the "system" button of the gamepad is pressed, which is not sent as part of the regular gamepad data. 
 * In this example, we toggle the "seat" of the gamepad between A and B, and trigger the event to update the LEDs and rumble. 
 * You can customize this function to handle other events, such as when Bluetooth is enabled/disabled, etc.
 */
static void my_platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
}

//
// Helpers
//
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d) {
    return (my_platform_instance_t*)&d->platform_data[0];
}

static void trigger_event_on_gamepad(uni_hid_device_t* d) {
    my_platform_instance_t* ins = get_my_platform_instance(d);

    if (d->report_parser.play_dual_rumble != NULL) {
        d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 150 /* duration ms */, 255 /* weak magnitude */,
                                          255 /* strong magnitude */);
    }

    if (d->report_parser.set_player_leds != NULL) {
        d->report_parser.set_player_leds(d, ins->gamepad_seat);
    }
}

//
// Entry Point
//
struct uni_platform* get_my_platform(void) {
    static struct uni_platform plat = {
        .name = "procon_platform",
        .init = my_platform_init,
        .on_init_complete = my_platform_on_init_complete,
        .on_device_discovered = my_platform_on_device_discovered,
        .on_device_connected = my_platform_on_device_connected,
        .on_device_disconnected = my_platform_on_device_disconnected,
        .on_device_ready = my_platform_on_device_ready,
        .on_oob_event = my_platform_on_oob_event,
        .on_controller_data = my_platform_on_controller_data,
        .get_property = my_platform_get_property,
    };

    return &plat;
}

void convert_gp(uni_gamepad_t *gp, mypad_t *mp){
    mp->A = (gp->buttons & BUTTON_A) ? 1 : 0;
    mp->B = (gp->buttons & BUTTON_B) ? 1 : 0;
    mp->X = (gp->buttons & BUTTON_X) ? 1 : 0;
    mp->Y = (gp->buttons & BUTTON_Y) ? 1 : 0;
    mp->UP = (gp->dpad == DPAD_UP) ? 1 : 0;
    mp->DOWN = (gp->dpad == DPAD_DOWN) ? 1 : 0;
    mp->LEFT = (gp->dpad == DPAD_LEFT) ? 1 : 0;
    mp->RIGHT = (gp->dpad == DPAD_RIGHT) ? 1 : 0;
    mp->TL = (gp->buttons & BUTTON_THUMB_L) ? 1 : 0;
    mp->TR = (gp->buttons & BUTTON_THUMB_R) ? 1 : 0;
    mp->MINUS = (gp->misc_buttons & MISC_BUTTON_SELECT) ? 1 : 0;
    mp->PLUS = (gp->misc_buttons & MISC_BUTTON_START) ? 1 : 0;
    mp->HOME = (gp->misc_buttons & MISC_BUTTON_SYSTEM) ? 1 : 0;
    mp->CAPTURE = (gp->misc_buttons & MISC_BUTTON_CAPTURE) ? 1 : 0;
    mp->L = (gp->buttons & BUTTON_SHOULDER_L) ? 1 : 0;
    mp->R = (gp->buttons & BUTTON_SHOULDER_R) ? 1 : 0;
    mp->ZL = (gp->buttons & BUTTON_TRIGGER_L) ? 1 : 0;
    mp->ZR = (gp->buttons & BUTTON_TRIGGER_R) ? 1 : 0;
    // Convert from -512..511 to -128..127
    mp->LX = (gp->axis_x * 256) / 512; 
    mp->LY = (gp->axis_y * 256) / 512;
    mp->RX = (gp->axis_rx * 256) / 512;
    mp->RY = (gp->axis_ry * 256) / 512;

    mp->connected = true;
}

void controller_dump(mypad_t* pad) {
    logi("A: %d, B: %d, X: %d, Y: %d", pad->A, pad->B, pad->X, pad->Y);
    logi("UP: %d, DOWN: %d, LEFT: %d, RIGHT: %d", pad->UP, pad->DOWN, pad->LEFT, pad->RIGHT);
    logi("L: %d, R: %d, ZL: %d, ZR: %d", pad->L, pad->R, pad->ZL, pad->ZR);
    logi("TL: %d, TR: %d, MINUS: %d, PLUS: %d", pad->TL, pad->TR, pad->MINUS, pad->PLUS);
    logi("HOME: %d, CAPTURE: %d", pad->HOME, pad->CAPTURE);
    logi("LX: %d, LY: %d, RX: %d, RY: %d\n", pad->LX, pad->LY, pad->RX, pad->RY);
}
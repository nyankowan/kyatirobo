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

#if 0
    uni_gamepad_mappings_t mappings = GAMEPAD_DEFAULT_MAPPINGS;

    // Inverted axis with inverted Y in RY.
    mappings.axis_x = UNI_GAMEPAD_MAPPINGS_AXIS_RX;
    mappings.axis_y = UNI_GAMEPAD_MAPPINGS_AXIS_RY;
    mappings.axis_ry_inverted = true;
    mappings.axis_rx = UNI_GAMEPAD_MAPPINGS_AXIS_X;
    mappings.axis_ry = UNI_GAMEPAD_MAPPINGS_AXIS_Y;

    // Invert A & B, X & Y
    mappings.button_a = UNI_GAMEPAD_MAPPINGS_BUTTON_B;
    mappings.button_b = UNI_GAMEPAD_MAPPINGS_BUTTON_A;
    mappings.button_x = UNI_GAMEPAD_MAPPINGS_BUTTON_Y;
    mappings.button_y = UNI_GAMEPAD_MAPPINGS_BUTTON_X;

    uni_gamepad_set_mappings(&mappings);
#endif
       //uni_bt_service_set_enabled(true);
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

    // As an example, if you want to filter out keyboards, do:
    // if (((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_KEYBOARD) == UNI_BT_COD_MINOR_KEYBOARD) {
    //     logi("Ignoring keyboard\n");
    //     return UNI_ERROR_IGNORE_DEVICE;
    // }
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
    //static uint8_t leds = 0;
    //static uint8_t enabled = true;
    static uni_controller_t prev = {0};
    uni_gamepad_t gp;

    // Optimization to avoid processing the previous data so that the console
    // does not get spammed with a lot of logs, but remove it from your project.
    if (memcmp(&prev, ctl, sizeof(*ctl)) == 0) {
        return;
    }
    prev = *ctl;
    // Print device Id before dumping gamepad.
    // This could be very CPU intensive and might crash the ESP32.
    // Remove these 2 lines in production code.
    //    logi("(%p), id=%d, \n", d, uni_hid_device_get_idx_for_instance(d));
    //    uni_controller_dump(ctl);

    switch (ctl->klass) {
        case UNI_CONTROLLER_CLASS_GAMEPAD:
            //gp = &ctl->gamepad;
            gp = uni_gamepad_remap(&ctl->gamepad);//ちゃんと動かない？

            // Update mypad
            mypad.B = (gp.buttons & BUTTON_A) ? 1 : 0;
            mypad.A = (gp.buttons & BUTTON_B) ? 1 : 0;
            mypad.Y = (gp.buttons & BUTTON_X) ? 1 : 0;
            mypad.X = (gp.buttons & BUTTON_Y) ? 1 : 0;
            mypad.UP = (gp.dpad == DPAD_UP) ? 1 : 0;
            mypad.DOWN = (gp.dpad == DPAD_DOWN) ? 1 : 0;
            mypad.LEFT = (gp.dpad == DPAD_LEFT) ? 1 : 0;
            mypad.RIGHT = (gp.dpad == DPAD_RIGHT) ? 1 : 0;
            mypad.TL = (gp.buttons & BUTTON_THUMB_L) ? 1 : 0;
            mypad.TR = (gp.buttons & BUTTON_THUMB_R) ? 1 : 0;
            mypad.MINUS = (gp.misc_buttons & MISC_BUTTON_SELECT) ? 1 : 0;
            mypad.PLUS = (gp.misc_buttons & MISC_BUTTON_START) ? 1 : 0;
            mypad.HOME = (gp.misc_buttons & MISC_BUTTON_SYSTEM) ? 1 : 0;
            mypad.CAPTURE = (gp.misc_buttons & MISC_BUTTON_CAPTURE) ? 1 : 0;
            mypad.L = (gp.buttons & BUTTON_SHOULDER_L) ? 1 : 0;
            mypad.R = (gp.buttons & BUTTON_SHOULDER_R) ? 1 : 0;
            mypad.ZL = (gp.buttons & BUTTON_TRIGGER_L) ? 1 : 0;
            mypad.ZR = (gp.buttons & BUTTON_TRIGGER_R) ? 1 : 0;
            // Convert from -512..511 to -128..127
            mypad.LX = (gp.axis_x * 256) / 512; 
            mypad.LY = (gp.axis_y * 256) / 512;
            mypad.RX = (gp.axis_rx * 256) / 512;
            mypad.RY = (gp.axis_ry * 256) / 512;
            mypad.battery_level = ctl->battery;
            mypad.connected = true;
            break;
        default:
            break;
    }
}
#define DEBUG_PROPERTY 1
#if DEBUG_PROPERTY
#define UNI_PROPERTY_NAME_UNI_AUTOFIRE_CPS "bp.uni.autofire"
#define UNI_PROPERTY_NAME_UNI_BB_FIRE_THRESHOLD "bp.uni.bb_fire"
#define UNI_PROPERTY_NAME_UNI_BB_MOVE_THRESHOLD "bp.uni.bb_move"
#define UNI_PROPERTY_NAME_UNI_C64_POT_MODE "bp.uni.c64pot"
#define UNI_PROPERTY_NAME_UNI_MODEL "bp.uni.model"
#define UNI_PROPERTY_NAME_UNI_MOUSE_EMULATION "bp.uni.mouseemu"
#define UNI_PROPERTY_NAME_UNI_SERIAL_NUMBER "bp.uni.serial"
#define UNI_PROPERTY_NAME_UNI_VENDOR "bp.uni.vendor"

static const uni_property_t my_platform_properties[] = {
    {UNI_PROPERTY_IDX_UNI_AUTOFIRE_CPS, UNI_PROPERTY_NAME_UNI_AUTOFIRE_CPS, UNI_PROPERTY_TYPE_U8,
     .default_value.u8 = 0},
    {UNI_PROPERTY_IDX_UNI_BB_FIRE_THRESHOLD, UNI_PROPERTY_NAME_UNI_BB_FIRE_THRESHOLD, UNI_PROPERTY_TYPE_U32,
     .default_value.u32 = 5000},
    {UNI_PROPERTY_IDX_UNI_BB_MOVE_THRESHOLD, UNI_PROPERTY_NAME_UNI_BB_MOVE_THRESHOLD, UNI_PROPERTY_TYPE_U32,
     .default_value.u32 = 1500},
    {UNI_PROPERTY_IDX_UNI_C64_POT_MODE, UNI_PROPERTY_NAME_UNI_C64_POT_MODE, UNI_PROPERTY_TYPE_U8,
     .default_value.u8 = 0},
    {UNI_PROPERTY_IDX_UNI_MODEL, UNI_PROPERTY_NAME_UNI_MODEL, UNI_PROPERTY_TYPE_STRING,
     .default_value.str = "Unknown", .flags = UNI_PROPERTY_FLAG_READ_ONLY},
    {UNI_PROPERTY_IDX_UNI_MOUSE_EMULATION, UNI_PROPERTY_NAME_UNI_MOUSE_EMULATION, UNI_PROPERTY_TYPE_U8,
     .default_value.u8 = 3},
    {UNI_PROPERTY_IDX_UNI_SERIAL_NUMBER, UNI_PROPERTY_NAME_UNI_SERIAL_NUMBER, UNI_PROPERTY_TYPE_U32,
     .default_value.u32 = 0, .flags = UNI_PROPERTY_FLAG_READ_ONLY},
    {UNI_PROPERTY_IDX_UNI_VENDOR, UNI_PROPERTY_NAME_UNI_VENDOR, UNI_PROPERTY_TYPE_STRING,
     .default_value.str = "Unknown", .flags = UNI_PROPERTY_FLAG_READ_ONLY},
};
#endif

static const uni_property_t* my_platform_get_property(uni_property_idx_t idx) {
#if DEBUG_PROPERTY
    if (idx >= UNI_PROPERTY_IDX_LAST && idx < UNI_PROPERTY_IDX_UNI_LAST) {
        return &my_platform_properties[idx - UNI_PROPERTY_IDX_LAST];
    }
#else
    ARG_UNUSED(idx);
#endif
    return NULL;
}

/**
 * Handle Out-of-Band events sent by Bluepad32. 
 * For example, you can use this to detect when the "system" button of the gamepad is pressed, which is not sent as part of the regular gamepad data. 
 * In this example, we toggle the "seat" of the gamepad between A and B, and trigger the event to update the LEDs and rumble. 
 * You can customize this function to handle other events, such as when Bluetooth is enabled/disabled, etc.
 */
static void my_platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
    // switch (event) {
    //     case UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON: {
    //         uni_hid_device_t* d = data;

    //         if (d == NULL) {
    //             loge("ERROR: my_platform_on_oob_event: Invalid NULL device\n");
    //             return;
    //         }
    //         logi("custom: on_device_oob_event(): %d\n", event);

    //         my_platform_instance_t* ins = get_my_platform_instance(d);
    //         ins->gamepad_seat = ins->gamepad_seat == GAMEPAD_SEAT_A ? GAMEPAD_SEAT_B : GAMEPAD_SEAT_A;

    //         trigger_event_on_gamepad(d);
    //         break;
    //     }

    //     case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
    //         logi("custom: Bluetooth enabled: %d\n", (bool)(data));
    //         break;

    //     default:
    //         logi("my_platform_on_oob_event: unsupported event: 0x%04x\n", event);
    //         break;
    // }
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
        .name = "custom",
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

void controller_dump(mypad_t* pad) {
    logi("A: %d, B: %d, X: %d, Y: %d", pad->A, pad->B, pad->X, pad->Y);
    logi("UP: %d, DOWN: %d, LEFT: %d, RIGHT: %d", pad->UP, pad->DOWN, pad->LEFT, pad->RIGHT);
    logi("L: %d, R: %d, ZL: %d, ZR: %d", pad->L, pad->R, pad->ZL, pad->ZR);
    logi("TL: %d, TR: %d, MINUS: %d, PLUS: %d", pad->TL, pad->TR, pad->MINUS, pad->PLUS);
    logi("HOME: %d, CAPTURE: %d", pad->HOME, pad->CAPTURE);
    logi("LX: %d, LY: %d, RX: %d, RY: %d\n", pad->LX, pad->LY, pad->RX, pad->RY);
}
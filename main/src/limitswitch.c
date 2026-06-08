#include "limitswitch.h"
#include "driver/gpio.h"

void limitswitches_init(limitswitch_t *lims, int *gpios ,int n){
    uint64_t gpio = 0;
    for(int i = 0;i < n;i++){
        lims[i].gpio_num = (gpio_num_t)gpios[i];
        lims[i].pressed = false;
        gpio |= 1ULL << gpios[i];
    }
    gpio_config_t io_conf = {
        .pin_bit_mask = gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);
}
void get_limitswitches_level(limitswitch_t *lims, int n){
    for(int i = 0;i < n; i++){
        lims[i].pressed = (bool)gpio_get_level(lims[i].gpio_num);
    }
}

void limitswitches_dump(limitswitch_t *lims, int n){
    for(int i = 0;i < n;i++){
        fprintf(stderr,"%d: %s, ",i,lims[i].pressed ? "pressed" : "not pressed");
    }
    fprintf(stderr,"\n");
}

    // uint64_t gpio = 0;
    // for(int i = 0; i < n; i++){
    //     gpio &= 1ULL << lims[i].gpio_num;
    // }
    // gpio_dump_io_configuration(stderr, gpio);
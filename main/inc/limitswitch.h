#include "soc/gpio_num.h"
#include <stdbool.h>

typedef struct{
    gpio_num_t gpio_num;
    bool pressed;
}limitswitch_t;

void limitswitches_init(limitswitch_t *lims, int *gpios ,int n);

void get_limitswitches_level(limitswitch_t *lims, int n);

void limitswitches_dump(limitswitch_t *lims, int n);





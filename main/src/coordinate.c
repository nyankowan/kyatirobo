#ifndef COORDINATE_H
#define COORDINATE_H

#define COORDINATE_TAG "Coordinate"
#include "coordinate.h"
#include "esp_log.h"

polar_t to_polar(direct_t d){
    polar_t p;
    p.r = sqrt(d.x*d.x + d.y*d.y);
    p.theta = atan2(d.y, d.x);
    if(p.theta<0)p.theta+=2*M_PI; // atan2の返り値は-π～πの範囲なので、0～2πに変換
    return p;
}

void coordinate_dump(direct_t *xy){
    ESP_LOGI(COORDINATE_TAG, "XY: (%.2f, %.2f), R: %.2f, Theta: %.2f\n", xy->x, xy->y, to_polar(*xy).r, to_polar(*xy).theta);
}
#endif // COORDINATE_H
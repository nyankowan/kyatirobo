#ifndef COORDINATE_H
#define COORDINATE_H
#include "coordinate.h"

polar to_polar(direct d){
    polar p;
    p.r = sqrt(d.x*d.x + d.y*d.y);
    p.theta = atan2(d.y, d.x);
    if(p.theta<0)p.theta+=2*M_PI; // atan2の返り値は-π～πの範囲なので、0～2πに変換
    return p;
}
#endif // COORDINATE_H
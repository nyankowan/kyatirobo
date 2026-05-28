#include <math.h>   //コンパイルするとき-lmでリンクする必要がある

struct direct {
    double x;
    double y;
} typedef direct_t;
struct polar {
    double r;
    double theta;//rad
} typedef polar_t;

polar_t to_polar(direct_t d);
void coordinate_dump(direct_t *xy);
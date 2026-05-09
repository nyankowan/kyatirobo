#include <math.h>   //コンパイルするとき-lmでリンクする必要がある

struct direct {
    double x;
    double y;
} typedef direct;
struct polar {
    double r;
    double theta;//rad
} typedef polar;

polar to_polar(direct d);
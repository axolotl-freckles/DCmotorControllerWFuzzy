#pragma once

#include <stdint.h>
#include <math.h>

const int64_t SAMPLE_PERIOD_us = 100000;
const float   SAMPLE_TIME_s    = (SAMPLE_PERIOD_us*1e-6);

inline float rad_s2rpm(const float val) {return val*(60/(2*M_PI));}
inline float rpm2rad_s(const float val) {return val*(2*M_PI/60);}
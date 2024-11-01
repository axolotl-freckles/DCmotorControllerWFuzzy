#pragma once

#include <stdint.h>
#include <math.h>

constexpr int64_t SAMPLE_TIME_us = 50000;
constexpr int64_t SAMPLE_TIME_ms = SAMPLE_TIME_us/1000;
constexpr float   SAMPLE_TIME_s  = (SAMPLE_TIME_us*1e-6);
constexpr int     ENCODER_SLITS  = 80;

inline float rad_s2rpm(const float val) {return val*(60/(2*M_PI));}
inline float rpm2rad_s(const float val) {return val*(2*M_PI/60);}

constexpr int PWM_RESOLUTION = 8;
constexpr int PWM_MAX        = (1<<PWM_RESOLUTION)-1;

constexpr int TELEMETRY_TX_PIN = 18;

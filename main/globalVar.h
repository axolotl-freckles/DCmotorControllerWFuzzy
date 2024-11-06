#pragma once

#include <stdint.h>
#include <math.h>

constexpr int64_t SAMPLE_TIME_us = 25000;
constexpr int64_t SAMPLE_TIME_ms = SAMPLE_TIME_us/1000;
constexpr float   SAMPLE_TIME_s  = (SAMPLE_TIME_us*1e-6);
constexpr int     ENCODER_SLITS  = 80;

inline float rad_s2rpm(const float val) {return val*(60/(2*M_PI));}
inline float rpm2rad_s(const float val) {return val*(2*M_PI/60);}

constexpr int PWM_RESOLUTION = 8;
constexpr int PWM_MAX        = (1<<PWM_RESOLUTION)-1;

constexpr int ENCODER_GPIO = 26;
constexpr int PWM_OUT_GPIO = 14;

constexpr int TELEMETRY_TX_PIN = 18;

constexpr   int QUARTER_TABLE_SIZE = 60;
constexpr float AMPLITUDE = 10.0;

const float sine_wave_90_deg_LUT[QUARTER_TABLE_SIZE] = {
	0.0000, 0.0262, 0.0523, 0.0785, 0.1045, 0.1305, 0.1564, 0.1822,
	0.2079, 0.2334, 0.2588, 0.2840, 0.3090, 0.3338, 0.3584, 0.3827,
	0.4067, 0.4305, 0.4540, 0.4772, 0.5000, 0.5225, 0.5446, 0.5664,
	0.5878, 0.6088, 0.6293, 0.6494, 0.6691, 0.6883, 0.7071, 0.7254,
	0.7431, 0.7604, 0.7771, 0.7934, 0.8090, 0.8241, 0.8387, 0.8526,
	0.8660, 0.8788, 0.8910, 0.9026, 0.9135, 0.9239, 0.9336, 0.9426,
	0.9511, 0.9588, 0.9659, 0.9724, 0.9781, 0.9832, 0.9877, 0.9914,
	0.9945, 0.9969, 0.9986, 0.9996
};

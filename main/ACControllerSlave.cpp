/**
 * @file ACControllerTask.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-11-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef AC_CONTROLLER_SLAVE_CPP
#define AC_CONTROLLER_SLAVE_CPP

#include "taskClass.hpp"

#include <algorithm>
#include <chrono>
using namespace std::chrono;

#include "esp_timer.h"
#include "esp_log.h"

#include "globalVar.h"
#include "DataProcessTask.cpp"
#include "pid.hpp"
#include "pwm.h"
#include "Filters.hpp"
#include "slaveWifi.hpp"

constexpr int64_t PWM_SAMPLE_TIMEus = 200;
constexpr float   PWM_SAMPLE_TIMEs  = PWM_SAMPLE_TIMEus*1e-6;

#define DEBUG_TAG "AC_CTRLR_SLAVE"

/**
 * @brief It isn't recommended to reduce the sampling time below 10ms with the
 * FreeRTOS task scheduler. A hardware timer is used instead.
 *
 * @param args The queue handler for the stator flux angular speed.
 */
void IRAM_ATTR pwm_output_handler(void* args) {
	static const QueueHandle_t spwm_config_q = static_cast<QueueHandle_t>(args);
	static float theta = 0.0f;

	spwm_config_t spwm_config;
	xQueuePeekFromISR(spwm_config_q, &spwm_config);

	// Inline integrator logic
	theta += spwm_config.angular_speed*PWM_SAMPLE_TIMEs;
	if (theta > M_TAU) theta -= M_TAU;
	switch (spwm_config.phase) {
		case B:
			theta += M_TAU/3; break;
		case C:
			theta -= M_TAU/3; break;
		default:
			break;
	}
	float phase = std::clamp((sin(theta)+1)/2, AC_OUT_MIN, AC_OUT_MAX);

	pwm_set_duty(PWM_CHANNEL, static_cast<uint32_t>(phase*PWM_MAX)&PWM_MAX);
}

inline void innit_slave_spwm(QueueHandle_t spwm_config_q) {
	static constexpr ledc_timer_t PWM_TIMER_SRC = LEDC_TIMER_1;
	static constexpr int   PWM_FREQ_Hz    = 20000;
	esp_timer_handle_t timer_handle;

	esp_timer_create_args_t timer_config = {
		.callback = pwm_output_handler,
		.arg      = (void*)spwm_config_q,
		.dispatch_method = ESP_TIMER_TASK,
		.name = "TRIFASIC SINE GEN",
		.skip_unhandled_events = false
	};
	ESP_ERROR_CHECK(esp_timer_create(&timer_config, &timer_handle));
	ESP_LOGI(DEBUG_TAG, "SPWM timer created");

	ledc_timer_config_t pwm_timer_config = {
		.speed_mode      = LEDC_HIGH_SPEED_MODE,
		.duty_resolution = static_cast<ledc_timer_bit_t>(PWM_RESOLUTION),
		.timer_num       = PWM_TIMER_SRC,
		.freq_hz         = PWM_FREQ_Hz,
		.clk_cfg         = LEDC_AUTO_CLK,
		.deconfigure     = false
	};
	ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer_config));

	ledc_channel_config_t pwm_channel_config = {
		.gpio_num   = PWM_OUT_GPIO,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.channel    = PWM_CHANNEL,
		.intr_type  = LEDC_INTR_DISABLE,
		.timer_sel  = PWM_TIMER_SRC,
		.duty       = 0x0F,
		.hpoint     = 0,
		.flags = {.output_invert = 0}
	};
	ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel_config));

	ESP_ERROR_CHECK(ledc_fade_func_install(0));
	ESP_LOGI(DEBUG_TAG, "PWM channel initialized");

	float angular_speed    = 0.0f;

	xQueueOverwrite(spwm_config_q, &angular_speed);
	ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, PWM_SAMPLE_TIMEus));

	ESP_LOGI(DEBUG_TAG, "Initilizing task");
}

#endif
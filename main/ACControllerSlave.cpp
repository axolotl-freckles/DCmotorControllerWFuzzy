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

constexpr int64_t PWM_SAMPLE_TIMEus = 200;
constexpr float   PWM_SAMPLE_TIMEs  = PWM_SAMPLE_TIMEus*1e-6;

static constexpr float OUT_MIN = 0.02f;
static constexpr float OUT_MAX = 0.98f;
#define DEBUG_TAG "AC_CTRLR_SLAVE"

/**
 * @brief It isn't recommended to reduce the sampling time below 10ms with the
 * FreeRTOS task scheduler. A hardware timer is used instead.
 *
 * @param args The queue handler for the stator flux angular speed.
 */
void IRAM_ATTR pwm_output_handler(void* args) {
	static const QueueHandle_t fluxAngularSpeed = static_cast<QueueHandle_t>(args);
	static float theta = 0.0f;

	float angular_speed = M_TAU;
	xQueuePeekFromISR(fluxAngularSpeed, &angular_speed);

	// Inline integrator logic
	theta += angular_speed*PWM_SAMPLE_TIMEs;
	if (theta > M_TAU) theta -= M_TAU;

	float phase_A = std::clamp((sin(theta)+1)/2, OUT_MIN, OUT_MAX);

	pwm_set_duty(PWM_CHANNEL, static_cast<uint32_t>(phase_A*PWM_MAX)&PWM_MAX);
}

class ACControllerSlave : public Task {
public:
	static constexpr ledc_timer_t PWM_TIMER_SRC = LEDC_TIMER_1;
	static constexpr int   PWM_FREQ_Hz    = 20000;
	static constexpr float SINE_DISP_MULT = 30.0f;

	void taskFunction() override {
		TickType_t _last_time_awake = xTaskGetTickCount();
		Data_out input_data;

		LowPass inputFilter(0.5, SAMPLE_TIME_s);
		Integrator fluxAngularPosition(SAMPLE_TIME_s);
		float angular_speed    = 0.0f;

		xQueueOverwrite(_fluxAngularSpeed_q, &angular_speed);
		ESP_ERROR_CHECK(esp_timer_start_periodic(_timer_handle, PWM_SAMPLE_TIMEus));

		high_resolution_clock::time_point task_st;
		high_resolution_clock::time_point task_en;

		ESP_LOGI(DEBUG_TAG, "Initilizing task");

		while (true) {
			xQueueReceive(_data_q, &input_data, _period_tks);
			xQueueReceive(_fluxAngularSpeed_q, &angular_speed, _period_tks);

			task_st = high_resolution_clock::now();

			_telemetry_data[0] = input_data.motor_speed;
			_telemetry_data[1] = angular_speed;

			float phase_A = fluxAngularPosition(angular_speed);
			float phase_B = phase_A + M_TAU/3;
			float phase_C = phase_A - M_TAU/3;
			_telemetry_data[2] = sin(phase_A)*SINE_DISP_MULT;
			_telemetry_data[3] = sin(phase_B)*SINE_DISP_MULT;
			_telemetry_data[4] = sin(phase_C)*SINE_DISP_MULT;

			if (fluxAngularPosition.integralAcumulator() > M_TAU) {
				fluxAngularPosition.setIntegralAcumulator(
					fluxAngularPosition.integralAcumulator()-M_TAU
				);
			}

			task_en = high_resolution_clock::now();
			_telemetry_data[N_TELEMETRY_CHANNELS-1] = duration_cast<microseconds>(task_en-task_st).count();
			for (int i=0; i<N_TELEMETRY_CHANNELS; i++) {
				xQueueOverwrite(_telemetry_channels[i], _telemetry_data+i);
			}
			vTaskDelayUntil(&_last_time_awake, _period_tks);
		}
	}

	ACControllerSlave(
		const char* name, uint32_t stack_size, UBaseType_t prio,
		QueueHandle_t fluxAngularSpeed, QueueHandle_t data_q,
		QueueHandle_t telemetry_channels[],
		TickType_t period_ms
	) : Task(name, stack_size, prio),
		_telemetry_data{0.0f},
		_period_tks(period_ms / portTICK_PERIOD_MS),
		_fluxAngularSpeed_q(fluxAngularSpeed), _data_q(data_q),
		_timer_handle(nullptr)
	{
		for (int i=0; i<N_TELEMETRY_CHANNELS; i++)
			_telemetry_channels[i] = telemetry_channels[i];

		esp_timer_create_args_t timer_config = {
			.callback = pwm_output_handler,
			.arg      = (void*)_fluxAngularSpeed_q,
			.dispatch_method = ESP_TIMER_TASK,
			.name = "TRIFASIC SINE GEN",
			.skip_unhandled_events = false
		};
		ESP_ERROR_CHECK(esp_timer_create(&timer_config, &_timer_handle));
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
	}

private:
	QueueHandle_t _telemetry_channels[N_TELEMETRY_CHANNELS];
	float _telemetry_data[N_TELEMETRY_CHANNELS];
	const TickType_t _period_tks;

	QueueHandle_t      _fluxAngularSpeed_q;
	QueueHandle_t      _data_q;
	esp_timer_handle_t _timer_handle;
};

#endif
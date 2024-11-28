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
#ifndef AC_CONTROLLER_TASK_CPP
#define AC_CONTROLLER_TASK_CPP

#include "taskClass.hpp"

#include <algorithm>
#include <chrono>
using namespace std::chrono;

#include "esp_timer.h"

#include "globalVar.h"
#include "DataProcessTask.cpp"
#include "pid.hpp"
#include "pwm.h"
#include "Filters.hpp"

constexpr int64_t PWM_SAMPLE_TIMEus = 200;
constexpr float   PWM_SAMPLE_TIMEs  = PWM_SAMPLE_TIMEus*1e-6;

constexpr float OUT_MIN = 0.05f;
constexpr float OUT_MAX = 0.95f;

/**
 * @brief It isn't recommended to reduce the sampling time below 10ms with the
 * FreeRTOS task scheduler. A hardware timer is used instead.
 *
 * @param args The queue handler for the stator flux angular speed.
 */
void IRAM_ATTR pwm_output_handler(void* args) {
	static const QueueHandle_t fluxAngularSpeed = static_cast<QueueHandle_t>(args);
	static float phase_A_theta = 0.0f;

	float angular_speed = M_TAU;
	xQueuePeekFromISR(fluxAngularSpeed, &angular_speed);

	// Inline integrator logic
	phase_A_theta += angular_speed*PWM_SAMPLE_TIMEs;
	if (phase_A_theta > M_TAU) phase_A_theta -= M_TAU;

	float phase_A = std::clamp((sin(phase_A_theta        )+1)/2, OUT_MIN, OUT_MAX);
	float phase_B = std::clamp((sin(phase_A_theta+M_TAU/3)+1)/2, OUT_MIN, OUT_MAX);
	float phase_C = std::clamp((sin(phase_A_theta-M_TAU/3)+1)/2, OUT_MIN, OUT_MAX);

	pwm_set_duty(A_PWM_CHANNEL, static_cast<uint32_t>(phase_A*PWM_MAX)&PWM_MAX);
	pwm_set_duty(B_PWM_CHANNEL, static_cast<uint32_t>(phase_B*PWM_MAX)&PWM_MAX);
	pwm_set_duty(C_PWM_CHANNEL, static_cast<uint32_t>(phase_C*PWM_MAX)&PWM_MAX);
}

class ACControllerTask : public Task {
public:
	static constexpr ledc_timer_t PWM_TIMER_SRC = LEDC_TIMER_1;
	static constexpr int   PWM_FREQ_Hz    = 20000;
	static constexpr float SINE_DISP_MULT = 30.0f;

	void taskFunction() override {
		TickType_t _last_time_awake = xTaskGetTickCount();
		Data_out input_data;

		LowPass inputFilter(0.5, SAMPLE_TIME_s);
		Integrator fluxAngularPosition(SAMPLE_TIME_s);
		float angular_speed = M_TAU;
		float setted_frecuency = 0.0f;

		xQueueOverwrite(_fluxAngularSpeed, &angular_speed);
		ESP_ERROR_CHECK(esp_timer_start_periodic(_timer_handle, PWM_SAMPLE_TIMEus));

		high_resolution_clock::time_point task_st;
		high_resolution_clock::time_point task_en;

		while (true) {
			xQueueReceive(_data_q, &input_data, _period_tks);

			task_st = high_resolution_clock::now();
			setted_frecuency = inputFilter(input_data.set_point);
			angular_speed = setted_frecuency * M_TAU;
			xQueueOverwrite(_fluxAngularSpeed, &angular_speed);

			_telemetry_data[0] = setted_frecuency;
			_telemetry_data[1] = input_data.motor_speed;

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

	ACControllerTask(
		const char* name, uint32_t stack_size, UBaseType_t prio,
		QueueHandle_t data_q,
		QueueHandle_t telemetry_channels[],
		TickType_t period_ms
	)
	:
		Task(name, stack_size, prio),
		_data_q(data_q),
		_telemetry_data{0.0f},
		_period_tks(period_ms / portTICK_PERIOD_MS),
		_timer_handle(nullptr)
	{
		for (int i=0; i<N_TELEMETRY_CHANNELS; i++)
			_telemetry_channels[i] = telemetry_channels[i];

		_fluxAngularSpeed = xQueueCreate(1, sizeof(float));
		esp_timer_create_args_t timer_config = {
			.callback = pwm_output_handler,
			.arg      = (void*)_fluxAngularSpeed,
			.dispatch_method = ESP_TIMER_TASK,
			.name = "TRIFASIC SINE GEN",
			.skip_unhandled_events = false
		};
		ESP_ERROR_CHECK(esp_timer_create(&timer_config, &_timer_handle));

		ledc_timer_config_t pwm_timer_config = {
			.speed_mode      = LEDC_HIGH_SPEED_MODE,
			.duty_resolution = static_cast<ledc_timer_bit_t>(PWM_RESOLUTION),
			.timer_num       = PWM_TIMER_SRC,
			.freq_hz         = PWM_FREQ_Hz,
			.clk_cfg         = LEDC_AUTO_CLK,
			.deconfigure     = false
		};
		ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer_config));

		ledc_channel_config_t channelA_config = { // CHANNEL A ###########
			.gpio_num   = A_PWM_OUT_GPIO,
			.speed_mode = LEDC_HIGH_SPEED_MODE,
			.channel    = A_PWM_CHANNEL,
			.intr_type  = LEDC_INTR_DISABLE,
			.timer_sel  = PWM_TIMER_SRC,
			.duty       = 0x0F,
			.hpoint     = 0,
			.flags = {.output_invert = 0}
		};
		ESP_ERROR_CHECK(ledc_channel_config(&channelA_config));
		ledc_channel_config_t channelB_config = { // CHANNEL B ###########
			.gpio_num   = B_PWM_OUT_GPIO,
			.speed_mode = LEDC_HIGH_SPEED_MODE,
			.channel    = B_PWM_CHANNEL,
			.intr_type  = LEDC_INTR_DISABLE,
			.timer_sel  = PWM_TIMER_SRC,
			.duty       = 0x0F,
			.hpoint     = 0,
			.flags = {.output_invert = 0}
		};
		ESP_ERROR_CHECK(ledc_channel_config(&channelB_config));
		ledc_channel_config_t channelC_config = { // CHANNEL C ###########
			.gpio_num   = C_PWM_OUT_GPIO,
			.speed_mode = LEDC_HIGH_SPEED_MODE,
			.channel    = C_PWM_CHANNEL,
			.intr_type  = LEDC_INTR_DISABLE,
			.timer_sel  = PWM_TIMER_SRC,
			.duty       = 0x0F,
			.hpoint     = 0,
			.flags = {.output_invert = 0}
		};
		ESP_ERROR_CHECK(ledc_channel_config(&channelC_config));

		ESP_ERROR_CHECK(ledc_fade_func_install(0));
	}

private:
	const QueueHandle_t _data_q;
	QueueHandle_t _telemetry_channels[N_TELEMETRY_CHANNELS];
	float _telemetry_data[N_TELEMETRY_CHANNELS];
	const TickType_t _period_tks;

	QueueHandle_t      _fluxAngularSpeed;
	esp_timer_handle_t _timer_handle;
};

#endif
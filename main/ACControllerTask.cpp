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

#include <chrono>
using namespace std::chrono;

#include "globalVar.h"
#include "DataProcessTask.cpp"
#include "pid.hpp"
#include "pwm.h"
#include "Filters.hpp"

class ACControllerTask : public Task {
public:
	static constexpr float SINE_DISP_MULT = 30.0f;
	void taskFunction() override {
		TickType_t _last_time_awake = xTaskGetTickCount();
		Data_out input_data;

		LowPass inputFilter(0.9, SAMPLE_TIME_s);
		Integrator fluxAngularPosition(SAMPLE_TIME_s);
		float angular_speed = M_TAU;
		float setted_frecuency = 0.0f;

		high_resolution_clock::time_point task_st;
		high_resolution_clock::time_point task_en;

		while (true) {
			xQueueReceive(_data_q, &input_data, _period_tks);

			task_st = high_resolution_clock::now();
			setted_frecuency = inputFilter(input_data.set_point);
			angular_speed = setted_frecuency * M_TAU;
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
			
			pwm_set_duty(PWM_CHANNEL, ((uint32_t)(fluxAngularPosition.integralAcumulator()+1)/2)&PWM_MAX);

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
		_period_tks(period_ms / portTICK_PERIOD_MS)
	{
		for (int i=0; i<N_TELEMETRY_CHANNELS; i++)
			_telemetry_channels[i] = telemetry_channels[i];
	}

private:
	const QueueHandle_t _data_q;
	QueueHandle_t _telemetry_channels[N_TELEMETRY_CHANNELS];
	float _telemetry_data[N_TELEMETRY_CHANNELS];
	const TickType_t _period_tks;
};

#endif
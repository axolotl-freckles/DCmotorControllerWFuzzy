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

#include "globalVar.h"
#include "DataProcessTask.cpp"

class ACControllerTask : public Task {
public:
	void taskFunction() override {
		TickType_t _last_time_awake = xTaskGetTickCount();
		Data_out input_data;

		while (true) {
			xQueueReceive(_data_q, &input_data, _period_tks);
			_telemetry_data[0] = input_data.set_point;
			_telemetry_data[1] = input_data.motor_speed;
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
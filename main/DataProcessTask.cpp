/**
 * @file DataProcessTask.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-11-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef DATA_PROCESS_TASK_CPP
#define DATA_PROCESS_TASK_CPP

#include "taskClass.hpp"

#include <math.h>

#include "freertos/queue.h"

#include "globalVar.h"

typedef struct {
	int     adc_read;
	int32_t motor_count;
} Raw_data;

typedef struct {
	float set_point;
	float motor_speed;
} Data_out;

class DataProcessTask : public Task {
public:
	// Reference units in Hz
	// static constexpr float REF_MIN = 30.0f;
	// static constexpr float REF_MAX = 60.0f;
	static constexpr float REF_MIN =  1.0f;
	static constexpr float REF_MAX = 30.0f;
	static constexpr float REF_CONV_FACTOR = (REF_MAX-REF_MIN)/ADC_MAX;

	// Motor speed in rad/s
	static constexpr float M_TAU = 2*M_PI;
	static constexpr float MOTOR_SPEED_CONV_FACTOR = M_TAU/(ENCODER_SLITS*SAMPLE_TIME_s);

	void taskFunction() override {
		TickType_t _last_time_awake = xTaskGetTickCount();
		Raw_data raw_data;
		Data_out data_out;

		int32_t prev_motor_count = 0;
		int32_t motor_count_diff = 0;

		while (true) {
			xQueueReceive(_raw_data_q, &raw_data, _period_tks);

			data_out.set_point   = (float)raw_data.adc_read*REF_CONV_FACTOR + REF_MIN;
			motor_count_diff     = raw_data.motor_count - prev_motor_count;
			data_out.motor_speed = (float)(motor_count_diff)*MOTOR_SPEED_CONV_FACTOR;
			prev_motor_count = raw_data.motor_count;

			xQueueOverwrite(_out_data_q, &data_out);
			vTaskDelayUntil(&_last_time_awake, _period_tks);
		}
	}

	DataProcessTask(
		const char* name, uint32_t stack_size, UBaseType_t prio,
		QueueHandle_t raw_data_q, QueueHandle_t out_data_q,
		TickType_t period_ms
	)
	:
		Task(name, stack_size, prio),
		_raw_data_q(raw_data_q), _out_data_q(out_data_q),
		_period_tks(period_ms / portTICK_PERIOD_MS)
	{ }

private:
	const QueueHandle_t _raw_data_q;
	const QueueHandle_t _out_data_q;
	const TickType_t _period_tks;
};

#endif
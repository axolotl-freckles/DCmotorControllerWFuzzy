/**
 * @file Telemetry.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-11-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef TELEMETRY_CPP
#define TELEMETRY_CPP

#include <initializer_list>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_mac.h"

#include "CommProtocol.hpp"
#include "globalVar.h"
#include "taskClass.hpp"

template<int n_channels>
class Telemetry : public Task {
public:
	static constexpr int BUFFSIZE = 11*n_channels + 1;
	static const TickType_t QUEUE_TIMEOUT = SAMPLE_TIME_ms / portTICK_PERIOD_MS;

	void taskFunction() {
		TickType_t previousWakeTime = xTaskGetTickCount();
		char buffer[BUFFSIZE];
		float datapoints[n_channels] = {0.0f};
		int offset;
		while (true) {
			offset = 0;
			for(int i=0; i<n_channels; i++) {
				(void)xQueueReceive(_channels[i], datapoints+i, QUEUE_TIMEOUT);
				offset += sprintf(buffer+offset, "%10.2e,", datapoints[i]);
			}
			buffer[BUFFSIZE-2] = '\n';
			buffer[BUFFSIZE-1] = '\0';

			_communicationProtocol->transmit(buffer, std::strlen(buffer));
			xTaskDelayUntil(&previousWakeTime, SAMPLE_TIME_ms * 5 / portTICK_PERIOD_MS);
		}
	}

	Telemetry(
		const char *name,
		uint32_t stack_size,
		CommProtocol* communicationProtocol,
		QueueHandle_t channels[n_channels]
	)
	:
		Task(name, stack_size, 3),
		_communicationProtocol(communicationProtocol)
	{
		for (int i=0; i<n_channels; i++) {
			_channels[i] = channels[i];
		}
	}

private:
	QueueHandle_t _channels[n_channels];
	CommProtocol* _communicationProtocol;
};

#endif

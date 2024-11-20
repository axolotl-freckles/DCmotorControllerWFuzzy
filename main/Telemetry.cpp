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

#include "driver/uart.h"

#include "esp_mac.h"

#include "globalVar.h"
#include "taskClass.hpp"

class CommProtocol {
public:
	virtual int transmit(const char* data, int size) = 0;
private:
};

class UART : public CommProtocol {
public:
	virtual int transmit(const char* data, int size) override {
		return uart_write_bytes(_uart_num, data, size);
	}

	UART(
		uart_port_t uart_num,
		int tx_pin,
		int baud_rate,
		uart_parity_t parity,
		uart_stop_bits_t stop_bits,
		uint8_t rx_flow_ctrl_thresh = 122,
		uint32_t backup_before_sleep = 0
	)
	:
		_uart_num(uart_num),
		_tx_pin(tx_pin)
	{
		uart_config_t uart_config = {
			.baud_rate = 115200,
			.data_bits = UART_DATA_8_BITS,
			.parity    = parity,
			.stop_bits = stop_bits,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.rx_flow_ctrl_thresh = rx_flow_ctrl_thresh,
			.flags = {.backup_before_sleep = backup_before_sleep}
		};

		ESP_ERROR_CHECK(
			uart_param_config(_uart_num, &uart_config)
		);
		ESP_ERROR_CHECK(
			uart_set_pin(_uart_num, tx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)
		);
		ESP_ERROR_CHECK(
			uart_driver_install(uart_num, UART_HW_FIFO_LEN(_uart_num)+1, 0, 0, NULL, 0)
		);
	}
private:
	const uart_port_t _uart_num;
	const int _tx_pin;
};

template<int n_channels>
class Telemetry : public Task {
public:
	static constexpr int BUFFSIZE = 11*n_channels + 1;
	static const TickType_t QUEUE_TIMEOUT = SAMPLE_TIME_ms / portTICK_PERIOD_MS;

	void taskFunction() {
		TickType_t previousWakeTime = xTaskGetTickCount();
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

			// uart_write_bytes(_uart_num, buffer, std::strlen(buffer));
			_communicationProtocol->transmit(buffer, std::strlen(buffer));
			xTaskDelayUntil(&previousWakeTime, SAMPLE_TIME_ms / portTICK_PERIOD_MS);
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
		CommProtocol(communicationProtocol)
	{
		for (int i=0; i<n_channels; i++) {
			_channels[i] = channels[i];
		}
	}

private:
	char buffer[BUFFSIZE];
	QueueHandle_t _channels[n_channels];
	CommProtocol* _communicationProtocol;
};

#endif

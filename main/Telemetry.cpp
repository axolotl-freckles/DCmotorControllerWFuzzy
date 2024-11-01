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

#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "driver/uart.h"

#include "esp_mac.h"

#include "globalVar.h"
#include "taskClass.hpp"

class Telemetry : public Task {
public:
	static constexpr int BUFFSIZE = 40;
	static const TickType_t QUEUE_TIMEOUT = SAMPLE_TIME_ms / portTICK_PERIOD_MS;

	void taskFunction() {
		TickType_t previousWakeTime = xTaskGetTickCount();
		float refer, error, error_derivative, control_signal;
		while (true) {
			(void)xQueueReceive(_refer_q, &refer, QUEUE_TIMEOUT);
			(void)xQueueReceive(_error_q, &error, QUEUE_TIMEOUT);
			(void)xQueueReceive(_errorDerivative_q, &error_derivative, QUEUE_TIMEOUT);
			(void)xQueueReceive(  _controlSignal_q,   &control_signal, QUEUE_TIMEOUT);

			// const char asdf[] = "0.00e+00";
			(void)sprintf(
				buffer, "%9.2e,%9.2e,%9.2e,%9.2e\n",
				refer, error, error_derivative, control_signal
			);
			uart_write_bytes(_uart_num, buffer, BUFFSIZE);
			xTaskDelayUntil(&previousWakeTime, SAMPLE_TIME_ms / portTICK_PERIOD_MS);
		}
	}

	Telemetry(
		const char *name,
		uint32_t stack_size,
		uart_port_t uart_num,
		int tx_pin,
		QueueHandle_t refer_q,
		QueueHandle_t error_q,
		QueueHandle_t errorDerivative_q,
		QueueHandle_t controlSignal_q
	)
	:
		Task(name, stack_size, 3),
		_uart_num(uart_num), _tx_pin(tx_pin),
		_refer_q(refer_q), _error_q(error_q), _errorDerivative_q(errorDerivative_q),
		_controlSignal_q(controlSignal_q)
	{
		uart_config_t uart_config = {
			.baud_rate = 115200,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS
		};

		ESP_ERROR_CHECK(
			uart_param_config(_uart_num, &uart_config)
		);
		ESP_ERROR_CHECK(
			uart_set_pin(_uart_num, tx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)
		);
		ESP_ERROR_CHECK(
			uart_driver_install(uart_num, 1, BUFFSIZE, 0, NULL, 0)
		);
	}

private:
	char buffer[BUFFSIZE];
	const uart_port_t _uart_num;
	const int _tx_pin;
	const QueueHandle_t _refer_q;
	const QueueHandle_t _error_q;
	const QueueHandle_t _errorDerivative_q;
	const QueueHandle_t _controlSignal_q;
};

#endif

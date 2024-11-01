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

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "driver/uart.h"

#include "globalVar.h"
#include "taskClass.hpp"

class Telemetry : public Task {
public:
	Telemetry(
		const char *name,
		uint32_t stack_size,
		uart_port_t uart_num,
		QueueHandle_t refer_q,
		QueueHandle_t error_q,
		QueueHandle_t errorDerivative_q,
		QueueHandle_t controlSignal_q
	)
	:
		Task(name, stack_size, 3),
		_uart_num(uart_num),
		_refer_q(refer_q), _error_q(error_q), _errorDerivative_q(errorDerivative_q),
		_controlSignal_q(controlSignal_q)
	{
		uart_config_t uart_config = {
			.baud_rate = 115200,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS
		};

		ESP_ERROR_CHECK(uart_param_config(_uart_num, &uart_config));
	}

private:
	const uart_port_t _uart_num;
	const QueueHandle_t _refer_q;
	const QueueHandle_t _error_q;
	const QueueHandle_t _errorDerivative_q;
	const QueueHandle_t _controlSignal_q;
};

#endif

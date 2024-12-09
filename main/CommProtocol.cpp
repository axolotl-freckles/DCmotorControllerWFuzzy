/**
 * @file CommProtocol.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-11-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef COMM_PROTOCOL_CPP
#define COMM_PROTOCOL_CPP

#include "CommProtocol.hpp"

int UART::transmit(const char* data, int size) {
	return uart_write_bytes(_uart_num, data, size);
}
UART::UART(
	uart_port_t uart_num,
	int tx_pin,
	int baud_rate,
	uart_parity_t parity,
	uart_stop_bits_t stop_bits,
	uint8_t rx_flow_ctrl_thresh,
	uint32_t backup_before_sleep
)
:
	_uart_num(uart_num),
	_tx_pin(tx_pin)
{
	uart_config_t uart_config = {
		.baud_rate = baud_rate,
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

#endif
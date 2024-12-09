/**
 * @file CommProtocol.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-11-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include "driver/uart.h"

class CommProtocol {
public:
	/**
	 * @brief Transmits data
	 * 
	 * @param data pointer to the data array to transmit
	 * @param size amount of data (in bytes) that will be transmitted
	 * @return the amount of bytes that were successfully transmitted
	 */
	virtual int transmit(const char* data, int size) = 0;
private:
};

class UART : public CommProtocol {
public:
	virtual int transmit(const char* data, int size) override;

	/**
	 * @brief Initializes the uart channel and constructs
	 * an UART CommProtocol object.
	 * 
	 * @param uart_num 
	 * @param tx_pin 
	 * @param baud_rate 
	 * @param parity 
	 * @param stop_bits 
	 * @param rx_flow_ctrl_thresh 
	 * @param backup_before_sleep 
	 */
	UART(
		uart_port_t uart_num,
		int tx_pin,
		int baud_rate,
		uart_parity_t    parity,
		uart_stop_bits_t stop_bits,
		uint8_t  rx_flow_ctrl_thresh = 122,
		uint32_t backup_before_sleep = 0
	);

private:
	const uart_port_t _uart_num;
	const int _tx_pin;
};

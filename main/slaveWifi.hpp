/**
 * @file slaveWifi.hpp
 * @author Zyanya ACMAX
 * @brief 
 * @version 0.1
 * @date 2024-12-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"
#include "mdns.h"

enum phase_t {A, B, C};

struct spwm_config_t {
	float angular_speed;
	phase_t phase;
};

void innit_slave_wifi(QueueHandle_t spwm_config_q);

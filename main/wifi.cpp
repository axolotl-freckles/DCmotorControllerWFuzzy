/**
 * @file wifi.cpp
 * @author Zyanya y ACMAX
 * @brief 
 * @version 0.1
 * @date 2024-12-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef WIFI_CPP
#define WIFI_CPP

#include <stdio.h>
#include <string.h>
#include <math.h>
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

#include "jsonParser.hpp"

#define WIFI_SSID CONFIG_WIFI_SSID        // SSID de Wi-Fi configurado en menuconfig
#define WIFI_PASS CONFIG_WIFI_PASSWORD    // Contraseña de Wi-Fi
#define MASTER_PORT 12345                 // Puerto del maestro
#define MDNS_SERVICE_TYPE "_spwm"         // Servicio mDNS del maestro

static const char *TAG = "SLAVE_DEVICE";

enum phase_t {A, B, C};

struct spwm_config_t {
	float angular_speed;
	phase_t phase;
};

// Eventos para la sincronización
static EventGroupHandle_t sync_event_group;
static constexpr int BIT_CONNECTED_TO_MASTER = BIT0; // Indica conexión al maestro
static constexpr int BIT_APPLY_CONFIG = 1<<1;

// static QueueHandle_t spwm_config_q   = xQueueCreate(1, sizeof(spwm_config_t));
static QueueHandle_t config_params_q = xQueueCreate(1, sizeof(spwm_config_t));

static esp_timer_handle_t config_apply_timer_handle = nullptr;

static bool connect_to_master(const char *ip, int port);
static void mdns_discovery_task(void *pvParameters);
static void slave_receive_task(void *pvParameters);
static void config_apply_h(void *params);
static inline void update_config(const char* config_JSON, int json_len);

// Función para manejar eventos Wi-Fi
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
		ESP_LOGI(TAG, "Connecting to Wi-Fi...");
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		ESP_LOGI(TAG, "Disconnected. Reconnecting...");
		esp_wifi_connect();
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));

		// Inicia mDNS para descubrir al maestro
		xTaskCreate(mdns_discovery_task, "mdns_discovery", 2080, NULL, 5, NULL);
	}
}

// Inicializar Wi-Fi
static void wifi_init(void) {
	esp_netif_init();
	esp_event_loop_create_default();
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&cfg);

	esp_event_handler_instance_register(WIFI_EVENT,
	                                    ESP_EVENT_ANY_ID,
	                                    &wifi_event_handler,
	                                    NULL,
	                                    NULL);
	esp_event_handler_instance_register(IP_EVENT,
	                                    IP_EVENT_STA_GOT_IP,
	                                    &wifi_event_handler,
	                                    NULL,
	                                    NULL);

	wifi_config_t wifi_config = {
		.sta = {
			.ssid = WIFI_SSID,
				.password = WIFI_PASS,
				.threshold = {.authmode = WIFI_AUTH_WPA2_PSK},
				.pmf_cfg = {
					.capable = true,
					.required = false
				},
			},
	};
	esp_wifi_set_mode(WIFI_MODE_STA);
	esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
	esp_wifi_start();
}

void innit_slave(QueueHandle_t spwm_config_q) {
	ESP_LOGI(TAG, "Starting device as slave");
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	sync_event_group = xEventGroupCreate();  // Crear grupo de eventos

	esp_timer_create_args_t oneshot_timer = {
		.callback = config_apply_h,
		.arg      = spwm_config_q,
		.dispatch_method = ESP_TIMER_TASK,
		.name = "Config. Apply",
		.skip_unhandled_events = false
	};
	esp_timer_create(&oneshot_timer, &config_apply_timer_handle);
	spwm_config_t default_config = {
		.angular_speed = 0,
		.phase = A
	};
	xQueueOverwrite(config_params_q, &default_config);

	wifi_init();  // Inicializar Wi-Fi
}

// Conectar al maestro
static bool connect_to_master(const char *ip, int port) {
	struct sockaddr_in dest_addr;
	dest_addr.sin_family = AF_INET;
	dest_addr.sin_port = htons(port);
	inet_pton(AF_INET, ip, &dest_addr.sin_addr);

	int sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
		return false;
	}

	ESP_LOGI(TAG, "Connecting to master at %s:%d...", ip, port);
	if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
		ESP_LOGE(TAG, "Socket connection failed: errno %d", errno);
		close(sock);
		return false;
	}

	ESP_LOGI(TAG, "Connected to master!");
	xEventGroupSetBits(sync_event_group, BIT_CONNECTED_TO_MASTER);

	// Inicia la tarea para recibir datos del maestro
	xTaskCreate(slave_receive_task, "slave_receive", 3072, (void *)sock, 5, NULL);
	return true;
}

// Tarea para descubrir al maestro mediante mDNS
static void mdns_discovery_task(void *pvParameters) {
	ESP_LOGI(TAG, "Searching for master via mDNS...");
	mdns_init();

	mdns_result_t *results = NULL;
	esp_err_t err = mdns_query_ptr(MDNS_SERVICE_TYPE, "_tcp", 10000, 10, &results);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "mDNS query failed: %s", esp_err_to_name(err));
		vTaskDelete(NULL);
		return;
	}

	if (!results) {
		ESP_LOGE(TAG, "No master found via mDNS");
		vTaskDelete(NULL);
		return;
	}

	// Conectar al maestro
	mdns_result_t *r = results;
	while (r) {
		char addr_str[128];
		inet_ntoa_r(r->addr->addr, addr_str, sizeof(addr_str));
		ESP_LOGI(TAG, "Found master at %s:%d", addr_str, r->port);

		if (connect_to_master(addr_str, r->port)) {
				break;  // Conexión exitosa
		}
		r = r->next;
	}

	mdns_query_results_free(results);
	vTaskDelete(NULL);
}

// Tarea para recibir datos del maestro
static void slave_receive_task(void *pvParameters) {
	int sock = (int)pvParameters;
	char rx_buffer[128];

	while (1) {
		int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
		if (len < 0) {
			ESP_LOGE(TAG, "Error receiving data: errno %d", errno);
			break;
		} else if (len == 0) {
			ESP_LOGI(TAG, "Connection closed by master");
			break;
		} else {
			rx_buffer[len] = '\0';  // Null-terminate the received data
			ESP_LOGI(TAG, "Received from master: %s", rx_buffer);

			// Procesar mensaje (por ahora solo imprimir)
			if (strstr(rx_buffer, "SYNC_SIGNAL")) {
				ESP_LOGI(TAG, "Synchronization signal received!");
				char* msg_st = strstr(rx_buffer, "SYNC_SIGNAL");
				if (msg_st == rx_buffer) {
					msg_st += 11;
				}
				else {
					*msg_st = '\0';
					msg_st = rx_buffer;
				}
				uint64_t time_to_apply = atoll(msg_st);
				if (time_to_apply > esp_timer_get_time()) {
					ESP_LOGI(TAG, "Time to apply: %lld", time_to_apply - esp_timer_get_time());

					esp_timer_start_once(config_apply_timer_handle, time_to_apply - esp_timer_get_time());
					send(sock, "ACK", 3, 0);
				}
				else {
					send(sock, "NAK", 3, 0);
					ESP_LOGI(TAG, "NAK, invalid time: %lld", time_to_apply);
					ESP_LOGI(TAG, "Current time     : %lld", esp_timer_get_time());
				}
			}
			else {
					ESP_LOGI(TAG, "Config. data received!");
					update_config(rx_buffer, len);
					send(sock, "ACK", 3, 0);
			}
		}
	}

	close(sock);
	vTaskDelete(NULL);
}

static void config_apply_h(void *params) {
	QueueHandle_t spwm_config_q = (QueueHandle_t)params;
	spwm_config_t config;
	xQueuePeekFromISR(config_params_q, &config);
	xQueueOverwriteFromISR(spwm_config_q, &(config.angular_speed), NULL);
	// ESP_LOGI("UPDATE", "w:%f", config.angular_speed);
}

static inline void update_config(const char* config_JSON, int json_len) {
	JSONDict newConfig;
	spwm_config_t config;
	xQueuePeek(config_params_q, &config, 1);
	parse_JSON(config_JSON, json_len, newConfig);

	JSONDict::iterator configEntry = newConfig.begin();

	if (newConfig.end() != (configEntry = newConfig.find("frequency"))) {
		ESP_LOGI(TAG, "frequency: %s", configEntry->second.c_str());
		config.angular_speed = std::stof(configEntry->second)*2*M_PI;
	}
	if (newConfig.end() != (configEntry = newConfig.find("phase"))) {
		ESP_LOGI(TAG, "phase: %s", configEntry->second.c_str());
		switch (configEntry->second[0]) {
			case 'A':
				config.phase = A; break;
			case 'B':
				config.phase = B; break;
			case 'C':
				config.phase = C; break;
			default:
				ESP_LOGI(TAG, "Invalid phase!");
		}
	}

	xQueueOverwrite(config_params_q, &config);
}

#endif
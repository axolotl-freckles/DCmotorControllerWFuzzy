/**
 * @file wifi_master.cpp
 * @author Zyanya
 * @brief 
 * @version 0.1
 * @date 2024-12-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef WIFI_MASTER_CPP
#define WIFI_MASTER_CPP

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "mdns.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#define TAG "SocketMaster"
#define MAX_SLAVES 1
#define SYNC_SIGNAL "SYNC_SIGNAL"
#define MDNS_SERVICE_TYPE_MASTER "_spwm"         // Servicio mDNS del maestro
#define MDNS_SERVICE_TYPE_SLAVE  "_slave"
#define SLAVE_PORT 2000
#define MASTER_PORT 12345                 // Puerto del maestro
#define MDNS_QUERY_TIMEOUT 50000

// Event group bits
#define SLAVE1_CONNECTED_BIT (1 << 0)
#define SLAVE2_CONNECTED_BIT (1 << 1)
#define SLAVE3_CONNECTED_BIT (1 << 2)
#define ALL_SLAVES_CONNECTED (SLAVE1_CONNECTED_BIT | SLAVE2_CONNECTED_BIT | SLAVE3_CONNECTED_BIT)

EventGroupHandle_t event_group;
static bool mdns_initialized = false;

// Start MDNS service
void start_mdns_service() {
    if (mdns_initialized) {
        ESP_LOGW(TAG, "MDNS already initialized.");
        return;
    }
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("esp32_master"));
    ESP_ERROR_CHECK(mdns_service_add("AC_CONTROL_MASTER", MDNS_SERVICE_TYPE_MASTER, "_tcp", MASTER_PORT, NULL, 0));
    ESP_LOGI(TAG, "MDNS started with hostname: esp32_master");

    mdns_initialized = true;
}

// Initialize Access Point
void init_wifi_as_ap() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = "ESP32_MASTER",
            .password = "12345678",
            .ssid_len = 0,
            .channel = 1,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = MAX_SLAVES
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Access Point started with SSID: ESP32_MASTER");
}

// Initialize communication with a slave
int initialize_socket(const char *ip, int port) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        return -1;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &server_addr.sin_addr) <= 0) {
        ESP_LOGE(TAG, "Invalid IP address: %s", ip);
        close(sock);
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to connect to slave at %s:%d", ip, port);
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "Connected to slave at %s:%d", ip, port);
    return sock;
}

// Send data to the slave
void send_data_to_slave(int sock, const char *data) {
    if (send(sock, data, strlen(data), 0) < 0) {
        ESP_LOGE(TAG, "Failed to send data");
    } else {
        ESP_LOGI(TAG, "Data sent to slave: %s", data);
    }
}

// Receive ACK from slave
bool receive_ack_from_slave(int sock) {
    char buffer[64];
    int len = recv(sock, buffer, sizeof(buffer) - 1, 0);
    if (len > 0) {
        buffer[len] = '\0';
        ESP_LOGI(TAG, "ACK received: %s", buffer);
        return strcmp(buffer, "ACK") == 0;
    } else {
        ESP_LOGE(TAG, "Failed to receive ACK");
        return false;
    }
}

// Close the socket
void close_socket(int sock) {
    close(sock);
    ESP_LOGI(TAG, "Connection closed");
}

// Configure motor using dynamic discovery and pre-configured settings
void configure_motor_with_sockets() {
    const char *initial_frequency = "50"; // Initial frequency (50 Hz)
    const char *phases[] = {"A", "B", "C"}; // Predefined phases
    const char *slave_ips[] = {NULL, NULL, NULL}; // To store discovered IPs
    int discovered_slaves = 0;

    // Discover slaves dynamically using MDNS
    ESP_LOGI(TAG, "Discovering slaves via MDNS...");
    for (int i = 0; i < MAX_SLAVES; i++) {
        mdns_result_t *result = NULL;
        esp_err_t err = mdns_query_ptr(MDNS_SERVICE_TYPE_SLAVE, "_tcp", SLAVE_PORT, MDNS_QUERY_TIMEOUT, &result);
        if (err == ESP_OK && result != NULL) {
            char ip_str[INET_ADDRSTRLEN];
            if (result->addr->addr.type == IPADDR_TYPE_V4) {
                // Convert IPv4 address to string
                inet_ntoa_r(result->addr->addr.u_addr.ip4, ip_str, sizeof(ip_str));
                slave_ips[i] = strdup(ip_str); // Save the IP as a string
                ESP_LOGI(TAG, "Discovered slave %d: %s", i + 1, slave_ips[i]);
                discovered_slaves++;
            } else {
                ESP_LOGE(TAG, "Unsupported IP address type for slave %d", i + 1);
            }
            mdns_query_results_free(result);
        } else {
            ESP_LOGW(TAG, "Slave %d not found via MDNS", i + 1);
        }
    }


    if (discovered_slaves < MAX_SLAVES) {
        ESP_LOGE(TAG, "Not all slaves were discovered (%d/%d). Exiting...", discovered_slaves, MAX_SLAVES);
        return;
    }

    // Configure each phase
    for (int i = 0; i < MAX_SLAVES; i++) {
        char json_data[128];
        snprintf(json_data, sizeof(json_data), "{\"frequency\":\"%s\", \"phase\":\"%s\"}", initial_frequency, phases[i]);

        int attempts = 0;
        const int max_attempts = 3;
        int sock = -1;

        while (attempts < max_attempts) {
            sock = initialize_socket(slave_ips[i], 12345); // Fixed port: 12345
            if (sock >= 0) {
                ESP_LOGI(TAG, "Successfully connected to slave %s after %d attempt(s)", slave_ips[i], attempts + 1);
                break;
            } else {
                ESP_LOGW(TAG, "Failed to connect to slave %s (attempt %d/%d)", slave_ips[i], attempts + 1, max_attempts);
                vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second before retrying
            }
            attempts++;
        }

        if (sock < 0) {
            ESP_LOGE(TAG, "Could not connect to slave %s after %d attempts. Skipping...", slave_ips[i], max_attempts);
            continue;
        }

        // Send JSON data to the slave
        send_data_to_slave(sock, json_data);

        // Wait for ACK from the slave
        if (!receive_ack_from_slave(sock)) {
            ESP_LOGE(TAG, "Failed to receive ACK from slave %s. Skipping...", slave_ips[i]);
            close_socket(sock);
            continue;
        }

        close_socket(sock);

        // Set the corresponding bit in the Event Group
        xEventGroupSetBits(event_group, 1 << i);
    }

    // Wait until all slaves are connected
    xEventGroupWaitBits(event_group, ALL_SLAVES_CONNECTED, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "All slaves are connected. Sending SYNC_SIGNAL...");

    // Send SYNC_SIGNAL to all slaves
    for (int i = 0; i < MAX_SLAVES; i++) {
        if (slave_ips[i] == NULL) continue;

        int sock = initialize_socket(slave_ips[i], 12345);
        if (sock >= 0) {
            send_data_to_slave(sock, SYNC_SIGNAL);
            close_socket(sock);
        } else {
            ESP_LOGE(TAG, "Failed to send SYNC_SIGNAL to slave %s", slave_ips[i]);
        }
    }

    // Free allocated memory for slave IPs
    for (int i = 0; i < MAX_SLAVES; i++) {
        if (slave_ips[i] != NULL) {
            free((void *)slave_ips[i]);
        }
    }
}

// Main application entry point
void innit_master_wifi(void) {
    printf("Starting three-phase motor configuration...\n");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wi-Fi as Access Point
    init_wifi_as_ap();

    // Start MDNS service
    start_mdns_service();

    // Initialize event group
    event_group = xEventGroupCreate();

    // Configure motor
    configure_motor_with_sockets();

    printf("Motor configuration completed.\n");
}

#endif

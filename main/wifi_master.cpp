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
#define MAX_SLAVES 3
#define SYNC_SIGNAL "SYNC_SIGNAL"

// Event group bits
#define SLAVE1_CONNECTED_BIT (1 << 0)
#define SLAVE2_CONNECTED_BIT (1 << 1)
#define SLAVE3_CONNECTED_BIT (1 << 2)
#define ALL_SLAVES_CONNECTED (SLAVE1_CONNECTED_BIT | SLAVE2_CONNECTED_BIT | SLAVE3_CONNECTED_BIT)

EventGroupHandle_t event_group;

// Array to track assigned phases
bool phases_assigned[3] = {false, false, false}; // Indices: 0 = A, 1 = B, 2 = C

// Array to track connected slaves
char slave_ips[MAX_SLAVES][16];

static bool mdns_initialized = false;

void start_mdns_service ()
{
    if (mdns_initialized)
    {
        ESP_LOGW(TAG, "mdns Inicializado");
        return;
    }
    ESP_ERROR_CHECK(mdns_init()); // Asegúrate de descomentar esta línea
    ESP_ERROR_CHECK(mdns_hostname_set("esp32_master"));
    ESP_LOGI(TAG, "MDNS iniciado con hostname: esp32_master");

    mdns_initialized = true;
}

// Initialize Access Point
void init_wifi_as_ap() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

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
    ESP_LOGI(TAG, "Access Point iniciado con SSID: ESP32_MASTER");
}



// Function to initialize communication with a slave
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

// Function to send data to the slave
void send_data_to_slave(int sock, const char *data) {
    if (send(sock, data, strlen(data), 0) < 0) {
        ESP_LOGE(TAG, "Failed to send data");
    } else {
        ESP_LOGI(TAG, "Data sent to slave: %s", data);
    }
}

// Function to receive ACK from slave
bool receive_ack_from_slave(int sock) {
    char buffer[64];
    int len = recv(sock, buffer, sizeof(buffer) - 1, 0);
    if (len > 0) {
        buffer[len] = '\0';
        ESP_LOGI(TAG, "ACK recibido: %s", buffer);
        return strcmp(buffer, "ACK") == 0;
    } else {
        ESP_LOGE(TAG, "Error al recibir ACK");
        return false;
    }
}

// Function to close the socket
void close_socket(int sock) {
    close(sock);
    ESP_LOGI(TAG, "Connection closed");
}

// Main function to configure motor
void configure_motor_with_sockets() {
    char slave_ip[64];
    char phase[2];
    char frequency[16];
    int freq_value;

    // Request frequency
    while (true) {
        printf("Enter motor frequency (30-60 Hz): ");
        fgets(frequency, sizeof(frequency), stdin);
        freq_value = atoi(frequency);
        if (freq_value >= 30 && freq_value <= 60) {
            break;
        }
        printf("Invalid frequency. It must be between 30 and 60 Hz.\n");
    }

    snprintf(frequency, sizeof(frequency), "%d", freq_value);

    // Configure each phase
    for (int i = 0; i < MAX_SLAVES; i++) {
        while (true) {
            printf("Enter the phase for the slave (A, B, C): ");
            fgets(phase, sizeof(phase), stdin);
            phase[strcspn(phase, "\n")] = '\0'; // Remove newline character

            // Validate that the phase is unique
            if (strcmp(phase, "A") == 0 && !phases_assigned[0]) {
                phases_assigned[0] = true;
                break;
            } else if (strcmp(phase, "B") == 0 && !phases_assigned[1]) {
                phases_assigned[1] = true;
                break;
            } else if (strcmp(phase, "C") == 0 && !phases_assigned[2]) {
                phases_assigned[2] = true;
                break;
            } else {
                printf("Invalid or duplicate phase. Try again.\n");
            }
        }

        printf("Enter the IP address of the slave for phase %s: ", phase);
        fgets(slave_ip, sizeof(slave_ip), stdin);
        slave_ip[strcspn(slave_ip, "\n")] = '\0'; // Remove newline character
        strncpy(slave_ips[i], slave_ip, sizeof(slave_ip));

        // Create the JSON
        char json_data[128];
        snprintf(json_data, sizeof(json_data), "{\"frequency\":\"%s\", \"phase\":\"%s\"}", frequency, phase);

        // Initialize socket and send data
        int sock = initialize_socket(slave_ip, 12345); // Fixed port: 12345
        if (sock >= 0) {
            send_data_to_slave(sock, json_data);

            // Wait for ACK
            if (!receive_ack_from_slave(sock)) {
                ESP_LOGE(TAG, "Failed to receive ACK from slave %s", slave_ip);
                close_socket(sock);
                return;
            }
            close_socket(sock);
        }

        // Set event group bit
        xEventGroupSetBits(event_group, 1 << i);
    }

    // Wait for all slaves to be connected
    xEventGroupWaitBits(event_group, ALL_SLAVES_CONNECTED, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "All slaves are connected. Sending SYNC_SIGNAL...");

    // Send SYNC_SIGNAL to all slaves
    for (int i = 0; i < MAX_SLAVES; i++) {
        int sock = initialize_socket(slave_ips[i], 12345);
        if (sock >= 0) {
            send_data_to_slave(sock, SYNC_SIGNAL);
            close_socket(sock);
        }
    }
}

// Main application entry point
void innit_master_wifi(void) {
    printf("Starting three-phase motor configuration...\n");
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
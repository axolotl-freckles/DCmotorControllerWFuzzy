#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdbool.h>
#include "esp_log.h"

#define TAG "SocketMaster"

// Array to track assigned phases
bool phases_assigned[3] = {false, false, false}; // Indices: 0 = A, 1 = B, 2 = C

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
    for (int i = 0; i < 3; i++) {
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

        // Create the JSON
        char json_data[128];
        snprintf(json_data, sizeof(json_data), "{\"frequency\":\"%s\", \"phase\":\"%s\"}", frequency, phase);

        // Initialize socket and send data
        int sock = initialize_socket(slave_ip, 12345); // Fixed port: 12345
        if (sock >= 0) {
            send_data_to_slave(sock, json_data);
            close_socket(sock);
        }
    }
}

// Main application entry point
void app_main(void) {
    printf("Starting three-phase motor configuration...\n");
    configure_motor_with_sockets();
    printf("Motor configuration completed.\n");
}

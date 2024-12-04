#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"

// Configuración de bits de evento
#define ACK_PHASE_A (1 << 0)
#define ACK_PHASE_B (1 << 1)
#define ACK_PHASE_C (1 << 2)
#define ALL_PHASES_ACK (ACK_PHASE_A | ACK_PHASE_B | ACK_PHASE_C)

static const char *TAG = "MotorControl";
static EventGroupHandle_t ack_event_group;

// Función para enviar JSON al slave
void send_json_to_slave(const char *slave_ip, const char *phase, const char *frequency) {
    // Construir JSON
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "frequency", frequency);
    cJSON_AddStringToObject(json, "phase", phase);
    char *json_string = cJSON_Print(json);

    // Configurar cliente HTTP
    esp_http_client_config_t config = {
        .url = slave_ip,
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_string, strlen(json_string));

    // Enviar solicitud
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "JSON enviado a %s: %s", slave_ip, json_string);

        // Simula recibir un ACK
        if (strcmp(phase, "A") == 0) {
            xEventGroupSetBits(ack_event_group, ACK_PHASE_A);
        } else if (strcmp(phase, "B") == 0) {
            xEventGroupSetBits(ack_event_group, ACK_PHASE_B);
        } else if (strcmp(phase, "C") == 0) {
            xEventGroupSetBits(ack_event_group, ACK_PHASE_C);
        }
    } else {
        ESP_LOGE(TAG, "Error enviando JSON a %s: %s", slave_ip, esp_err_to_name(err));
    }

    // Limpiar
    esp_http_client_cleanup(client);
    cJSON_Delete(json);
    free(json_string);
}

// Tarea para configuración 
void configure_motor_task(void *arg) {
    char slave_ip[64];
    char phase[2];
    char frequency[16];
    int freq_value;

    // Solicitar frecuencia
    while (true) {
        printf("Ingrese la frecuencia para el motor (30-60 Hz): ");
        fgets(frequency, sizeof(frequency), stdin);
        freq_value = atoi(frequency);
        if (freq_value >= 30 && freq_value <= 60) {
            break;
        }
        printf("Frecuencia inválida. Debe estar entre 30 y 60 Hz.\n");
    }

    snprintf(frequency, sizeof(frequency), "%d", freq_value);

    // Configurar cada fase
    for (int i = 0; i < 3; i++) {
        printf("Ingrese la IP del slave para la fase %c: ", 'A' + i);
        fgets(slave_ip, sizeof(slave_ip), stdin);
        slave_ip[strcspn(slave_ip, "\n")] = '\0'; // Eliminar nueva línea

        snprintf(phase, sizeof(phase), "%c", 'A' + i);

        // Enviar JSON al slave
        send_json_to_slave(slave_ip, phase, frequency);
    }

    vTaskDelete(NULL);
}

// Tarea para monitorear sincronización de fases
void sync_task(void *arg) {
    while (true) {
        EventBits_t bits = xEventGroupWaitBits(ack_event_group, ALL_PHASES_ACK, pdTRUE, pdTRUE, portMAX_DELAY);
        if ((bits & ALL_PHASES_ACK) == ALL_PHASES_ACK) {
            ESP_LOGI(TAG, "Todas las fases sincronizadas. Motor listo para arrancar.");
            // Activar señal de sincronización
            break;
        }
    }
    vTaskDelete(NULL);
}

// Inicializar WiFi
void wifi_init() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_ap_config = {
        .ap = {
            .ssid = "ESP32_AP",
            .password = "AP_Password",
            .ssid_len = strlen("ESP32_AP"),
            .max_connection = 3,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Punto de acceso iniciado: SSID=ESP32_AP, Password=AP_Password");
}

void app_main(void) {
    ack_event_group = xEventGroupCreate();

    wifi_init();

    // Crear tareas para configuración y sincronización
    xTaskCreate(configure_motor_task, "Configure Motor Task", 4096, NULL, 5, NULL);
    xTaskCreate(sync_task, "Sync Task", 2048, NULL, 5, NULL);
}

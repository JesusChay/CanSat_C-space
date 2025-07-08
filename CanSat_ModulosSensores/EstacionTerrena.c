#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_now.h"
#include <string.h>
#include <stdbool.h>

// Identificadores por cada carga
typedef enum {
    PRIMARY_LOAD = 1,     // Identificador de la primaria
    SECONDARY_LOAD = 2    // Identificador de la secundaria
} load_type_t;

// Estructura base que incluye el identificador
typedef struct {
    load_type_t load_type;
} load_header_t;

// Estructura de datos de la carga primaria
typedef struct {
    load_header_t header;  // Identificador
    float speed;
    float accelx, accely, accelz;
    float gyrox, gyroy, gyroz;
    float magx, magy, magz;
    float altitud;
    float latitude, longitude;
    bool decoupling_status;
} primary_load_data_t;

// Estructura de datos de la carga secundaria
typedef struct {
    load_header_t header;  // Identificador
    float temperature, humidity, pressure;
    float accelx, accely, accelz;
    float gyrox, gyroy, gyroz;
    float altitud;
    float latitude, longitude;
} secondary_load_data_t;

// Inicializar WiFi
void wifi_init() {
    nvs_flash_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t wifi_drvconfig = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_drvconfig));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Configurar para larga distancia
    esp_wifi_set_max_tx_power(80);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);  //  Larga distancia
    
    uint8_t STA_MAC_Addr[6];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, STA_MAC_Addr);
    printf("MAC ADDR: %02X:%02X:%02X:%02X:%02X:%02X\n", 
           STA_MAC_Addr[0], STA_MAC_Addr[1], STA_MAC_Addr[2], 
           STA_MAC_Addr[3], STA_MAC_Addr[4], STA_MAC_Addr[5]);
}

// Inicializar ESP-NOW
void espnow_init() {
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_peer_info_t peer = {
        .channel = 1,
        .encrypt = false,
    };
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

// Callback para recibir los datos en el ESP-NOW
static void RecibirMensajes_ESPNOW_CALLBACK(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    if (data_len < sizeof(load_header_t)) {
        printf("Mensaje demasiado corto\n");
        return;
    }

    // Obtener el tipo de carga del encabezado
    load_header_t *header = (load_header_t *)data;
    
    switch(header->load_type) {
        case PRIMARY_LOAD:
            if (data_len == sizeof(primary_load_data_t)) {
                primary_load_data_t *received_data = (primary_load_data_t *)data;
                printf("[PRIMARY] %.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%s\n",
                       received_data->speed, 
                       received_data->accelx, received_data->accely, received_data->accelz,
                       received_data->gyrox, received_data->gyroy, received_data->gyroz,
                       received_data->magx, received_data->magy, received_data->magz,
                       received_data->altitud, received_data->latitude, received_data->longitude,
                       received_data->decoupling_status ? "true" : "false");
            } else {
                printf("Tamano incorrecto para datos primarios: %d (esperado %d)\n", 
                       data_len, sizeof(primary_load_data_t));
            }
            break;
            
        case SECONDARY_LOAD:
            if (data_len == sizeof(secondary_load_data_t)) {
                secondary_load_data_t *received_data = (secondary_load_data_t *)data;
                printf("[SECONDARY] %.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f\n",
                       received_data->temperature,
                       received_data->accelx, received_data->accely, received_data->accelz,
                       received_data->gyrox, received_data->gyroy, received_data->gyroz,
                       received_data->humidity, received_data->pressure,
                       received_data->altitud, received_data->latitude, received_data->longitude);
            } else {
                printf("Tamano incorrecto para datos secundarios: %d (esperado %d)\n", 
                       data_len, sizeof(secondary_load_data_t));
            }
            break;
            
        default:
            printf("No hay cargas conectadas: %d\n", header->load_type);
            break;
    }
}

void app_main() {
    wifi_init();
    espnow_init();
    ESP_ERROR_CHECK(esp_now_register_recv_cb(RecibirMensajes_ESPNOW_CALLBACK));
    
    while (1) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_err.h"
#include "esp_system.h"

#define TINY_GSM_MODEM_SIM7600
#define UART_PORT       UART_NUM_1
#define TXD_PIN         17
#define RXD_PIN         16
#define BUF_SIZE        512
#define MAX_PHOTOS      4

#include <HTTPClient.h>
#include "TinyGsmClient.h"

static const char *TAG = "UART_RECEIVER";
static int current_photo_index = 0;

void init_spiffs(void) {
    ESP_LOGI(TAG, "🗂️ Inicializando SPIFFS...");
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 10,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Error al montar SPIFFS (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "✅ SPIFFS montado correctamente");
    }
}

void save_photo(const uint8_t *data, uint8_t len) {
    // Construir el nombre del archivo
    char filename[32];
    snprintf(filename, sizeof(filename), "/spiffs/photo_%d.txt", current_photo_index);

    // Si ya existe, lo borramos
    remove(filename);

    // Guardar nuevo archivo
    FILE *f = fopen(filename, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "❌ No se pudo abrir %s para escritura", filename);
        return;
    }

    for (int i = 0; i < len; i++) {
        fprintf(f, "%02X ", data[i]);
    }
    fprintf(f, "\n");
    fclose(f);
    ESP_LOGI(TAG, "💾 Imagen guardada en %s", filename);

    // Avanzar índice circular
    current_photo_index = (current_photo_index + 1) % MAX_PHOTOS;
}

void list_photos(void) {
    DIR *dir = opendir("/spiffs");
    if (!dir) {
        ESP_LOGE(TAG, "❌ No se pudo abrir el directorio /spiffs");
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        ESP_LOGI(TAG, "📂 Archivo: %s", entry->d_name);
    }
    closedir(dir);
}

void uart_receiver_task(void *arg) {
    uint8_t comm;
    uint8_t len_body = 0;
    uint8_t body[BUF_SIZE];
    uint8_t crc_rx;

    while (1) {
        int read = uart_read_bytes(UART_PORT, &comm, 1, portMAX_DELAY);
        if (read != 1) continue;

        uint8_t crc = comm;

        if (uart_read_bytes(UART_PORT, &len_body, 1, pdMS_TO_TICKS(20)) == 1) {
            crc ^= len_body;

            if (len_body > 0) {
                int body_len = uart_read_bytes(UART_PORT, body, len_body, portMAX_DELAY);
                if (body_len != len_body) {
                    ESP_LOGE(TAG, "❌ Body incompleto");
                    continue;
                }

                for (int i = 0; i < len_body; i++) {
                    crc ^= body[i];
                }
            }
        } else {
            len_body = 0;
        }

        if (uart_read_bytes(UART_PORT, &crc_rx, 1, portMAX_DELAY) != 1) {
            ESP_LOGE(TAG, "❌ No se recibió CRC");
            continue;
        }

        if (crc == crc_rx) {
            ESP_LOGI(TAG, "✅ CRC OK | COMM: 0x%02X | LEN: %d", comm, len_body);
            if (len_body > 0) {
                ESP_LOG_BUFFER_HEXDUMP(TAG, body, len_body, ESP_LOG_INFO);
                save_photo(body, len_body);  // Guardar como imagen individual
            }
        } else {
            ESP_LOGE(TAG, "❌ CRC FAIL: local 0x%02X vs recibido 0x%02X", crc, crc_rx);
        }
    }
}

void setup() {
    init_spiffs();
    list_photos();

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    xTaskCreate(uart_receiver_task, "uart_rx_task", 4096, NULL, 10, NULL);
}

void loop () {}

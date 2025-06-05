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
#define MAX_PHOTOS      4

#include <HTTPClient.h>
#include "TinyGsmClient.h"
#include "board_master.hpp"

static const char *TAG = "UART_RECEIVER";
static int current_photo_index = 0;

namespace {
    uart::BoardMaster master;
}

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


void setup() {
    master.init();
    master.begin();

    xTaskNotify(master.talk_task, 0x0, eSetValueWithOverwrite);
}

void loop () {}

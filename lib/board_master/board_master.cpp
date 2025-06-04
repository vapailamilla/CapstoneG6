#include "board_master.hpp"

namespace uart {
/* ============= Timer callback functions ============= */
static void timeout_handler(TimerHandle_t timer) {
  ESP_LOGW(TAG, "UART response timeout reached");
  // Retrieve the pointer to the BoardMaster from the timer ID
  BoardMaster * master = ( BoardMaster * ) pvTimerGetTimerID( timer );
  /// TODO: reset corresponding variables
};

/* ============= Constructor, Destructor, init() and begin() ============= */
/* Constructor */
BoardMaster::BoardMaster() {
};

/* Destructor */
BoardMaster::~BoardMaster() {
  // Uninstall the UART driver
  uart_driver_delete(UART_PORT);
  // Reset the pins to their default state
  gpio_reset_pin((gpio_num_t)TX_PIN);
  gpio_reset_pin((gpio_num_t)RX_PIN);
  gpio_reset_pin(BUILTIN_LED);
  /// Kill FreeRTOS tasks (check that their handles are not NULL)
  if (talk_task)  
    vTaskDelete(talk_task);
  if (response_handling_task)
    vTaskDelete(response_handling_task);
}; // end destructor

/* init() */
esp_err_t BoardMaster::init() {
  // Default to no errors
  esp_err_t ret{ESP_OK};
  /********************************
   * GPIO and UART configuration  *
   ********************************/
  /* UART driver. If any of this fail, abort */
  uart_config_t uart_config = {
      .baud_rate  = BAUD_RATE,
      .data_bits  = UART_DATA_8_BITS,
      .parity     = UART_PARITY_EVEN,
      .stop_bits  = UART_STOP_BITS_1,
      .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
  };
  ESP_ERROR_CHECK( uart_param_config(UART_PORT, &uart_config) );
  ESP_ERROR_CHECK( uart_set_pin(UART_PORT, TX_PIN, RX_PIN, -1, -1) );
  ESP_ERROR_CHECK( uart_driver_install(UART_PORT, 256, 256, 256, &uart_queue, 0) );
  /* Configure the BUILTIN_LED GPIO pin */
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << BUILTIN_LED),  // Select which pin(s) to configure
    .mode         = GPIO_MODE_OUTPUT,       // Set pin to output mode 
    .pull_up_en   = GPIO_PULLUP_DISABLE,    // Disable internal pull-up
    .pull_down_en = GPIO_PULLDOWN_ENABLE,   // Disable internal pull-down
    .intr_type    = GPIO_INTR_DISABLE       // No interrupt on this pin
  };
  gpio_config(&io_conf);

  /* Create the UART sync timer */
  response_timer = xTimerCreateStatic("response timeout",
                                  pdMS_TO_TICKS(UART_SYNC_TIMEOUT),
                                  pdFALSE, this, timeout_handler,
                                  &response_timer_state);

  ESP_GOTO_ON_FALSE(response_timer == NULL, ESP_FAIL, err, TAG, "Couldn't create the UART sync timer!");
  ESP_LOGD(TAG, "UART Worker Init Done");

  ESP_LOGD(TAG, "BoardMaster Init Done");

  return ret;

 err:
  ESP_LOGE(TAG, "Error code (0x%x)", ret);
  ESP_LOGE(TAG, "Fatal failure, restarting in 5 seconds...");
  vTaskDelay(pdMS_TO_TICKS(5000));
  esp_restart();
}

/* begin() */
esp_err_t BoardMaster::begin() {
  esp_err_t ret{ESP_OK};
  /* Attempt to create listening task */
  /// TODO: Transform into static tasks
  ESP_GOTO_ON_ERROR(ret = xTaskCreatePinnedToCore(talk, "talk_task",
                                                  4096, (void *)this, 
                                                  20, &talk_task, 0) == pdPASS ? ESP_OK : ESP_FAIL,
                    err, TAG, "Couldn't create the talking task!");
  /* Attempt to create command handling task */
  ESP_GOTO_ON_ERROR(ret = xTaskCreatePinnedToCore(handle_response, "handle_reponse",
                                                  4096, (void *)this, 
                                                  8, &response_handling_task, 1) == pdPASS ? ESP_OK : ESP_FAIL,
                    err, TAG, "Couldn't create the response handling task!");
  
  return ret;

 err:
  ESP_LOGE(TAG, "Fatal failure, restarting in 5 seconds...");
  vTaskDelay(pdMS_TO_TICKS(5000));
  esp_restart();
} // end begin

/* ============= FreeRTOS tasks (talk and handle_response) ============= */
void BoardMaster::talk(void *p) {
  esp_err_t ret{ESP_OK}; // This is needed for the ESP_GOTO_ON_FALSE macro
  BoardMaster *master = reinterpret_cast<BoardMaster *>(p);
  command_t command;
  while(true) {
    // Wait for this task to be notified
    command = (command_t)ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    ESP_LOGD(TAG, "Sending out command 0x%02x", command);
    master->crc = command;
    master->command_buffer[0] = command;
    switch (command) {
      case CONF:
        master->build_conf();
        break;
      // Commands without body
      case STATUS:
      case NEXT_PHOTO:
      case RESET:
        command_len = 2;
        master->command_buffer[1] = crc;
        break;
      default:
        break;
    } 
  }
}

/* ============= TX Helper functions ============= */
void BoardMaster::build_conf() {
  command_buffer[1] = 0x04;
  command_len = 7;                  // Command byte + length byte + 4 epoch bytes + crc
  timeval time;
  gettimeofday(&time, nullptr);
  for (int i=0; i<4; i++) {
    // Add the time epoch bytes to the buffer
    command_buffer[2 + i] = (time.tv_sec & (0xFF << (8*i))) >> 8*i;
    crc ^= command_buffer[2 + i];   // Include bytes into crc calculation
  }
  command_buffer[6] = crc;          // crc at the end of the buffer
}


esp_err_t BoardMaster::send_command(command_t command) {
  esp_err_t ret{ESP_OK};
  ret = uart_write(command_buffer, command_len);
  return ret;
}

esp_err_t BoardMaster::uart_write(unsigned char *buff, size_t len) {
  esp_err_t ret{ESP_OK};
  if (buff == NULL || len == 0) {
    return ESP_FAIL;
  }
  ret = uart_write_bytes(UART_PORT, (const uint8_t*) buff, len) >= len ? ESP_OK : ESP_FAIL;

  return ret;
} // end uart_write

} // end namespace
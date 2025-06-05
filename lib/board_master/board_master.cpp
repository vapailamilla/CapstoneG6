#include "board_master.hpp"

namespace uart {
/* ============= Timer callback functions ============= */
static void timeout_handler(TimerHandle_t timer) {
  ESP_LOGW(TAG, "UART response timeout reached");
  // Retrieve the pointer to the BoardMaster from the timer ID
  BoardMaster * master = ( BoardMaster * ) pvTimerGetTimerID( timer );
  /// TODO: reset corresponding variables
  master->timeout_reached = true;
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
    } // end switch
    // Send the command and start the response timer
    master->send_command(command);
    master->timeout_reached = true;
    xTimerStart(master->response_timer, portMAX_DELAY);
    // Notify the response_handling_task
    xTaskNotify(master->response_handling_task, command, eSetValueWithOverwrite);
  } // end while
} // end talk

void BoardMaster::handle_response(void *p) {
  esp_err_t ret{ESP_OK}; // This is needed for the ESP_GOTO_ON_FALSE macro
  command_t command;
  BoardMaster *master = reinterpret_cast<BoardMaster *>(p);
  unsigned char rx_buffer[512]; // A buffer for reception.
  uint32_t rx_count = 0;        // A byte counter
  uint32_t body_len = 0;        // The length of the received command
  uint8_t  body_len_count = 0;  /// TODO: Receive more than one byte for the LEN portion
  int bytes_received;
  while(true) {
    // Wait for this task to be notified
    command = (command_t)ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    while (!timeout_reached) {
      bytes_received = uart_read_bytes(UART_PORT, rx_buffer, 512, 0);
      if (bytes_received <= 0) continue;
      /*==============================*/
      
      /* Bytes where received, handle them depending on rx_state */
      for (int idx=0; idx<bytes_received; idx++) {
        switch (rx_state) {
          case WAITING_RES:
            ESP_LOGD(TAG, "Received response byte 0x%02X", rx_buffer[idx]);
            /// TODO: Handle cases where response byte includes NACK or JUST_RESET
            switch (command) {
              /* Commands without body sequences */
              case CONF:
                rx_state = WAITING_CRC;           // Next byte is the CRC
                break;
              /* Commands with body sequences */
              default:
                rx_state = WAITING_LEN;
                body_len = 0;
                break;
            } // end switch
            break;
          
          case WAITING_LEN :
            /// TODO: Allow receiving more than one length byte
            body_len = rx_buffer[idx];
            crc ^= body_len;
            ESP_LOGV(TAG, "Body length: %lu bytes", body_len);
            rx_state = WAITING_BODY;        // Next bytes received will be the command body
            break;
          
          case WAITING_BODY:
            /// TODO: handle this differently for NEXT_PHOTO command
            switch(command) {
              case STATUS:
                while (rx_count < body_len && idx < bytes_received) {
                  response_buff[rx_count] = rx_buffer[idx];   // Add byte to buffer
                  crc ^= rx_buffer[idx];
                  idx++;                                    // Move to next byte
                  rx_count++;                               // Up the byte count
                }
                --idx;                                      // Move back a place so CRC is correctly recieved
                for (int p=0; p < body_len / 8; p++) {
                  uint32_t photo_size = 0;
                  uint32_t photo_timestamp = 0;
                  for (int t=0; t<4; t++) {
                    photo_size |= response_buff[2+8*p] << 8*(3-t);
                    photo_timestamp |= response_buff[2+4+8*p] << 8*(3-t);
                  }
                  ESP_LOGI(TAG, "Photo %d:\tSize=%lu B\tTimestamp=%lu", p+1, photo_size);
                }
                break;

              case NEXT_PHOTO:
                break;

            } // end switch
            ESP_LOGV(TAG, "Body bytes received: %lu", rx_count);
            if (rx_count < body_len) break;             // Not enough bytes received yet
            master->response_len = rx_count;            // Inform the master about the amount of received bytes
            rx_state = WAITING_CRC;                     // Next byte is CRC
            break;
          
          case WAITING_CRC:
            ESP_LOGV(TAG, "Local CRC = 0x%02X\tReceived CRC = 0x%02X", crc, rx_buffer[idx]);
            if (crc != rx_buffer[idx]){
              ESP_LOGE(TAG, "Bad CRC!");
            }
            break;
          
          case RESET_RX:
            idx = bytes_received;  // break the for loop (this should never execute)
            break;
        } // end switch(rx_state)
      } // end for

    } // end while
     done:
      /// TODO: What if there's still bytes in the buffer? This must be a function, not a goto tag
      rx_count = 0;               // reset byte count
      rx_state = WAITING_RES;     // Back to the beginning
      // Stop the UART timeout timer
      ESP_GOTO_ON_FALSE(  xTimerStop(master->response_timer, 0), 
                          ESP_FAIL, err, TAG, 
                          "Couldn't stop the UART response timer!"  );
      /* Tell the talking task we're done */
      ESP_LOGD(TAG, "Notifying talking task (0x%02X)", command);
      xTaskNotify(master->talk_task,command, eSetValueWithoutOverwrite );
      // Send response to the Host
  } // end while(true)
 err:
  ESP_LOGE(TAG, "Fatal failure, restarting in 5 seconds...");
  vTaskDelay(pdMS_TO_TICKS(5000));
  esp_restart();
} // end handle_response

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
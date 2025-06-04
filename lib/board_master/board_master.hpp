/**
 * @file board_master.hpp
 * @author Simón Jaramillo <sjaramillo5@uc.cl>
 * @date June 2025
 * 
 * @brief This file contains the declarations for a BoardMaster
 * class in charge of controlling the communication interface
 * between both boards of CapstoneCAM.
 */

#pragma once

/* Espressif includes */
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_check.h>
#include <esp_err.h>
#include <esp_log.h>
/* Standard includes */
#include <sys/time.h>
/* FreeRTOS includes */
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/timers.h>
/* Custom includes */

namespace uart {
/**
 * @brief A common log tag for this component
 */
static const char* TAG = "BoardMaster";

/**
 * @brief The default UART port, baudrate and timeout
 */
static const uart_port_t UART_PORT{UART_NUM_2};
static const int BAUD_RATE{3000000};
static const int UART_SYNC_TIMEOUT{3000};

/**
 * @anchor gpio_definitions
 * @name GPIO definitions
 * 
 * @brief these are constants for easy connection to ESP32 DevKit-C.
 */
/**@{*/
constexpr static const gpio_num_t BUILTIN_LED{GPIO_NUM_NC};   // No built in LED in this kit
constexpr static const gpio_num_t TX_PIN{GPIO_NUM_19};
constexpr static const gpio_num_t RX_PIN{GPIO_NUM_18};
/*@}*/

/**
 * @anchor enums
 * @name Enums used for the BoardMaster
 */
/**@{*/

typedef enum {
  CONF,                                       // Configure RTC and Initialize Components
  STATUS,                                     // Request System Status
  NEXT_PHOTO,                                 // Request next photo in queue
  MAX_COMMAND,
  RESET = 0x7F,                               // Reset ESP
} command_t;

typedef enum {
  ACK,                    // Command correctly received
  NACK,                   // Bad CRC or not enough bytes
  JUST_RESET              // The peripheral has just been reset (no configuration received yet)
} err_code_t;
/*@}*/

/**
 * @brief A wrapper class for everything related to this library
 */
class BoardMaster {
  /**
   * @anchor ctor
   * @name Constructor, Destructor, Begins, Getters and Setters
   *
   * @brief All the needed setup functions.
   */
  /**@{*/
 public:
  /**
   * @brief The constructor for BoardMaster
   */
  BoardMaster();
  /**
   * @brief Deinitilize UART interface and destroy this object
   */
  ~BoardMaster();
  /**
   * @brief Initilize the UART interface
   */
  esp_err_t init();
  /**
   * @brief Begin talking into the UART interface
   */
  esp_err_t begin();
  /**@}*/

  /**
   * @anchor free_rtos
   * @name FreeRTOS tasks declarations
   * 
   * @brief All the needed FreeRTOS tasks, members and functions are declared here.
   * @todo Turn tasks into static tasks
   */
  /**@{*/
 private:
  /**
   * @brief A handle for the UART talking task. Used to send commands in real time.
   */
  TaskHandle_t talk_task;
  /**
   * @brief The function in charge of talking and listening to the UART line.
   *        This is intended to run as a FreeRTOS task
   * @todo Turn this task into a static task
   * 
   * @param p A pointer to a `BoardMaster` instance
   */
  static void talk(void *p);
  /**
   * @brief A handle for the response handling task. Used to process the peripheral
   * responses to the BoardMaster's commands
   */
  TaskHandle_t response_handling_task;
  /**
   * @brief The function in charge of receiving the responses.
   * 
   * @param p A pointer to a `BoardMaster` instance
   */
  static void handle_response(void *p);
  /**@} */

  /**
   * @anchor master_members
   * @name Class members
   * 
   * @brief All the variables needed for the BoardMaster
   */
 private:
  /**@{ */
  /**
   * @brief The UART event queue used to receive incoming bytes
   */
  inline static QueueHandle_t uart_queue;
  /**
   * @brief The UART response timer handle. This timer allows the `BoardMaster`
   * to abort waiting for the peripheral response
   * 
   * @note This timer is meant to be created as a Static Timer
   */
  static inline TimerHandle_t response_timer;
  /**
   * @brief The buffer that holds the response timer state
   */
  static inline StaticTimer_t response_timer_state;
  /**
   * @brief The rolling CRC
   */
  inline static uint8_t crc;
  /**
   * @brief A buffer for the bytes to be sent
   * @todo Make the length of this buffer configurable
   */
  inline static uint8_t command_buffer[512];
  /**
   * @brief The amount of body-bytes to be sent
   */
  inline static uint32_t command_len;
  /**
   * @brief The buffer where the peripheral response will be received
   */
  inline static uint8_t response_buff[512];
  /**
   * @brief The amount of characters to be received into `response_buff`
   */
  inline static uint8_t response_len = 1;
  /**@} */

  /**
   * @anchor master_methods
   * @name Class functions
   * 
   * @brief All the function members needed for the `BoardMaster`
   */
  /**@{ */
  esp_err_t send_command(command_t command);
 private:
  /**
   * @brief Sends bytes to the UART line
   * @param buff A pointer to the buffer to be sent
   * @param len The length of the buffer (the amount of bytes to send)
   */
  static esp_err_t uart_write(unsigned char *buff, size_t len);
  /**
   * @brief Build the CONF command sequence in the `command_buffer`
   */
  void BoardMaster::build_conf();
  /**@} */

}; // end BoardMaster

} // end namespace
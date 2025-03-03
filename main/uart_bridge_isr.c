/**
 * @file esp32_dual_uart.c
 * @brief ESP32 UART Bridge for bidirectional communication between two UART ports.
 * 
 * This implementation enables seamless communication between two UART ports,
 * using FreeRTOS tasks and queues for efficient data transfer. The design is fully event-driven,
 * reducing CPU usage by utilizing the ESP-IDF UART driver’s event queue mechanism.
 *
 *  * 
 * ## Queue Usage
 * - Two event queues (`uart1_event_queue`, `uart2_event_queue`) handle UART events.
 * - Two data queues (`uart1_queue`, `uart2_queue`) ensure synchronization between receiver and forwarding tasks.
 * - Queues eliminate the need for explicit locking mechanisms.
 * 
 * ## Deadlock Prevention & Synchronization
 * - No mutexes or semaphores are used as queue operations are inherently thread-safe.
 * - Each UART has a separate task for event handling and data forwarding, preventing conflicts.
 * - Instead of `portMAX_DELAY`, a shorter timeout (`pdMS_TO_TICKS(10)`) is used in `uart_read_bytes()`
 *   to optimize CPU usage and prevent indefinite blocking, ensuring smooth data flow.
 */

 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include "esp_log.h"
 #include "driver/uart.h"
 #include "driver/gpio.h"
 #include <string.h>
 
 #define BUF_SIZE (128)
 #define UART1_NUM UART_NUM_1
 #define UART2_NUM UART_NUM_2
 
 #define UART1_TX_PIN 1
 #define UART1_RX_PIN 2
 #define UART1_RTS_PIN 4
 #define UART1_CTS_PIN 3
 
 #define UART2_TX_PIN 5
 #define UART2_RX_PIN 6
 #define UART2_BT_EN_PIN 7

 #define DEBUG 1 // Set to 1 to enable debug traces, 0 to disable
 
 static const char *TAG = "UART_BRIDGE";
 static QueueHandle_t uart1_queue;
 static QueueHandle_t uart2_queue;
 static QueueHandle_t uart1_event_queue;
 static QueueHandle_t uart2_event_queue;
 
 /**
  * @brief Task to periodically print a keep-alive message
  */
 void keep_alive_task(void *arg) {
     while (1) {
         ESP_LOGI(TAG, "Keep-alive message");
         vTaskDelay(pdMS_TO_TICKS(5000));
     }
 }
 
 
 /**
  * @brief Structure to hold UART data packets
  */
 typedef struct {
     uint8_t data[BUF_SIZE]; ///< Data buffer
     int len; ///< Data length
 } uart_packet_t;
 

 


 /**
  * @brief Initializes UART configuration
  */
 void uart_init() {
     ESP_LOGI(TAG, "Initializing UART...");
     
     uart_driver_install(UART1_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart1_event_queue, 0);
     uart_driver_install(UART2_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart2_event_queue, 0);
     
     uart_param_config(UART1_NUM, &(uart_config_t){
         .baud_rate = 115200,
         .data_bits = UART_DATA_8_BITS,
         .parity = UART_PARITY_ODD,
         .stop_bits = UART_STOP_BITS_1,
         .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
         .source_clk = UART_SCLK_DEFAULT,
     });
     uart_param_config(UART2_NUM, &(uart_config_t){
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_ODD,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    });
    uart_set_pin(UART1_NUM, UART1_TX_PIN, UART1_RX_PIN, UART1_RTS_PIN, UART1_CTS_PIN);
    uart_set_pin(UART2_NUM, UART2_TX_PIN, UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

}     
    

     /**
 * @brief Initializes UART communication with the specified baud rate.
 * 
 * @param port UART port (e.g., UART_NUM_1)
 * @param baud_rate Desired baud rate (9600, 19200, etc.)
 */
void init_uart_with_params(uart_port_t port, int baud_rate) {
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(port, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(port, UART2_TX_PIN, UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

 
 /**
  * @brief Sends an AT command and verifies the response.
  *
  * @param cmd The AT command to send.
  * @param expected_response The expected response string.
  * @return true if the expected response is received, false otherwise.
  */
 bool send_at_command(uart_port_t port, const char *cmd, const char *expected_response) {
     uint8_t data[BUF_SIZE];
 
     #ifdef DEBUG
     ESP_LOGI(TAG, "Sending command: %s", cmd);
     #endif
 
     uart_write_bytes(port, cmd, strlen(cmd));
     vTaskDelay(pdMS_TO_TICKS(500));
 
     int len = uart_read_bytes(port, data, BUF_SIZE, pdMS_TO_TICKS(1000));
     if (len > 0) {
         data[len] = '\0';
 
         #ifdef DEBUG
         ESP_LOGI(TAG, "Received: %s", (char *)data);
         #endif
 
         return (strstr((char *)data, expected_response) != NULL);
     }
     return false;
 }
 
 /**
  * @brief Enables AT mode on JDY-31 / HC-05 by pulling the ENABLE pin low.
  */
 void enter_AT_mode(gpio_num_t enable_pin) {
     gpio_set_direction(enable_pin, GPIO_MODE_OUTPUT);
     gpio_set_level(enable_pin, 0); // Hold LOW to enter AT mode
     vTaskDelay(pdMS_TO_TICKS(500));
 }
 
 /**
  * @brief Exits AT mode by setting the ENABLE pin high.
  */
 void exit_AT_mode(gpio_num_t enable_pin) {
     gpio_set_level(enable_pin, 1); // Set HIGH to exit AT mode
     vTaskDelay(pdMS_TO_TICKS(500));
 }
 


 /**
 * @brief Detects the baud rate of JDY-31 / HC-05 and configures Bluetooth parameters.
 *
 * @param port UART port (e.g., UART_NUM_1)
 * @param enable_pin BT module enable PIN
 * @param device_name New Bluetooth device name
 * @param target_baud_rate Desired baud rate (9600, 19200, etc.)
 * @param pin Bluetooth enable PIN
 * @param mode Bluetooth mode (0 = Slave, 1 = Master)
 * @param auto_connect Auto-connect mode (0 = Manual, 1 = Automatic)
 * @return Detected baud rate if successful, -1 if detection fails.
 */

int detect_and_configure_bt_module(
    uart_port_t port,
    gpio_num_t enable_pin,
    const char *device_name,
    int target_baud_rate,
    const char *pin,
    int mode,
    int auto_connect
){
     int baudRates[] = {9600, 19200, 38400, 57600, 115200};
     uint8_t data[BUF_SIZE];
 
     enter_AT_mode(enable_pin); // Enter AT mode
 
     for (int i = 0; i < (sizeof(baudRates) / sizeof(baudRates[0])); i++) {
         init_uart_with_params(port, baudRates[i]);
         vTaskDelay(pdMS_TO_TICKS(500));
 
         // Send AT command
         const char *testCmd = "AT\r\n";
         uart_write_bytes(port, testCmd, strlen(testCmd));
         vTaskDelay(pdMS_TO_TICKS(200));
 
         int len = uart_read_bytes(port, data, BUF_SIZE, pdMS_TO_TICKS(100));
         if (len > 0) {
             data[len] = '\0';
 
             if (strstr((char *)data, "OK") != NULL) {
                 #ifdef DEBUG
                 ESP_LOGI(TAG, "Detected baud rate: %d", baudRates[i]);
                 #endif
 
                 // Now configure the module
                 init_uart_with_params(port, baudRates[i]);
 
                 // Set baud rate (JDY-31 & HC-05 compatible)
                 char baud_cmd[32];
                 snprintf(baud_cmd, sizeof(baud_cmd), "AT+BAUD%d\r\n", (target_baud_rate / 1200) - 3);
                 send_at_command(port, baud_cmd, "OK");
 
                 // Set Bluetooth name
                 char name_cmd[32];
                 snprintf(name_cmd, sizeof(name_cmd), "AT+NAME%s\r\n", device_name);
                 send_at_command(port,name_cmd, "OK");
 
                 // Set PIN
                 char pin_cmd[32];
                 snprintf(pin_cmd, sizeof(pin_cmd), "AT+PIN%s\r\n", pin);
                 send_at_command(port, pin_cmd, "OK");
 
                 // Set role (Slave/Master)
                 char role_cmd[16];
                 snprintf(role_cmd, sizeof(role_cmd), "AT+ROLE%d\r\n", mode);
                 send_at_command(port, role_cmd, "OK");
 
                 // Set auto-connect
                 char auto_cmd[16];
                 snprintf(auto_cmd, sizeof(auto_cmd), "AT+AUTO%d\r\n", auto_connect);
                 send_at_command(port, auto_cmd, "OK");
 
                 #ifdef DEBUG
                 ESP_LOGI(TAG, "Bluetooth module configured successfully!");
                 #endif
 
                 // Reinitialize UART with new baud rate
                 init_uart_with_params(port, target_baud_rate);
                 exit_AT_mode(enable_pin);
                 return target_baud_rate;
             }
         }
     }

 exit_AT_mode(enable_pin);
     #ifdef DEBUG
     ESP_LOGE(TAG, "Failed to detect baud rate!");
     #endif
     return -1;
 }
     

 

 /**
  * @brief UART event handling task
  * @param arg UART port number
  */
 static void uart_event_task(void *arg) {
     uart_event_t event;
     uart_packet_t packet;
     int uart_num = (int)arg;
     QueueHandle_t event_queue = (uart_num == UART1_NUM) ? uart1_event_queue : uart2_event_queue;
     QueueHandle_t data_queue = (uart_num == UART1_NUM) ? uart1_queue : uart2_queue;
     
     while (1) {
         if (xQueueReceive(event_queue, &event, portMAX_DELAY)) {
             if (event.type == UART_DATA && event.size > 0) {
                 packet.len = uart_read_bytes(uart_num, packet.data, event.size, pdMS_TO_TICKS(10));
                 if (packet.len > 0) {
                     xQueueSend(data_queue, &packet, portMAX_DELAY);
                 }
             }
         }
     }
 }
 
 /**
  * @brief Task to forward data from UART1 to UART2
  */
 void uart_forward_task_uart1_to_uart2(void *arg) {
     uart_packet_t packet;
     while (1) {
         if (xQueueReceive(uart1_queue, &packet, portMAX_DELAY)) {
             if (packet.len > 0) {
                 #if DEBUG
                 ESP_LOGI(TAG, "Forwarding %d bytes from UART1 to UART2", packet.len);
                 #endif
                 uart_write_bytes(UART2_NUM, (const char *)packet.data, packet.len);
             }
         }
     }
 }
 
 /**
  * @brief Task to forward data from UART2 to UART1
  */
 void uart_forward_task_uart2_to_uart1(void *arg) {
     uart_packet_t packet;
     while (1) {
         if (xQueueReceive(uart2_queue, &packet, portMAX_DELAY)) {
             if (packet.len > 0) {
                 #if DEBUG
                 ESP_LOGI(TAG, "Forwarding %d bytes from UART2 to UART1", packet.len);
                 #endif
                 uart_write_bytes(UART1_NUM, (const char *)packet.data, packet.len);
             }
         }
     }
 }
 


 /**
  * @brief Main application entry point
  */
 void app_main() {
     ESP_LOGI(TAG, "Initializing UART bridge...");

     int configuredBaud = detect_and_configure_bt_module(UART2_NUM, UART2_BT_EN_PIN, "ESP32_BT", 115200, "1234", 0, 1);
    
     if (configuredBaud > 0) {
         #ifdef DEBUG
         ESP_LOGI(TAG, "Bluetooth module successfully configured at %d baud!", configuredBaud);
         #endif
     } else {
         #ifdef DEBUG
         ESP_LOGE(TAG, "Failed to configure Bluetooth module.");
         #endif
     }
 #ifdef DEBUG
     xTaskCreate(keep_alive_task, "keep_alive_task", 2048, NULL, 5, NULL);
 #endif
     
     uart1_queue = xQueueCreate(10, sizeof(uart_packet_t));
     uart2_queue = xQueueCreate(10, sizeof(uart_packet_t));
     uart1_event_queue = xQueueCreate(10, sizeof(uart_event_t));
     uart2_event_queue = xQueueCreate(10, sizeof(uart_event_t));
     
     uart_init();
     
     xTaskCreate(uart_event_task, "uart1_event_task", 4096, (void *)UART1_NUM, 10, NULL);
     xTaskCreate(uart_event_task, "uart2_event_task", 4096, (void *)UART2_NUM, 10, NULL);
     xTaskCreate(uart_forward_task_uart1_to_uart2, "uart1_to_uart2_task", 4096, NULL, 10, NULL);
     xTaskCreate(uart_forward_task_uart2_to_uart1, "uart2_to_uart1_task", 4096, NULL, 10, NULL);
 }

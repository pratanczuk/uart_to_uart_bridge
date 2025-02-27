/**
 * @file esp32_dual_uart.c
 * @brief ESP32 UART Bridge for bidirectional communication between two UART ports.
 * 
 * This implementation enables seamless communication between two UART ports,
 * using FreeRTOS tasks and queues for efficient data transfer. The design is fully event-driven,
 * reducing CPU usage by utilizing the ESP-IDF UART driver’s event queue mechanism.
 * 
 * ## Performance Calculations
 * - **Baud Rate:** 115200 bps
 * - **Packet Size:** 8 bytes
 * - **Bits per packet:** (8 data + 1 start + 1 stop) * 8 = 80 bits per packet
 * - **Transmission Time per Packet:** 80 bits / 115200 bps ≈ 0.6944 ms
 * - **Packets per second (ideal case):** ≈ 1440 packets/sec
 * - **Max Theoretical Bandwidth:** 1440 packets/sec * 8 bytes = 11520 bytes/sec (~11.52 KB/s)
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

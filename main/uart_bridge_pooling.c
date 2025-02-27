#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include <string.h>

#define UART1_TX_PIN 1
#define UART1_RX_PIN 2
#define UART1_RTS_PIN 4
#define UART1_CTS_PIN 3

#define UART2_TX_PIN 5
#define UART2_RX_PIN 6

#define UART1_NUM UART_NUM_1
#define UART2_NUM UART_NUM_2

#define BUF_SIZE (128)

static const char *TAG = "UART_BRIDGE";

/**
 * @brief Periodically logs a keep-alive message.
 * @param arg Unused parameter.
 */
void keep_alive_task(void *arg) {
    while (1) {
        ESP_LOGI(TAG, "Keep-alive message");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static QueueHandle_t uart1_queue;
static QueueHandle_t uart2_queue;
static SemaphoreHandle_t uart1_semaphore;
static SemaphoreHandle_t uart2_semaphore;

/**
 * @brief Structure to store UART packets.
 */
typedef struct {
    uint8_t data[BUF_SIZE]; // Data buffer
    int len; // Data length
} uart_packet_t;

/**
 * @brief Task to forward data from UART1 to UART2.
 * @param arg Unused parameter.
 */
void uart_forward_task_uart1_to_uart2(void *arg) {
    uart_packet_t packet;
    while (1) {
        if (xQueueReceive(uart1_queue, &packet, portMAX_DELAY)) {
            if (packet.len > 0) {
                xSemaphoreTake(uart2_semaphore, portMAX_DELAY);
                ESP_LOGI(TAG, "Forwarding %d bytes from UART1 to UART2", packet.len);
                uart_write_bytes(UART2_NUM, (const char *)packet.data, packet.len);
                xSemaphoreGive(uart2_semaphore);
            }
        }
    }
}

/**
 * @brief Task to forward data from UART2 to UART1.
 * @param arg Unused parameter.
 */
void uart_forward_task_uart2_to_uart1(void *arg) {
    uart_packet_t packet;
    while (1) {
        if (xQueueReceive(uart2_queue, &packet, portMAX_DELAY)) {
            if (packet.len > 0) {
                xSemaphoreTake(uart1_semaphore, portMAX_DELAY);
                ESP_LOGI(TAG, "Forwarding %d bytes from UART2 to UART1", packet.len);
                uart_write_bytes(UART1_NUM, (const char *)packet.data, packet.len);
                xSemaphoreGive(uart1_semaphore);
            }
        }
    }
}

/**
 * @brief Task to read data from UART1 and queue it for processing.
 * @param arg Unused parameter.
 */
void uart_rx_task_uart1(void *arg) {
    uart_packet_t packet;
    while (1) {
        xSemaphoreTake(uart1_semaphore, portMAX_DELAY);
        packet.len = uart_read_bytes(UART1_NUM, packet.data, BUF_SIZE, 10 / portTICK_PERIOD_MS);
        xSemaphoreGive(uart1_semaphore);
        if (packet.len > 0) {
            xQueueSend(uart1_queue, &packet, portMAX_DELAY);
        }
    }
}

/**
 * @brief Task to read data from UART2 and queue it for processing.
 * @param arg Unused parameter.
 */
void uart_rx_task_uart2(void *arg) {
    uart_packet_t packet;
    while (1) {
        xSemaphoreTake(uart2_semaphore, portMAX_DELAY);
        packet.len = uart_read_bytes(UART2_NUM, packet.data, BUF_SIZE, 10 / portTICK_PERIOD_MS);
        xSemaphoreGive(uart2_semaphore);
        if (packet.len > 0) {
            xQueueSend(uart2_queue, &packet, portMAX_DELAY);
        }
    }
}

/**
 * @brief Initializes UART configuration.
 */
void uart_init() {
    ESP_LOGI(TAG, "Initializing UART...");

    uart_driver_install(UART1_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    uart_driver_install(UART2_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    
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
 * @brief Main application entry point.
 */
void app_main() {
    ESP_LOGI(TAG, "Initializing UART bridge...");

#ifdef DEBUG
    xTaskCreate(keep_alive_task, "keep_alive_task", 2048, NULL, 5, NULL);
#endif

    uart_init();

    uart1_queue = xQueueCreate(10, BUF_SIZE);
    uart2_queue = xQueueCreate(10, BUF_SIZE);
    uart1_semaphore = xSemaphoreCreateBinary();
    uart2_semaphore = xSemaphoreCreateBinary();
    
    xSemaphoreGive(uart1_semaphore);
    xSemaphoreGive(uart2_semaphore);

    xTaskCreate(uart_rx_task_uart1, "uart1_rx_task", 4096, NULL, 10, NULL);
    xTaskCreate(uart_rx_task_uart2, "uart2_rx_task", 4096, NULL, 10, NULL);
    xTaskCreate(uart_forward_task_uart1_to_uart2, "uart1_to_uart2_task", 4096, NULL, 10, NULL);
    xTaskCreate(uart_forward_task_uart2_to_uart1, "uart2_to_uart1_task", 4096, NULL, 10, NULL);
}

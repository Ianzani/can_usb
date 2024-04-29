#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const uart_port_t uart_num = UART_NUM_0;
static QueueHandle_t uart0_queue;

static void uart_event(void *pvParameters)
{
    uart_event_t event;
    uint8_t buffer[256] = {};

    while (1) {
        
        if (xQueueReceive(uart0_queue, (void*) &event, 10 / portTICK_PERIOD_MS)) {

            switch (event.type) {
                case UART_DATA:
                    uart_read_bytes(uart_num, buffer, event.size, 10 / portTICK_PERIOD_MS);
                    uart_write_bytes(uart_num, (const char*) buffer, event.size);
                break;

                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                case UART_BREAK:
                case UART_PARITY_ERR:
                case UART_PATTERN_DET:
                
                default:
                break;
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    free(buffer);
}

void app_main(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 256, 0, 20, &uart0_queue, 0));

    xTaskCreate(uart_event, "uart_event_task", 2048, NULL, 12, NULL);
}

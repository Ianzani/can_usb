#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/twai.h"


typedef struct {
    uint8_t data[8];
    uint8_t len;
} uart_can_buffer_t;


QueueHandle_t queue_uart_can = NULL;


static const uart_port_t uart_num = UART_NUM_0;
static QueueHandle_t uart0_queue = NULL;

/* -------------- Tasks ----------------- */
static void uart_event(void *pvParameters);
static void can_send(void *pvParameters);
/* ------------ Private Functions ------------- */
static void init_uart(const uart_port_t uart_num);
static void init_can(void);
static void can_send_message(const uart_can_buffer_t *buffer);


void app_main(void)
{
    init_uart(uart_num);
    init_can();

    queue_uart_can = xQueueCreate(10, sizeof(uart_can_buffer_t));

    xTaskCreate(uart_event, "uart_event_task", 2048, NULL, 12, NULL);
    xTaskCreate(can_send, "can_send", 1024, NULL, 12, NULL);
}

static void uart_event(void *pvParameters)
{
    uart_event_t event;
    uart_can_buffer_t buffer = {};

    while (1) {
        
        if (xQueueReceive(uart0_queue, (void*) &event, 10 / portTICK_PERIOD_MS)) {

            switch (event.type) {
                case UART_DATA:
                    buffer.len = uart_read_bytes(uart_num, buffer.data, event.size, 10 / portTICK_PERIOD_MS);
                    xQueueSend(queue_uart_can, &buffer, portMAX_DELAY);
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
}

static void can_send(void *pvParameters) 
{
    uart_can_buffer_t buffer = {};

    while (1) {

        if (xQueueReceive(queue_uart_can, &buffer, 1 / portTICK_PERIOD_MS) == pdTRUE) {

            uart_write_bytes(uart_num, (const char*) buffer.data, buffer.len);
            can_send_message(&buffer);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void init_uart(const uart_port_t uart_port)
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
    ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_port, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_port, 256, 0, 20, &uart0_queue, 0));
}

static void init_can(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_18, TWAI_MODE_NO_ACK);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
}

static void can_send_message(const uart_can_buffer_t *buffer)
{
    twai_message_t tx_msg = {
        .extd = 0,              // Standard Format message (11-bit ID)
        .rtr = 0,               // Send a data frame
        .ss = 0,                // Not single shot
        .self = 1,              // Message is a self reception request (loopback)
        .dlc_non_comp = 0,      // DLC is less than 8
        .identifier = 0x555,
        .data_length_code = buffer->len,
        .data = {}
    };

    uint8_t i;
    for(i = 0; i < buffer->len; i++) {
        tx_msg.data[i] = buffer->data[i];
    }

    twai_transmit(&tx_msg, portMAX_DELAY);
}
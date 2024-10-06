#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/twai.h"


#define PROTOCOL_HEADER_BYTE                (0x4eU)
#define PROTOCOL_FOOTER_BYTE                (0x9aU)
#define PROTOCOL_MAX_PAYLOAD_LEN            (8U)

#define UART_NUM                            (UART_NUM_0)


typedef struct {
    uint8_t data[PROTOCOL_MAX_PAYLOAD_LEN];
    uint8_t len;
} uart_can_buffer_t;

typedef enum {
    CHECK_HEADER,
    CHECK_DATA,
    CHECK_FOOTER
} decode_states_e;


static QueueHandle_t queue_uart_to_can = NULL;
static QueueHandle_t uart0_queue = NULL;


/* -------------- Tasks ----------------- */
static void uart_event(void *pvParameters);
static void can_send(void *pvParameters);
/* ------------ Private Functions ------------- */
static void init_uart(const uart_port_t uart_num);
static void init_can(void);
static void can_send_message(const uart_can_buffer_t *buffer);
static void decode_and_send_uart_tx(uint8_t * data, uint8_t len);


void app_main(void)
{
    init_uart(UART_NUM);
    init_can();

    queue_uart_to_can = xQueueCreate(10, sizeof(uart_can_buffer_t));

    xTaskCreate(uart_event, "uart_event_task", 2048, NULL, 12, NULL);
    xTaskCreate(can_send, "can_send", 2048, NULL, 12, NULL);
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

static void uart_event(void *pvParameters)
{
    uart_event_t event = {};
    uart_can_buffer_t buffer_rx = {};

    while (1) {
        
        if (xQueueReceive(uart0_queue, (void*) &event, 10 / portTICK_PERIOD_MS)) {
            switch (event.type) {

                case UART_DATA:
                    buffer_rx.len = uart_read_bytes(UART_NUM, buffer_rx.data, event.size, 10 / portTICK_PERIOD_MS);
                    decode_and_send_uart_tx(buffer_rx.data, buffer_rx.len);
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

static void decode_and_send_uart_tx(uint8_t * data, uint8_t len)
{
    static uint8_t msg_buffer[2] = {};
    static uint8_t msg_index = 0;
    static decode_states_e states = CHECK_HEADER;

    for (uint8_t i = 0; i < len; i++) {

        switch (states) {
            case CHECK_HEADER:
                if (data[i] == PROTOCOL_HEADER_BYTE) {
                    states = CHECK_DATA;
                }
            break;

            case CHECK_DATA:
                msg_buffer[msg_index++] = data[i];

                if (msg_index > 1) {
                    msg_index = 0;
                    states = CHECK_FOOTER;
                }
            break;

            case CHECK_FOOTER:
                if (data[i] == PROTOCOL_FOOTER_BYTE) {
                    uart_can_buffer_t buffer = {
                        .data = {msg_buffer[0], msg_buffer[1]},
                        .len = sizeof(msg_buffer)
                    };

                    xQueueSend(queue_uart_to_can, &buffer, portMAX_DELAY);
                }
                states = CHECK_HEADER;
            break;
        }

    }
}

static void can_send(void *pvParameters) 
{
    uart_can_buffer_t msg = {};

    while (1) {

        if (xQueueReceive(queue_uart_to_can, &msg, 1 / portTICK_PERIOD_MS) == pdTRUE) {
            can_send_message(&msg);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void can_send_message(const uart_can_buffer_t *msg)
{
    twai_message_t tx_msg = {
        .extd = 0,              
        .rtr = 0,               
        .ss = 0,                
        .self = 0,              
        .dlc_non_comp = 0,      
        .identifier = 0x555,
        .data_length_code = msg->len,
        .data = {}
    };

    for(uint8_t i = 0; i < msg->len; i++) {
        tx_msg.data[i] = msg->data[i];
    }

    twai_transmit(&tx_msg, portMAX_DELAY);
}
/* uart_echo_example_main.c  -- Modified for A/B LED control + case 'I' handshake */

#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_timer.h"
// #include "esp_rom/esp_rom.h"


#define ECHO_TEST_TXD        (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD        (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_UART_PORT_NUM   (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE  (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE (CONFIG_EXAMPLE_TASK_STACK_SIZE)
#define BUF_SIZE             (1024)

#define ULTRASONIC_TRIG_PIN GPIO_NUM_4    // Feather pin “4” / A5
#define ULTRASONIC_ECHO_PIN GPIO_NUM_15   // Feather pin “15”/ A8

static void echo_task(void *arg) {
    // 1) Configure and install UART driver
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 2) Allocate buffer for incoming data
    uint8_t *data = malloc(BUF_SIZE);
    if (!data) {
        ESP_LOGE("echo_task", "Buffer allocation failed");
        vTaskDelete(NULL);
        return;
    }

    // 3) Send handshake so LabVIEW can detect connection
    uart_write_bytes(ECHO_UART_PORT_NUM, "Commands", strlen("Commands"));

    // 4) Main loop: read → interpret
    while (1) {
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';  // null-terminate

            switch (data[0]) {
                case 'I':
                    // Connection initialization: LED on + respond
                    gpio_set_level(GPIO_NUM_13, 1);
                    uart_write_bytes(ECHO_UART_PORT_NUM, "ESP32\r\n", strlen("ESP32\r\n"));
                    break;
                case 'A':
                    // Turn LED13 ON
                    gpio_set_level(GPIO_NUM_13, 1);
                    uart_write_bytes(ECHO_UART_PORT_NUM, "LED ON\r\n", strlen("LED ON\r\n"));
                    break;
                case 'B':
                    // Turn LED13 OFF
                    gpio_set_level(GPIO_NUM_13, 0);
                    uart_write_bytes(ECHO_UART_PORT_NUM, "LED OFF\r\n", strlen("LED OFF\r\n"));
                    break;
                default:
                    // ignore other commands
                    break;
            }
        }
    }
}

// ——— Initialization ———
static void ultrasonic_init(void) {
    // TRIG = output, start low
    gpio_reset_pin(ULTRASONIC_TRIG_PIN);
    gpio_set_direction(ULTRASONIC_TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);
    
    gpio_reset_pin(ULTRASONIC_ECHO_PIN);
    gpio_set_direction(ULTRASONIC_ECHO_PIN, GPIO_MODE_INPUT);
}

// ——— Measure once, return distance in cm ———
static float ultrasonic_read_cm(void) {
    // 1) Fire a 10 µs pulse
    gpio_set_level(ULTRASONIC_TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);

    // 2) Wait for echo to go high (with timeout)
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(ULTRASONIC_ECHO_PIN) == 0) {
        if (esp_timer_get_time() - start > 20000) {
            // no echo within 20 ms → out of range
            return -1.0f;
        }
    }

    // 3) Timestamp rising edge
    int64_t t1 = esp_timer_get_time();

    // 4) Wait for echo to go low (with timeout)
    while (gpio_get_level(ULTRASONIC_ECHO_PIN) == 1) {
        if (esp_timer_get_time() - t1 > 20000) {
            // stuck high → error
            return -2.0f;
        }
    }
    int64_t t2 = esp_timer_get_time();

    // 5) Compute distance
    int64_t dt = t2 - t1;  // round-trip time in µs
    float distance = (dt / 2.0f) * 0.0343f;
    return distance;
}

// ——— Example task: print every 500 ms ———
static void ultrasonic_task(void *arg) {
    while (1) {
        float d = ultrasonic_read_cm();
        if (d < 0) {
            printf("Ultrasonic: no reading (err=%.1f)\n", d);
        } else {
            printf("Ultrasonic: %.1f cm\n", d);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void) {

    // 1) Init ultrasonic hardware
    ultrasonic_init();
    // Initialize GPIO13 for on-board LED
    gpio_reset_pin(GPIO_NUM_13);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_13, 0);  // LED starts OFF

    // Launch the UART echo / command task
    // xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);

    // 3) Launch the ultrasonic measurement task
    xTaskCreate(ultrasonic_task, "ultra", 2048, NULL, 5, NULL);
}

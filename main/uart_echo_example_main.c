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

// UART CONFIG, DONT CHANGE
#define ECHO_TEST_TXD        (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD        (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_UART_PORT_NUM   (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE  (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE (CONFIG_EXAMPLE_TASK_STACK_SIZE)
#define BUF_SIZE             (1024)

// LED heartbeat
static uint8_t  s_led_state    = 1;
static uint32_t flash_period   = 1000;
static uint32_t flash_period_dec = 100;
TaskHandle_t    myTaskHandle   = NULL;

// Servo PWM
#define SERVO_GPIO1         12
#define SERVO_GPIO2         13
#define SERVO_PWM_FREQUENCY 50
#define SERVO_PWM_TIMER     LEDC_TIMER_0
#define SERVO_PWM_CHANNEL_1 LEDC_CHANNEL_0
#define SERVO_PWM_CHANNEL_2 LEDC_CHANNEL_1
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500
#define SERVO_MAX_DEGREE         180

// Motor GPIOs
#define MOTOR_A_IN1_GPIO  GPIO_NUM_7
#define MOTOR_A_IN2_GPIO  GPIO_NUM_8
#define MOTOR_B_IN3_GPIO  GPIO_NUM_26
#define MOTOR_B_IN4_GPIO  GPIO_NUM_25

#define ULTRASONIC_TRIG_PIN GPIO_NUM_4    // Feather pin “4” / A5
#define ULTRASONIC_ECHO_PIN GPIO_NUM_15   // Feather pin “15”/ A8

//——— INITIALIZATION FUNCTIONS ———

static void init_led(void) {
    gpio_reset_pin(GPIO_NUM_13);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_13, s_led_state);
}

static void init_servo(void) {
    // timer
    ledc_timer_config_t timer_cfg = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = SERVO_PWM_TIMER,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = SERVO_PWM_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);
    // channel 1
    ledc_channel_config_t ch1 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = SERVO_PWM_CHANNEL_1,
        .timer_sel  = SERVO_PWM_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = SERVO_GPIO1,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch1);
    // channel 2
    ledc_channel_config_t ch2 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = SERVO_PWM_CHANNEL_2,
        .timer_sel  = SERVO_PWM_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = SERVO_GPIO2,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch2);
}

static void init_motor(void) {
    gpio_set_direction(MOTOR_A_IN1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_A_IN2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_IN3_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_IN4_GPIO, GPIO_MODE_OUTPUT);
    // ensure stopped
    gpio_set_level(MOTOR_A_IN1_GPIO, 0);
    gpio_set_level(MOTOR_A_IN2_GPIO, 0);
    gpio_set_level(MOTOR_B_IN3_GPIO, 0);
    gpio_set_level(MOTOR_B_IN4_GPIO, 0);
}

static void ultrasonic_init(void) {
    // TRIG = output, start low
    gpio_reset_pin(ULTRASONIC_TRIG_PIN);
    gpio_set_direction(ULTRASONIC_TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);
    
    gpio_reset_pin(ULTRASONIC_ECHO_PIN);
    gpio_set_direction(ULTRASONIC_ECHO_PIN, GPIO_MODE_INPUT);
}


//——— SERVO HELPERS ———

static uint32_t servo_angle_to_duty_us(int angle) {
    return SERVO_MIN_PULSEWIDTH_US +
        ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle) / SERVO_MAX_DEGREE;
}

static void set_servo_angle(int channel, int angle) {
    uint32_t duty_us = servo_angle_to_duty_us(angle);
    uint32_t duty = (duty_us * (1 << 13)) / (1000000 / SERVO_PWM_FREQUENCY);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

static void move_servo_smooth(int channel, int start_angle, int end_angle, int delay_ms) {
    int step = (end_angle > start_angle) ? 1 : -1;
    for (int a = start_angle; a != end_angle; a += step) {
        set_servo_angle(channel, a);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    set_servo_angle(channel, end_angle);
}


//——— MOTOR PRIMITIVES ———

static void motor_forward(void) {
    gpio_set_level(MOTOR_A_IN1_GPIO, 1);
    gpio_set_level(MOTOR_A_IN2_GPIO, 0);
    gpio_set_level(MOTOR_B_IN3_GPIO, 1);
    gpio_set_level(MOTOR_B_IN4_GPIO, 0);
}

static void motor_reverse(void) {
    gpio_set_level(MOTOR_A_IN1_GPIO, 0);
    gpio_set_level(MOTOR_A_IN2_GPIO, 1);
    gpio_set_level(MOTOR_B_IN3_GPIO, 0);
    gpio_set_level(MOTOR_B_IN4_GPIO, 1);
}

static void motor_left(void) {
    gpio_set_level(MOTOR_A_IN1_GPIO, 0);
    gpio_set_level(MOTOR_A_IN2_GPIO, 0);
    gpio_set_level(MOTOR_B_IN3_GPIO, 1);
    gpio_set_level(MOTOR_B_IN4_GPIO, 0);
}

static void motor_right(void) {
    gpio_set_level(MOTOR_A_IN1_GPIO, 1);
    gpio_set_level(MOTOR_A_IN2_GPIO, 0);
    gpio_set_level(MOTOR_B_IN3_GPIO, 0);
    gpio_set_level(MOTOR_B_IN4_GPIO, 0);
}


//——— ULTRASONIC HELPERS ———

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


//——— Blink & random-servo demo ———

static void blink_led(void) {
    gpio_set_level(GPIO_NUM_13, s_led_state);
}

static void blink_task(void *arg) {
    int angle1 = 90, angle2 = 90;
    while (1) {
        s_led_state = !s_led_state;
        blink_led();

        // random new angles
        int na1 = esp_random() % 181;
        int na2 = 70 + (esp_random() % 41);
        move_servo_smooth(SERVO_PWM_CHANNEL_1, angle1, na1, 10);
        move_servo_smooth(SERVO_PWM_CHANNEL_2, angle2, na2, 10);
        angle1 = na1;  angle2 = na2;

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


//——— TASKS ———

// ——— Example task: print every 500 ms ———
static void ultrasonic_task(void *arg) {
    char buffer[32];  // Buffer for formatting the UART message
    while (1) {
        float d = ultrasonic_read_cm();
        if (d < 0) {
            snprintf(buffer, sizeof(buffer), "U:ERR:%.1f\r\n", d);
        } else {
            snprintf(buffer, sizeof(buffer), "U:%.1f\r\n", d);
        }
        uart_write_bytes(ECHO_UART_PORT_NUM, buffer, strlen(buffer));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void echo_task(void *arg) { // TODO: UPDATE TO PROPERLY SEND NEW COMMANDS W LABVIEW
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

                case 'R':
                    //Start ultrasonic task?
                    xTaskCreate(ultrasonic_task, "ultra", 2048, NULL, 5, NULL);

                default:
                    // ignore other commands
                    break;
            }
        }
    }
}


void app_main(void) {

    // 1) Init all hardware
    init_led();
    init_servo();
    init_motor();
    ultrasonic_init();

    // Launch the UART echo / command task
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);

    // 3) Launch the ultrasonic measurement task
    // xTaskCreate(ultrasonic_task, "ultra", 2048, NULL, 5, NULL);
}

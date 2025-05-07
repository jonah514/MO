/* uart_echo_example_main.c  -- Modified for A/B LED control + case 'I' handshake */

#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "driver/adc.h"
#include <math.h>
#include "lcd_1602.h" // We'll create this header file

// UART CONFIG, DONT CHANGE
#define UART_TX_PIN         (CONFIG_EXAMPLE_UART_TXD)
#define UART_RX_PIN         (CONFIG_EXAMPLE_UART_RXD)
#define UART_PORT_NUM       (CONFIG_EXAMPLE_UART_PORT_NUM)
#define UART_BAUD_RATE      (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define UART_BUF_SIZE       (1024)
#define UART_TASK_STACK     (CONFIG_EXAMPLE_TASK_STACK_SIZE)

//— pins & PWM for servo ————————————————————————————
#define SERVO_GPIO1         GPIO_NUM_4 //tilt
#define SERVO_GPIO2         GPIO_NUM_5 //pan
#define SERVO_GPIO3         GPIO_NUM_13  //arm
#define SERVO_FREQ_HZ       50
#define SERVO_TIMER         LEDC_TIMER_0
#define SERVO_CHANNEL_1     LEDC_CHANNEL_0
#define SERVO_CHANNEL_2     LEDC_CHANNEL_1
#define SERVO_CHANNEL_3     LEDC_CHANNEL_2
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2500
#define SERVO_MAX_DEG       180
#define SERVO_ARM_REST      90  // Default rest position (adjust after testing)
#define SERVO_ARM_DELTA     5  // Degrees to move up/down (adjust after testing)

//— motor GPIOs ——————————————————————————————————————
#define MOTOR_A_IN1         GPIO_NUM_7
#define MOTOR_A_IN2         GPIO_NUM_8
#define MOTOR_B_IN3         GPIO_NUM_26
#define MOTOR_B_IN4         GPIO_NUM_25

//— ultrasonic pins ————————————————————————————————
#define ULTRA_TRIG_PIN      GPIO_NUM_21
#define ULTRA_ECHO_PIN      GPIO_NUM_36

//— microphone pin ————————————————————————————————
#define MIC_PIN             GPIO_NUM_34
#define MIC_THRESHOLD       2000    // ADC raw value threshold (adjust after testing)
#define MIC_CHECK_DELAY_MS  50      // How often to check microphone

// LCD pin definitions
#define LCD_RS_PIN      GPIO_NUM_14
#define LCD_E_PIN       GPIO_NUM_32
#define LCD_D4_PIN      GPIO_NUM_15
#define LCD_D5_PIN      GPIO_NUM_33
#define LCD_D6_PIN      GPIO_NUM_27
#define LCD_D7_PIN      GPIO_NUM_12

//— modes ————————————————————————————————————————
typedef enum {
    MODE_REMOTE = 0,
    MODE_RANDOM = 1,
    MODE_VOICE  = 2
} operation_mode_t;
static volatile operation_mode_t current_mode = MODE_REMOTE;

//— shared distance (cm) updated by timer callback ———————————
static volatile float g_ultrasonic_cm = -1.0f;

// Count how many times MO has bumped into something
static volatile uint32_t obstacle_count = 0;

//— task handles —————————————————————————————————————
static TaskHandle_t randomTaskHandle = NULL;
static TaskHandle_t voiceTaskHandle  = NULL;

// Add a global variable to track the current arm position
static int current_arm_position = SERVO_ARM_REST;

// Define custom characters for square eyes
const uint8_t FULL_BLOCK[8] = {
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111
};

// For blinking, we'll use a completely empty character
const uint8_t EMPTY_BLOCK[8] = {
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000
};

// Middle line for half-closed eyes
const uint8_t MIDDLE_LINE[8] = {
    0b00000,
    0b00000,
    0b00000,
    0b11111,
    0b11111,
    0b00000,
    0b00000,
    0b00000
};

// Increase task stack sizes to prevent stack overflow
#define STANDARD_TASK_STACK_SIZE 4096  // Increased from default 2048

// Function to display normal square eyes (5x2 blocks per eye, centered)
static void display_normal_eyes(void) {
    lcd_clear();
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // First row - both eyes
    // We have 16 columns total, with 5 blocks per eye = 10 blocks
    // That leaves 6 spaces to distribute: 1 at start, 4 between eyes, 1 at end
    
    // Left eye (first row)
    lcd_set_cursor(1, 0);
    for (int i = 0; i < 5; i++) {
        lcd_write_char(0);  // Full block
    }
    
    // Space between eyes
    lcd_write_char(' ');
    lcd_write_char(' ');
    lcd_write_char(' ');
    lcd_write_char(' ');
    
    // Right eye (first row)
    for (int i = 0; i < 5; i++) {
        lcd_write_char(0);  // Full block
    }
    
    // Second row - both eyes
    // Left eye (second row)
    lcd_set_cursor(1, 1);
    for (int i = 0; i < 5; i++) {
        lcd_write_char(0);  // Full block
    }
    
    // Space between eyes
    lcd_write_char(' ');
    lcd_write_char(' ');
    lcd_write_char(' ');
    lcd_write_char(' ');
    
    // Right eye (second row)
    for (int i = 0; i < 5; i++) {
        lcd_write_char(0);  // Full block
    }
}

// Function to display half-closed eyes (blinking transition)
static void display_half_closed_eyes(void) {
    lcd_clear();
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // First row - both eyes
    // Left eye (first row)
    lcd_set_cursor(1, 0);
    for (int i = 0; i < 5; i++) {
        lcd_write_char(0);  // Full block
    }
    
    // Space between eyes
    lcd_write_char(' ');
    lcd_write_char(' ');
    lcd_write_char(' ');
    lcd_write_char(' ');
    
    // Right eye (first row)
    for (int i = 0; i < 5; i++) {
        lcd_write_char(0);  // Full block
    }
    
    // Second row - both eyes with middle line
    // Left eye (second row)
    lcd_set_cursor(1, 1);
    for (int i = 0; i < 5; i++) {
        lcd_write_char(2);  // Middle line
    }
    
    // Space between eyes
    lcd_write_char(' ');
    lcd_write_char(' ');
    lcd_write_char(' ');
    lcd_write_char(' ');
    
    // Right eye (second row)
    for (int i = 0; i < 5; i++) {
        lcd_write_char(2);  // Middle line
    }
}

// Function to display closed eyes (fully blinked)
static void display_closed_eyes(void) {
    lcd_clear();
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // First row - both eyes
    // Left eye (first row)
    lcd_set_cursor(1, 0);
    for (int i = 0; i < 5; i++) {
        lcd_write_char(1);  // Empty block
    }
    
    // Space between eyes
    lcd_write_char(' ');
    lcd_write_char(' ');
    lcd_write_char(' ');
    lcd_write_char(' ');
    
    // Right eye (first row)
    for (int i = 0; i < 5; i++) {
        lcd_write_char(1);  // Empty block
    }
    
    // Second row - both eyes
    // Left eye (second row)
    lcd_set_cursor(1, 1);
    for (int i = 0; i < 5; i++) {
        lcd_write_char(2);  // Middle line
    }
    
    // Space between eyes
    lcd_write_char(' ');
    lcd_write_char(' ');
    lcd_write_char(' ');
    lcd_write_char(' ');
    
    // Right eye (second row)
    for (int i = 0; i < 5; i++) {
        lcd_write_char(2);  // Middle line
    }
}

// Function to perform a natural blink animation
static void blink_eyes(void) {
    // Half-closed eyes
    display_half_closed_eyes();
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Fully closed eyes
    display_closed_eyes();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Half-closed eyes again
    display_half_closed_eyes();
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Back to normal
    display_normal_eyes();
}

static void display_angry_eyes(void) {
    display_normal_eyes();
}

// Function to initialize the LCD
static void init_lcd(void) {
    // Add a delay before initialization to ensure power is stable
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Initialize LCD in 4-bit mode
    lcd_init(LCD_RS_PIN, LCD_E_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);
    
    // Add a longer delay after initialization
    vTaskDelay(pdMS_TO_TICKS(300));
    
    // Create custom characters with longer delays between operations
    lcd_create_char(0, FULL_BLOCK);   // Full block for normal eyes
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_create_char(1, EMPTY_BLOCK);  // Empty block for closed eyes
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_create_char(2, MIDDLE_LINE);  // Middle line for half-closed eyes
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Clear display and home cursor
    lcd_clear();
    vTaskDelay(pdMS_TO_TICKS(50));
    
    printf("LCD initialization complete\n");
}

//——— SERVO HELPERS ———

static uint32_t angle_to_duty(int angle) {
    return SERVO_MIN_US +
         ((SERVO_MAX_US - SERVO_MIN_US) * angle) / SERVO_MAX_DEG;
}

static void set_servo(int channel, int angle) {
    uint32_t us = angle_to_duty(angle);
    uint32_t duty = (us * (1 << 13)) / (1000000 / SERVO_FREQ_HZ);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

//———————————————————————————————————————————————
//— Initialization routines
//———————————————————————————————————————————————

static void init_led(void) {
    gpio_reset_pin(GPIO_NUM_13);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_13, 0);
}

static void init_servo(void) {
    // timer
    ledc_timer_config_t t = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = SERVO_TIMER,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = SERVO_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t);

    for (int ch = SERVO_CHANNEL_1; ch <= SERVO_CHANNEL_3; ch++) {
        ledc_channel_config_t c = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = ch,
            .timer_sel  = SERVO_TIMER,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = (ch == SERVO_CHANNEL_1 ? SERVO_GPIO1 :
                           (ch == SERVO_CHANNEL_2 ? SERVO_GPIO2 : SERVO_GPIO3)),
            .duty       = 0,
            .hpoint     = 0
        };
        ledc_channel_config(&c);
    }

    // Initialize both servos to center position (90 degrees)
    set_servo(SERVO_CHANNEL_1, 90);  // Center tilt
    set_servo(SERVO_CHANNEL_2, 90);  // Center pan
    set_servo(SERVO_CHANNEL_3, SERVO_ARM_REST);  // Default rest position
    current_arm_position = SERVO_ARM_REST;
    
    printf("Servos initialized to center position\r\n");
}

static void init_motor(void) {
    // Reset pins and set as outputs
    gpio_reset_pin(MOTOR_A_IN1);
    gpio_reset_pin(MOTOR_A_IN2);
    gpio_reset_pin(MOTOR_B_IN3);
    gpio_reset_pin(MOTOR_B_IN4);
    
    gpio_set_direction(MOTOR_A_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_A_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_IN3, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_IN4, GPIO_MODE_OUTPUT);
    
    // ensure stopped
    gpio_set_level(MOTOR_A_IN1, 0);
    gpio_set_level(MOTOR_A_IN2, 0);
    gpio_set_level(MOTOR_B_IN3, 0);
    gpio_set_level(MOTOR_B_IN4, 0);
    
    printf("Motor pins initialized with enables HIGH\r\n");
}

static void init_ultrasonic(void) {
    gpio_reset_pin(ULTRA_TRIG_PIN);
    gpio_set_direction(ULTRA_TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ULTRA_TRIG_PIN, 0);

    gpio_reset_pin(ULTRA_ECHO_PIN);
    gpio_set_direction(ULTRA_ECHO_PIN, GPIO_MODE_INPUT);
}

static void init_microphone(void) {
    // Configure ADC1 channel 6 (GPIO34)
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    printf("Microphone ADC initialized\r\n");
}

//———————————————————————————————————————————————
//— Motor primitives
//———————————————————————————————————————————————

static void motor_forward(void) {
    gpio_set_level(MOTOR_A_IN1, 1);
    gpio_set_level(MOTOR_A_IN2, 0);
    gpio_set_level(MOTOR_B_IN3, 1);
    gpio_set_level(MOTOR_B_IN4, 0);
}
static void motor_reverse(void) {
    gpio_set_level(MOTOR_A_IN1, 0);
    gpio_set_level(MOTOR_A_IN2, 1);
    gpio_set_level(MOTOR_B_IN3, 0);
    gpio_set_level(MOTOR_B_IN4, 1);
}
static void motor_left(void) {
    gpio_set_level(MOTOR_A_IN1, 0);
    gpio_set_level(MOTOR_A_IN2, 1);
    gpio_set_level(MOTOR_B_IN3, 1);
    gpio_set_level(MOTOR_B_IN4, 0);
}
static void motor_right(void) {
    gpio_set_level(MOTOR_A_IN1, 1);
    gpio_set_level(MOTOR_A_IN2, 0);
    gpio_set_level(MOTOR_B_IN3, 0);
    gpio_set_level(MOTOR_B_IN4, 1);
}
static void motor_stop(void) {
    gpio_set_level(MOTOR_A_IN1, 0);
    gpio_set_level(MOTOR_A_IN2, 0);
    gpio_set_level(MOTOR_B_IN3, 0);
    gpio_set_level(MOTOR_B_IN4, 0);
}


//———————————————————————————————————————————————
//— Ultrasonic measurement via esp_timer
//———————————————————————————————————————————————

// ——— Measure once, return distance in cm ———
static void ultrasonic_timer_cb(void* arg) {
    // send 10µs trigger
    gpio_set_level(ULTRA_TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(ULTRA_TRIG_PIN, 0);

    // wait for echo high
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(ULTRA_ECHO_PIN) == 0 &&
           esp_timer_get_time() - t0 < 20000) {}
    if (gpio_get_level(ULTRA_ECHO_PIN) == 0) {
        g_ultrasonic_cm = -1.0f;
        return;
    }

    // measure pulse width
    int64_t t1 = esp_timer_get_time();
    while (gpio_get_level(ULTRA_ECHO_PIN) == 1 &&
           esp_timer_get_time() - t1 < 20000) {}
    int64_t t2 = esp_timer_get_time();
    float dt = (float)(t2 - t1);  // µs
    g_ultrasonic_cm = (dt * 0.0343f) / 2.0f;
}


//——— Expressive reaction on obstacle ————————————————————
// quick LED+servo animation
static void express_react(void) {
    // Store original arm position to restore later
    int original_arm_pos = current_arm_position;
    
    // Show surprised eyes
    display_normal_eyes();
    
    // Raise arm by 35 degrees (subtract since servo orientation)
    int raised_arm_pos = original_arm_pos - 35;
    if (raised_arm_pos < 0) raised_arm_pos = 0;  // Ensure we don't go below 0
    set_servo(SERVO_CHANNEL_3, raised_arm_pos);
    
    // LED flash x3
    for (int i = 0; i < 3; i++) {
        gpio_set_level(GPIO_NUM_13, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(GPIO_NUM_13, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Blink eyes
    blink_eyes();
    
    // wave servo 2 out-and-back (pan)
    for (int a = 90; a <= 150; a += 10) {
        set_servo(SERVO_CHANNEL_2, a);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    for (int a = 150; a >= 30; a -= 10) {
        set_servo(SERVO_CHANNEL_2, a);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    set_servo(SERVO_CHANNEL_2, 90);

    // wave servo 1 out-and-back (tilt)
    for (int a = 90; a >= 30; a -= 10) {
        set_servo(SERVO_CHANNEL_1, a);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    // restore center
    set_servo(SERVO_CHANNEL_1, 90);
    
    // Return arm to original position
    set_servo(SERVO_CHANNEL_3, original_arm_pos);
}

//——— Anger reaction - spin in circles and move servos ————————————————————
static void express_anger(void) {
    // Show angry eyes
    display_angry_eyes();
    
    // Store original servo positions to restore later
    int original_arm_pos = current_arm_position;
    
    // Stop any current movement first
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Raise arm fully
    set_servo(SERVO_CHANNEL_3, original_arm_pos - 90);
    
    // Spin in circles twice (clockwise)
    for (int circle = 0; circle < 2; circle++) {
        // Start spinning right
        motor_right();
        
        // Move servos while spinning
        for (int pos = 30; pos <= 150; pos += 20) {
            // Move tilt servo
            set_servo(SERVO_CHANNEL_1, pos);
            
            // Move pan servo in opposite direction
            set_servo(SERVO_CHANNEL_2, 180 - pos);
            
            // Flash LED
            gpio_set_level(GPIO_NUM_13, pos % 40 < 20 ? 1 : 0);
            
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // Move servos back while continuing to spin
        for (int pos = 150; pos >= 30; pos -= 20) {
            // Move tilt servo
            set_servo(SERVO_CHANNEL_1, pos);
            
            // Move pan servo in opposite direction
            set_servo(SERVO_CHANNEL_2, 180 - pos);
            
            // Flash LED
            gpio_set_level(GPIO_NUM_13, pos % 40 < 20 ? 1 : 0);
            
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        obstacle_count = 0;
    }
    
    // Stop spinning
    motor_stop();
    
    // Return servos to center position
    set_servo(SERVO_CHANNEL_1, 90);
    set_servo(SERVO_CHANNEL_2, 90);
    set_servo(SERVO_CHANNEL_3, original_arm_pos);
    
    // Turn off LED
    gpio_set_level(GPIO_NUM_13, 0);
    
    // Return to normal eyes
    display_normal_eyes();
}

//——— TASKS ———

//———————————————————————————————————————————————
//— Random-mode task (priority 5)
//———————————————————————————————————————————————

static void random_task(void* arg) {
    const float THRESHOLD = 20.0f;  // cm
    while (1) {
        // Check if obstacle count exceeds threshold for anger reaction
        if (obstacle_count > 5) {
            // Execute anger behavior
            express_anger();
            
            // Reset obstacle count
            obstacle_count = 0;
            
            // Send message to UART
            uart_write_bytes(UART_PORT_NUM, "ANGER TRIGGERED: TOO MANY OBSTACLES!\r\n", 38);
            
            // Continue with normal operation after expressing anger
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        if (g_ultrasonic_cm > 0 && g_ultrasonic_cm < THRESHOLD) {
            // obstacle encountered
            obstacle_count++;
            
            // 1. Stop and back up slightly
            motor_stop();
            vTaskDelay(pdMS_TO_TICKS(500));
            motor_reverse();
            vTaskDelay(pdMS_TO_TICKS(300));  // back up for 300ms
            motor_stop();
            vTaskDelay(pdMS_TO_TICKS(200));  // Brief pause before turning
            
            // 2. Random turn (left or right) for random duration
            if (esp_random() % 2 == 0) {
                motor_left();
            } else {
                motor_right();
            }
            // Random turn duration between 500ms and 1500ms
            // Increased duration for more noticeable turns
            int turn_duration = (esp_random() % 1001) + 500;  // 500 to 1500ms
            vTaskDelay(pdMS_TO_TICKS(turn_duration));
            motor_stop();
            vTaskDelay(pdMS_TO_TICKS(200));  // Brief pause after turning

            // 3. Express reaction after completing the turn
            express_react();
        }
        
        // Continue forward
        motor_forward();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

//———————————————————————————————————————————————
//— Voice-mode task (priority 4) NOT FINISHED
//———————————————————————————————————————————————

static void voice_task(void* arg) {
    // suspend self until mode=VOICE
    vTaskSuspend(NULL);
    while (1) {
        // TODO: integrate microphone ISR / DMA here
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void microphone_task(void* arg) {
    const int SAMPLE_COUNT = 10;  // Number of samples to average
    int samples[SAMPLE_COUNT];
    int sample_index = 0;
    int sum = 0;
    float baseline = 0;
    const int CALIBRATION_SAMPLES = 50;  // Reduced from 100 to avoid long blocking
    const float SOUND_THRESHOLD = 200.0f;  // Deviation threshold for sound detection
    bool in_cooldown = false;  // Flag to prevent multiple triggers
    int startup_delay = 20;    // Ignore first few readings to let mic stabilize
    
    // Initialize array to zeros initially
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        samples[i] = 0;
    }
    
    // Establish baseline once at startup - with error handling
    printf("Calibrating baseline - please keep quiet...\n");
    int baseline_sum = 0;
    int valid_samples = 0;
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        int raw_value = adc1_get_raw(ADC1_CHANNEL_6);
        if (raw_value >= 0) {  // Check for valid reading
            baseline_sum += raw_value;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Ensure we have at least some valid samples
    if (valid_samples > 0) {
        baseline = (float)baseline_sum / valid_samples;
    } else {
        // Default baseline if no valid readings
        baseline = 2000.0f;
    }
    printf("Baseline established: %.1f (from %d samples)\n", baseline, valid_samples);
    
    // Initialize samples array with baseline value
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        samples[i] = (int)baseline;
    }
    sum = (int)baseline * SAMPLE_COUNT;
    
    while (1) {
        // Get new sample with error handling
        int new_value;
        
        // Add error handling for ADC reading
        new_value = adc1_get_raw(ADC1_CHANNEL_6);
        if (new_value < 0) {
            // Invalid reading, use baseline instead
            new_value = (int)baseline;
        }
        
        // Update running sum: subtract oldest sample, add new sample
        sum -= samples[sample_index];
        sum += new_value;
        samples[sample_index] = new_value;
        
        // Move to next position in circular buffer
        sample_index = (sample_index + 1) % SAMPLE_COUNT;
        
        // Calculate average if we have enough samples
        float avg = (float)sum / SAMPLE_COUNT;
        
        // Calculate deviation from baseline
        float deviation = fabsf(avg - baseline);
        
        // Skip processing during startup delay
        if (startup_delay > 0) {
            startup_delay--;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        
        // Reduce serial output frequency to avoid overwhelming the system
        if (sample_index == 0) {  // Only print once per full buffer cycle
            printf("Deviation: %.1f\n", deviation);
        }
        
        // Check if sound threshold is exceeded and not in cooldown
        if (deviation > SOUND_THRESHOLD && !in_cooldown) {
            printf("Loud sound detected! Stopping and reacting...\n");
            
            // Set cooldown flag
            in_cooldown = true;
            
            // Suspend other movement tasks - with error checking
            if (randomTaskHandle != NULL && eTaskGetState(randomTaskHandle) != eSuspended) {
                vTaskSuspend(randomTaskHandle);
            }
            if (voiceTaskHandle != NULL && eTaskGetState(voiceTaskHandle) != eSuspended) {
                vTaskSuspend(voiceTaskHandle);
            }
            
            // Stop all movement
            motor_stop();
            
            // React with LED and servos
            express_react();
            
            // Resume tasks in their previous state - with error checking
            if (current_mode == MODE_RANDOM && randomTaskHandle != NULL) {
                vTaskResume(randomTaskHandle);
            } else if (current_mode == MODE_VOICE && voiceTaskHandle != NULL) {
                vTaskResume(voiceTaskHandle);
            }
            
            // Reset cooldown flag after a longer delay to prevent multiple triggers
            vTaskDelay(pdMS_TO_TICKS(2000));  // 2000ms cooldown
            in_cooldown = false;
            
            // Reset the samples array to baseline values to avoid false triggers
            for (int i = 0; i < SAMPLE_COUNT; i++) {
                samples[i] = (int)baseline;
            }
            sum = (int)baseline * SAMPLE_COUNT;
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms delay between readings
    }
}

//——— Diagnostic task → LabVIEW (priority 3) —————————
static void diagnostic_task(void* arg) {
    char out[64];
    while (1) {
        snprintf(out, sizeof(out), "OBSTCNT %lu\r\n", obstacle_count);

        uart_write_bytes(UART_PORT_NUM, out, strlen(out));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//———————————————————————————————————————————————
//— UART & mode manager (priority 10)
//———————————————————————————————————————————————

static void echo_task(void* arg) {
    // 1) UART driver install
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE*2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 2) Handshake & buffer
    uint8_t* buf = malloc(UART_BUF_SIZE);
    uart_write_bytes(UART_PORT_NUM, "Commands\r\n", strlen("Commands\r\n"));

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, buf, UART_BUF_SIZE-1, pdMS_TO_TICKS(50));
        if (len <= 0) continue;
        buf[len] = '\0';
        char cmd = buf[0];
        
        // Echo received command to serial output
        char echo_msg[64];
        snprintf(echo_msg, sizeof(echo_msg), "Received command: %c\r\n", cmd);
        printf("%s", echo_msg);
        
        switch (cmd) {
            case 'I':
                // connection init
                gpio_set_level(GPIO_NUM_13, 1);
                uart_write_bytes(UART_PORT_NUM, "ESP32\r\n", strlen("ESP32\r\n"));
                break;

            // — MODE SWITCHES —
            case '1':  // RANDOM
                current_mode = MODE_RANDOM;
                vTaskResume(randomTaskHandle);
                vTaskSuspend(voiceTaskHandle);
                uart_write_bytes(UART_PORT_NUM, "MODE RANDOM\r\n", 13);
                break;
            case '2':  // REMOTE
                current_mode = MODE_REMOTE;
                vTaskSuspend(randomTaskHandle);
                vTaskSuspend(voiceTaskHandle);
                uart_write_bytes(UART_PORT_NUM, "MODE REMOTE\r\n", 13);
                break;
            case '3':  // VOICE
                current_mode = MODE_VOICE;
                vTaskSuspend(randomTaskHandle);
                vTaskResume(voiceTaskHandle);
                uart_write_bytes(UART_PORT_NUM, "MODE VOICE\r\n", 12);
                break;

            // — REMOTE-CONTROL COMMANDS —
            case 'F': if (current_mode==MODE_REMOTE) motor_forward();  break;
            case 'G': if (current_mode==MODE_REMOTE) motor_reverse();  break;
            case 'L': if (current_mode==MODE_REMOTE) motor_left();     break;
            case 'H': if (current_mode==MODE_REMOTE) motor_right();    break;
            case 'S': if (current_mode==MODE_REMOTE) motor_stop();     break;

            // — ON-DEMAND DISTANCE —
            case 'U': {
                char out[32];
                if (g_ultrasonic_cm < 0) {
                    snprintf(out, sizeof(out), "DIST ERR\r\n");
                } else {
                    snprintf(out, sizeof(out), "DIST %.1f\r\n", g_ultrasonic_cm);
                }
                uart_write_bytes(UART_PORT_NUM, out, strlen(out));
            } break;

            // — ARM SERVO COMMANDS —
            case '7': {  // Raise arm
                int new_pos = current_arm_position + SERVO_ARM_DELTA;
                if (new_pos > SERVO_MAX_DEG) new_pos = SERVO_MAX_DEG;
                set_servo(SERVO_CHANNEL_3, new_pos);
                current_arm_position = new_pos;  // Update the current position
                
                char response[32];
                snprintf(response, sizeof(response), "UP: %d\r\n", new_pos);
                uart_write_bytes(UART_PORT_NUM, response, strlen(response));
            } break;
            
            case '8': {  // Lower arm
                int new_pos = current_arm_position - SERVO_ARM_DELTA;
                if (new_pos < 0) new_pos = 0;
                set_servo(SERVO_CHANNEL_3, new_pos);
                current_arm_position = new_pos;  // Update the current position
                
                char response[32];
                snprintf(response, sizeof(response), "DOWN: %d\r\n", new_pos);
                uart_write_bytes(UART_PORT_NUM, response, strlen(response));
            } break;

            // — SPECIAL COMMANDS —
            case 'X': {  // ANGER command
                if (current_mode == MODE_RANDOM) {
                    // Suspend random task temporarily
                    vTaskSuspend(randomTaskHandle);
                    
                    // Execute anger behavior
                    express_anger();
                    
                    // Resume random task
                    vTaskResume(randomTaskHandle);
                    
                    uart_write_bytes(UART_PORT_NUM, "ANGER EXPRESSED\r\n", 17);
                } else {
                    uart_write_bytes(UART_PORT_NUM, "ANGER IGNORED (not in random mode)\r\n", 36);
                }
            } break;

            default:
                break;
        }
    }
}

void app_main(void) {
    // 1) Hardware init
    init_led();
    
    // Add delay between initializations
    vTaskDelay(pdMS_TO_TICKS(100));
    
    init_lcd();
    
    // Add delay between initializations
    vTaskDelay(pdMS_TO_TICKS(100));
    
    init_servo();
    
    // Add delay between initializations
    vTaskDelay(pdMS_TO_TICKS(100));
    
    init_motor();
    
    // Add delay between initializations
    vTaskDelay(pdMS_TO_TICKS(100));
    
    init_ultrasonic();
    
    // Add delay between initializations
    vTaskDelay(pdMS_TO_TICKS(100));
    
    init_microphone();
    
    // Explicitly ensure motors are stopped after initialization
    motor_stop();
    
    // Add delay before displaying eyes
    vTaskDelay(pdMS_TO_TICKS(200));
    display_normal_eyes();

    // 2) Start periodic ultrasonic timer (200 ms)
    const esp_timer_create_args_t targs = {
        .callback = ultrasonic_timer_cb,
        .name     = "ultra_timer"
    };
    esp_timer_handle_t tm;
    esp_timer_create(&targs, &tm);
    esp_timer_start_periodic(tm, 200000);

    // 3) Create mode tasks with increased stack sizes
    xTaskCreate(random_task, "MODE_RANDOM", STANDARD_TASK_STACK_SIZE, NULL, 5, &randomTaskHandle);
    vTaskSuspend(randomTaskHandle);  // Ensure random task is suspended initially
    motor_stop();

    xTaskCreate(voice_task, "MODE_VOICE", STANDARD_TASK_STACK_SIZE, NULL, 4, &voiceTaskHandle);
    vTaskSuspend(voiceTaskHandle);  // Ensure voice task is suspended initially

    xTaskCreate(diagnostic_task, "DIAG", STANDARD_TASK_STACK_SIZE, NULL, 3, NULL);

    // 4) UART & mode‐manager
    xTaskCreate(echo_task, "MODE_UART", UART_TASK_STACK, NULL, 10, NULL);

    // Create microphone monitoring task (high priority to ensure quick response)
    xTaskCreate(microphone_task, "MIC_MONITOR", STANDARD_TASK_STACK_SIZE, NULL, 8, NULL);
    
    printf("All tasks created successfully\n");
}
#include "lcd_1602.h"
#include <string.h>

// Pin definitions
static gpio_num_t rs_pin, e_pin, d4_pin, d5_pin, d6_pin, d7_pin;

// Commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// Flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// Flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// Flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

static uint8_t display_function;
static uint8_t display_control;
static uint8_t display_mode;

// Helper functions
static void pulse_enable(void) {
    gpio_set_level(e_pin, 0);
    esp_rom_delay_us(1);
    gpio_set_level(e_pin, 1);
    esp_rom_delay_us(1);
    gpio_set_level(e_pin, 0);
    esp_rom_delay_us(100); // Commands need > 37us to settle
}

static void write_4bits(uint8_t value) {
    gpio_set_level(d4_pin, (value >> 0) & 0x01);
    gpio_set_level(d5_pin, (value >> 1) & 0x01);
    gpio_set_level(d6_pin, (value >> 2) & 0x01);
    gpio_set_level(d7_pin, (value >> 3) & 0x01);
    pulse_enable();
}

static void send(uint8_t value, uint8_t mode) {
    gpio_set_level(rs_pin, mode);
    
    // 4-bit mode, send high 4 bits first
    write_4bits(value >> 4);
    write_4bits(value);
}

static void command(uint8_t value) {
    send(value, 0);
}

void lcd_init(gpio_num_t rs, gpio_num_t e, gpio_num_t d4, gpio_num_t d5, gpio_num_t d6, gpio_num_t d7) {
    // Save pin numbers
    rs_pin = rs;
    e_pin = e;
    d4_pin = d4;
    d5_pin = d5;
    d6_pin = d6;
    d7_pin = d7;
    
    // Configure pins
    gpio_reset_pin(rs_pin);
    gpio_reset_pin(e_pin);
    gpio_reset_pin(d4_pin);
    gpio_reset_pin(d5_pin);
    gpio_reset_pin(d6_pin);
    gpio_reset_pin(d7_pin);
    
    gpio_set_direction(rs_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(e_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(d4_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(d5_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(d6_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(d7_pin, GPIO_MODE_OUTPUT);
    
    // Initialize display
    display_function = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
    
    // Wait for LCD to become ready (docs suggest 40ms+ after power rises to 2.7V)
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Put LCD into 4-bit mode
    // First write 0x03, wait, write 0x03 again, wait, write 0x03 yet again, wait
    // Then write 0x02 to finally enter 4-bit mode
    gpio_set_level(rs_pin, 0);
    write_4bits(0x03);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    write_4bits(0x03);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    write_4bits(0x03);
    esp_rom_delay_us(150);
    
    write_4bits(0x02);
    
    // Set # of lines, font size, etc.
    command(LCD_FUNCTIONSET | display_function);
    
    // Turn the display on with no cursor or blinking
    display_control = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    command(LCD_DISPLAYCONTROL | display_control);
    
    // Clear display
    lcd_clear();
    
    // Initialize to default text direction (left-to-right)
    display_mode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    command(LCD_ENTRYMODESET | display_mode);
}

void lcd_clear(void) {
    command(LCD_CLEARDISPLAY);
    vTaskDelay(pdMS_TO_TICKS(2)); // This command takes a long time
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    static const uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void lcd_write_char(char c) {
    send(c, 1);
}

void lcd_write_string(const char *str) {
    while (*str) {
        lcd_write_char(*str++);
    }
}

void lcd_create_char(uint8_t location, const uint8_t *charmap) {
    location &= 0x7; // We only have 8 CGRAM locations (0-7)
    command(LCD_SETCGRAMADDR | (location << 3));
    
    for (int i = 0; i < 8; i++) {
        send(charmap[i], 1);
    }
    
    // Return to DDRAM address mode
    command(LCD_SETDDRAMADDR);
}
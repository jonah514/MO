#ifndef LCD_1602_H
#define LCD_1602_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Function prototypes
void lcd_init(gpio_num_t rs, gpio_num_t e, gpio_num_t d4, gpio_num_t d5, gpio_num_t d6, gpio_num_t d7);
void lcd_clear(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_write_char(char c);
void lcd_write_string(const char *str);
void lcd_create_char(uint8_t location, const uint8_t *charmap);

#endif // LCD_1602_H
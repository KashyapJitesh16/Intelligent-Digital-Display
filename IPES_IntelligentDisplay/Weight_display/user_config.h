#ifndef USER_CONFIG_H
#define USER_CONFIG_H
#include "driver/gpio.h"

// I2C
#define ESP32_SCL_NUM (GPIO_NUM_7)
#define ESP32_SDA_NUM (GPIO_NUM_15)


#define Backlight_Testing
#define EXAMPLE_Rotate_90 //软件实现旋转


#define EXAMPLE_LCD_H_RES 320
#define EXAMPLE_LCD_V_RES 820


#define EXAMPLE_LCD_IO_SPI_CS      (gpio_num_t)0
#define EXAMPLE_LCD_IO_SPI_SCK     (gpio_num_t)2
#define EXAMPLE_LCD_IO_SPI_SDO     (gpio_num_t)1

#define EXAMPLE_LCD_IO_RGB_DE      (gpio_num_t)40
#define EXAMPLE_LCD_IO_RGB_PCLK    (gpio_num_t)41
#define EXAMPLE_LCD_IO_RGB_VSYNC   (gpio_num_t)39
#define EXAMPLE_LCD_IO_RGB_HSYNC   (gpio_num_t)38
#define EXAMPLE_LCD_IO_RGB_DISP
#define EXAMPLE_LCD_IO_RGB_RESET   (gpio_num_t)16 

#define EXAMPLE_LCD_IO_RGB_R0      (gpio_num_t)17
#define EXAMPLE_LCD_IO_RGB_R1      (gpio_num_t)46
#define EXAMPLE_LCD_IO_RGB_R2      (gpio_num_t)3
#define EXAMPLE_LCD_IO_RGB_R3      (gpio_num_t)8
#define EXAMPLE_LCD_IO_RGB_R4      (gpio_num_t)18

#define EXAMPLE_LCD_IO_RGB_G0      (gpio_num_t)14
#define EXAMPLE_LCD_IO_RGB_G1      (gpio_num_t)13
#define EXAMPLE_LCD_IO_RGB_G2      (gpio_num_t)12
#define EXAMPLE_LCD_IO_RGB_G3      (gpio_num_t)11
#define EXAMPLE_LCD_IO_RGB_G4      (gpio_num_t)10
#define EXAMPLE_LCD_IO_RGB_G5      (gpio_num_t)9

#define EXAMPLE_LCD_IO_RGB_B0      (gpio_num_t)21
#define EXAMPLE_LCD_IO_RGB_B1      (gpio_num_t)5
#define EXAMPLE_LCD_IO_RGB_B2      (gpio_num_t)45
#define EXAMPLE_LCD_IO_RGB_B3      (gpio_num_t)48
#define EXAMPLE_LCD_IO_RGB_B4      (gpio_num_t)47


#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (8 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     5


#endif
#pragma once

#include "driver/gpio.h"

// I2C

#define I2C_MASTER_SCL_GPIO GPIO_NUM_1
#define I2C_MASTER_SDA_GPIO GPIO_NUM_2

// TLC5948

#define TLC5948_PIN_MOSI GPIO_NUM_35    // SIN (Serial Data Input)
#define TLC5948_PIN_SCLK GPIO_NUM_36    // SCLK (Serial Clock)
#define TLC5948_PIN_MISO GPIO_NUM_37    // SCLK (Serial Clock)
#define TLC5948_PIN_XLAT GPIO_NUM_47    // LAT (Data Latch)
#define TLC5948_PIN_GSCLK GPIO_NUM_6    // GSCLK (Grayscale Clock)

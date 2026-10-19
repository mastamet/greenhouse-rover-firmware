#ifndef SH1107_H
#define SH1107_H

#include <stdint.h>
#include "driver/i2c.h"

// Initialize the SH1107 display over I2C
void sh1107_init(i2c_port_t i2c_num);

// Clear the screen buffer
void sh1107_clear(void);

// Draw a string at a specific page (0 to 15) and column (0 to 127)
void sh1107_draw_string(uint8_t page, uint8_t col, const char *str);

// Push the local buffer to the physical OLED screen via I2C
void sh1107_refresh(void);

#endif
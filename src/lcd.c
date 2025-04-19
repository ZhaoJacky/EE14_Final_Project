// Function implementations of the functions that control the LCD connected
// to the STM32 L432KC Nucleo Board.

// Includes the header file.
#include "lcd.h"

void enable_lcd() {
    host_serial_init();
    i2c_init(I2C1, D1, D0);
}

void write_lcd(uint8_t data) {
    unsigned char initial[1] = {data};
    i2c_write(I2C1, 0x27, initial ,1);
}

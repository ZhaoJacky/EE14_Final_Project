// Header file that contains the functions that control an LCD connected
// to a STM32 L432KC Nucleo Board

#include <stm32l432xx.h>
#include "ee14lib.h"

// Before using the commands contained in this file, please make sure
// to initialize I2C on the Nucleo Board.
void SysTick_initialize(void);
void delay_ms(int s);
void write_lcd(uint8_t byte, int data);
uint8_t construct_lcd_nibble(uint8_t nibble, int enable, int data);
void enable_lcd();
// Header file that contains the functions that control an LCD connected
// to a STM32 L432KC Nucleo Board

#include <stm32l432xx.h>
#include "ee14lib.h"

// Before using the commands contained in this file, please make sure
// to initialize I2C on the Nucleo Board.

void write_lcd(uint8_t data);
void enable_lcd();
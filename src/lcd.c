// Function implementations of the functions that control the LCD connected
// to the STM32 L432KC Nucleo Board.

// Includes the header file.
#include "lcd.h"
#include <string.h>

void SysTick_initialize(void) {
    SysTick->CTRL = 0; //disables systick counter
    SysTick->LOAD = 3999; 
    //LOAD indicates how often an interrupt is generated (4Mhz/LOAD = frequency
    //of interrupt generation)

    // This sets the priority of the interrupt to 15 (2^4 - 1), which is the
    // largest supported value (aka lowest priority)
    NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);

    SysTick->VAL = 0; //resets counter to zero, when program starts, is set to load value, and counts down
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; //selects processor clock
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; //reaching zero sends interrupt request
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //enables counter
}

volatile unsigned int n = 0;
void delay_ms(int s){
    volatile unsigned int target = n + s;
    while(n < target){}
}

void SysTick_Handler(void){ //automatically gets called when interrupt occurs
    n++; //total number of times interrupt has occurred
}

/*
Uses Professor Bell's i2c.c file to initialize the i2c protocol. It's
just a wrapper function for it to be honest.
*/
void enable_lcd() {
    host_serial_init();
    i2c_init(I2C1, D1, D0);

    write_lcd(0x30, 0); // Sets 8 bit-mode 3 times (necessary to turn on).
    delay_ms(2);
    write_lcd(0x30, 0); // Make sure the LCD is in 8-bit mode.
    delay_ms(2);
    write_lcd(0x30, 0); // Make sure the LCD is in 8-bit mode.
    delay_ms(2);
    write_lcd(0x20, 0); // Switches LCD to 4-bit mode.
    delay_ms(2);

    write_lcd(0x28, 0); // Sets font and line number.
    delay_ms(2);
    write_lcd(0x08, 0); // Turns off display
    delay_ms(2);
    write_lcd(0x01, 0); // Clears Display
    delay_ms(2);
    write_lcd(0x06, 0); // Sets how the display responds when it receives data (increments).
    delay_ms(2);
    write_lcd(0x0C, 0); // Turns on the display
    delay_ms(2);
    write_lcd(0x01, 0); // Clears the display again for a second time.
    delay_ms(2);
}

/* Name: write_lcd
 * Arguments: v `32
 * 1) 8 bit integer indicating the ascii code of the desired character
 * 2) integer indicated if the data bit should be set to 1 or 0
 * Purpose:
 * Bit masks an ascii character to separate it into the upper and lower nibbles. 
 * Calls construct nibble function to create 4 necessary bytes, and stores them in 
 * an array. Sends the byte array to the lcd.
 * Output: 
 * void function; no output. 
*/
void write_lcd(uint8_t byte, int data) {
    uint8_t nibble1 = (byte & 0xF0); //most significant
    uint8_t nibble2 = (byte & 0x0F) << 4; //least significant

    unsigned char byte_bus[4]; //enable goes 0 1 0 1
    for(int i = 0; i < 4; i++) {
        if(i < 2) {
            byte_bus[i] = construct_lcd_nibble(nibble1, (i + 1) % 2, data);
        } else {
            byte_bus[i] = construct_lcd_nibble(nibble2, (i + 1) % 2, data);
        }
    }
    
    //send array to LCD
    i2c_write(I2C1, 0x27, byte_bus, 4);
    
}

/* Name: construct_lcd_nibble
 * Arguments: 
 * 1) nibble containing high or low nibble from original ascii character
 * 2) integer indicated if the data bit should be set to 1 or 0
 * Purpose:
 * Offsets bits to assign them to proper bits that correspond to their designated
 * function for the LCD to interpret
 * 
 * Output: 
 * A byte containing properly assigned bits to be sent to LCD
*/
//actually constructs a byte, but it contains the nibble
uint8_t construct_lcd_nibble(uint8_t nibble, int enable, int data) {

    uint8_t i2c_byte = nibble;

    if(data == 0) { // RS = 0 (command) RS = 1 (data)
        i2c_byte &= ~(1 << 0);
    } else {
        i2c_byte |= (1 << 0);
    }

    i2c_byte &= ~(1 << 1); // RW always write(0)
    
    if(enable) { //enable is one
        i2c_byte |= (1 << 2); // E = 1
    } else { //enable is zero
        i2c_byte &= ~(1 << 2); // E = 0
    }
    
    i2c_byte |= (1 << 3); // Backlight LED
    
    return i2c_byte;
}

void print_string_lcd(char *c) {
    int length = strlen(c);
    for (int i = 0; i < length; i++){
        write_lcd(c[i], 1);
    }
}
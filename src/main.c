/*
EE14 Final Project: keyboard
*/

#include "stm32l432xx.h"
#include "ee14lib.h"
#include "lcd.h"
#include "dac.h"
#include <stdio.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "sine_table.h"

const int NUM_NOTES = 8;
const EE14Lib_Pin b[] = { D3, D2, D5, D6, D9, D10, D11, D12 }; //array of pins
                            // B   A   G   F   E    D    C
const EE14Lib_Pin sp = A4;

// Mapping of Nucleo pin number to GPIO port
static GPIO_TypeDef * g_GPIO_port[D13+1] = {
    GPIOA,GPIOA,GPIOA,GPIOA,  // A0=PA0,A1=PA1,A2=PA3,A3=PA4
    GPIOA,GPIOA,GPIOA,GPIOA,  // A4=PA5,A5=PA6,A6=PA7,A7=PA2
    GPIOA,GPIOA,GPIOA,GPIOB,  // D0=PA10,D1=PA9,D2=PA12,D3=PB0
    GPIOB,GPIOB,GPIOB,GPIOC,  // D4=PB7,D5=PB6,D6=PB1,D7=PC14
    GPIOC,GPIOA,GPIOA,GPIOB,  // D8=PC15,D9=PA8,D10=PA11,D11=PB5
    GPIOB,GPIOB               // D12=PB4,D13=PB3.
};
  
// Mapping of Nucleo pin number to GPIO pin
// Using this plust g_GPIO_port[] above, we can translate a Nucleo pin name into
// the chip's actual GPIO port and pin number.
static uint8_t g_GPIO_pin[D13+1] = {
    0,1,3,4,    // A0=PA0,A1=PA1,A2=PA3,A3=PA4
    5,6,7,2,    // A4=PA5,A5=PA6,A6=PA7,A7=PA2
    10,9,12,0,  // D0=PA10,D1=PA9,D2=PA12,D3=PB0
    7,6,1,14,   // D4=PB7,D5=PB6,D6=PB1,D7=PC14
    15,8,11,5,  // D8=PC15,D9=PA8,D10=PA11,D11=PB5
    4,3         // D12=PB4,D13=PB3.
};

//for the print functions
int _write(int file, char *data, int len){
    serial_write(USART2, data, len);
    return len;
}

//check which buttons are pressed
bool check_pressed(EE14Lib_Pin pin) {
    GPIO_TypeDef* port = g_GPIO_port[pin];
    uint8_t pin_offset = g_GPIO_pin[pin];
    
    if (!(port->IDR & (0x1UL << pin_offset))) { //if pressed
        return true;
    }
    return false;
}

//print the names of the notes
void note_name(EE14Lib_Pin pin) {
    if      (pin == D2)  write_lcd('B', 1);
    else if (pin == D5)  write_lcd('A', 1);
    else if (pin == D6)  write_lcd('G', 1);
    else if (pin == D9)  write_lcd('F', 1);
    else if (pin == D10) write_lcd('E', 1);
    else if (pin == D11) write_lcd('D', 1);
    else if (pin == D12) write_lcd('C', 1);
}

void run(EE14Lib_Pin pin) {
    unsigned int freq_DAC = 0;
    bool printed = false;
    GPIO_TypeDef* port = g_GPIO_port[pin];
    uint8_t pin_offset = g_GPIO_pin[pin];
    
    while (!(port->IDR & (0x1UL << pin_offset))) { //if pressed
        if(freq_DAC == 0) {
            freq_DAC = freq_select(pin);
            TIM7_Init(freq_DAC);  
        }
        if (printed == false) {
            note_name(pin);
            for (volatile int i = 0; i < 1000; i++);
            printed = true;
        }
    }
    if(printed == true) {
        write_lcd(0x01, 0);
        printed = false;
    }
    
    freq_DAC = 0;
    TIM7_Init(freq_DAC);
}



//play when button is pressed
// void play(int *notes, int size_array) {
//     EE14Lib_Pin pin;
//     for (int i = 0; i < NUM_NOTES; i++) {
//         if (notes[i] == 1) { //if pressed
//             pin = b[i]; //set pin 
//         }
//     }
//     GPIO_TypeDef* port = g_GPIO_port[pin]; 
//     uint8_t pin_offset = g_GPIO_pin[pin];

//     for (int i = 0; i < size_array; i++) { //loop through the sound sample
//         if(!(port->IDR & 0x1UL << pin_offset)) { //check if button is pressed
//             // send information to DAC

//         } else { //button released
//             i = size_array - 1;
//         }    
//     }
// }

//config pins used for the buttons
void setup() {
    gpio_config_mode(b[0], INPUT); //button 0   //A0 //PA0
    gpio_config_pullup(b[0], PULL_UP);

    gpio_config_mode(b[1], INPUT); //button 1   //D2 //PA12
    gpio_config_pullup(b[1], PULL_UP);

    gpio_config_mode(b[2], INPUT); //button 2   //A1 //PA1
    gpio_config_pullup(b[2], PULL_UP);

    gpio_config_mode(b[3], INPUT); //button 3   //D5 //PB6
    gpio_config_pullup(b[3], PULL_UP);

    gpio_config_mode(b[4], INPUT); //button 4   //D9 //PA8
    gpio_config_pullup(b[4], PULL_UP);

    gpio_config_mode(b[5], INPUT); //button 5   //D10 //PA11
    gpio_config_pullup(b[5], PULL_UP);

    gpio_config_mode(b[6], INPUT); //button 6   //D11 //PB5
    gpio_config_pullup(b[6], PULL_UP);

    gpio_config_mode(b[7], INPUT); //button 7   //D12 //PB4
    gpio_config_pullup(b[7], PULL_UP);
}

// display the notes on LCD
// void display_note(int *notes) {
//     for (int i = 0; i < NUM_NOTES; i++) {
//         if (notes[i] == 1) { //if pressed
//             note_name(i);
//         }
//     }
// }

// uint16_t lookup_sine(int x, int *sine_table) {
//     //x is output in degrees
//     x = fmod(x, 360); //x might be Larger than 360 
//     if (x < 90) return sine_table[x]; 
//     if (x < 180) return sine_table[180-x]; 
//     if (x < 270) return 4096 - sine_table[x-180]; 
//     return 4096 - sine_table[360-x]; 
// }

// print starting message to the LCD
void print_start(void) {
    write_lcd('h', 1);
    write_lcd('e', 1);
    write_lcd('l', 1);
    write_lcd('l', 1);
    write_lcd('o', 1);

    write_lcd(' ', 1);

    write_lcd('w', 1);
    write_lcd('o', 1);
    write_lcd('r', 1);
    write_lcd('l', 1);
    write_lcd('d', 1);

    write_lcd('!', 1);

    delay_ms(2000);

    write_lcd(0x01, 0);
}

volatile int i = 0;
// float mult = 1.0f;

void TIM7_IRQHandler(void) {
    int output;
    if (TIM7->SR & TIM_SR_UIF) {           // Check update interrupt flag
        TIM7->SR &= ~TIM_SR_UIF;           // Clear interrupt flag

        // float sine_wave = sine[i] * mult;
        // output = sine_wave;
        output = sine[i];                 // Get next sample from sine table
    
        DAC->DHR12R2 = output;             // Load output to DAC channel 2
        DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG2;  // Trigger DAC conversion
        
        i++;                                 // Advance to next sample
        if (i >= 360)   i = 0;               // Wrap around when end is reached
    }
}

unsigned int freq_select(EE14Lib_Pin note) {
    unsigned int freq = 0;
    if      (note == D12) freq = 23600;
    else if (note == D11) freq = 26500;
    else if (note == D10) freq = 30000;
    else if (note == D9)  freq = 31500;
    else if (note == D6)  freq = 35700;
    else if (note == D5)  freq = 40000;
    else if (note == D2)  freq = 44900;

    return freq;
}

int main() {
    host_serial_init();
    SysTick_initialize();
    DAC_Channel2_Init(); //initialize pin A4 --> PA5
    setup(); //config button pins
    enable_lcd(); //config lcd
    print_start(); //print starting message
    while(1) {
        for (int i = 0; i < NUM_NOTES; i++) {
            run(b[i]);
        }
    }
}


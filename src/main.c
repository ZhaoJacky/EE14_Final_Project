#include "stm32l432xx.h"
#include "ee14lib.h"
#include "lcd.h"

const int SIZE_ARRAY = 1000;
const int NUM_NOTES = 8;
const EE14Lib_Pin b[] = { A0, D2, A1, D5, D9, D10, D11, D12 }; //array of pins
//button 0 //A0 //PA0
//button 1 //D2 //PA12
//button 2 //A1 //PA1
//button 4 //D9 //PA8
//button 5 //D10 //PA11
//button 6 //D11 //PB5
//button 7 //D12 //PB4

const EE14Lib_Pin sp = D1;

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

int _write(int file, char *data, int len){
    serial_write(USART2, data, len);
    return len;
}

void test_button(int *notes, int button_index){
    if(notes[button_index]){
        gpio_write(D1, 1);
    } else {
        gpio_write(D1, 0);
    }
}

void test_all(int *notes) {
    int count = 0;
    for(int i = 0; i < NUM_NOTES; i++) {
        if(notes[i]) { //if the button is pressed
            count++;
        }
    }
    if(count > 0) { //if any button was pressed
        gpio_write(D1, 1); //turn on 
    } else { //if no buttons pressed
        gpio_write(D1, 0); //turn off
    }
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

//play when button is pressed
void play(int *notes, int size_array) {
    EE14Lib_Pin pin;
    for (int i = 0; i < NUM_NOTES; i++) {
        if (notes[i] == 1) { //if pressed
            pin = b[i]; //set pin 
        }
    }
    GPIO_TypeDef* port = g_GPIO_port[pin]; 
    uint8_t pin_offset = g_GPIO_pin[pin];

    for (int i = 0; i < size_array; i++) {
        if(!(port->IDR & 0x1UL << pin_offset)) { 
            // send information to DAC
        } else { //button released
            i = size_array - 1;
        }    
    }

}


void setup() {
    gpio_config_mode(b[0], INPUT); //button 0   //A0 //PA0
    gpio_config_pullup(b[0], PULL_UP);

    gpio_config_mode(b[1], INPUT); //button 1 //D2 //PA12
    gpio_config_pullup(b[1], PULL_UP);

    gpio_config_mode(b[2], INPUT); //button 2 //A1 //PA1
    gpio_config_pullup(b[2], PULL_UP);

    gpio_config_mode(b[3], INPUT); //button 3 //D5 //PB6
    gpio_config_pullup(b[3], PULL_UP);

    gpio_config_mode(b[4], INPUT); //button 4 //D9 //PA8
    gpio_config_pullup(b[4], PULL_UP);

    gpio_config_mode(b[5], INPUT); //button 5 //D10 //PA11
    gpio_config_pullup(b[5], PULL_UP);

    gpio_config_mode(b[6], INPUT); //button 6 //D11 //PB5
    gpio_config_pullup(b[6], PULL_UP);

    gpio_config_mode(b[7], INPUT); //button 7 //D12 //PB4
    gpio_config_pullup(b[7], PULL_UP);

    //speaker, analog output //A2??
    gpio_config_mode(D1, OUTPUT);
}

int main() {
    setup(); //configure button MODER's & PUPDR's
    gpio_write(D1, 0); //1 = on , 0 = off
    while(1) {        
        int notes[NUM_NOTES];
        for (int i = 0; i < NUM_NOTES; i++) { //initialize with all zeroes
            notes[i] = 0;
        }
        for(int i = 0; i < NUM_NOTES; i++) { //assign pressed (1) or unpressed (0)
            if(check_pressed(b[i])) {
                notes[i] = 1;
            } else {
                notes[i] = 0;
            }
        }
        // test_button(notes, 7);
        // test_all(notes);
        play(notes, SIZE_ARRAY);
    }
}

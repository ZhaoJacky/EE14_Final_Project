/*
EE14 Final Project: keyboard
*/

#include "stm32l432xx.h"
#include "ee14lib.h"
#include "lcd.h"
#include "dac.h"
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "sine_table.h"

volatile int i = 0;
bool mode = 0; // 0 = High Notes and 1 = Low Notes
bool song_mode = 0; // 0 = OFF and 1 = ON
int fade_num = 4096;
const int NUM_NOTES = 9;
const EE14Lib_Pin b[] = { A1, D2, D5, D6, D9, D10, D11, D12, A0}; //array of pins
                        
const EE14Lib_Pin sp = A4;
bool song_printed = false;
volatile unsigned int song_count = 0; //A1  //PA1 
// // 0 = OFF; 1 = Twinkle; 2 = Belong; 3 = Rick Roll

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

// //for the print functions
int _write(int file, char *data, int len){
    serial_write(USART2, data, len);
    return len;
}

//button mode
void EXTI0_IRQHandler (void) {
    if (EXTI->PR1 & EXTI_PR1_PIF0) { //A0  //PA0
        mode = !mode;
        EXTI->PR1 |= EXTI_PR1_PIF0;   //   clear bits(by writing a 1)
    }
}

//change modes (song mode: on/off, change songs)
void EXTI1_IRQHandler (void) {
    if (EXTI->PR1 & EXTI_PR1_PIF1) { //A1  //PA1
        
        if (song_count == 0 || song_count == 3) {
           song_mode = !song_mode; 
        }
        song_count = (song_count + 1) % 4;
        song_printed = false;
        
        EXTI->PR1 |= EXTI_PR1_PIF1;   //   clear bits(by writing a 1)
    }
}

void button1_config(void) { //A0  //PA0
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; // Clear any existing mapping for EXTI0

    EXTI->FTSR1 |= EXTI_FTSR1_FT0;

    EXTI->IMR1 |= EXTI_IMR1_IM0;

    NVIC_SetPriority(EXTI0_IRQn, 2);  // Set the priority of EXTI line 0 interrupt (priority 2)
    NVIC_EnableIRQ(EXTI0_IRQn); // Enable the EXTI0 interrupt in the NVIC
}

void button3_config(void) { //A1  //PA1
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1; // Clear any existing mapping for EXTI0

    EXTI->FTSR1 |= EXTI_FTSR1_FT1;

    EXTI->IMR1 |= EXTI_IMR1_IM1;

    NVIC_SetPriority(EXTI1_IRQn, 2);  // Set the priority of EXTI line 0 interrupt (priority 2)
    NVIC_EnableIRQ(EXTI1_IRQn); // Enable the EXTI0 interrupt in the NVIC
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

bool check_pressed(EE14Lib_Pin pin) {
    GPIO_TypeDef* port = g_GPIO_port[pin];
    uint8_t pin_offset = g_GPIO_pin[pin];
    
    if (!(port->IDR & (0x1UL << pin_offset))) {
        return true;
    } 
    return false;
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
        if (printed == false && !song_mode) {
            note_name(pin);
            printed = true;
        }
    }
    if(printed == true && !song_mode) {
        write_lcd(0x01, 0);
        printed = false;
    }
    
    freq_DAC = 0;
    fade_num = 4096;
    TIM7_Init(freq_DAC);
}

//config pins used for the buttons
void setup() {
    gpio_config_mode(b[0], INPUT); 
    gpio_config_pullup(b[0], PULL_UP);

    gpio_config_mode(b[1], INPUT); 
    gpio_config_pullup(b[1], PULL_UP);

    gpio_config_mode(b[2], INPUT); 
    gpio_config_pullup(b[2], PULL_UP);

    gpio_config_mode(b[3], INPUT); 
    gpio_config_pullup(b[3], PULL_UP);

    gpio_config_mode(b[4], INPUT); 
    gpio_config_pullup(b[4], PULL_UP);

    gpio_config_mode(b[5], INPUT); 
    gpio_config_pullup(b[5], PULL_UP);

    gpio_config_mode(b[6], INPUT); 
    gpio_config_pullup(b[6], PULL_UP);

    gpio_config_mode(b[7], INPUT); 
    gpio_config_pullup(b[7], PULL_UP);

    gpio_config_mode(b[8], INPUT); 
    gpio_config_pullup(b[8], PULL_UP);
}

void TIM7_IRQHandler(void) {
    int output;
    if (TIM7->SR & TIM_SR_UIF) {           // Check update interrupt flag
        TIM7->SR &= ~TIM_SR_UIF;           // Clear interrupt flag

        if(mode == 0) { 
            output = (sine2[i] * fade_num) >> 12; //higher octave
        } else {
            output = (sine[i] * fade_num) >> 12;
        }
        
        DAC->DHR12R2 = output;             // Load output to DAC channel 2
        DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG2;  // Trigger DAC conversion
        
        i++;                                 // Advance to next sample
        if (i >= 90 && mode == 0) { //0 = higher
            i = 0;
            if(fade_num > 0) fade_num -= 8;
        } else if (i >= 360 && mode == 1) {
            i = 0;
            if(fade_num > 0) fade_num -= 16;
        }
    }
}  

unsigned int freq_select(EE14Lib_Pin note) {
    unsigned int freq = 0;
    if      (note == D12) freq = 23600; //C
    else if (note == D11) freq = 26500; //D
    else if (note == D10) freq = 30000; //E
    else if (note == D9)  freq = 31500; //F
    else if (note == D6)  freq = 35700; //G
    else if (note == D5)  freq = 40000; //A
    else if (note == D2)  freq = 44900; //B

    return freq;
}

void Twinkle() {
    write_lcd(0x01, 0); //clear LCD
    delay_ms(100);

    print_string_lcd("Guess the song:");
    write_lcd(0xC0, 0); //new line
    delay_ms(50);
    print_string_lcd("CCGGAAG FFEEDDC");
}

void Belong() {
    write_lcd(0x01, 0); //clear LCD
    delay_ms(100);

    print_string_lcd("Guess the song:");
    delay_ms(2000);
    write_lcd(0x01, 0); //clear LCD
    delay_ms(500);
    print_string_lcd("GEEDCCGEDED");
    write_lcd(0xC0, 0); //new line
    delay_ms(50);
    print_string_lcd("DGGEEDGEDED");
}

void Rick_roll(){
    write_lcd(0x01, 0); //clear LCD
    delay_ms(100);

    print_string_lcd("Guess the song:");
    write_lcd(0xC0, 0); //new line
    delay_ms(50);
    print_string_lcd("CDFDAAG CDFDGGF");
}

int main() {
    host_serial_init();
    SysTick_initialize();
    button1_config();
    button3_config();
    DAC_Channel2_Init(); //initialize pin A4 --> PA5
    setup(); //config button pins
    enable_lcd(); //config lcd
    
    //print starting message
    print_string_lcd("Welcome to");
    write_lcd(0xC0, 0);
    delay_ms(50);
    print_string_lcd("Electric Piano!");
    write_lcd(0xC0, 0);
    delay_ms(50);

    delay_ms(2000);
    write_lcd(0x01, 0);
    delay_ms(500);
    while(1) {
        if (song_mode && !song_printed) {
            if (song_count == 1) { //twinkle twinkle
                Twinkle();
            } else if (song_count == 2) {
                Belong();
            } else if (song_count == 3) {
                Rick_roll();
            }
            song_printed = true;
        }
        
        for (int i = 0; i < NUM_NOTES; i++) {
            run(b[i]);
        }
    }
}




#include <stm32l432xx.h>
#include "ee14lib.h"
#include <stdio.h>
#include <string.h>

volatile unsigned int count = 0;

void  SysTick_Handler(void) {
    count++;
}

void SysTick_initialize(void) {
    SysTick->CTRL = 0;
    SysTick->LOAD = 3999; //reload value register
    // This sets the priority of the interrupt to 15 (2^4 - 1), which is the
    // largest supported value (aka lowest priority)
    NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
    SysTick->VAL = 0;//current value register
    //Control and status and 
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;//indicates the clock source
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; //enable interrupt request
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //enable the counter or not
}

void delay_ms(int ms) {
    //record current millisecond count
    volatile int enter_count = count;
    //add the delay
    volatile int delay_count = enter_count + ms;
    //spin in loop itil count exceeds the number
    while (count <= delay_count) {}
}

int main() {
    //Systick initialize
    SysTick_initialize();

    //gpio interrupt
    //speaker 
    //LCD 

    delay_ms(1000);
    
}
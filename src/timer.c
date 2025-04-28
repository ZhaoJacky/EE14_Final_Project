#include "ee14lib.h"

// Initializes the TIM7 timer.
void TIM7_Init(unsigned int freq_hz)
{
    // Enable TIM7 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;

    // Disable TIM7 while configuring
    TIM7->CR1 &= ~TIM_CR1_CEN;

    // Calculate prescaler and ARR
    TIM7->PSC = 0;             
    TIM7->ARR = (4000000 / freq_hz) - 1;

    // Clear counter
    TIM7->CNT = 0;

    // Enable update interrupt
    TIM7->DIER |= TIM_DIER_UIE;

    // Enable TIM7 interrupt in NVIC
    NVIC_EnableIRQ(TIM7_IRQn);

    // Start TIM7
    TIM7->CR1 |= TIM_CR1_CEN;
}



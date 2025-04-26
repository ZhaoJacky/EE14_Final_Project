#include "ee14lib.h"

// /*
//  * Name:      timer_config
//  * Purpose:   Enables the clock for each timer and configure PWM frequency by 
//  *            performing calculations to modify the auto-reload register (ARR) 
//  *            and prescaler (PSC) of the timer for each pin. Finally, enable 
//  *            the timer itself.
//  * Arguments: timer: pointer to the timer of the chosen pin (TIM1, TIM2, etc.)
//  *            freq_hz: value of the frequency of signal in Hz
//  * Returns:   EE14Lib_Err_OK if nothing went wrong
//  *            EE14Lib_Err_NOT_IMPLEMENTED for invalid configurations.
//  */
// EE14Lib_Err timer_config(TIM_TypeDef* const timer, const unsigned int freq_hz)
// {
//     // Enable the clock for the timer
//     if(timer == TIM1){
//         RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
//     }
//     else if(timer == TIM2){
//         RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
//     }
//     else if(timer == TIM15){
//         RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
//     }
//     else if(timer == TIM16){
//         RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
//     }
//     else if( timer == TIM7){
//         RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
//     }
//     else{
//         return EE14Lib_Err_NOT_IMPLEMENTED;
//     }

//     timer -> PSC = 65535;

//     // //maximize prescalar, minimize arr (reload value)
//     // // unsigned int r = (4000000/freq_hz)/(65535+1);
//     // // timer -> ARR = r;

//     unsigned int arr = (4000000 / ((65535 + 1) * freq_hz)) - 1;
//     timer->ARR = arr;

    

//     /*Set the prescaler value as: original frequency over desired
//     (4MHz/desired frequency), divided by the max value of ARR, minimizing
//     prescalar and maximizing arr, allowing widest range of duty cycles (colors)
//     */
//     // unsigned int p = (4000000/freq_hz)/(65535);
//     // timer -> PSC = p;

//     // Set the reload value
//     //unsigned int n = 4000000/(p+1); //calculate new PWM frequency after prescaler is applied
//     // divide new frequency by desired, then subtract 1 for ARR value & assign to register
//     //timer -> ARR = n/freq_hz - 1; 
//     // timer -> ARR = 90;

//     // // Set the main output enable
//     // timer->BDTR |= TIM_BDTR_MOE;

//     // And enable the timer itself
//     timer->CR1 |= TIM_CR1_CEN;

//     // Enables the timer to generate an interrupt.
//     timer->DIER |= TIM_DIER_UIE;

//     return EE14Lib_Err_OK;    
// }

// Initializes the TIM7 timer.
void TIM7_Init(unsigned int freq_hz)
{
    // Enable TIM7 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;

    // Disable TIM7 while configuring
    TIM7->CR1 &= ~TIM_CR1_CEN;
    TIM7->CR1 &= ~TIM_CR2_MMS_1;

    // Calculate prescaler and ARR
    unsigned int clk = 4000000; // Timer clock (4 MHz)
    TIM7->PSC = 0;              // No prescaler for now
    TIM7->ARR = (clk / freq_hz) - 1;

    // Clear counter
    TIM7->CNT = 0;

    // Enable update interrupt
    TIM7->DIER |= TIM_DIER_UIE;

    // Enable TIM7 interrupt in NVIC
    NVIC_EnableIRQ(TIM7_IRQn);

    // Start TIM7
    TIM7->CR1 |= TIM_CR1_CEN;
    TIM7->CR1 |= TIM_CR2_MMS_1;
}



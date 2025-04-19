#include "stm32l432xx.h"
#include "ee14lib.h"

void DAC_Channel1_Init(void) {
    
    // Enables the clock for the DAC clock.
    RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;

    // Disable DAC.
    DAC->CR &= ~(DAC_CR_EN1 | DAC_CR_EN2);

    // Enables normal mode.
    DAC->MCR &= ~DAC_MCR_MODE2;

    // Enables the trigger for DAC channel 1.
    DAC->CR |= DAC_CR_TEN1;

    // Select the software trigger.
    DAC->CR |= DAC_CR_TSEL1;

    // Enable the DAC channel 1.

    // Enable the clock of GPIO port A.
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // Set PA5's I/O mode is analog.
    GPIOA->MODER |= GPIO_MODER_MODE5;
}


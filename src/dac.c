#include "stm32l432xx.h"
#include "ee14lib.h"
#include "dac.h"

void DAC_Channel2_Init(void) {
    
    // Enables the clock for the DAC clock.
    RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;

    // Disable DAC.
    DAC->CR &= ~(DAC_CR_EN1 | DAC_CR_EN2);

    // Enables normal mode.
    DAC->MCR &= ~DAC_MCR_MODE2;

    // Enables the trigger for DAC channel 2.
    DAC->CR |= DAC_CR_TEN2;

    // Select the software trigger.
    DAC->CR |= DAC_CR_TSEL2;

    // Enable the DAC channel 2.
    DAC->CR |= DAC_CR_EN2; 

    // Enable the clock of GPIO port A.
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // Set PA5's I/O mode as analog.
    GPIOA->MODER |= GPIO_MODER_MODE5; // Set the mode as analog (11)
}


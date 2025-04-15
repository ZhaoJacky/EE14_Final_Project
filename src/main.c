#include "stm32l432xx.h"
#include "ee14lib.h"

void set_digit(volatile int digit);
void initialize_button();

// Number of milliseconds that have passed since the program started.
volatile int interrupt_num = 0;
volatile int counter = 0;

// This function MUST be named SysTick_Handler for the CMSIS framework
// code to link to it correctly.
void  SysTick_Handler(void) {
    interrupt_num += 1;
}

// Delays the program by the specificed number of milliseconds.
void delay_ms(volatile int delay) {
    int delay_time = interrupt_num + delay;
    while(delay_time > interrupt_num);
}

void SysTick_initialize(void) {
    
    // This line disables the SysTick Timer by setting the SysTick control
    // and status register to 0.
    SysTick->CTRL = 0;

    // This line makes it so that an interrupt is generated after a specific
    // number of clock cycles. The counter starts counting down from 3999 to
    // 0, then generates an interrupt, so an interrupt is generated every
    // 4000 clock cycles.
    SysTick->LOAD = 3999;

    // This sets the priority of the interrupt to 15 (2^4 - 1), which is the
    // largest supported value (aka lowest priority)
    NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);

    // This line sets the value of the SysTick counter after each clock cycle.
    // When the program starts, VAL is set to the reload value, and counts down
    // from there.
    SysTick->VAL = 0;

    // Changes the bit associated with CLCKSOURCE in the control and status
    // register to 1, which selects Processor clock (AHB).
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    // Changes the bit associated with TICKINT in the control and status
    // register to 1, which makes it so counting down to 0 asserts the
    // SysTick exception request.
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    // This line enables the SysTick counter.
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

}

// Truth table for the segments 
bool digit_arr[10][7] = {{0, 0, 0, 0, 0, 0, 1}, // 0
                         {1, 0, 0, 1, 1, 1, 1}, // 1
                         {0, 0, 1, 0, 0, 1, 0}, // 2
                         {0, 0, 0, 0, 1, 1, 0}, // 3
                         {1, 0, 0, 1, 1, 0, 0}, // 4
                         {0, 1, 0, 0, 1, 0, 0}, // 5
                         {0, 1, 0, 0, 0, 0, 0}, // 6
                         {0, 0, 0, 1, 1, 1, 1}, // 7
                         {0, 0, 0, 0, 0, 0, 0}, // 8
                         {0, 0, 0, 1, 1, 0, 0}}; // 9

// Sets the output pins for a specific digit.
void set_digit(int digit) {
    gpio_write(D10, digit_arr[digit][0]); // Segment A
    gpio_write(A2, digit_arr[digit][1]); // Segment B
    gpio_write(D2, digit_arr[digit][2]); // Segment C
    gpio_write(D5, digit_arr[digit][3]); // Segment D
    gpio_write(D3, digit_arr[digit][4]); // Segment E
    gpio_write(D6, digit_arr[digit][5]); // Segment F
    gpio_write(A1, digit_arr[digit][6]); // Segment G

}

// Initializes the button to a pull-up button and set it to input mode.
void initialize_button() {
    gpio_config_pullup(A6, 0b01);
    gpio_config_mode(A6, 0b00);
}

void config_gpio_interrupt(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI7); // Specifies which ports lines are we using to generate interrupts.
    EXTI->FTSR1 |= (EXTI_FTSR1_FT7); // Specifies the conditions to generate an interrupt.
    EXTI->IMR1 |= (EXTI_IMR1_IM7); // Allows the specific pin to be able to generate an interrupt.
    NVIC_SetPriority(EXTI9_5_IRQn, 2);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EXTI9_5_IRQHandler() {
    gpio_write(D1, 1);
    if(EXTI->PR1 & EXTI_PR1_PIF7) {
        counter++;
    } 
    EXTI->PR1 |= EXTI_PR1_PIF7;
    delay_ms(1000);
    gpio_write(D1, 0);
}

int main()
{
    // Take a look in gpio.c for the new GPIO functions which should make your
    // life a little easier for configuring/reading/writing GPIO pins.

    SysTick_initialize();
    
    // Sets the pins connected to the digit anodes to output mode.
    // gpio_config_direction(D9, 0b01); // Ten's Place
    // gpio_config_direction(D4, 0b01); // One's Place

    // Sets the pins connected to the segments to output mode.
    // gpio_config_direction(D2, 0b01);
    // gpio_config_direction(D3, 0b01);
    // gpio_config_direction(D5, 0b01);
    // gpio_config_direction(D6, 0b01);
    // gpio_config_direction(A1, 0b01);
    // gpio_config_direction(A2, 0b01);
    // gpio_config_direction(D10, 0b01);
    config_gpio_interrupt();
    initialize_button();
    gpio_config_mode(D1, 0b01);
    
    
    while(1) {
        
        // if(GPIOA->ODR & GPIO_ODR_OD9) {
        //     gpio_write(D1, 0);
        // } 

        // set_digit(ones_place);
        // gpio_write(D9, 0);
        // gpio_write(D4, 1);
        // delay_ms(5);

        // set_digit(tens_place);
        // gpio_write(D9, 1);
        // gpio_write(D4, 0);
        // delay_ms(5);
    }

    // Good luck!
}


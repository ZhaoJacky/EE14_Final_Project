// #include "stm32l432xx.h"
// #include "ee14lib.h"

// volatile unsigned int count = 0;
// volatile unsigned int ex_count1 = 0; //A0  //PA0
// volatile unsigned int ex_count2 = 0; //D2  //PA12
// volatile unsigned int ex_count3 = 0; //A1  //PA1
// volatile unsigned int ex_count4 = 0; //D5  //PB6
// volatile unsigned int ex_count5 = 0; //D9  //PA8
// volatile unsigned int ex_count6 = 0; //D10 //PA11
// volatile unsigned int ex_count7 = 0; //D11 //PB5
// volatile unsigned int ex_count8 = 0; //D12 //PB4

// void  SysTick_Handler(void) {
//     count++;
// }

// //button 1
// void EXTI0_IRQHandler (void) {
//     if (EXTI->PR1 & EXTI_PR1_PIF0) { //A0  //PA0
//         ex_count1++;  
//         EXTI->PR1 = EXTI_PR1_PIF0;   //   clear bits(by writing a 1)
//     }
// }

// //button 3
// void EXTI1_IRQHandler (void) {
//     if (EXTI->PR1 & EXTI_PR1_PIF1) { //A1  //PA1
//         ex_count3++;  
//         EXTI->PR1 = EXTI_PR1_PIF1;   //   clear bits(by writing a 1)
//     }
// }

// //button 2 and 6
// void EXTI15_10_IRQHandler (void) {
//     if (EXTI->PR1 & EXTI_PR1_PIF11) { //D10 //PA11
//         ex_count6++;  
//         EXTI->PR1 = EXTI_PR1_PIF11;   //   clear bits(by writing a 1)
//     } else if (EXTI->PR1 & EXTI_PR1_PIF12) { //D2  //PA12
//         ex_count2++;  
//         EXTI->PR1 = EXTI_PR1_PIF12;   //   clear bits(by writing a 1)
//     }
// }

// //button 4, 5 and 7
// void EXTI9_5_IRQHandler (void) {
//     if (EXTI->PR1 & EXTI_PR1_PIF5) { //D11 //PB5
//         ex_count7++;  
//         EXTI->PR1 = EXTI_PR1_PIF5;   //   clear bits(by writing a 1)
//     } else if (EXTI->PR1 & EXTI_PR1_PIF8) {
//         ex_count5++;  
//         EXTI->PR1 = EXTI_PR1_PIF8;   //   clear bits(by writing a 1)
//     } else if (EXTI->PR1 & EXTI_PR1_PIF6) { //D5  //PB6
//         ex_count4++;  
//         EXTI->PR1 = EXTI_PR1_PIF6;   //   clear bits(by writing a 1)
//     }
// }

// //button 8
// void EXTI4_IRQHandler (void) { //D12 //PB4
//     if (EXTI->PR1 & EXTI_PR1_PIF4) {
//         ex_count8++;  
//         EXTI->PR1 = EXTI_PR1_PIF4;   //   clear bits(by writing a 1)
//     }
// }

// //  test this function by wrting some code (turning LED on/off perhaps)
// void delay_ms(int ms) {
//     //record current millisecond count
//     volatile int enter_count = count;
//     //add the delay
//     volatile int delay_count = enter_count + ms;
//     //spin in loop itil count exceeds the number
//     while (count <= delay_count) {}
// }

// void SysTick_initialize(void) {
//     SysTick->CTRL = 0;
//     SysTick->LOAD = 3999; //reload value register
//     // This sets the priority of the interrupt to 15 (2^4 - 1), which is the
//     // largest supported value (aka lowest priority)
//     NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
//     SysTick->VAL = 0;//current value register
//     //Control and status and 
//     SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;//indicates the clock source
//     SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; //enable interrupt request
//     SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //enable the counter or not
// }

// void button1_config(void) { //A0  //PA0
//     RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

//     SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; // Clear any existing mapping for EXTI0

//     EXTI->FTSR1 |= EXTI_FTSR1_FT0;

//     EXTI->IMR1 |= EXTI_IMR1_IM0;

//     NVIC_SetPriority(EXTI0_IRQn, 2);  // Set the priority of EXTI line 0 interrupt (priority 2)
//     NVIC_EnableIRQ(EXTI0_IRQn); // Enable the EXTI0 interrupt in the NVIC
// }

// void button2_config(void){ //D2  //PA12
//     RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

//     SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI12; // Clear any existing mapping for EXTI0

//     EXTI->FTSR1 |= EXTI_FTSR1_FT12;

//     EXTI->IMR1 |= EXTI_IMR1_IM12;

//     NVIC_SetPriority(EXTI15_10_IRQn, 2);  // Set the priority of EXTI line 0 interrupt (priority 2)
//     NVIC_EnableIRQ(EXTI15_10_IRQn); // Enable the EXTI0 interrupt in the NVIC
// }

// void button3_config(void) { //A1  //PA1
//     RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

//     SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1; // Clear any existing mapping for EXTI0

//     EXTI->FTSR1 |= EXTI_FTSR1_FT1;

//     EXTI->IMR1 |= EXTI_IMR1_IM1;

//     NVIC_SetPriority(EXTI1_IRQn, 2);  // Set the priority of EXTI line 0 interrupt (priority 2)
//     NVIC_EnableIRQ(EXTI1_IRQn); // Enable the EXTI0 interrupt in the NVIC
// }

// void button4_config(void) { //D5  //PB6
//     RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
//     RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; //if use PB

//     SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI6; // Clear any existing mapping for EXTI0
//     SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PB; //if use PB

//     EXTI->FTSR1 |= EXTI_FTSR1_FT6;

//     EXTI->IMR1 |= EXTI_IMR1_IM6;

//     NVIC_SetPriority(EXTI9_5_IRQn, 2);  // Set the priority of EXTI line 0 interrupt (priority 2)
//     NVIC_EnableIRQ(EXTI9_5_IRQn); // Enable the EXTI0 interrupt in the NVIC
// }

// void button5_config(void) { //D9  //PA8
//     RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

//     SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI8; // Clear any existing mapping for EXTI0

//     EXTI->FTSR1 |= EXTI_FTSR1_FT8;

//     EXTI->IMR1 |= EXTI_IMR1_IM8;

//     NVIC_SetPriority(EXTI9_5_IRQn, 2);  // Set the priority of EXTI line 0 interrupt (priority 2)
//     NVIC_EnableIRQ(EXTI9_5_IRQn); // Enable the EXTI0 interrupt in the NVIC
// }

// void button6_config(void) { //D10 //PA11
//     RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

//     SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI11; // Clear any existing mapping for EXTI0

//     EXTI->FTSR1 |= EXTI_FTSR1_FT11;

//     EXTI->IMR1 |= EXTI_IMR1_IM11;

//     NVIC_SetPriority(EXTI15_10_IRQn, 2);  // Set the priority of EXTI line 0 interrupt (priority 2)
//     NVIC_EnableIRQ(EXTI15_10_IRQn); // Enable the EXTI0 interrupt in the NVIC
// }

// void button7_config(void) { //D11 //PB5
//     RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
//     RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; //if use PB

//     SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI5; // Clear any existing mapping for EXTI0
//     SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PB; //if use PB

//     EXTI->FTSR1 |= EXTI_FTSR1_FT5;

//     EXTI->IMR1 |= EXTI_IMR1_IM5;

//     NVIC_SetPriority(EXTI9_5_IRQn, 2);  // Set the priority of EXTI line 0 interrupt (priority 2)
//     NVIC_EnableIRQ(EXTI9_5_IRQn); // Enable the EXTI0 interrupt in the NVIC
// }

// void button8_config(void) { //D12 //PB4
//     RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
//     RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; //if use PB

//     SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI4; // Clear any existing mapping for EXTI0
//     SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB; //if use PB

//     EXTI->FTSR1 |= EXTI_FTSR1_FT4;

//     EXTI->IMR1 |= EXTI_IMR1_IM4;

//     NVIC_SetPriority(EXTI4_IRQn, 2);  // Set the priority of EXTI line 0 interrupt (priority 2)
//     NVIC_EnableIRQ(EXTI4_IRQn); // Enable the EXTI0 interrupt in the NVIC
// }

// void setup() {
//     SysTick_initialize();
//     config_gpio_interrupt();
    
//     gpio_config_direction(A0, 0b00);   //button is input
//     gpio_config_pullup(A0, 0b01); 
//     gpio_config_direction(D1, 0b01);  
// }

// int main()
// {
//     SysTick_initialize();
    
//     button8_config();
//     gpio_config_mode(D12, 0b00);   //button A1 
//     gpio_config_pullup(D12, 0b01); 

//     // button2_config();
//     // gpio_config_direction(A0, 0b00);   //button A0 
//     // gpio_config_pullup(A0, 0b01); 

//     gpio_config_mode(D1, 0b01);   //LED

//     while(1) {            
//         if(ex_count8 % 2 == 1) {
//             gpio_write(D1, 1);
//             delay_ms(100);
//         } else {
//             gpio_write(D1, 0);
//         }
//     }

//     //STEP 1: Config all pins

//     //STEP 2: If a button is pressed, the note corresponding to that button is played
//     //        and diplay show the note name

//     //STEP 3: DAC
// }
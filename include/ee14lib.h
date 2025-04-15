/* Header file for all EE14lib functions. */

#ifndef EE14LIB_H
#define EE14LIB_H

#include "stm32l432xx.h"
#include <stdbool.h>

// Pin names on the Nucleo silkscreen, copied from the Arduino Nano form factor
typedef enum {A0,A1,A2,A3,A4,A5,A6,A7,
	  D0,D1,D2,D3,D4,D5,D6,D7,D8,D9,D10,D11,D12,D13 } EE14Lib_Pin ;

typedef int EE14Lib_Err;

#define EE14Lib_Err_OK 0
#define EE14Lib_Err_INEXPLICABLE_FAILURE -1
#define EE14Lib_Err_NOT_IMPLEMENTED -2
#define EE14Lib_ERR_INVALID_CONFIG -3

// GPIO modes
#define INPUT 0b00
#define OUTPUT 0b01

// GPIO pullup modes
#define PULL_OFF 0b00
#define PULL_UP 0b01
#define PULL_DOWN 0b10
// Both on is an error

EE14Lib_Err gpio_config_direction(EE14Lib_Pin pin, unsigned int direction);
EE14Lib_Err gpio_config_pullup(EE14Lib_Pin pin, unsigned int mode);
EE14Lib_Err gpio_config_alternate_function(EE14Lib_Pin pin, unsigned int function);
void gpio_write(EE14Lib_Pin pin, bool value);
bool gpio_read(EE14Lib_Pin pin);

EE14Lib_Err timer_config_pwm(TIM_TypeDef* const timer, const unsigned int freq_hz);
EE14Lib_Err timer_config_channel_pwm(TIM_TypeDef* const timer, const EE14Lib_Pin pin, const unsigned int duty);

#endif
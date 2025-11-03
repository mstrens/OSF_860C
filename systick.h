#pragma once

#include "cybsp.h"
#include "SEGGER_RTT.h"
#include "main.h"

extern volatile uint32_t ui32_ms_counter;
// new wheel and cadence variables
// =============== VARIABLES PARTAGÉES =============== 
extern volatile uint32_t ui32_pwm_ticks;          // compteur soft 19kHz
extern volatile uint32_t ui32_cadence_last_ticks[6];   // timestamps pédalage (codes 0..5)
extern volatile uint32_t ui32_wheel_last_pwm_ticks; // dernier front roue (ui32_pwm_ticks)

extern uint8_t ui8_pas_counter; // counter to detect a full pedal rotation (after 20 valid transitions)


void SysTick_Handler(void) ;


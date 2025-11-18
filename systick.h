#pragma once

#include "cybsp.h"
#include "SEGGER_RTT.h"
#include "main.h"
#include "common.h"

extern volatile uint32_t ui32_ms_counter;
// new wheel and cadence variables
// =============== VARIABLES PARTAGÉES =============== 
extern volatile uint32_t ui32_pwm_ticks;          // compteur soft 19kHz
extern volatile uint32_t ui32_cadence_last_ticks[6];   // timestamps pédalage (codes 0..5)
extern volatile uint32_t ui32_wheel_last_pwm_ticks; // dernier front roue (ui32_pwm_ticks)

extern uint8_t ui8_pas_counter; // counter to detect a full pedal rotation (after 20 valid transitions)
extern uint16_t ui16_lead_total_q8_8; // sum of lead base (from a table based on velocity) and correction (based on Id)

void SysTick_Handler(void) ;
void update_lead_angle(void);
void update_duty_cycle(void);
void systick_security_checks(void);



#define PHASE_PEAK_TRIP2 ((uint32_t)(PHASE_PEAK_ADC_NOMINAL * PHASE_PEAK_ADC_NOMINAL * PHASE_PEAK_TRIP_RATIO * PHASE_PEAK_TRIP_RATIO) )  // I^2
#define PHASE_RMS_WARN2  ((uint32_t)(PHASE_PEAK_ADC_NOMINAL * PHASE_PEAK_ADC_NOMINAL * PHASE_RMS_WARN_RATIO * PHASE_RMS_WARN_RATIO))
#define IMOTOR_RMS_WARN2 ((uint32_t)(PHASE_PEAK_ADC_NOMINAL * PHASE_PEAK_ADC_NOMINAL * IMOTOR_RMS_WARN_RATIO * IMOTOR_RMS_WARN_RATIO * 3.0 )) // *3 because motor is sum of the 3 phase

#define IDC_FAST_TRIP         (((uint16_t)(IDC_NOMINAL_AMPERE * IDC_FAST_TRIP_RATIO * 100.0 / 16.0)) << 5)     // 30A = 180 ADC 10 bit; here in 15 bits for IDC
#define IDC_SLOW_WARN         (((uint16_t)(IDC_NOMINAL_AMPERE * IDC_SLOW_WARN_RATIO * 100.0 / 16.0)) << 5)    // 30A = 180 ADC 10 bit; here in 15 bits for IDC


#define PHASE_RMS_ALPHA       8         // pour IIR: alpha = 1/256



#pragma once

#include "cybsp.h"
#include "SEGGER_RTT.h"
#include "main.h"



// copied from TSDZ2
// motor states
#define BLOCK_COMMUTATION 			            0
#define SINEWAVE_INTERPOLATION_60_DEGREES 	    0x80

// power variables
extern volatile uint8_t ui8_controller_duty_cycle_ramp_up_inverse_step;
extern volatile uint8_t ui8_controller_duty_cycle_ramp_down_inverse_step;
extern volatile uint16_t ui16_adc_voltage_cut_off;
extern volatile uint8_t ui8_adc_battery_current_filtered;
extern volatile uint8_t ui8_controller_adc_battery_current_target;
extern volatile uint8_t ui8_g_duty_cycle;
extern volatile uint8_t ui8_fw_hall_counter_offset;
extern volatile uint8_t ui8_fw_hall_counter_offset_max;
extern volatile uint8_t ui8_field_weakening_enabled;
extern volatile uint16_t ui16_hall_counter_total;    
extern volatile uint8_t ui8_controller_duty_cycle_target;
extern volatile uint16_t ui16_hall_calib_cnt[6];
extern uint8_t ui8_hall_ref_angles[8];  // was 6 in tsdz2
extern const uint8_t ui8_hall_counter_offsets[8]; // was 6 in tsdz2
extern volatile uint8_t ui8_hall_sensors_state;

// Sensors
extern volatile uint8_t ui8_brake_state;
extern volatile uint16_t ui16_adc_voltage;
extern volatile uint16_t ui16_adc_torque;
//extern volatile uint16_t ui16_adc_throttle; // mstrens : moved to ebike_app.c

extern volatile uint16_t ui16_adc_torque_filtered  ; 
extern volatile uint16_t ui16_adc_torque_actual_rotation ;
extern volatile uint16_t ui16_adc_torque_previous_rotation ;
extern volatile uint8_t ui8_adc_torque_rotation_reset ;

extern volatile uint8_t ui8_pas_new_transition;

// cadence sensor
extern volatile uint16_t ui16_cadence_sensor_ticks;

// wheel speed sensor
extern volatile uint16_t ui16_wheel_speed_sensor_ticks;
extern volatile uint32_t ui32_wheel_speed_sensor_ticks_total;

// battery soc
extern volatile uint8_t ui8_battery_SOC_saved_flag;
extern volatile uint8_t ui8_battery_SOC_reset_flag;

// end of code copied from TSDZ2

// added by ms because used in ebike_app.c
extern volatile uint8_t ui8_g_foc_angle;
extern uint8_t ui8_foc_angle_multiplicator;

// added by mstrens because defined in ebike_app.c and used in motor.c
extern uint8_t ui8_adc_battery_overcurrent;

// added by mstrens to debug

#if ( GENERATE_DATA_FOR_REGRESSION_ANGLES == (1) )
extern uint16_t ticks_intervals[8]; // ticks intervals between 2 pattern changes;
extern uint8_t ticks_intervals_status; // 0 =  new data can be written; 1 data being written; 2 all data written, must be transmitted
extern uint16_t previous_hall_pattern_change_ticks;  // save the ticks of last pattern change
#endif

void CCU80_0_IRQHandler(); // called when ccu8 Slice 4 reaches 840  counting UP (= 1/4 of 19mhz cycles)
void CCU80_1_IRQHandler(); // called when ccu8 Slice 4 reaches 840  counting DOWN (= 1/4 of 19mhz cycles)
void POSIF0_1_IRQHandler(); // called when posif generate a SR 1 ( used currently to debug)

uint32_t getHallPosition(void);
void posif_init_position();
void update_shadow_pattern(uint8_t current_pattern);

void motor_enable_pwm(void) ;
void motor_disable_pwm(void) ;

void get_hall_pattern();

void set_rotor_angle( uint8_t angle, uint8_t duty_cycle);

void check_current_during(uint32_t during_ms, uint16_t max_A);

void log_hall_sensor_position();

uint16_t get_current_adc_10bits();

uint32_t calculate_average_angle(uint8_t pattern);

void update_foc_pid();

void update_foc_optimiser(void);

//__RAM_FUNC static inline void calculate_id_part1();

//__RAM_FUNC static inline void calculate_id_part2();
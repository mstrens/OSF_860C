#pragma once


#include "ebike_app.h"

#ifdef USE_TSDZ8_VLCD5_VERSION

extern volatile struct_configuration_variables m_configuration_variables;

// display menu
extern uint8_t ui8_assist_level;
extern uint8_t ui8_assist_level_temp ;
extern uint8_t ui8_assist_level_01_flag ;
extern uint8_t ui8_riding_mode_temp ;
extern uint8_t ui8_lights_flag ;
extern uint8_t ui8_lights_on_5s;
extern uint8_t ui8_menu_flag ;
extern uint8_t ui8_menu_index;
extern uint8_t ui8_data_index;
extern uint8_t ui8_menu_counter;
extern uint8_t ui8_display_function_code;
extern uint8_t ui8_display_function_code_temp;
extern uint8_t ui8_menu_function_enabled;
extern uint8_t ui8_display_data_enabled;
extern uint16_t ui16_display_data;
extern uint16_t ui16_data_value;
extern uint8_t ui8_auto_display_data_flag;
extern uint8_t ui8_auto_display_data_status;
extern uint8_t ui8_auto_data_number_display;
extern uint16_t ui16_display_data_factor;
extern uint8_t ui8_delay_display_function;
extern uint8_t ui8_display_data_on_startup;
extern uint8_t ui8_set_parameter_enabled_temp;
extern uint8_t ui8_auto_display_data_enabled_temp;
extern uint8_t ui8_street_mode_enabled_temp;
extern uint8_t ui8_torque_sensor_adv_enabled_temp;
extern uint8_t ui8_assist_without_pedal_rotation_temp;
extern uint8_t ui8_walk_assist_enabled_array[2];
extern uint8_t ui8_display_battery_soc;
extern uint8_t ui8_display_riding_mode;
extern uint8_t ui8_display_lights_configuration;
extern uint8_t ui8_display_alternative_lights_configuration;
extern uint8_t ui8_display_torque_sensor_flag_1;
extern uint8_t ui8_display_torque_sensor_flag_2;
extern uint8_t ui8_display_torque_sensor_value;
extern uint8_t ui8_display_torque_sensor_step;
extern uint8_t ui8_display_function_status[3][5];
extern uint8_t ui8_lights_configuration_2;
extern uint8_t ui8_lights_configuration_3;
extern uint8_t ui8_lights_configuration_temp;

// system
extern uint8_t ui8_riding_mode_parameter;
extern volatile uint8_t ui8_system_state;
extern volatile uint8_t ui8_motor_enabled;
extern uint8_t ui8_assist_without_pedal_rotation_threshold;
extern uint8_t ui8_lights_state;
extern uint8_t ui8_lights_button_flag;
extern uint8_t ui8_field_weakening_erps_delta;
//extern uint8_t ui8_optional_ADC_function;
extern uint8_t ui8_walk_assist_level;

// battery
extern uint16_t ui16_battery_voltage_filtered_x10;
extern uint16_t ui16_battery_voltage_calibrated_x10;
extern volatile uint16_t ui16_battery_voltage_soc_filtered_x10;
extern uint16_t ui16_battery_power_x10;															  
extern uint16_t ui16_battery_power_filtered_x10;
extern uint16_t ui16_actual_battery_capacity;
extern uint32_t ui32_wh_x10;
extern uint32_t ui32_wh_sum_x10;
extern volatile uint32_t ui32_wh_x10_offset;
extern uint32_t ui32_wh_since_power_on_x10;
extern volatile uint16_t ui16_battery_SOC_percentage_x10;
extern volatile uint8_t ui8_battery_SOC_init_flag;
extern uint8_t ui8_battery_state_of_charge;

// power control
extern uint8_t ui8_duty_cycle_ramp_up_inverse_step;
extern uint8_t ui8_duty_cycle_ramp_up_inverse_step_default;
extern uint8_t ui8_duty_cycle_ramp_down_inverse_step;
extern uint8_t ui8_duty_cycle_ramp_down_inverse_step_default;
extern uint16_t ui16_battery_voltage_filtered_x1000;
extern uint16_t ui16_battery_no_load_voltage_filtered_x10;
extern uint8_t ui8_battery_current_filtered_x10;
extern uint8_t ui8_adc_battery_current_max; // 112 = 18A
extern uint8_t ui8_adc_battery_current_target;
extern uint8_t ui8_duty_cycle_target;
extern uint16_t ui16_duty_cycle_percent;
extern volatile uint8_t ui8_adc_motor_phase_current_max; // 187
extern uint8_t ui8_error_battery_overcurrent;
extern uint8_t ui8_adc_battery_overcurrent; //112 + 50
extern uint8_t ui8_adc_battery_current_max_temp_1;
extern uint8_t ui8_adc_battery_current_max_temp_2;
extern uint32_t ui32_adc_battery_power_max_x1000_array[2];

// Motor ERPS
extern uint16_t ui16_motor_speed_erps;

// cadence sensor
extern uint16_t ui16_cadence_ticks_count_min_speed_adj;
extern uint8_t ui8_pedal_cadence_RPM;
extern uint8_t ui8_motor_deceleration;

// torque sensor
extern uint8_t ui8_pedal_torque_per_10_bit_ADC_step_x100;
extern uint8_t ui8_pedal_torque_per_10_bit_ADC_step_calc_x100;
extern uint16_t ui16_adc_pedal_torque_offset;
extern uint16_t ui16_adc_pedal_torque_offset_init;
extern uint16_t ui16_adc_pedal_torque_offset_cal;
extern uint16_t ui16_adc_pedal_torque_offset_min;
extern uint16_t ui16_adc_pedal_torque_offset_max;
extern uint8_t ui8_adc_pedal_torque_offset_error;
extern uint16_t ui16_adc_coaster_brake_threshold;
extern uint16_t ui16_adc_pedal_torque;
extern uint16_t ui16_adc_pedal_torque_delta;
extern uint16_t ui16_adc_pedal_torque_delta_temp;
extern uint16_t ui16_adc_pedal_torque_delta_no_boost;
extern uint16_t ui16_pedal_torque_x100;
extern uint16_t ui16_human_power_x10;
extern uint16_t ui16_human_power_filtered_x10;
extern uint8_t ui8_torque_sensor_calibrated;
extern uint16_t ui16_pedal_weight_x100;
extern uint16_t ui16_pedal_torque_step_temp;
extern uint8_t ui8_torque_sensor_calibration_flag;
extern uint8_t ui8_torque_sensor_calibration_started;
extern uint8_t ui8_pedal_torque_per_10_bit_ADC_step_x100_array[2];
extern uint8_t ui8_eMTB_based_on_power;

// wheel speed sensor
extern uint16_t ui16_wheel_speed_x10;
extern uint8_t ui8_wheel_speed_max_array[2];

// wheel speed display
extern uint8_t ui8_display_ready_flag;
extern uint8_t ui8_startup_counter;
extern uint16_t ui16_oem_wheel_speed_time;
extern uint8_t ui8_oem_wheel_diameter;
extern uint32_t ui32_odometer_compensation_mm;

// throttle control
extern uint8_t ui8_adc_throttle_assist;
extern uint8_t ui8_throttle_adc_in;
extern uint8_t ui8_throttle_mode_array[2];

// cruise control
extern uint8_t ui8_cruise_threshold_speed_x10_array[2];
extern uint8_t ui8_cruise_button_flag;

// walk assist
extern uint8_t ui8_walk_assist_flag;
extern uint8_t ui8_walk_assist_speed_target_x10;
extern uint8_t ui8_walk_assist_duty_cycle_counter;
extern uint8_t ui8_walk_assist_duty_cycle_target;
extern uint8_t ui8_walk_assist_duty_cycle_max;
extern uint8_t ui8_walk_assist_adj_delay;
extern uint16_t ui16_walk_assist_wheel_speed_counter;
extern uint16_t ui16_walk_assist_erps_target;
extern uint16_t ui16_walk_assist_erps_min;
extern uint16_t ui16_walk_assist_erps_max;
extern uint8_t ui8_walk_assist_speed_flag;

// startup boost
extern uint8_t ui8_startup_boost_at_zero;
extern uint8_t ui8_startup_boost_flag;
extern uint8_t ui8_startup_boost_enabled_temp;
extern uint16_t ui16_startup_boost_factor_array[120];

// smooth start
extern uint8_t ui8_smooth_start_flag;
extern uint8_t ui8_smooth_start_counter;
extern uint8_t ui8_smooth_start_counter_set;

// startup assist
extern uint8_t ui8_startup_assist_flag;
extern uint8_t ui8_startup_assist_adc_battery_current_target;

// motor temperature control
extern uint16_t ui16_adc_motor_temperature_filtered;
extern uint16_t ui16_motor_temperature_filtered_x10;
extern uint8_t ui8_motor_temperature_max_value_to_limit_array[2];
extern uint8_t ui8_motor_temperature_min_value_to_limit_array[2];


// array for oem display
extern uint8_t ui8_data_index_array[];
extern uint8_t ui8_delay_display_array[];

// array for riding parameters
extern uint8_t  ui8_riding_mode_parameter_array[8][5];

extern uint8_t ui8_default_flash_state;

// ADC Values from motor.c
extern volatile uint16_t ui16_adc_voltage;
extern volatile uint16_t ui16_adc_torque;
extern volatile uint16_t ui16_adc_throttle;

// power from motor.c
extern volatile uint8_t ui8_g_duty_cycle;

extern uint8_t ui8_pwm_duty_cycle_max;



void fillRxBuffer();
void uart_receive_package();
void uart_send_package();

#endif
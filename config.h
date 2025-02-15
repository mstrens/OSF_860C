/*
 * config.h
 *
 *  Automatically created by TSDS2 Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "cybsp.h"
#include "cy_utils.h"

#ifdef USE_CONFIG
typedef struct _config
{
    uint16_t main_version;
    uint16_t sub_version;
    uint16_t reserve_1;
    uint16_t reserve_2;
    uint16_t reserve_3;
    uint16_t reserve_4;
    uint16_t reserve_5;
    uint16_t reserve_6;
    uint16_t reserve_7;
    uint16_t reserve_8;
    uint16_t motor_type;
    uint8_t torque_sensor_calibrated;
    uint8_t motor_acceleration;
    uint8_t motor_assistance_without_pedal_rotation;
    uint8_t assistance_without_pedal_rotation_threshold;
     uint8_t  pedal_torque_per_10_bit_adc_step_x100;
    uint16_t pedal_torque_adc_max;
    uint16_t startup_boost_torque_factor;
     uint8_t  motor_blocked_counter_threshold;
     uint8_t  motor_blocked_battery_current_threshold_x10;
     uint8_t  motor_blocked_erps_threshold;
     uint8_t  startup_boost_cadence_step;
     uint8_t  battery_current_max;
    uint16_t target_max_battery_power;
    uint16_t target_max_battery_capacity;
     uint8_t  battery_cells_number;
     uint8_t  motor_deceleration;
     uint8_t  battery_low_voltage_cut_off;
     uint8_t  actual_battery_voltage_percent;
     uint8_t  actual_battery_capacity_percent;
    float li_ion_cell_overvolt;
    float li_ion_cell_reset_soc_percent;
    float li_ion_cell_volts_full;
    float li_ion_cell_volts_3_of_4;
    float li_ion_cell_volts_2_of_4;
    float li_ion_cell_volts_1_of_4;
    float li_ion_cell_volts_5_of_6;
    float li_ion_cell_volts_4_of_6;
    float li_ion_cell_volts_3_of_6;
    float li_ion_cell_volts_2_of_6;
    float li_ion_cell_volts_1_of_6;
    float li_ion_cell_volts_empty;
    uint16_t wheel_perimeter;
    uint8_t  wheel_max_speed;
    uint8_t  enable_lights;
    uint8_t  enable_walk_assist;
    uint8_t  enable_brake_sensor;
    uint8_t  enable_throttle;
    uint8_t  enable_temperature_limit;
    uint8_t  enable_street_mode_on_startup;
    uint8_t  enable_set_parameter_on_startup;
    uint8_t  enable_odometer_compensation;
    uint8_t  startup_boost_on_startup;
    uint8_t  torque_sensor_adv_on_startup;
    uint8_t  lights_configuration_on_startup;
    uint8_t  riding_mode_on_startup;
    uint8_t  lights_configuration_1;
    uint8_t  lights_configuration_2;
    uint8_t  lights_configuration_3;
    uint8_t  street_mode_power_limit_enabled;
    uint16_t street_mode_power_limit;
    uint8_t  street_mode_speed_limit;
    uint8_t  street_mode_throttle_enabled;
    uint8_t  street_mode_cruise_enabled;
    uint8_t  adc_throttle_min_value;
    uint8_t  adc_throttle_max_value;
    uint8_t  motor_temperature_min_value_limit;
    uint8_t  motor_temperature_max_value_limit;
    uint8_t  enable_temperature_error_min_limit;
    uint8_t  enable_vlcd6;
    uint8_t  enable_vlcd5;
    uint8_t  enable_xh18;
    uint8_t  enable_display_working_flag;
    uint8_t  enable_display_always_on;
    uint8_t  enable_wheel_max_speed_from_display;
    uint8_t  delay_menu_on;
    uint8_t  coaster_brake_enabled;
    uint8_t  coaster_brake_torque_threshold;
    uint8_t  enable_auto_data_display;
    uint8_t  startup_assist_enabled;
    uint8_t  auto_data_number_display;
    uint8_t  delay_display_data_1;
    uint8_t  delay_display_data_2;
    uint8_t  delay_display_data_3;
    uint8_t  delay_display_data_4;
    uint8_t  delay_display_data_5;
    uint8_t  delay_display_data_6;
    uint8_t  display_data_1;
    uint8_t  display_data_2;
    uint8_t  display_data_3;
    uint8_t  display_data_4;
    uint8_t  display_data_5;
    uint8_t  display_data_6;
    uint16_t power_assist_level_1;
    uint16_t power_assist_level_2;
    uint16_t power_assist_level_3;
    uint16_t power_assist_level_4;
    uint8_t  torque_assist_level_1;
    uint8_t  torque_assist_level_2;
    uint8_t  torque_assist_level_3;
    uint8_t  torque_assist_level_4;
    uint8_t  cadence_assist_level_1;
    uint8_t  cadence_assist_level_2;
    uint8_t  cadence_assist_level_3;
    uint8_t  cadence_assist_level_4;
    uint8_t  emtb_assist_level_1;
    uint8_t  emtb_assist_level_2;
    uint8_t  emtb_assist_level_3;
    uint8_t  emtb_assist_level_4;
    uint8_t  walk_assist_level_1;
    uint8_t  walk_assist_level_2;
    uint8_t  walk_assist_level_3;
    uint8_t  walk_assist_level_4;
    uint8_t  walk_assist_threshold_speed_x10;
    uint8_t  walk_assist_debounce_enabled;
    uint8_t  walk_assist_debounce_time;
    uint8_t  cruise_target_speed_level_1;
    uint8_t  cruise_target_speed_level_2;
    uint8_t  cruise_target_speed_level_3;
    uint8_t  cruise_target_speed_level_4;
    uint8_t  cruise_mode_walk_enabled;
    uint8_t  cruise_threshold_speed;
    uint8_t  pedal_torque_adc_offset;
    uint8_t  units_type;
    uint8_t  assist_throttle_min_value;
    uint8_t  assist_throttle_max_value;
    uint8_t  street_mode_walk_enabled;
    uint8_t  data_display_on_startup;
    uint8_t  field_weakening_enabled;
    uint8_t  pedal_torque_adc_offset_adj;
    uint8_t  pedal_torque_adc_range_adj;
    uint8_t  pedal_torque_adc_angle_adj;
    uint8_t  pedal_torque_per_10_bit_adc_step_adv_x100;
    uint8_t  soc_percent_calc;
    uint8_t  startup_boost_at_zero;
    uint8_t  enablec850;
    uint8_t  street_mode_throttle_legal;
    uint8_t  brake_temperature_switch;
    uint8_t  emtb_based_on_power;
    uint8_t  smooth_start_enabled;
    uint8_t  smooth_start_set_percent;
    uint8_t  temperature_sensor_type;
    uint8_t  cruise_mode_enabled;
    uint8_t  throttle_mode;
    uint8_t  street_mode_throttle_mode;
    uint8_t  assist_level_1_of_5_percent;
    uint8_t  alternative_miles;
} struct_config;



/* Duplicated for easier manual update
#define OFF_MODE                                  0
#define POWER_ASSIST_MODE                         1
#define TORQUE_ASSIST_MODE                        2
#define CADENCE_ASSIST_MODE                       3
#define eMTB_ASSIST_MODE                          4
#define HYBRID_ASSIST_MODE						  5
#define CRUISE_MODE                               6
#define WALK_ASSIST_MODE                          7
#define TORQUE_SENSOR_CALIBRATION_MODE            8								   

// Index of the data too display
ui16_battery_SOC_percentage_x10          1
ui16_battery_voltage_calibrated_x10      2
ui8_battery_current_filtered_x10         3
ui16_battery_power_filtered_x10          4
ui16_adc_throttle                        5
ui16_adc_torque                          6
ui8_pedal_cadence_RPM                    7
ui16_human_power_filtered_x10            8
ui16_adc_pedal_torque_delta              9
ui32_wh_x10                              10
ui16_motor_speed_erps                    11
ui16_duty_cycle_percent                  12
*/

// added by mstrens
#define _MAIN_VERSION 1
#define _SUB_VERSION 1
#define _RESERVE_1   2
#define _RESERVE_2   0
#define _RESERVE_3   0
#define _RESERVE_4   0
#define _RESERVE_5   0
#define _RESERVE_6   0
#define _RESERVE_7   0
#define _RESERVE_8   0

#define _MOTOR_TYPE 0              // 0 = 48V, 1 = 36V ; is used to change the FOC_ANGLE_MULTIPLIER in main.h
#define _TORQUE_SENSOR_CALIBRATED 0
#define _MOTOR_ACCELERATION  35
#define _MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION 0 // when enabled (1), assistance is provided when the pedal is pressed more than the thershold herafer
#define _ASSISTANCE_WITHOUT_PEDAL_ROTATION_THRESHOLD 20
#define _PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100 67    // used to calculate the correct ratio between the assistance factor and the human power + for total power
#define _PEDAL_TORQUE_ADC_MAX 300                    // value from ADC when max weigth is apply on one pedal
#define _STARTUP_BOOST_TORQUE_FACTOR 250
#define _MOTOR_BLOCKED_COUNTER_THRESHOLD 2
#define _MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X10 20  // mstrens : it was 30 for tsdz2, I reduce it for testing
#define _MOTOR_BLOCKED_ERPS_THRESHOLD 10       // electric RPS mstrens it was 20 for tsdz2, TSDZ8 has 4 poles instead of 8, so 2 more ticks for the same speed

#define _STARTUP_BOOST_CADENCE_STEP 25       
#define _BATTERY_CURRENT_MAX        5          // A mstrens: it was 13 for tsdz2, reduce for testing
#define _TARGET_MAX_BATTERY_POWER 500
#define _TARGET_MAX_BATTERY_CAPACITY 500
#define _BATTERY_CELLS_NUMBER 10               // mstrens 10 for 36V battery and 13 for 48V battery
#define _MOTOR_DECELERATION 35
#define _BATTERY_LOW_VOLTAGE_CUT_OFF 30          // Volt mstrens : it was 39, changed to 30V for testing// could use 30 for 36 batt and 39 for 48V battery
#define _ACTUAL_BATTERY_VOLTAGE_PERCENT 100
#define _ACTUAL_BATTERY_CAPACITY_PERCENT 90
#define _LI_ION_CELL_OVERVOLT 4.35
#define _LI_ION_CELL_RESET_SOC_PERCENT 4.10
#define _LI_ION_CELL_VOLTS_FULL 4.10
#define _LI_ION_CELL_VOLTS_3_OF_4 3.85
#define _LI_ION_CELL_VOLTS_2_OF_4 3.60
#define _LI_ION_CELL_VOLTS_1_OF_4 3.35
#define _LI_ION_CELL_VOLTS_5_OF_6 3.94
#define _LI_ION_CELL_VOLTS_4_OF_6 3.76
#define _LI_ION_CELL_VOLTS_3_OF_6 3.60
#define _LI_ION_CELL_VOLTS_2_OF_6 3.44
#define _LI_ION_CELL_VOLTS_1_OF_6 3.26
#define _LI_ION_CELL_VOLTS_EMPTY 3.10
#define _WHEEL_PERIMETER 2200
#define _WHEEL_MAX_SPEED 25         // km/h
#define _ENABLE_LIGHTS 1            
#define _ENABLE_WALK_ASSIST 1       // 1 enable  
#define _ENABLE_BRAKE_SENSOR 1                   
#define _ENABLE_THROTTLE 1          // 1 enable 
#define _ENABLE_TEMPERATURE_LIMIT 0
#define _ENABLE_STREET_MODE_ON_STARTUP 1
#define _ENABLE_SET_PARAMETER_ON_STARTUP 0
#define _ENABLE_ODOMETER_COMPENSATION 0         
#define _STARTUP_BOOST_ON_STARTUP 0
#define _TORQUE_SENSOR_ADV_ON_STARTUP 0
#define _LIGHTS_CONFIGURATION_ON_STARTUP 0
#define _RIDING_MODE_ON_STARTUP 3     // POWER 1, TORQUE 2, CADENCE 3, eMTB 4, HYBRID 5, CRUISE 6, WALK  7

#define _LIGHTS_CONFIGURATION_1 1
#define _LIGHTS_CONFIGURATION_2 9
#define _LIGHTS_CONFIGURATION_3 10
#define _STREET_MODE_POWER_LIMIT_ENABLED 0
#define _STREET_MODE_POWER_LIMIT 250
#define _STREET_MODE_SPEED_LIMIT 25
#define _STREET_MODE_THROTTLE_ENABLED 1    // not used anymore
#define _STREET_MODE_CRUISE_ENABLED 1     
#define _ADC_THROTTLE_MIN_VALUE 55            // It is ADC 8 bits ; For tsdz2, it was 47, for tsdz8 it could be 45; I increased due to some errors
#define _ADC_THROTTLE_MAX_VALUE 176            // It is ADC 8 bits ; For TSDZ2, it was 176, for tsdz8 it could be 180; we keep 176 ; this is mapped to 255
#define _MOTOR_TEMPERATURE_MIN_VALUE_LIMIT 65
#define _MOTOR_TEMPERATURE_MAX_VALUE_LIMIT 95
#define _ENABLE_TEMPERATURE_ERROR_MIN_LIMIT 0
#define _ENABLE_VLCD6 0                    
#define _ENABLE_VLCD5 1                    
#define _ENABLE_XH18 0                     
#define _ENABLE_DISPLAY_WORKING_FLAG 1     
#define _ENABLE_DISPLAY_ALWAYS_ON 0        
#define _ENABLE_WHEEL_MAX_SPEED_FROM_DISPLAY 0 // allow to change the max speed from the display  
#define _DELAY_MENU_ON 50                  
#define _COASTER_BRAKE_ENABLED 0            
#define _COASTER_BRAKE_TORQUE_THRESHOLD 30
#define _ENABLE_AUTO_DATA_DISPLAY 1
#define _STARTUP_ASSIST_ENABLED 0          

#define _AUTO_DATA_NUMBER_DISPLAY 6         // number of data to display in sequence (max 6)
#define _DELAY_DISPLAY_DATA_1 50
#define _DELAY_DISPLAY_DATA_2 50
#define _DELAY_DISPLAY_DATA_3 50
#define _DELAY_DISPLAY_DATA_4 50
#define _DELAY_DISPLAY_DATA_5 50
#define _DELAY_DISPLAY_DATA_6 50
#define _DISPLAY_DATA_1 2             // ui16_battery_SOC_percentage_x10
#define _DISPLAY_DATA_2 3             // ui16_battery_voltage_calibrated_x10
#define _DISPLAY_DATA_3 2             // ui32_wh_x10 
#define _DISPLAY_DATA_4 3              // ui8_pedal_cadence_RPM
#define _DISPLAY_DATA_5 2              // ui16_battery_power_filtered_x10
#define _DISPLAY_DATA_6 3              // ui16_human_power_filtered_x10

#define _POWER_ASSIST_LEVEL_1 50
#define _POWER_ASSIST_LEVEL_2 100
#define _POWER_ASSIST_LEVEL_3 160
#define _POWER_ASSIST_LEVEL_4 260
#define _TORQUE_ASSIST_LEVEL_1 50
#define _TORQUE_ASSIST_LEVEL_2 80
#define _TORQUE_ASSIST_LEVEL_3 120
#define _TORQUE_ASSIST_LEVEL_4 160
#define _CADENCE_ASSIST_LEVEL_1 80
#define _CADENCE_ASSIST_LEVEL_2 100
#define _CADENCE_ASSIST_LEVEL_3 130
#define _CADENCE_ASSIST_LEVEL_4 160
#define _EMTB_ASSIST_LEVEL_1 60
#define _EMTB_ASSIST_LEVEL_2 100
#define _EMTB_ASSIST_LEVEL_3 140
#define _EMTB_ASSIST_LEVEL_4 180
#define _WALK_ASSIST_LEVEL_1 30
#define _WALK_ASSIST_LEVEL_2 35
#define _WALK_ASSIST_LEVEL_3 40
#define _WALK_ASSIST_LEVEL_4 45
#define _WALK_ASSIST_THRESHOLD_SPEED_X10 60
#define _WALK_ASSIST_DEBOUNCE_ENABLED 0           
#define _WALK_ASSIST_DEBOUNCE_TIME 60
#define _CRUISE_TARGET_SPEED_LEVEL_1 15
#define _CRUISE_TARGET_SPEED_LEVEL_2 18
#define _CRUISE_TARGET_SPEED_LEVEL_3 21
#define _CRUISE_TARGET_SPEED_LEVEL_4 24
#define _CRUISE_MODE_WALK_ENABLED 0                
#define _CRUISE_THRESHOLD_SPEED 10
#define _PEDAL_TORQUE_ADC_OFFSET 150        // to be tested; it is the ADC value when no pressured is applied on the pedal
#define _UNITS_TYPE 0                        
#define _ASSIST_THROTTLE_MIN_VALUE 0        // adc values are mapped between min and max values
#define _ASSIST_THROTTLE_MAX_VALUE 255
#define _STREET_MODE_WALK_ENABLED 1
#define _DATA_DISPLAY_ON_STARTUP 1             
#define _FIELD_WEAKENING_ENABLED 0             
#define _PEDAL_TORQUE_ADC_OFFSET_ADJ 20
#define _PEDAL_TORQUE_ADC_RANGE_ADJ 20
#define _PEDAL_TORQUE_ADC_ANGLE_ADJ 36
#define _PEDAL_TORQUE_PER_10_BIT_ADC_STEP_ADV_X100 34  // to be tested
#define _SOC_PERCENT_CALC 0
#define _STARTUP_BOOST_AT_ZERO 0
#define _ENABLEC850 0                          
#define _STREET_MODE_THROTTLE_LEGAL 0          // is not used anymore
#define _BRAKE_TEMPERATURE_SWITCH 0            // is not used
#define _eMTB_BASED_ON_POWER 1
#define _SMOOTH_START_ENABLED 1                
#define _SMOOTH_START_SET_PERCENT 35
#define _TEMPERATURE_SENSOR_TYPE 0            // not used
#define _CRUISE_MODE_ENABLED 1                
#define _THROTTLE_MODE 4               // see below: when 0, throttle ADC is not used and converted; perhaps it is filled by java based on other parameters
#define _STREET_MODE_THROTTLE_MODE 4   // see below: with the display, it is probably possible to switch from THOTTLE_MODE to STREET_MODE_TROTTLE_MODE
#define _ASSIST_LEVEL_1_OF_5_PERCENT 60
#define _ALTERNATIVE_MILES 0                 



// THROTTLE_MODE and STREEMODE_THROTTLE_MODE can be Disabled (=0), pedaling (1), 6KM/h Only (2) , 6KM/h & pedaling (3), Unconditionnal (4)
// Only with brake sensors enabled<br>\nSet Optional ADC to Throttle\n<html>");
// for street mode, Trottle mode must be at the same or higher level
// Throttle mode
// throttle mode
//#define DISABLED					0
//#define PEDALING					1
//#define W_O_P_6KM_H_ONLY			2
//#define W_O_P_6KM_H_AND_PEDALING    3
//#define UNCONDITIONAL				4

// startup boost mode
//#define CADENCE					0
//#define SPEED						1


extern uint16_t ui16_actual_battery_capacity;// = (uint16_t)(((uint32_t) TARGET_MAX_BATTERY_CAPACITY * ACTUAL_BATTERY_CAPACITY_PERCENT) / 100);


extern uint8_t ui8_pedal_torque_per_10_bit_ADC_step_x100;

extern uint16_t c_ADC_TORQUE_SENSOR_RANGE_TARGET;
extern uint16_t c_ADC_TORQUE_SENSOR_RANGE_TARGET_MIN;
extern uint16_t c_ADC_TORQUE_SENSOR_RANGE_TARGET_MAX;

extern uint16_t c_ADC_TORQUE_SENSOR_RANGE;
extern uint16_t c_ADC_TORQUE_SENSOR_CALIBRATION_OFFSET;
extern uint16_t c_ADC_TORQUE_SENSOR_MIDDLE_OFFSET_ADJ;
extern uint16_t c_ADC_TORQUE_SENSOR_OFFSET_ADJ;
extern uint16_t c_ADC_TORQUE_SENSOR_DELTA_ADJ;
extern uint16_t c_ADC_TORQUE_SENSOR_RANGE_INGREASE_X100;
extern uint16_t c_ADC_TORQUE_SENSOR_DELTA_WITH_WEIGHT;
extern uint16_t c_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_CALC_X100;
extern uint16_t c_ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT;

extern uint8_t c_DELAY_LIGHTS_ON;
extern uint8_t c_DELAY_FUNCTION_STATUS;

extern uint16_t c_LI_ION_CELL_VOLTS_8_X100;
extern uint16_t c_LI_ION_CELL_VOLTS_7_X100;
extern uint16_t c_LI_ION_CELL_VOLTS_6_X100;
extern uint16_t c_LI_ION_CELL_VOLTS_5_X100	;
extern uint16_t c_LI_ION_CELL_VOLTS_4_X100;
extern uint16_t c_LI_ION_CELL_VOLTS_3_X100;
extern uint16_t c_LI_ION_CELL_VOLTS_2_X100	;
extern uint16_t c_LI_ION_CELL_VOLTS_1_X100	;
extern uint16_t c_LI_ION_CELL_VOLTS_0_X100	;
extern uint16_t c_BATTERY_SOC_VOLTS_8_X10	;
extern uint16_t c_BATTERY_SOC_VOLTS_7_X10	;
extern uint16_t c_BATTERY_SOC_VOLTS_6_X10	;
extern uint16_t c_BATTERY_SOC_VOLTS_5_X10	;
extern uint16_t c_BATTERY_SOC_VOLTS_4_X10	;
extern uint16_t c_BATTERY_SOC_VOLTS_3_X10	;
extern uint16_t c_BATTERY_SOC_VOLTS_2_X10	;
extern uint16_t c_BATTERY_SOC_VOLTS_1_X10	;
extern uint16_t c_BATTERY_SOC_VOLTS_0_X10	;

// cruise threshold (speed limit min km/h x10)
extern uint16_t c_CRUISE_THRESHOLD_SPEED_X10;	
extern uint16_t c_CRUISE_OFFROAD_THRESHOLD_SPEED_X10;
extern uint16_t c_CRUISE_STREET_THRESHOLD_SPEED_X10;

extern uint8_t ui8_pwm_duty_cycle_max;


void init_extra_fields_config ();
void upload_m_config();
#endif
#endif /* CONFIG_H_ */

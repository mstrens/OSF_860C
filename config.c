#include "xmc_flash.h"
#include "config.h"
#include "common.h" // to know the uint type
#include "main.h"

#ifdef USE_TSDZ8_VLCD5_VERSION

struct_config m_config = {
_MAIN_VERSION,
_SUB_VERSION,
_RESERVE_1   ,
_RESERVE_2   ,
_RESERVE_3   ,
_RESERVE_4   ,
_RESERVE_5   ,
_RESERVE_6   ,
_RESERVE_7   ,
_RESERVE_8   ,
_MOTOR_TYPE ,
_TORQUE_SENSOR_CALIBRATED ,
_MOTOR_ACCELERATION  ,
_MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION ,
_ASSISTANCE_WITHOUT_PEDAL_ROTATION_THRESHOLD ,
_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100 ,
_PEDAL_TORQUE_ADC_MAX ,
_STARTUP_BOOST_TORQUE_FACTOR ,
_MOTOR_BLOCKED_COUNTER_THRESHOLD ,
_MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X10 ,
_MOTOR_BLOCKED_ERPS_THRESHOLD ,
_STARTUP_BOOST_CADENCE_STEP ,      
_BATTERY_CURRENT_MAX        ,
_TARGET_MAX_BATTERY_POWER ,
_TARGET_MAX_BATTERY_CAPACITY ,
_BATTERY_CELLS_NUMBER ,
_MOTOR_DECELERATION ,
_BATTERY_LOW_VOLTAGE_CUT_OFF ,
_ACTUAL_BATTERY_VOLTAGE_PERCENT ,
_ACTUAL_BATTERY_CAPACITY_PERCENT ,
_LI_ION_CELL_OVERVOLT ,
_LI_ION_CELL_RESET_SOC_PERCENT ,
_LI_ION_CELL_VOLTS_FULL ,
_LI_ION_CELL_VOLTS_3_OF_4 ,
_LI_ION_CELL_VOLTS_2_OF_4 ,
_LI_ION_CELL_VOLTS_1_OF_4 ,
_LI_ION_CELL_VOLTS_5_OF_6 ,
_LI_ION_CELL_VOLTS_4_OF_6 ,
_LI_ION_CELL_VOLTS_3_OF_6 ,
_LI_ION_CELL_VOLTS_2_OF_6 ,
_LI_ION_CELL_VOLTS_1_OF_6 ,
_LI_ION_CELL_VOLTS_EMPTY ,
_WHEEL_PERIMETER ,
_WHEEL_MAX_SPEED ,
_ENABLE_LIGHTS ,
_ENABLE_WALK_ASSIST ,
_ENABLE_BRAKE_SENSOR ,
_ENABLE_THROTTLE ,
_ENABLE_TEMPERATURE_LIMIT ,
_ENABLE_STREET_MODE_ON_STARTUP ,
_ENABLE_SET_PARAMETER_ON_STARTUP ,
_ENABLE_ODOMETER_COMPENSATION ,
_STARTUP_BOOST_ON_STARTUP ,
_TORQUE_SENSOR_ADV_ON_STARTUP ,
_LIGHTS_CONFIGURATION_ON_STARTUP ,
_RIDING_MODE_ON_STARTUP ,
_LIGHTS_CONFIGURATION_1 ,
_LIGHTS_CONFIGURATION_2 ,
_LIGHTS_CONFIGURATION_3 ,
_STREET_MODE_POWER_LIMIT_ENABLED ,
_STREET_MODE_POWER_LIMIT ,
_STREET_MODE_SPEED_LIMIT ,
_STREET_MODE_THROTTLE_ENABLED ,
_STREET_MODE_CRUISE_ENABLED ,
_ADC_THROTTLE_MIN_VALUE ,
_ADC_THROTTLE_MAX_VALUE ,
_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT ,
_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT ,
_ENABLE_TEMPERATURE_ERROR_MIN_LIMIT ,
_ENABLE_VLCD6 ,
_ENABLE_VLCD5 ,
_ENABLE_XH18 ,
_ENABLE_DISPLAY_WORKING_FLAG ,
_ENABLE_DISPLAY_ALWAYS_ON ,
_ENABLE_WHEEL_MAX_SPEED_FROM_DISPLAY ,
_DELAY_MENU_ON ,
_COASTER_BRAKE_ENABLED ,
_COASTER_BRAKE_TORQUE_THRESHOLD ,
_ENABLE_AUTO_DATA_DISPLAY ,
_STARTUP_ASSIST_ENABLED ,
_AUTO_DATA_NUMBER_DISPLAY ,
_DELAY_DISPLAY_DATA_1 ,
_DELAY_DISPLAY_DATA_2 ,
_DELAY_DISPLAY_DATA_3 ,
_DELAY_DISPLAY_DATA_4 ,
_DELAY_DISPLAY_DATA_5 ,
_DELAY_DISPLAY_DATA_6 ,
_DISPLAY_DATA_1 ,
_DISPLAY_DATA_2 ,
_DISPLAY_DATA_3 ,
_DISPLAY_DATA_4 ,
_DISPLAY_DATA_5 ,
_DISPLAY_DATA_6 ,
_POWER_ASSIST_LEVEL_1 ,
_POWER_ASSIST_LEVEL_2 ,
_POWER_ASSIST_LEVEL_3 ,
_POWER_ASSIST_LEVEL_4 ,
_TORQUE_ASSIST_LEVEL_1 ,
_TORQUE_ASSIST_LEVEL_2 ,
_TORQUE_ASSIST_LEVEL_3 ,
_TORQUE_ASSIST_LEVEL_4 ,
_CADENCE_ASSIST_LEVEL_1 ,
_CADENCE_ASSIST_LEVEL_2 ,
_CADENCE_ASSIST_LEVEL_3 ,
_CADENCE_ASSIST_LEVEL_4 ,
_EMTB_ASSIST_LEVEL_1 ,
_EMTB_ASSIST_LEVEL_2 ,
_EMTB_ASSIST_LEVEL_3 ,
_EMTB_ASSIST_LEVEL_4 ,
_WALK_ASSIST_LEVEL_1 ,
_WALK_ASSIST_LEVEL_2 ,
_WALK_ASSIST_LEVEL_3 ,
_WALK_ASSIST_LEVEL_4 ,
_WALK_ASSIST_THRESHOLD_SPEED_X10 ,
_WALK_ASSIST_DEBOUNCE_ENABLED ,
_WALK_ASSIST_DEBOUNCE_TIME ,
_CRUISE_TARGET_SPEED_LEVEL_1 ,
_CRUISE_TARGET_SPEED_LEVEL_2 ,
_CRUISE_TARGET_SPEED_LEVEL_3 ,
_CRUISE_TARGET_SPEED_LEVEL_4 ,
_CRUISE_MODE_WALK_ENABLED ,
_CRUISE_THRESHOLD_SPEED ,
_PEDAL_TORQUE_ADC_OFFSET ,
_UNITS_TYPE ,
_ASSIST_THROTTLE_MIN_VALUE ,
_ASSIST_THROTTLE_MAX_VALUE ,
_STREET_MODE_WALK_ENABLED ,
_DATA_DISPLAY_ON_STARTUP ,
_FIELD_WEAKENING_ENABLED ,
_PEDAL_TORQUE_ADC_OFFSET_ADJ ,
_PEDAL_TORQUE_ADC_RANGE_ADJ ,
_PEDAL_TORQUE_ADC_ANGLE_ADJ ,
_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_ADV_X100 ,
_SOC_PERCENT_CALC ,
_STARTUP_BOOST_AT_ZERO ,
_ENABLEC850 ,
_STREET_MODE_THROTTLE_LEGAL ,
_BRAKE_TEMPERATURE_SWITCH ,
_eMTB_BASED_ON_POWER ,
_SMOOTH_START_ENABLED ,
_SMOOTH_START_SET_PERCENT ,
_TEMPERATURE_SENSOR_TYPE ,
_CRUISE_MODE_ENABLED ,
_THROTTLE_MODE ,
_STREET_MODE_THROTTLE_MODE ,
_ASSIST_LEVEL_1_OF_5_PERCENT ,
_ALTERNATIVE_MILES ,
} ; 

uint16_t c_ADC_TORQUE_SENSOR_RANGE_TARGET; 
uint16_t c_ADC_TORQUE_SENSOR_RANGE_TARGET_MIN;
uint16_t c_ADC_TORQUE_SENSOR_RANGE_TARGET_MAX;

uint16_t c_ADC_TORQUE_SENSOR_RANGE;
uint16_t c_ADC_TORQUE_SENSOR_CALIBRATION_OFFSET;
uint16_t c_ADC_TORQUE_SENSOR_MIDDLE_OFFSET_ADJ;
uint16_t c_ADC_TORQUE_SENSOR_OFFSET_ADJ;
uint16_t c_ADC_TORQUE_SENSOR_DELTA_ADJ;
uint16_t c_ADC_TORQUE_SENSOR_RANGE_INGREASE_X100;
uint16_t c_ADC_TORQUE_SENSOR_DELTA_WITH_WEIGHT;
uint16_t c_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_CALC_X100;
uint16_t c_ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT;

uint8_t c_DELAY_LIGHTS_ON;
uint8_t c_DELAY_FUNCTION_STATUS;

uint16_t c_LI_ION_CELL_VOLTS_8_X100;
uint16_t c_LI_ION_CELL_VOLTS_7_X100;
uint16_t c_LI_ION_CELL_VOLTS_6_X100;
uint16_t c_LI_ION_CELL_VOLTS_5_X100	;
uint16_t c_LI_ION_CELL_VOLTS_4_X100;
uint16_t c_LI_ION_CELL_VOLTS_3_X100;
uint16_t c_LI_ION_CELL_VOLTS_2_X100	;
uint16_t c_LI_ION_CELL_VOLTS_1_X100	;
uint16_t c_LI_ION_CELL_VOLTS_0_X100	;
uint16_t c_BATTERY_SOC_VOLTS_8_X10	;
uint16_t c_BATTERY_SOC_VOLTS_7_X10	;
uint16_t c_BATTERY_SOC_VOLTS_6_X10	;
uint16_t c_BATTERY_SOC_VOLTS_5_X10	;
uint16_t c_BATTERY_SOC_VOLTS_4_X10	;
uint16_t c_BATTERY_SOC_VOLTS_3_X10	;
uint16_t c_BATTERY_SOC_VOLTS_2_X10	;
uint16_t c_BATTERY_SOC_VOLTS_1_X10	;
uint16_t c_BATTERY_SOC_VOLTS_0_X10	;

// cruise threshold (speed limit min km/h x10)
uint16_t c_CRUISE_THRESHOLD_SPEED_X10;	
uint16_t c_CRUISE_OFFROAD_THRESHOLD_SPEED_X10;
uint16_t c_CRUISE_STREET_THRESHOLD_SPEED_X10;

// defined in ebike_app.c
extern uint8_t ui8_auto_data_number_display;
extern uint8_t ui8_delay_display_function ;
extern  uint8_t ui8_display_data_on_startup ;
extern  uint8_t ui8_set_parameter_enabled_temp ;
extern  uint8_t ui8_auto_display_data_enabled_temp ;
extern  uint8_t ui8_street_mode_enabled_temp ;
extern  uint8_t ui8_torque_sensor_adv_enabled_temp ;
extern  uint8_t ui8_assist_without_pedal_rotation_temp ;
extern  uint8_t ui8_walk_assist_enabled_array[2] ;

extern uint8_t ui8_lights_configuration_2;// = LIGHTS_CONFIGURATION_2;
extern uint8_t ui8_lights_configuration_3;// = LIGHTS_CONFIGURATION_3;
extern uint8_t ui8_lights_configuration_temp;// = LIGHTS_CONFIGURATION_ON_STARTUP;

extern uint8_t ui8_assist_without_pedal_rotation_threshold; // = ASSISTANCE_WITHOUT_PEDAL_ROTATION_THRESHOLD;
 
 
extern uint8_t ui8_motor_deceleration; // = MOTOR_DECELERATION; 

extern uint8_t ui8_pedal_torque_per_10_bit_ADC_step_calc_x100 ;//= PEDAL_TORQUE_PER_10_BIT_ADC_STEP_CALC_X100;  //
extern uint16_t ui16_adc_pedal_torque_offset;// = PEDAL_TORQUE_ADC_OFFSET;      // 150
extern uint16_t ui16_adc_pedal_torque_offset_init;// = PEDAL_TORQUE_ADC_OFFSET; // 150
extern uint16_t ui16_adc_pedal_torque_offset_cal;// = PEDAL_TORQUE_ADC_OFFSET;  // 150
extern uint16_t ui16_adc_pedal_torque_offset_min;// = PEDAL_TORQUE_ADC_OFFSET - ADC_TORQUE_SENSOR_OFFSET_THRESHOLD; //150-30
extern uint16_t ui16_adc_pedal_torque_offset_max;// = PEDAL_TORQUE_ADC_OFFSET + ADC_TORQUE_SENSOR_OFFSET_THRESHOLD; // 150 + 30

extern uint8_t ui8_torque_sensor_calibrated;// = TORQUE_SENSOR_CALIBRATED;
 
extern uint8_t ui8_wheel_speed_max_array[2];// = {WHEEL_MAX_SPEED,STREET_MODE_SPEED_LIMIT};
extern uint8_t ui8_eMTB_based_on_power;

extern uint8_t ui8_throttle_mode_array[2]; // defined in ebike.app.c

extern uint8_t ui8_cruise_threshold_speed_x10_array[2]; // defined in ebike_app.c

extern uint8_t ui8_startup_boost_at_zero;// = STARTUP_BOOST_AT_ZERO;
 
extern  uint8_t ui8_startup_boost_enabled_temp;// = STARTUP_BOOST_ON_STARTUP;
 
extern  uint8_t ui8_data_index_array[DATA_INDEX_ARRAY_DIM] ; // {DISPLAY_DATA_1,DISPLAY_DATA_2,DISPLAY_DATA_3,DISPLAY_DATA_4,DISPLAY_DATA_5,DISPLAY_DATA_6};
extern  uint8_t ui8_delay_display_array[DATA_INDEX_ARRAY_DIM]; // = {DELAY_DISPLAY_DATA_1,DELAY_DISPLAY_DATA_2,DELAY_DISPLAY_DATA_3,DELAY_DISPLAY_DATA_4,DELAY_DISPLAY_DATA_5,DELAY_DISPLAY_DATA_6};

extern uint8_t  ui8_riding_mode_parameter_array[8][5];
//	{POWER_ASSIST_LEVEL_OFF, POWER_ASSIST_LEVEL_ECO, POWER_ASSIST_LEVEL_TOUR, POWER_ASSIST_LEVEL_SPORT, POWER_ASSIST_LEVEL_TURBO},
//	{TORQUE_ASSIST_LEVEL_0, TORQUE_ASSIST_LEVEL_1, TORQUE_ASSIST_LEVEL_2, TORQUE_ASSIST_LEVEL_3, TORQUE_ASSIST_LEVEL_4},
//	{CADENCE_ASSIST_LEVEL_0, CADENCE_ASSIST_LEVEL_1, CADENCE_ASSIST_LEVEL_2, CADENCE_ASSIST_LEVEL_3, CADENCE_ASSIST_LEVEL_4},
//	{EMTB_ASSIST_LEVEL_0, EMTB_ASSIST_LEVEL_1, EMTB_ASSIST_LEVEL_2, EMTB_ASSIST_LEVEL_3, EMTB_ASSIST_LEVEL_4},
//	{POWER_ASSIST_LEVEL_OFF, POWER_ASSIST_LEVEL_ECO, POWER_ASSIST_LEVEL_TOUR, POWER_ASSIST_LEVEL_SPORT, POWER_ASSIST_LEVEL_TURBO},
//	{CRUISE_TARGET_SPEED_LEVEL_0, CRUISE_TARGET_SPEED_LEVEL_1, CRUISE_TARGET_SPEED_LEVEL_2, CRUISE_TARGET_SPEED_LEVEL_3, CRUISE_TARGET_SPEED_LEVEL_4},
//	{WALK_ASSIST_LEVEL_0, WALK_ASSIST_LEVEL_1, WALK_ASSIST_LEVEL_2, WALK_ASSIST_LEVEL_3, WALK_ASSIST_LEVEL_4},
//	{0, 0, 0, 0, 0}
	

void init_extra_fields_config (){
    #if (USE_CONFIG_FROM_COMPILATION != 1 ) 
    upload_m_config(); // try to get the user preference from flash at 0X1000F000; 
                        //When version is not compatible (not the same main version), setup is done with the values from compilation
                        // Normally the motor is then blocked with an error code = E09.
                        // still a define in main.h allows to let the motor run with the compilation config.
                        // this can be usefull for testing/debugging (avoid changes in XLS) 
    #endif
    // battery
    ui16_actual_battery_capacity = (uint16_t)(((uint32_t) m_config.target_max_battery_capacity * m_config.actual_battery_capacity_percent ) / 100);
 
    // torque sensor
    ui8_pedal_torque_per_10_bit_ADC_step_x100 = m_config.pedal_torque_per_10_bit_adc_step_x100;  // 67
 
    // Torque sensor range values
    c_ADC_TORQUE_SENSOR_RANGE = m_config.pedal_torque_adc_max - m_config.pedal_torque_adc_offset;
    c_ADC_TORQUE_SENSOR_RANGE_TARGET	  =		160;

    // Torque sensor offset values
    if (m_config.torque_sensor_calibrated){
        c_ADC_TORQUE_SENSOR_CALIBRATION_OFFSET = (((6 * c_ADC_TORQUE_SENSOR_RANGE) / c_ADC_TORQUE_SENSOR_RANGE_TARGET) + 1);
        c_ADC_TORQUE_SENSOR_MIDDLE_OFFSET_ADJ = (((20 * c_ADC_TORQUE_SENSOR_RANGE) / c_ADC_TORQUE_SENSOR_RANGE_TARGET) + 1);
        c_ADC_TORQUE_SENSOR_OFFSET_ADJ = ((( m_config.pedal_torque_adc_offset_adj * c_ADC_TORQUE_SENSOR_RANGE) 
                                            / c_ADC_TORQUE_SENSOR_RANGE_TARGET) + 1);
    } else {
        c_ADC_TORQUE_SENSOR_CALIBRATION_OFFSET = 6 ; 
        c_ADC_TORQUE_SENSOR_MIDDLE_OFFSET_ADJ = 20; 
        c_ADC_TORQUE_SENSOR_OFFSET_ADJ = ((( m_config.pedal_torque_adc_offset_adj * c_ADC_TORQUE_SENSOR_RANGE) 
                                            / c_ADC_TORQUE_SENSOR_RANGE_TARGET) + 1);
    }
    // adc torque range parameters for remapping
    c_ADC_TORQUE_SENSOR_DELTA_ADJ = ((c_ADC_TORQUE_SENSOR_MIDDLE_OFFSET_ADJ * 2) - c_ADC_TORQUE_SENSOR_CALIBRATION_OFFSET
         - c_ADC_TORQUE_SENSOR_CALIBRATION_OFFSET);
    c_ADC_TORQUE_SENSOR_RANGE_INGREASE_X100 = ((c_ADC_TORQUE_SENSOR_RANGE_TARGET * 50) / c_ADC_TORQUE_SENSOR_RANGE);
    
    c_ADC_TORQUE_SENSOR_RANGE_TARGET_MIN 	=	(uint16_t)((float)((c_ADC_TORQUE_SENSOR_RANGE_TARGET / 2) \
    * (((c_ADC_TORQUE_SENSOR_RANGE_TARGET / 2) / ADC_TORQUE_SENSOR_ANGLE_COEFF + ADC_TORQUE_SENSOR_ANGLE_COEFF) / ADC_TORQUE_SENSOR_ANGLE_COEFF)));

    c_ADC_TORQUE_SENSOR_RANGE_TARGET_MAX 	=	(uint16_t)((c_ADC_TORQUE_SENSOR_RANGE_TARGET_MIN * (100 + m_config.pedal_torque_adc_range_adj)) / 100); //PEDAL_TORQUE_ADC_RANGE_ADJ

    // parameters of the adc torque step for human power calculation
    #define PEDAL_TORQUE_PER_10_BIT_ADC_STEP_BASE_X100	34 // base adc step for remapping
    #define WEIGHT_ON_PEDAL_FOR_STEP_CALIBRATION		24 // Kg
    #define PERCENT_TORQUE_SENSOR_RANGE_WITH_WEIGHT		75 // % of torque sensor range with weight
    c_ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT	=	(uint16_t)((c_ADC_TORQUE_SENSOR_RANGE_TARGET * PERCENT_TORQUE_SENSOR_RANGE_WITH_WEIGHT) / 100);

    c_ADC_TORQUE_SENSOR_DELTA_WITH_WEIGHT	=	(uint16_t)(((((c_ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT \
    * c_ADC_TORQUE_SENSOR_RANGE_TARGET_MIN) / c_ADC_TORQUE_SENSOR_RANGE_TARGET)	* (100 + m_config.pedal_torque_adc_range_adj) / 100) \
    * (c_ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT - c_ADC_TORQUE_SENSOR_CALIBRATION_OFFSET + c_ADC_TORQUE_SENSOR_OFFSET_ADJ \
    - ((c_ADC_TORQUE_SENSOR_DELTA_ADJ * c_ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT) / c_ADC_TORQUE_SENSOR_RANGE_TARGET))) /
     c_ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT);

    c_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_CALC_X100 =	(uint8_t)((uint16_t)(((WEIGHT_ON_PEDAL_FOR_STEP_CALIBRATION * 167) \
    / ((c_ADC_TORQUE_SENSOR_DELTA_WITH_WEIGHT * c_ADC_TORQUE_SENSOR_RANGE_TARGET_MAX) \
    / (c_ADC_TORQUE_SENSOR_RANGE_TARGET_MAX - (((c_ADC_TORQUE_SENSOR_RANGE_TARGET_MAX - c_ADC_TORQUE_SENSOR_DELTA_WITH_WEIGHT) * 10) \
    / m_config.pedal_torque_adc_angle_adj ))) \
    * m_config.pedal_torque_per_10_bit_adc_step_adv_x100) / PEDAL_TORQUE_PER_10_BIT_ADC_STEP_BASE_X100));

    // delay lights function (0.1 sec)
    uint8_t c_DELAY_LIGHTS_ON = m_config.delay_menu_on;	//	 	DELAY_MENU_ON    // 5sec

    // delay function status (0.1 sec)
    c_DELAY_FUNCTION_STATUS =   (uint8_t) (c_DELAY_LIGHTS_ON / 2) ; //2,5 sec

    // cell bars
    if ((m_config.enable_vlcd6) || (m_config.enable_xh18) ){
        c_LI_ION_CELL_VOLTS_6_X100	= (uint16_t)((float) m_config.li_ion_cell_overvolt * 100);
        c_LI_ION_CELL_VOLTS_5_X100	= (uint16_t)((float) m_config.li_ion_cell_reset_soc_percent * 100);
        c_LI_ION_CELL_VOLTS_4_X100	=	(uint16_t)((float) m_config.li_ion_cell_volts_full * 100);
        c_LI_ION_CELL_VOLTS_3_X100	=	(uint16_t)((float) m_config.li_ion_cell_volts_3_of_4 * 100);
        c_LI_ION_CELL_VOLTS_2_X100	=	(uint16_t)((float) m_config.li_ion_cell_volts_2_of_4 * 100);
        c_LI_ION_CELL_VOLTS_1_X100	=	(uint16_t)((float) m_config.li_ion_cell_volts_1_of_4 * 100);
        c_LI_ION_CELL_VOLTS_0_X100	=	(uint16_t)((float) m_config.li_ion_cell_volts_empty * 100);
        c_BATTERY_SOC_VOLTS_6_X10	=		(uint16_t)(m_config.battery_cells_number * ((float) m_config.li_ion_cell_overvolt * 10));
        c_BATTERY_SOC_VOLTS_5_X10	=		(uint16_t)(m_config.battery_cells_number * ((float) m_config.li_ion_cell_reset_soc_percent * 10));
        c_BATTERY_SOC_VOLTS_4_X10	=		(uint16_t)(m_config.battery_cells_number * ((float) m_config.li_ion_cell_volts_full * 10));
        c_BATTERY_SOC_VOLTS_3_X10	=		(uint16_t)(m_config.battery_cells_number * ((float) m_config.li_ion_cell_volts_3_of_4 * 10));
        c_BATTERY_SOC_VOLTS_2_X10	=		(uint16_t)(m_config.battery_cells_number * ((float) m_config.li_ion_cell_volts_2_of_4 * 10));
        c_BATTERY_SOC_VOLTS_1_X10	=		(uint16_t)(m_config.battery_cells_number * ((float) m_config.li_ion_cell_volts_1_of_4 * 10));
        c_BATTERY_SOC_VOLTS_0_X10	=		(uint16_t)(m_config.battery_cells_number * ((float) m_config.li_ion_cell_volts_empty * 10));
    } else { // ENABLE_VLCD5 or 850C
        c_LI_ION_CELL_VOLTS_8_X100	=	(uint16_t)((float)m_config.li_ion_cell_overvolt * 100.0);
        c_LI_ION_CELL_VOLTS_7_X100	=	(uint16_t)((float)m_config.li_ion_cell_reset_soc_percent * 100.0);
        c_LI_ION_CELL_VOLTS_6_X100	=	(uint16_t)((float)m_config.li_ion_cell_volts_full * 100);
        c_LI_ION_CELL_VOLTS_5_X100	=	(uint16_t)((float)m_config.li_ion_cell_volts_5_of_6 * 100);
        c_LI_ION_CELL_VOLTS_4_X100	=	(uint16_t)((float)m_config.li_ion_cell_volts_4_of_6 * 100);
        c_LI_ION_CELL_VOLTS_3_X100	=	(uint16_t)((float)m_config.li_ion_cell_volts_3_of_6 * 100);
        c_LI_ION_CELL_VOLTS_2_X100	=	(uint16_t)((float)m_config.li_ion_cell_volts_2_of_6 * 100);
        c_LI_ION_CELL_VOLTS_1_X100	=	(uint16_t)((float)m_config.li_ion_cell_volts_1_of_6 * 100);
        c_LI_ION_CELL_VOLTS_0_X100	=	(uint16_t)((float)m_config.li_ion_cell_volts_empty * 100);
        c_BATTERY_SOC_VOLTS_8_X10	=		(uint16_t)(m_config.battery_cells_number * ((float)m_config.li_ion_cell_overvolt * 10));
        c_BATTERY_SOC_VOLTS_7_X10	=		(uint16_t)(m_config.battery_cells_number * ((float)m_config.li_ion_cell_reset_soc_percent * 10));
        c_BATTERY_SOC_VOLTS_6_X10	=		(uint16_t)(m_config.battery_cells_number * ((float)m_config.li_ion_cell_volts_full * 10));
        c_BATTERY_SOC_VOLTS_5_X10	=		(uint16_t)(m_config.battery_cells_number * ((float)m_config.li_ion_cell_volts_5_of_6 * 10));
        c_BATTERY_SOC_VOLTS_4_X10	=		(uint16_t)(m_config.battery_cells_number * ((float)m_config.li_ion_cell_volts_4_of_6 * 10));
        c_BATTERY_SOC_VOLTS_3_X10	=		(uint16_t)(m_config.battery_cells_number * ((float)m_config.li_ion_cell_volts_3_of_6 * 10));
        c_BATTERY_SOC_VOLTS_2_X10	=		(uint16_t)(m_config.battery_cells_number * ((float)m_config.li_ion_cell_volts_2_of_6 * 10));
        c_BATTERY_SOC_VOLTS_1_X10	=		(uint16_t)(m_config.battery_cells_number * ((float)m_config.li_ion_cell_volts_1_of_6 * 10));
        c_BATTERY_SOC_VOLTS_0_X10	=		(uint16_t)(m_config.battery_cells_number * ((float)m_config.li_ion_cell_volts_empty * 10));
    }
    // cruise threshold (speed limit min km/h x10)
    c_CRUISE_THRESHOLD_SPEED_X10 = m_config.cruise_threshold_speed * 10;
    #define CRUISE_THRESHOLD_SPEED_X10_DEFAULT		80
    c_CRUISE_OFFROAD_THRESHOLD_SPEED_X10 = c_CRUISE_THRESHOLD_SPEED_X10;
    if (c_CRUISE_THRESHOLD_SPEED_X10 < CRUISE_THRESHOLD_SPEED_X10_DEFAULT) {
        c_CRUISE_STREET_THRESHOLD_SPEED_X10 = CRUISE_THRESHOLD_SPEED_X10_DEFAULT;
    } else {	
        c_CRUISE_STREET_THRESHOLD_SPEED_X10	= c_CRUISE_THRESHOLD_SPEED_X10;
    }

    ui8_auto_data_number_display = m_config.auto_data_number_display;
    ui8_delay_display_function = m_config.delay_menu_on;
    ui8_display_data_on_startup = m_config.data_display_on_startup;
    ui8_set_parameter_enabled_temp = m_config.enable_set_parameter_on_startup;
    ui8_auto_display_data_enabled_temp = m_config.enable_auto_data_display;
    ui8_street_mode_enabled_temp = m_config.enable_street_mode_on_startup;
    ui8_torque_sensor_adv_enabled_temp = m_config.torque_sensor_adv_on_startup; // TORQUE_SENSOR_ADV_ON_STARTUP;
    ui8_assist_without_pedal_rotation_temp = m_config.motor_assistance_without_pedal_rotation; // MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION;
    ui8_walk_assist_enabled_array[0] = m_config.enable_walk_assist; // {ENABLE_WALK_ASSIST,STREET_MODE_WALK_ENABLED};
    ui8_walk_assist_enabled_array[1] = m_config.street_mode_walk_enabled; // {ENABLE_WALK_ASSIST,STREET_MODE_WALK_ENABLED};

    ui8_lights_configuration_2 = m_config.lights_configuration_2; // LIGHTS_CONFIGURATION_2;
    ui8_lights_configuration_3 = m_config.lights_configuration_3; //LIGHTS_CONFIGURATION_3;
    ui8_lights_configuration_temp = m_config.lights_configuration_on_startup; // LIGHTS_CONFIGURATION_ON_STARTUP;

    ui8_assist_without_pedal_rotation_threshold = m_config.assistance_without_pedal_rotation_threshold; // ASSISTANCE_WITHOUT_PEDAL_ROTATION_THRESHOLD;
    
    ui16_actual_battery_capacity = (uint16_t)(((uint32_t) m_config.target_max_battery_capacity *
                         m_config.actual_battery_voltage_percent) / 100); // TARGET_MAX_BATTERY_CAPACITY * ACTUAL_BATTERY_CAPACITY_PERCENT) / 100);
 
    ui8_motor_deceleration = m_config.motor_deceleration;// MOTOR_DECELERATION;

    ui8_pedal_torque_per_10_bit_ADC_step_calc_x100 = c_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_CALC_X100;  //
    ui16_adc_pedal_torque_offset = m_config.pedal_torque_adc_offset;// PEDAL_TORQUE_ADC_OFFSET;      // 150
    ui16_adc_pedal_torque_offset_init = m_config.pedal_torque_adc_offset;// PEDAL_TORQUE_ADC_OFFSET; // 150
    ui16_adc_pedal_torque_offset_cal =  m_config.pedal_torque_adc_offset;//PEDAL_TORQUE_ADC_OFFSET;  // 150
    ui16_adc_pedal_torque_offset_min = m_config.pedal_torque_adc_offset - ADC_TORQUE_SENSOR_OFFSET_THRESHOLD; //150-30
    ui16_adc_pedal_torque_offset_max = m_config.pedal_torque_adc_offset + ADC_TORQUE_SENSOR_OFFSET_THRESHOLD; // 150 + 30

    ui8_torque_sensor_calibrated = m_config.torque_sensor_calibrated;// TORQUE_SENSOR_CALIBRATED;
 
     ui8_wheel_speed_max_array[0] = m_config.wheel_max_speed;
     ui8_wheel_speed_max_array[1] = m_config.street_mode_speed_limit; // {WHEEL_MAX_SPEED,STREET_MODE_SPEED_LIMIT};

    ui8_eMTB_based_on_power =  m_config.emtb_based_on_power ;//eMTB_BASED_ON_POWER;

    ui8_throttle_mode_array[0] = m_config.throttle_mode;
    ui8_throttle_mode_array[1] = m_config.street_mode_throttle_mode;

    // initialisation of variables used in ebike_app.c
    ui8_cruise_threshold_speed_x10_array[0] = c_CRUISE_OFFROAD_THRESHOLD_SPEED_X10;
    ui8_cruise_threshold_speed_x10_array[1] = c_CRUISE_STREET_THRESHOLD_SPEED_X10;

    ui8_startup_boost_at_zero = m_config.startup_boost_at_zero;// STARTUP_BOOST_AT_ZERO;
 
    ui8_startup_boost_enabled_temp = m_config.startup_boost_on_startup;// STARTUP_BOOST_ON_STARTUP;

    ui8_data_index_array[0] = m_config.display_data_1;//{DISPLAY_DATA_1,DISPLAY_DATA_2,DISPLAY_DATA_3,DISPLAY_DATA_4,DISPLAY_DATA_5,DISPLAY_DATA_6};
    ui8_data_index_array[1] = m_config.display_data_2;
    ui8_data_index_array[2] = m_config.display_data_3;
    ui8_data_index_array[3] = m_config.display_data_4;
    ui8_data_index_array[4] = m_config.display_data_5;
    ui8_data_index_array[5] = m_config.display_data_6;
    
    ui8_delay_display_array[0] = m_config.delay_display_data_1; //{DELAY_DISPLAY_DATA_1,DELAY_DISPLAY_DATA_2,DELAY_DISPLAY_DATA_3,DELAY_DISPLAY_DATA_4,DELAY_DISPLAY_DATA_5,DELAY_DISPLAY_DATA_6};
    ui8_delay_display_array[1] = m_config.delay_display_data_2;
    ui8_delay_display_array[2] = m_config.delay_display_data_3;
    ui8_delay_display_array[3] = m_config.delay_display_data_4;
    ui8_delay_display_array[4] = m_config.delay_display_data_5;
    ui8_delay_display_array[5] = m_config.delay_display_data_6;

    // array for riding parameters
    //	{POWER_ASSIST_LEVEL_OFF, POWER_ASSIST_LEVEL_ECO, POWER_ASSIST_LEVEL_TOUR, POWER_ASSIST_LEVEL_SPORT, POWER_ASSIST_LEVEL_TURBO},
    ui8_riding_mode_parameter_array[0][0] = 0;
    ui8_riding_mode_parameter_array[0][1] = m_config.power_assist_level_1 >> 1;
    ui8_riding_mode_parameter_array[0][2] = m_config.power_assist_level_2 >> 1;
    ui8_riding_mode_parameter_array[0][3] = m_config.power_assist_level_3 >> 1;
    ui8_riding_mode_parameter_array[0][4] = m_config.power_assist_level_4 >> 1; 
    //	{TORQUE_ASSIST_LEVEL_0, TORQUE_ASSIST_LEVEL_1, TORQUE_ASSIST_LEVEL_2, TORQUE_ASSIST_LEVEL_3, TORQUE_ASSIST_LEVEL_4},
    ui8_riding_mode_parameter_array[1][0] = 0;
    ui8_riding_mode_parameter_array[1][1] = m_config.torque_assist_level_1;
    ui8_riding_mode_parameter_array[1][2] = m_config.torque_assist_level_2;
    ui8_riding_mode_parameter_array[1][3] = m_config.torque_assist_level_3;
    ui8_riding_mode_parameter_array[1][4] = m_config.torque_assist_level_4; 
//	{CADENCE_ASSIST_LEVEL_0, CADENCE_ASSIST_LEVEL_1, CADENCE_ASSIST_LEVEL_2, CADENCE_ASSIST_LEVEL_3, CADENCE_ASSIST_LEVEL_4},
    ui8_riding_mode_parameter_array[2][0] = 0;
    ui8_riding_mode_parameter_array[2][1] = m_config.cadence_assist_level_1;
    ui8_riding_mode_parameter_array[2][2] = m_config.cadence_assist_level_2;
    ui8_riding_mode_parameter_array[2][3] = m_config.cadence_assist_level_3;
    ui8_riding_mode_parameter_array[2][4] = m_config.cadence_assist_level_4; 
//	{EMTB_ASSIST_LEVEL_0, EMTB_ASSIST_LEVEL_1, EMTB_ASSIST_LEVEL_2, EMTB_ASSIST_LEVEL_3, EMTB_ASSIST_LEVEL_4},
    ui8_riding_mode_parameter_array[3][0] = 0;
    ui8_riding_mode_parameter_array[3][1] = m_config.emtb_assist_level_1;
    ui8_riding_mode_parameter_array[3][2] = m_config.emtb_assist_level_2;
    ui8_riding_mode_parameter_array[3][3] = m_config.emtb_assist_level_3;
    ui8_riding_mode_parameter_array[3][4] = m_config.emtb_assist_level_4; 
//	{POWER_ASSIST_LEVEL_OFF, POWER_ASSIST_LEVEL_ECO, POWER_ASSIST_LEVEL_TOUR, POWER_ASSIST_LEVEL_SPORT, POWER_ASSIST_LEVEL_TURBO},
    ui8_riding_mode_parameter_array[4][0] = 0;
    ui8_riding_mode_parameter_array[4][1] = m_config.power_assist_level_1 >> 1;
    ui8_riding_mode_parameter_array[4][2] = m_config.power_assist_level_2 >> 1;
    ui8_riding_mode_parameter_array[4][3] = m_config.power_assist_level_3 >> 1;
    ui8_riding_mode_parameter_array[4][4] = m_config.power_assist_level_4 >> 1; 
//	{CRUISE_TARGET_SPEED_LEVEL_0, CRUISE_TARGET_SPEED_LEVEL_1, CRUISE_TARGET_SPEED_LEVEL_2, CRUISE_TARGET_SPEED_LEVEL_3, CRUISE_TARGET_SPEED_LEVEL_4},
    ui8_riding_mode_parameter_array[5][0] = 0;
    ui8_riding_mode_parameter_array[5][1] = m_config.cruise_target_speed_level_1;
    ui8_riding_mode_parameter_array[5][2] = m_config.cruise_target_speed_level_2;
    ui8_riding_mode_parameter_array[5][3] = m_config.cruise_target_speed_level_3;
    ui8_riding_mode_parameter_array[5][4] = m_config.cruise_target_speed_level_4; 
//	{WALK_ASSIST_LEVEL_0, WALK_ASSIST_LEVEL_1, WALK_ASSIST_LEVEL_2, WALK_ASSIST_LEVEL_3, WALK_ASSIST_LEVEL_4},
    ui8_riding_mode_parameter_array[6][0] = 0;
    ui8_riding_mode_parameter_array[6][1] = m_config.walk_assist_level_1;
    ui8_riding_mode_parameter_array[6][2] = m_config.walk_assist_level_2;
    ui8_riding_mode_parameter_array[6][3] = m_config.walk_assist_level_3;
    ui8_riding_mode_parameter_array[6][4] = m_config.walk_assist_level_4; 
//	{0, 0, 0, 0, 0}
    ui8_riding_mode_parameter_array[7][0] = 0;
    ui8_riding_mode_parameter_array[7][1] = 0;
    ui8_riding_mode_parameter_array[7][2] = 0;
    ui8_riding_mode_parameter_array[7][3] = 0;
    ui8_riding_mode_parameter_array[7][4] = 0; 

    // used for special tests
    #if (PROCESS == FIND_BEST_GLOBAL_HALL_OFFSET)  
    ui8_pwm_duty_cycle_max = PWM_DUTY_CYCLE_MAX_FIND_BEST_GLOBAL_HALL_OFFSET;        
    #elif (PROCESS == TEST_WITH_FIXED_DUTY_CYCLE)
    ui8_pwm_duty_cycle_max = PWM_DUTY_CYCLE_MAX_TEST_WITH_FIXED_DUTY_CYCLE;
    #else
    ui8_pwm_duty_cycle_max = PWM_DUTY_CYCLE_MAX_NORMAL_OPERATIONS;	//max duty cycle for normal operations       
    #endif
}

void upload_m_config(){
    uint16_t * pConfig = (uint16_t *) ADDRESS_OF_M_CONFIG_FLASH;  // point to the begin of user preference parameters in flash 
    if ( *pConfig != m_config.main_version) {
        return; // discard flash parameters (and use those from compilation) if the main version is different
                // in ebike_app.c there is another check that force an ERROR_MOTOR_CHECK to block the motor
    } else {           
        m_config.main_version = *pConfig++; // read the value given by the pointer, and afterward increment it
        m_config.sub_version = *pConfig++;
        m_config.reserve_1 = *pConfig++;
        m_config.reserve_2 = *pConfig++;
        m_config.reserve_3 = *pConfig++;
        m_config.reserve_4 = *pConfig++;
        m_config.reserve_5 = *pConfig++;
        m_config.reserve_6 = *pConfig++;
        m_config.reserve_7 = *pConfig++;
        m_config.reserve_8 = *pConfig++;
        m_config.motor_type = *pConfig++;
        m_config.torque_sensor_calibrated = *pConfig++;
        m_config.motor_acceleration = *pConfig++;
        m_config.motor_assistance_without_pedal_rotation = *pConfig++;
        m_config.assistance_without_pedal_rotation_threshold = *pConfig++;
        m_config.pedal_torque_per_10_bit_adc_step_x100 = *pConfig++;
        m_config.pedal_torque_adc_max = *pConfig++;
        m_config.startup_boost_torque_factor = *pConfig++;
        m_config.motor_blocked_counter_threshold = *pConfig++;
        m_config.motor_blocked_battery_current_threshold_x10 = *pConfig++;
        m_config.motor_blocked_erps_threshold = *pConfig++;
        m_config.startup_boost_cadence_step = *pConfig++;
        m_config.battery_current_max = *pConfig++;
        m_config.target_max_battery_power = *pConfig++;
        m_config.target_max_battery_capacity = *pConfig++;
        m_config.battery_cells_number = *pConfig++;
        m_config.motor_deceleration = *pConfig++;
        m_config.battery_low_voltage_cut_off = *pConfig++;
        m_config.actual_battery_voltage_percent = *pConfig++;
        m_config.actual_battery_capacity_percent = *pConfig++;
        m_config.li_ion_cell_overvolt = ((float)*pConfig++)/100.0;
        m_config.li_ion_cell_reset_soc_percent = ((float)*pConfig++)/100.0;
        m_config.li_ion_cell_volts_full = ((float)*pConfig++)/100.0;
        m_config.li_ion_cell_volts_3_of_4 = ((float)*pConfig++)/100.0;
        m_config.li_ion_cell_volts_2_of_4 = ((float)*pConfig++)/100.0;
        m_config.li_ion_cell_volts_1_of_4 = ((float)*pConfig++)/100.0;
        m_config.li_ion_cell_volts_5_of_6 = ((float)*pConfig++)/100.0;
        m_config.li_ion_cell_volts_4_of_6 = ((float)*pConfig++)/100.0;
        m_config.li_ion_cell_volts_3_of_6 = ((float)*pConfig++)/100.0;
        m_config.li_ion_cell_volts_2_of_6 = ((float)*pConfig++)/100.0;
        m_config.li_ion_cell_volts_1_of_6 = ((float)*pConfig++)/100.0;
        m_config.li_ion_cell_volts_empty = ((float)*pConfig++)/100.0;
        m_config.wheel_perimeter = *pConfig++;
        m_config.wheel_max_speed = *pConfig++;
        m_config.enable_lights = *pConfig++;
        m_config.enable_walk_assist = *pConfig++;
        m_config.enable_brake_sensor = *pConfig++;
        m_config.enable_throttle = *pConfig++;
        m_config.enable_temperature_limit = *pConfig++;
        m_config.enable_street_mode_on_startup = *pConfig++;
        m_config.enable_set_parameter_on_startup = *pConfig++;
        m_config.enable_odometer_compensation = *pConfig++;
        m_config.startup_boost_on_startup = *pConfig++;
        m_config.torque_sensor_adv_on_startup = *pConfig++;
        m_config.lights_configuration_on_startup = *pConfig++;
        m_config.riding_mode_on_startup = *pConfig++;
        m_config.lights_configuration_1 = *pConfig++;
        m_config.lights_configuration_2 = *pConfig++;
        m_config.lights_configuration_3 = *pConfig++;
        m_config.street_mode_power_limit_enabled = *pConfig++;
        m_config.street_mode_power_limit = *pConfig++;
        m_config.street_mode_speed_limit = *pConfig++;
        m_config.street_mode_throttle_enabled = *pConfig++;
        m_config.street_mode_cruise_enabled = *pConfig++;
        m_config.adc_throttle_min_value = *pConfig++;
        m_config.adc_throttle_max_value = *pConfig++;
        m_config.motor_temperature_min_value_limit = *pConfig++;
        m_config.motor_temperature_max_value_limit = *pConfig++;
        m_config.enable_temperature_error_min_limit = *pConfig++;
        m_config.enable_vlcd6 = *pConfig++;
        m_config.enable_vlcd5 = *pConfig++;
        m_config.enable_xh18 = *pConfig++;
        m_config.enable_display_working_flag = *pConfig++;
        m_config.enable_display_always_on = *pConfig++;
        m_config.enable_wheel_max_speed_from_display = *pConfig++;
        m_config.delay_menu_on = *pConfig++;
        m_config.coaster_brake_enabled = *pConfig++;
        m_config.coaster_brake_torque_threshold = *pConfig++;
        m_config.enable_auto_data_display = *pConfig++;
        m_config.startup_assist_enabled = *pConfig++;
        m_config.auto_data_number_display = *pConfig++;
        m_config.delay_display_data_1 = *pConfig++;
        m_config.delay_display_data_2 = *pConfig++;
        m_config.delay_display_data_3 = *pConfig++;
        m_config.delay_display_data_4 = *pConfig++;
        m_config.delay_display_data_5 = *pConfig++;
        m_config.delay_display_data_6 = *pConfig++;
        m_config.display_data_1 = *pConfig++;
        m_config.display_data_2 = *pConfig++;
        m_config.display_data_3 = *pConfig++;
        m_config.display_data_4 = *pConfig++;
        m_config.display_data_5 = *pConfig++;
        m_config.display_data_6 = *pConfig++;
        m_config.power_assist_level_1 = *pConfig++;
        m_config.power_assist_level_2 = *pConfig++;
        m_config.power_assist_level_3 = *pConfig++;
        m_config.power_assist_level_4 = *pConfig++;
        m_config.torque_assist_level_1 = *pConfig++;
        m_config.torque_assist_level_2 = *pConfig++;
        m_config.torque_assist_level_3 = *pConfig++;
        m_config.torque_assist_level_4 = *pConfig++;
        m_config.cadence_assist_level_1 = *pConfig++;
        m_config.cadence_assist_level_2 = *pConfig++;
        m_config.cadence_assist_level_3 = *pConfig++;
        m_config.cadence_assist_level_4 = *pConfig++;
        m_config.emtb_assist_level_1 = *pConfig++;
        m_config.emtb_assist_level_2 = *pConfig++;
        m_config.emtb_assist_level_3 = *pConfig++;
        m_config.emtb_assist_level_4 = *pConfig++;
        m_config.walk_assist_level_1 = *pConfig++;
        m_config.walk_assist_level_2 = *pConfig++;
        m_config.walk_assist_level_3 = *pConfig++;
        m_config.walk_assist_level_4 = *pConfig++;
        m_config.walk_assist_threshold_speed_x10 = *pConfig++;
        m_config.walk_assist_debounce_enabled = *pConfig++;
        m_config.walk_assist_debounce_time = *pConfig++;
        m_config.cruise_target_speed_level_1 = *pConfig++;
        m_config.cruise_target_speed_level_2 = *pConfig++;
        m_config.cruise_target_speed_level_3 = *pConfig++;
        m_config.cruise_target_speed_level_4 = *pConfig++;
        m_config.cruise_mode_walk_enabled = *pConfig++;
        m_config.cruise_threshold_speed = *pConfig++;
        m_config.pedal_torque_adc_offset = *pConfig++;
        m_config.units_type = *pConfig++;
        m_config.assist_throttle_min_value = *pConfig++;
        m_config.assist_throttle_max_value = *pConfig++;
        m_config.street_mode_walk_enabled = *pConfig++;
        m_config.data_display_on_startup = *pConfig++;
        m_config.field_weakening_enabled = *pConfig++;
        m_config.pedal_torque_adc_offset_adj = *pConfig++;
        m_config.pedal_torque_adc_range_adj = *pConfig++;
        m_config.pedal_torque_adc_angle_adj = *pConfig++;
        m_config.pedal_torque_per_10_bit_adc_step_adv_x100 = *pConfig++;
        m_config.soc_percent_calc = *pConfig++;
        m_config.startup_boost_at_zero = *pConfig++;
        m_config.enablec850 = *pConfig++;
        m_config.street_mode_throttle_legal = *pConfig++;
        m_config.brake_temperature_switch = *pConfig++;
        m_config.emtb_based_on_power = *pConfig++;
        m_config.smooth_start_enabled = *pConfig++;
        m_config.smooth_start_set_percent = *pConfig++;
        m_config.temperature_sensor_type = *pConfig++;
        m_config.cruise_mode_enabled = *pConfig++;
        m_config.throttle_mode = *pConfig++;
        m_config.street_mode_throttle_mode = *pConfig++;
        m_config.assist_level_1_of_5_percent = *pConfig++;
        m_config.alternative_miles = *pConfig++;
    } // end of good config version
}

#endif
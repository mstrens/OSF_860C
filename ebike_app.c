/*
 * TongSheng TSDZ2 motor controller firmware
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */
#include "cybsp.h"
#include "cy_utils.h"
#include "cy_retarget_io.h"
#include "ebike_app.h"
#include "main.h"
#include "motor.h"
#include "common.h"
//#include "uart.h"

// for debug


// Motor init state
#define MOTOR_INIT_STATE_RESET                  0
#define MOTOR_INIT_STATE_NO_INIT                1
#define MOTOR_INIT_STATE_INIT_START_DELAY       2
#define MOTOR_INIT_STATE_INIT_WAIT_DELAY        3
#define MOTOR_INIT_OK                           4

// Motor init status
#define MOTOR_INIT_STATUS_RESET                 0
#define MOTOR_INIT_STATUS_GOT_CONFIG            1
#define MOTOR_INIT_STATUS_INIT_OK               2

// Communications package frame type
#define COMM_FRAME_TYPE_ALIVE                         0
#define COMM_FRAME_TYPE_STATUS                        1
#define COMM_FRAME_TYPE_PERIODIC                      2
#define COMM_FRAME_TYPE_CONFIGURATIONS                3
#define COMM_FRAME_TYPE_FIRMWARE_VERSION              4

// variables for various system functions
uint8_t ui8_m_system_state = ERROR_NOT_INIT; // start with system error because configurations are empty at startup
static uint8_t ui8_m_motor_init_state = MOTOR_INIT_STATE_RESET;
static uint8_t ui8_m_motor_init_status = MOTOR_INIT_STATUS_RESET;
static uint8_t m_ui8_got_configurations_timer = 0;

// Initial configuration values
static uint16_t ui16_battery_low_voltage_cut_off_x10 = 300; 	// 36 V battery, 30.0V (3.0 * 10)
static uint8_t ui8_voltage_cut_off_flag = 0;
static uint16_t ui16_wheel_perimeter = 2050 ;               	// 26'' wheel: 2050 mm perimeter
static uint8_t ui8_wheel_speed_max = 25;                  		// 25 Km/h
uint8_t ui8_pedal_torque_per_10_bit_ADC_step_x100 = 67;
static uint8_t ui8_target_battery_max_power_div25 = 20;    		// 500W (500/25 = 20)
static uint8_t ui8_target_battery_max_power_div25_temp = 0;
static uint8_t ui8_optional_ADC_function = 0;               	// 0 = no function

// system
static uint8_t ui8_assist_level_flag = 0;
static uint8_t ui8_riding_mode = OFF_MODE;
uint8_t ui8_riding_mode_parameter = 0;
static uint8_t ui8_walk_assist_parameter = 0;
//static uint8_t ui8_cruise_parameter = 0;
uint8_t ui8_motor_enabled = 1;
uint8_t ui8_assist_without_pedal_rotation_threshold = 0;
static uint8_t ui8_assist_without_pedal_rotation_enabled = 0;
static uint8_t ui8_lights_configuration = 0;
static uint8_t ui8_assist_with_error_enabled = 0;
uint8_t ui8_lights_state = 0;
static uint8_t ui8_field_weakening_feature_enabled = 0;
static uint8_t ui8_field_weakening_erps_delta = 0;
uint8_t ui8_error_battery_overcurrent = 0;
static uint8_t ui8_error_battery_overcurrent_counter = 0;
static uint8_t ui8_battery_overcurrent_delay = 2;
uint8_t ui8_adc_battery_overcurrent = (uint8_t)(ADC_10_BIT_BATTERY_CURRENT_MAX + ADC_10_BIT_BATTERY_EXTRACURRENT);
static uint16_t ui16_adc_voltage_shutdown = 0;
static uint8_t ui8_voltage_shutdown_flag = 0;

// power control
static uint8_t ui8_battery_current_max = DEFAULT_VALUE_BATTERY_CURRENT_MAX;
uint8_t ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
uint8_t ui8_duty_cycle_ramp_up_inverse_step_default = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
uint8_t ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
uint8_t ui8_duty_cycle_ramp_down_inverse_step_default = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
uint16_t ui16_battery_voltage_filtered_x1000 = 0;
static uint8_t ui8_battery_current_filtered_x5 = 0;
static uint8_t ui8_motor_current_filtered_x5 = 0;
static uint16_t ui16_adc_battery_voltage_filtered = 0;
uint8_t ui8_adc_battery_current_max = ADC_10_BIT_BATTERY_CURRENT_MAX;
uint8_t ui8_adc_battery_current_target = 0;
uint8_t ui8_duty_cycle_target = 0;
volatile uint16_t ui16_adc_motor_phase_current_max = ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX;

extern volatile uint16_t ui16_adc_motor_phase_current;


// Motor ERPS
uint16_t ui16_motor_speed_erps = 0;

// cadence sensor
uint16_t ui16_cadence_ticks_count_min_speed_adj = CADENCE_SENSOR_CALC_COUNTER_MIN;
static uint8_t ui8_pedal_cadence_RPM = 0;

// torque sensor
static uint16_t ui16_adc_pedal_torque_offset = ADC_TORQUE_SENSOR_OFFSET_DEFAULT;
static uint16_t ui16_adc_pedal_torque_offset_init = ADC_TORQUE_SENSOR_OFFSET_DEFAULT;
static uint16_t ui16_adc_pedal_torque_offset_cal = ADC_TORQUE_SENSOR_OFFSET_DEFAULT;
static uint16_t ui16_adc_pedal_torque_offset_set = ADC_TORQUE_SENSOR_OFFSET_DEFAULT;
static uint16_t ui16_adc_pedal_torque_offset_min = ADC_TORQUE_SENSOR_OFFSET_DEFAULT - ADC_TORQUE_SENSOR_OFFSET_THRESHOLD;
static uint16_t ui16_adc_pedal_torque_offset_max = ADC_TORQUE_SENSOR_OFFSET_DEFAULT + ADC_TORQUE_SENSOR_OFFSET_THRESHOLD;
static uint8_t ui8_adc_pedal_torque_offset_error = 0;
static uint8_t ui8_adc_torque_calibration_offset = 0;
static uint8_t ui8_adc_torque_middle_offset_adj = 0;
static uint8_t ui8_adc_pedal_torque_offset_adj = 0;
static uint8_t ui8_adc_pedal_torque_delta_adj = 0;
static uint8_t ui8_adc_pedal_torque_range_adj = 0;
static uint16_t ui16_adc_pedal_torque_range = 0;
static uint16_t ui16_adc_pedal_torque_range_ingrease_x100 = 0;
static uint8_t ui8_adc_pedal_torque_angle_adj = 0;
static uint16_t ui16_adc_pedal_torque_range_target_max = 0;
uint16_t ui16_adc_coaster_brake_threshold = 0;
uint8_t ui8_coaster_brake_enabled = 0;
static uint8_t ui8_coaster_brake_torque_threshold = 0;
static uint16_t ui16_adc_pedal_torque = 0;
static uint16_t ui16_adc_pedal_torque_delta = 0;
static uint16_t ui16_adc_pedal_torque_delta_temp = 0;
static uint16_t ui16_adc_pedal_torque_delta_no_boost = 0;
static uint16_t ui16_pedal_torque_x100 = 0;
static uint8_t ui8_torque_sensor_calibration_enabled = 0;
static uint8_t ui8_hybrid_torque_parameter = 0;
static uint8_t ui8_eMTB_based_on_power = 1;

// wheel speed sensor
static uint16_t ui16_wheel_speed_x10 = 0;
static uint8_t ui8_speed_limit_high_exceeded = 0;

// motor temperature control
static uint16_t ui16_motor_temperature_filtered_x10 = 0;
static uint8_t ui8_motor_temperature_max_value_to_limit = 0;
static uint8_t ui8_motor_temperature_min_value_to_limit = 0;

// throttle control
static uint8_t ui8_adc_throttle_assist = 0;
static uint8_t ui8_throttle_adc_in = 0;
static uint8_t ui8_throttle_virtual = 0;
static uint8_t ui8_throttle_legal = 0;
static uint8_t ui8_throttle_feature_enabled = 0;

// cruise control
static uint8_t ui8_cruise_PID_initialize = 1;
static uint16_t ui16_wheel_speed_target_received_x10 = 0;
static uint8_t ui8_cruise_legal = 0;

// walk assist
static uint8_t ui8_walk_assist_speed_target_x10 = 0;
static uint8_t ui8_walk_assist_duty_cycle_counter = 0;
static uint8_t ui8_walk_assist_duty_cycle_target = WALK_ASSIST_DUTY_CYCLE_MIN;
static uint8_t ui8_walk_assist_duty_cycle_max = WALK_ASSIST_DUTY_CYCLE_MIN;
static uint8_t ui8_walk_assist_adj_delay = WALK_ASSIST_ADJ_DELAY_MIN;
static uint16_t ui16_walk_assist_wheel_speed_counter = 0;
static uint16_t ui16_walk_assist_erps_target = 0;
static uint16_t ui16_walk_assist_erps_min = 0;
static uint16_t ui16_walk_assist_erps_max = 0;
static uint8_t ui8_walk_assist_speed_flag = 0;

// startup boost
static uint8_t ui8_startup_boost_enabled = 0;
static uint8_t ui8_startup_boost_at_zero = 0;
static uint8_t ui8_startup_boost_flag = 0;
static uint16_t ui16_startup_boost_factor_array[120];
static uint8_t ui8_startup_boost_cadence_step = 0;

// smooth start
static uint8_t ui8_smooth_start_enabled = 1;
static uint8_t ui8_smooth_start_flag = 0;
static uint8_t ui8_smooth_start_counter = 0;
static uint8_t ui8_smooth_start_counter_set = SMOOTH_START_RAMP_DEFAULT;
static uint8_t ui8_smooth_start_counter_set_temp = SMOOTH_START_RAMP_DEFAULT;

// startup assist
static uint8_t ui8_startup_assist_flag = 0;
static uint8_t ui8_startup_assist_adc_battery_current_target = 0;

// UART
#define UART_NUMBER_DATA_BYTES_TO_RECEIVE   88
#define UART_NUMBER_DATA_BYTES_TO_SEND      29

volatile uint8_t ui8_received_package_flag = 0;
volatile uint8_t ui8_rx_buffer[UART_NUMBER_DATA_BYTES_TO_RECEIVE];
//volatile uint8_t ui8_rx_counter = 0;
volatile uint8_t ui8_rx_cnt = 0;
volatile uint8_t ui8_rx_len = 0;
volatile uint8_t ui8_tx_buffer[UART_NUMBER_DATA_BYTES_TO_SEND];
// initialize the ui8_tx_counter like at the end of send operation to enable the first send.
//volatile uint8_t ui8_tx_counter = UART_NUMBER_DATA_BYTES_TO_SEND + 1;
static volatile uint8_t ui8_m_tx_buffer_index;
volatile uint8_t ui8_i;
volatile uint8_t ui8_byte_received;
volatile uint8_t ui8_state_machine = 0;
static uint16_t ui16_crc_rx;
static uint16_t ui16_crc_tx;
//volatile uint8_t ui8_message_ID = 0;
static uint8_t ui8_comm_error_counter = 0;

static void communications_controller(void);
static void communications_process_packages(uint8_t ui8_frame_type);
//static void uart_receive_package(void);
//static void uart_send_package(void);
		
// system functions
static void get_battery_voltage(void);
static void get_pedal_torque(void);
static void calc_wheel_speed(void);
static void calc_cadence(void);

static void ebike_control_lights(void);
static void ebike_control_motor(void);
static void check_system(void);

static void set_motor_ramp(void);
static void apply_startup_boost(void);
static void apply_smooth_start(void);

static void apply_power_assist(void);
static void apply_torque_assist(void);
static void apply_cadence_assist(void);
static void apply_emtb_assist(void);
static void apply_hybrid_assist(void);
static void apply_cruise(void);
static void apply_walk_assist(void);
static void apply_calibration_assist(void);
static void apply_throttle(void);
static void apply_temperature_limiting(void);
static void apply_speed_limit(void);

uint8_t ui8_pwm_duty_cycle_max;
// added for using testing mode
uint8_t ui8_test_mode_flag = DEFAULT_TEST_MODE_FLAG ; // can be changed in uc_probe
uint8_t ui8_battery_current_target_testing = DEFAULT_BATTERY_CURRENT_TARGET_TESTING_A ; // value is in A ; this is a default value that can be changed with uc_probe
uint8_t ui8_duty_cycle_target_testing = DEFAULT_DUTY_CYCLE_TARTGET_TESTING; // max is 245, this is a default value that can be changed with uc_probe
#define AVERAGING_CNT 40 // 25 msec per cycle; 40 = 1 sec
uint32_t ui32_adc_battery_current_filtered_15b_accumulated =0;
uint32_t ui32_adc_battery_current_filtered_15b_cnt = AVERAGING_CNT;
uint32_t ui32_battery_current_filtered_avg_mA ;
	


/*
// ********************* init ******************************
void ebike_app_init(void)
{
	// minimum value for these displays
	if ((m_config.enable_vlcd6) || (m_config.enablec850) ){//#if ENABLE_VLCD6 || ENABLE_850C
		if (ui8_delay_display_function < 70) {
			ui8_delay_display_function = 70;
		}
	}
	
	// set low voltage cutoff (16 bit) ; 39V => 390*100/87= 448adcfor 48V battery
	ui16_adc_voltage_cut_off = ((uint32_t) m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 * 100U) /
		 BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000;
	
	// check if assist without pedal rotation threshold is valid (safety)
	if (ui8_assist_without_pedal_rotation_threshold > 100) {
		ui8_assist_without_pedal_rotation_threshold = 100;
	}
	// set duty cycle ramp up inverse step default
	ui8_duty_cycle_ramp_up_inverse_step_default = map_ui8((uint8_t) m_config.motor_acceleration, //35
				(uint8_t) 0,
				(uint8_t) 100,
				(uint8_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT, //194
				(uint8_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN); //24
	
	// set duty cycle ramp down inverse step default
	ui8_duty_cycle_ramp_down_inverse_step_default = map_ui8((uint8_t) m_config.motor_deceleration, // 35
				(uint8_t) 0,
                (uint8_t) 100,
                (uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,    // 73
                (uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);       // 9
	
	// Smooth start counter set ; counter is e.g. about 160
	ui8_smooth_start_counter_set = map_ui8((uint8_t) m_config.smooth_start_set_percent, //35
				(uint8_t) 0,
                (uint8_t) 100,
                (uint8_t) 255,
                (uint8_t) SMOOTH_START_RAMP_MIN);                             //30
	
	// set pedal torque per 10_bit DC_step x100 advanced (calibrated) or default(not calibrated)
	ui8_pedal_torque_per_10_bit_ADC_step_x100_array[TORQUE_STEP_DEFAULT] = m_config.pedal_torque_per_10_bit_adc_step_x100; // PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100; // 67
	if (ui8_torque_sensor_calibrated) {
		ui8_pedal_torque_per_10_bit_ADC_step_x100_array[TORQUE_STEP_ADVANCED] = m_config.pedal_torque_per_10_bit_adc_step_adv_x100;//  PEDAL_TORQUE_PER_10_BIT_ADC_STEP_ADV_X100; //34
	}
	else {
		ui8_pedal_torque_per_10_bit_ADC_step_x100_array[TORQUE_STEP_ADVANCED] = m_config.pedal_torque_per_10_bit_adc_step_x100;//  PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100;
	}
	
	// parameters status on startup
	// set parameters on startup                         Currently m_configuration_variables are taken from eeprom.c (to be changed later on)
	ui8_display_function_status[0][OFF] = m_configuration_variables.ui8_set_parameter_enabled;
	// auto display data on startup
	ui8_display_function_status[1][OFF] = m_configuration_variables.ui8_auto_display_data_enabled;
	// street mode on startup
	ui8_display_function_status[0][ECO] = m_configuration_variables.ui8_street_mode_enabled;
	// startup boost on startup
	ui8_display_function_status[1][ECO] = m_configuration_variables.ui8_startup_boost_enabled;
	// torque sensor adv on startup
	ui8_display_function_status[2][ECO] = m_configuration_variables.ui8_torque_sensor_adv_enabled;
	// assist without pedal rotation on startup
	ui8_display_function_status[1][TURBO] = m_configuration_variables.ui8_assist_without_pedal_rotation_enabled;
	// system error enabled on startup
	ui8_display_function_status[2][TURBO] = m_configuration_variables.ui8_assist_with_error_enabled;
	// riding mode on startup
	ui8_display_riding_mode = m_configuration_variables.ui8_riding_mode;
	// lights configuration on startup
	ui8_display_lights_configuration = m_configuration_variables.ui8_lights_configuration;
	
	// percentage remaining battery capacity x10 at power on
	ui16_battery_SOC_percentage_x10 = ((uint16_t) m_configuration_variables.ui8_battery_SOC_percentage_8b) << 2;
		 
	// battery SOC checked at power on
	if (ui16_battery_SOC_percentage_x10) {
		// calculate watt-hours x10 at power on
		ui32_wh_x10_offset = ((uint32_t)(1000 - ui16_battery_SOC_percentage_x10) * ui16_actual_battery_capacity) / 100;
		
		ui8_battery_SOC_init_flag = 1;
	}

	// make startup boost array This array start with a high value and decrease gradually
	ui16_startup_boost_factor_array[0] = m_config.startup_boost_torque_factor; //  STARTUP_BOOST_TORQUE_FACTOR; //300
	uint8_t ui8_i;
	for (ui8_i = 1; ui8_i < 120; ui8_i++)
	{
		uint16_t ui16_temp = (ui16_startup_boost_factor_array[ui8_i - 1] * m_config.startup_boost_cadence_step) >> 8;//delta*20/256   Sbased on TARTUP_BOOST_CADENCE_STEP
		ui16_startup_boost_factor_array[ui8_i] = ui16_startup_boost_factor_array[ui8_i - 1] - ui16_temp;
	}
	
	// enable data displayed on startup
	//#if DATA_DISPLAY_ON_STARTUP
	if (m_config.data_display_on_startup) {
		ui8_display_data_enabled = 1;
	}	
	

	// calculate max adc battery current from the received battery current limit // 13*100/16 = 81
	ui8_adc_battery_current_max_temp_1 = (uint8_t)((uint16_t)(m_configuration_variables.ui8_battery_current_max * 100U) 
		/ BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100); //16 means 0,16A per adc step (so for TSDZ2 it was 13 * 100 /16 = 81 adc steps)

	// calculate the max adc battery power from the power limit received in offroad mode // 500 *100*1000/16
	ui32_adc_battery_power_max_x1000_array[OFFROAD_MODE] = (uint32_t)((uint32_t) m_config.target_max_battery_power * 100U * 1000U) //   TARGET_MAX_BATTERY_POWER
		/ BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100; //16 means 0,16A per adc step
	
	// calculate the max adc battery power from the received power limit in street mode
	ui32_adc_battery_power_max_x1000_array[STREET_MODE] = (uint32_t)((uint32_t) m_config.street_mode_power_limit * 100U * 1000U) //  STREET_MODE_POWER_LIMIT
		/ BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100; //16
	
	// set max motor phase current // used in motor.c to perform some checks // 
	uint16_t ui16_temp = ui8_adc_battery_current_max_temp_1 * ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX;//187 (if 30A); temp = 81*187 = 15147
	ui8_adc_motor_phase_current_max = (uint8_t)(ui16_temp / ADC_10_BIT_BATTERY_CURRENT_MAX); //112 (if 18A) so 15147/112 = 135 for TSDZ2
	// limit max motor phase current if higher than configured hardware limit (safety)
	if (ui8_adc_motor_phase_current_max > ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX) { //187
		ui8_adc_motor_phase_current_max = ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX;  //187
	}
}
*/

void ebike_app_controller(void) // is called every 25ms by main()
{
	// calculate motor ERPS = electrical rotation per sec ; ui16_hall_counter_total is the number of tick (4usec/tick) for a full electric rotation
    // 0x8000 was the value for TSDZ2; 
	// TSDZ8 should test on a value that is 2 * because there is 4 poles instead of 8 and so it takes more ticks for the same mecanical speed
	//if ((ui16_hall_counter_total >= 0x8000 ) || (ui16_hall_counter_total < 10)) { For TSDZ2
	// > 0X8000 = >32000 ; *4 usec = 0,131 sec per electric rotation ; for TSDZ2 * 8 = 1 sec per rotation = 60 rotations mecanical /sec
	// normally this should not happens because there is already a check in motor.c that set ui16_hall_counter_total = 0xffff when enlapsed time is more than a value
	// So, we should not exceed a uint16_t variable
	if ((ui16_hall_counter_total >= 0xF000 ) || (ui16_hall_counter_total < 10)) {
        ui16_motor_speed_erps = 0;  // speed is 0 if number of ticks is to high
    }
	else 
	{
        //ui16_motor_speed_erps = (uint16_t)(HALL_COUNTER_FREQ >> 2) / (uint16_t)(ui16_tmp >> 2); // 250000/nrOfTicks; so in sec
		ui16_motor_speed_erps = ((uint32_t) HALL_COUNTER_FREQ) / ui16_hall_counter_total; // 250000/nrOfTicks; so rotation in sec
	}
	// calculate the wheel speed
	calc_wheel_speed();
	
	// calculate the cadence and set limits from wheel speed
	calc_cadence();

	// Calculate filtered Battery Voltage (mV)
    get_battery_voltage(); // get a ui16_adc_voltage filtered value and convert it in mv with ui16_battery_voltage_filtered_x1000
	
    
    // Calculate filtered Battery Current (Ampx5)
    ui8_battery_current_filtered_x5 = (uint8_t)(((uint16_t) ui8_adc_battery_current_filtered * BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100) / 20);
	
	// Calculate filtered Motor Current (Ampx5)
    ui8_motor_current_filtered_x5 = (uint8_t)(((uint16_t) ui16_adc_motor_phase_current * BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100) / 20);
	
	// get pedal torque ; calculate ui16_pedal_torque_x100 and ui16_human_power_x10 (human power)
	get_pedal_torque();
	
	// send/receive data, ebike control lights, calc oem wheelspeed, 
	// check system, check battery soc, every 4 cycles (25ms * 4)
	
    static uint8_t ui8_counter;

	switch (ui8_counter++ & 0x03) {
		case 0: 
			communications_controller();
			break;
		case 1:
			ebike_control_lights();
			break;
		case 2:
			communications_controller(); 
			break;
		case 3:
			check_system();
			break;
	}
	
	// use received data and sensor input to control motor
    ebike_control_motor();

    /*------------------------------------------------------------------------

     NOTE: regarding function call order

     Do not change order of functions if not absolutely sure it will
     not cause any undesirable consequences.

     ------------------------------------------------------------------------*/
}


static void ebike_control_motor(void) // is called every 25ms by ebike_app_controller()
{
    // reset control variables (safety)
    ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;     // 194
    ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;  //73
    ui8_adc_battery_current_target = 0;
    ui8_duty_cycle_target = 0;
	
	// field weakening enable
	if ((ui8_field_weakening_feature_enabled)
		&& (ui16_motor_speed_erps > MOTOR_SPEED_FIELD_WEAKENING_MIN)
		&& (ui8_adc_battery_current_filtered < ui8_controller_adc_battery_current_target)
		&& (!ui8_adc_throttle_assist)) {
			ui8_field_weakening_erps_delta = ui16_motor_speed_erps - MOTOR_SPEED_FIELD_WEAKENING_MIN;
			ui8_fw_hall_counter_offset_max = ui8_field_weakening_erps_delta >> 5;
			if (ui8_fw_hall_counter_offset_max > FW_HALL_COUNTER_OFFSET_MAX) {
				ui8_fw_hall_counter_offset_max = FW_HALL_COUNTER_OFFSET_MAX;
			}
			ui8_field_weakening_enabled = 1;
	}
	else {
		ui8_field_weakening_enabled = 0;
	}

	// added by mstrens (test mode flag)
	 // ********************* here the 2 main ways to run the motor (one for test/calibration, the other for normal use) *****************
	if (ui8_test_mode_flag == NORMAL_RUNNING_MODE) {
    // select riding mode and calculate ui8_adc_battery_current_target and ui8_duty_cycle_target is 255 (or 0)
    //        It also adapt the ramp up and down inverse step that has an impact on how fast the motor react to a change.
		switch (ui8_riding_mode) {
			case POWER_ASSIST_MODE: apply_power_assist(); break;
			case TORQUE_ASSIST_MODE: apply_torque_assist(); break;
			case CADENCE_ASSIST_MODE: apply_cadence_assist(); break;
			case eMTB_ASSIST_MODE: apply_emtb_assist(); break;
			case HYBRID_ASSIST_MODE: apply_hybrid_assist(); break;
			case CRUISE_MODE: apply_cruise(); break;
			case WALK_ASSIST_MODE: apply_walk_assist(); break;
			case MOTOR_CALIBRATION_MODE: apply_calibration_assist(); break;
		}
	} else { // we are in testing mode 
		// for testing, we force the 4 parameters used to control the motor
		ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;     // 194
		ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;  //73
		// Note : this code suppose that Throttle is connected ?? 
		// set current target to the testing value and check with max
		ui8_adc_battery_current_target = (uint16_t) ui8_battery_current_target_testing *100 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;
		if (ui8_adc_battery_current_target > ADC_10_BIT_BATTERY_CURRENT_MAX)
				ui8_adc_battery_current_target = ADC_10_BIT_BATTERY_CURRENT_MAX;
		// set duty cycle target to the tesing value and check against max
		ui8_duty_cycle_target = ui8_duty_cycle_target_testing;
		if (ui8_duty_cycle_target >= ui8_pwm_duty_cycle_max ) {
			ui8_duty_cycle_target = ui8_pwm_duty_cycle_max;  
		}
		ui8_riding_mode_parameter =  50; // if it is set on 0 , it means that there is no assist and motor stays/goes off in safety checks
	}

    // select optional ADC function
    switch (ui8_optional_ADC_function) {
		case THROTTLE_CONTROL:
			apply_throttle();
			break;
		case TEMPERATURE_CONTROL:
			apply_temperature_limiting();
			if (ui8_throttle_virtual) {apply_throttle();}
			break;
		default:
			if (ui8_throttle_virtual) {apply_throttle();}
			break;
    }

    // speed limit :  reduce ui8_adc_battery_current_target progressively (up to 0) when close to speed limit (or exceed)
    apply_speed_limit();
	
	// check if motor init delay has to be done (from v.1.1.0)
	switch (ui8_m_motor_init_state)	{
	  case MOTOR_INIT_STATE_INIT_START_DELAY:
		m_ui8_got_configurations_timer = 40;
		ui8_m_motor_init_state = MOTOR_INIT_STATE_INIT_WAIT_DELAY;
		// no break to execute next code

	  case MOTOR_INIT_STATE_INIT_WAIT_DELAY:
		if (m_ui8_got_configurations_timer > 0) {
			m_ui8_got_configurations_timer--;
		}
		else {
			ui8_m_motor_init_state = MOTOR_INIT_OK;
			ui8_m_motor_init_status = MOTOR_INIT_STATUS_INIT_OK;
			ui8_m_system_state &= ~ERROR_NOT_INIT;
		}
	  break;
	}
	// Check battery voltage if lower than shutdown value (safety)
	if ((ui16_adc_voltage < ui16_adc_voltage_shutdown)
	  &&(ui8_m_motor_init_state == MOTOR_INIT_OK)) {
		ui8_m_system_state |= ERROR_FATAL; // Undervoltage
		ui8_voltage_shutdown_flag = 1;
	}
	
	// Check battery Over-current (read current here in case PWM interrupt for some error was disabled)
	//the resistance/gain in TSDZ8 is 4X smaller than in TSDZ2; still ADC is 12 bits instead of 10; so ADC 12bits TSDZ8 = ADC 10 bits TSDZ2
	// in TSDZ2, we used only the 8 lowest bits of adc; 1 adc step = 0,16A
	// In tsdz8, the resistance is (I expect) 0.003 Ohm ; So 1A => 0,003V => 0,03V (gain aop is 10)*4096/5Vcc = 24,576 steps
	//      SO 1 adc step = 1/24,576 = 0,040A
	// For 10 A, TSDZ2 should gives 10/0,16 = 62 steps
	// For 10 A, TSDZ8 shoud give 10*24,576 steps
	// to convert TSDZ8 steps in the same units as TSDZ2, we shoud take ADC *62/245,76 = 0,25 and divide by 4 (or >>2)
	// current is available in gr0 ch1 result 8 in queue 0 p2.8 and/or in gr0 ch0 result in 12 (p2.8)
	// here we take the average of the 2 conversions and so we should use >>3 instead of >>2
	// Still due to IIR filtering, we have to add >>2 because it is returned in 14 bits instead of 12
	uint8_t ui8_temp_adc_current = ((XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 15 ) & 0xFFFF) +
									(XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 15 ) & 0xFFFF)) >>5  ;  // >>2 for IIR, >>2 for ADC12 to ADC10 , >>1 for averaging		
	if ( ui8_temp_adc_current > ui8_adc_battery_overcurrent){ // 112+50 in tsdz2 (*0,16A) => 26A
		ui8_error_battery_overcurrent = ERROR_BATTERY_OVERCURRENT ;
	}
	  
    	/*
		// Read in assembler to ensure data consistency (conversion overrun)
		// E07 (E04 blinking for XH18)
		#ifndef __CDT_PARSER__ // avoid Eclipse syntax check
		__asm
			ld a, 0x53eb // ADC1->DB5RL
			cp a, _ui8_adc_battery_overcurrent
			jrc 00011$
			mov _ui8_error_battery_overcurrent+0, #ERROR_BATTERY_OVERCURRENT
		00011$:
		__endasm;
		#endif
		*/
	if (ui8_battery_overcurrent_delay > 0) {  // OVERCURRENT_DELAY
		if (ui8_error_battery_overcurrent) {
			ui8_error_battery_overcurrent_counter++;
		}
		else {
			ui8_error_battery_overcurrent_counter = 0;
		}
		if (ui8_error_battery_overcurrent_counter >= ui8_battery_overcurrent_delay) {
			ui8_m_system_state |= ui8_error_battery_overcurrent;
		}
	}
	
	// calculate an average current in mA (to find parameters giving lowest current)
	ui32_adc_battery_current_filtered_15b_accumulated += ui32_adc_battery_current_filtered_15b;
	ui32_adc_battery_current_filtered_15b_cnt--;
	if (ui32_adc_battery_current_filtered_15b_cnt == 0){
		ui32_battery_current_filtered_avg_mA = (ui32_adc_battery_current_filtered_15b_accumulated  *1000 
				/ AVERAGING_CNT / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100) >> 5;
		ui32_adc_battery_current_filtered_15b_cnt = AVERAGING_CNT ;
		ui32_adc_battery_current_filtered_15b_accumulated = 0;
	}

    // reset control parameters if... (safety)
    if ((ui8_brake_state)
	  || (ui8_m_system_state & ERROR_MOTOR_BLOCKED)
	  || (ui8_m_system_state & ERROR_BATTERY_OVERCURRENT)
	  || (ui8_m_system_state & ERROR_THROTTLE)
	  || (ui8_m_system_state & ERROR_FATAL)
	  || (!ui8_motor_enabled)
	  || (!ui8_assist_level_flag)
	  || ((ui8_m_system_state)&&(!ui8_assist_with_error_enabled))) {
        ui8_controller_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
		ui8_controller_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
		ui8_controller_adc_battery_current_target = 0;
		ui8_controller_duty_cycle_target = 0;
    }
	else { // motor can run (no safety issue)
        // limit max current if higher than configured hardware limit (safety)
        if (ui8_adc_battery_current_max > ADC_10_BIT_BATTERY_CURRENT_MAX) {
            ui8_adc_battery_current_max = ADC_10_BIT_BATTERY_CURRENT_MAX;
        }

        // limit target current if higher than max value (safety)
        if (ui8_adc_battery_current_target > ui8_adc_battery_current_max) {
            ui8_adc_battery_current_target = ui8_adc_battery_current_max;
        }

        // limit target duty cycle if higher than max value
        if (ui8_duty_cycle_target > PWM_DUTY_CYCLE_MAX) {
            ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
        }
		
        // limit target duty cycle ramp up inverse step if lower than min value (safety)
        if (ui8_duty_cycle_ramp_up_inverse_step < PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN) {
            ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
        }

        // limit target duty cycle ramp down inverse step if lower than min value (safety)
        if (ui8_duty_cycle_ramp_down_inverse_step < PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN) {
            ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
        }
		
        // set duty cycle ramp up in controller
        ui8_controller_duty_cycle_ramp_up_inverse_step = ui8_duty_cycle_ramp_up_inverse_step;

        // set duty cycle ramp down in controller
        ui8_controller_duty_cycle_ramp_down_inverse_step = ui8_duty_cycle_ramp_down_inverse_step;

        // set target battery current in controller
        ui8_controller_adc_battery_current_target = ui8_adc_battery_current_target;

        // set target duty cycle in controller
        ui8_controller_duty_cycle_target = ui8_duty_cycle_target;
	}
	
    // check if the motor should be enabled or disabled
    if (ui8_motor_enabled
		&& ((ui8_m_system_state & ERROR_MOTOR_BLOCKED)
			|| (ui8_m_system_state & ERROR_BATTERY_OVERCURRENT)
			|| (ui8_m_system_state & ERROR_THROTTLE)
			|| (ui8_m_system_state & ERROR_FATAL)
			|| ((ui16_motor_speed_erps == 0)
				&& (!ui8_adc_battery_current_target)
				&& (!ui8_g_duty_cycle)))) {
        ui8_motor_enabled = 0;
        motor_disable_pwm();
    }
	else if (!ui8_motor_enabled
			&& (ui16_motor_speed_erps < ERPS_SPEED_OF_MOTOR_REENABLING) // enable the motor only if it rotates slowly or is stopped
			&& (ui8_adc_battery_current_target)
			&& (!ui8_brake_state)) {
		ui8_motor_enabled = 1;
		ui8_g_duty_cycle = PWM_DUTY_CYCLE_STARTUP;
		//ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
		//ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
		ui8_fw_hall_counter_offset = 0;
		motor_enable_pwm();
	}

	/*  is not in 860C tsdz2 version ?????
	// current limit with power limit
	ui8_adc_battery_current_max_temp_2 = (uint8_t)((uint32_t)(ui32_adc_battery_power_max_x1000_array[m_configuration_variables.ui8_street_mode_enabled]
		/ ui16_battery_voltage_filtered_x1000));
	
	// set max battery current
	ui8_adc_battery_current_max = ui8_min(ui8_adc_battery_current_max_temp_1, ui8_adc_battery_current_max_temp_2);
			
	mstest6=ui8_adc_battery_current_target;
	*/
}


// calculate motor ramp depending on speed and cadence
static void set_motor_ramp(void)
{
	uint8_t ui8_tmp;
	if (ui16_wheel_speed_x10 >= 200) {
        ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
        ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
    }
	else {
        ui8_duty_cycle_ramp_up_inverse_step = map_ui8((uint8_t)(ui16_wheel_speed_x10>>2),
                (uint8_t)10, // 10*4 = 40 -> 4 kph
                (uint8_t)50, // 50*4 = 200 -> 20 kph
                (uint8_t)ui8_duty_cycle_ramp_up_inverse_step_default,
                (uint8_t)PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);
        ui8_tmp = map_ui8(ui8_pedal_cadence_RPM,
                (uint8_t)20, // 20 rpm
                (uint8_t)70, // 70 rpm
                (uint8_t)ui8_duty_cycle_ramp_up_inverse_step_default,
                (uint8_t)PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);
        if (ui8_tmp < ui8_duty_cycle_ramp_up_inverse_step) {
            ui8_duty_cycle_ramp_up_inverse_step = ui8_tmp;
		}
        ui8_duty_cycle_ramp_down_inverse_step = map_ui8((uint8_t)(ui16_wheel_speed_x10>>2),
                (uint8_t)10, // 10*4 = 40 -> 4 kph
                (uint8_t)50, // 50*4 = 200 -> 20 kph
                (uint8_t)ui8_duty_cycle_ramp_down_inverse_step_default,
                (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
        ui8_tmp = map_ui8(ui8_pedal_cadence_RPM,
                (uint8_t)20, // 20 rpm
                (uint8_t)70, // 70 rpm
                (uint8_t)ui8_duty_cycle_ramp_down_inverse_step_default,
                (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
        if (ui8_tmp < ui8_duty_cycle_ramp_down_inverse_step) {
            ui8_duty_cycle_ramp_down_inverse_step = ui8_tmp;
		}
    }
}



// calculate startup boost & new pedal torque delta
static void apply_startup_boost(void)
{
	// startup boost mode
	switch (ui8_startup_boost_at_zero) {
		case CADENCE:
			ui8_startup_boost_flag = 1;
			break;
		case SPEED:
			if (!ui16_wheel_speed_x10) {
				ui8_startup_boost_flag = 1;
			}
			else if (ui8_pedal_cadence_RPM > 45) {
				ui8_startup_boost_flag = 0;
			}
			break;
	}
	// pedal torque delta & startup boost
	if (ui8_startup_boost_flag) {
		uint32_t ui32_temp = ((uint32_t)(ui16_adc_pedal_torque_delta * ui16_startup_boost_factor_array[ui8_pedal_cadence_RPM])) / 100;
		ui16_adc_pedal_torque_delta += (uint16_t) ui32_temp;
	}
}


// calculate smooth start & new pedal torque delta
static void apply_smooth_start(void)
{
	if ((!ui8_pedal_cadence_RPM)&&(!ui16_motor_speed_erps)) {
		ui8_smooth_start_flag = 1;
		ui8_smooth_start_counter = ui8_smooth_start_counter_set;
	}
	else if (ui8_smooth_start_flag) {
		if (ui8_smooth_start_counter > 0) {
			ui8_smooth_start_counter--;
		}
		else {
			ui8_smooth_start_flag = 0;
		}
		// pedal torque delta & smooth start
		uint16_t ui16_temp = 100 - ((ui8_smooth_start_counter * 100) / ui8_smooth_start_counter_set);
		ui16_adc_pedal_torque_delta = (ui16_adc_pedal_torque_delta * ui16_temp) / 100;
	}
}


static void apply_power_assist(void)
{
	uint8_t ui8_power_assist_multiplier_x50 = ui8_riding_mode_parameter;
	
	// check for assist without pedal rotation when there is no pedal rotation
	if (ui8_assist_without_pedal_rotation_enabled) {
		if ((!ui8_pedal_cadence_RPM) &&
		   (ui16_adc_pedal_torque_delta > (120 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 1;
		}
	}
	
  	// startup boost
	if (ui8_startup_boost_enabled) {
		apply_startup_boost();
	}
	
	if ((ui8_pedal_cadence_RPM)||(ui8_startup_assist_adc_battery_current_target)) {
		// calculate torque on pedals + torque startup boost
		uint32_t ui32_pedal_torque_x100 = (uint32_t)(ui16_adc_pedal_torque_delta * ui8_pedal_torque_per_10_bit_ADC_step_x100);
	
		// calculate power assist by multiplying human power with the power assist multiplier
		uint32_t ui32_power_assist_x100 = (((uint32_t)(ui8_pedal_cadence_RPM * ui8_power_assist_multiplier_x50))
            * ui32_pedal_torque_x100) >> 9; // see note below

		/*------------------------------------------------------------------------

		NOTE: regarding the human power calculation

		(1) Formula: pedal power = torque * rotations per second * 2 * pi
		(2) Formula: pedal power = torque * rotations per minute * 2 * pi / 60
		(3) Formula: pedal power = torque * rotations per minute * 0.1047
		(4) Formula: pedal power = torque * 100 * rotations per minute * 0.001047
		(5) Formula: pedal power = torque * 100 * rotations per minute / 955
		(6) Formula: pedal power * 100  =  torque * 100 * rotations per minute * (100 / 955)
		(7) Formula: assist power * 100  =  torque * 100 * rotations per minute * (100 / 955) * (ui8_power_assist_multiplier_x50 / 50)
		(8) Formula: assist power * 100  =  torque * 100 * rotations per minute * (2 / 955) * ui8_power_assist_multiplier_x50
		(9) Formula: assist power * 100  =  torque * 100 * rotations per minute * ui8_power_assist_multiplier_x50 / 480

		------------------------------------------------------------------------*/

		// calculate target current
		uint32_t ui32_battery_current_target_x100 = (ui32_power_assist_x100 * 1000) / ui16_battery_voltage_filtered_x1000;
	
		// set battery current target in ADC steps
		uint16_t ui16_adc_battery_current_target = (uint16_t)ui32_battery_current_target_x100 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;
	
		// set motor acceleration / deceleration
		set_motor_ramp();
	
		// set battery current target
		if (ui16_adc_battery_current_target > ui8_adc_battery_current_max) {
			ui8_adc_battery_current_target = ui8_adc_battery_current_max;
		}
		else {
			ui8_adc_battery_current_target = ui16_adc_battery_current_target;
		}
	
		// set startup assist battery current target
		if (ui8_startup_assist_flag) {
			if (ui8_adc_battery_current_target > ui8_startup_assist_adc_battery_current_target) {
				ui8_startup_assist_adc_battery_current_target = ui8_adc_battery_current_target;
			}
			ui8_adc_battery_current_target = ui8_startup_assist_adc_battery_current_target;
		}
		else {
			ui8_startup_assist_adc_battery_current_target = 0;
		}
	
		// set duty cycle target
		if (ui8_adc_battery_current_target) {
			ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
		}
		else {
			ui8_duty_cycle_target = 0;
		}
	}
}


static void apply_torque_assist(void)
{
	// smooth start
	if (ui8_smooth_start_enabled) {
		apply_smooth_start();
	}
	
	// check for assist without pedal rotation when there is no pedal rotation
	if (ui8_assist_without_pedal_rotation_enabled) {
		if ((!ui8_pedal_cadence_RPM)&&
			(ui16_adc_pedal_torque_delta > (120 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 1;
		}
	}
	
	if (ui8_assist_with_error_enabled) {
		ui8_pedal_cadence_RPM = 1;
	}

    // calculate torque assistance
    if (((ui16_adc_pedal_torque_delta)&&(ui8_pedal_cadence_RPM))
	  ||(ui8_startup_assist_adc_battery_current_target)) {
        // get the torque assist factor
        uint8_t ui8_torque_assist_factor = ui8_riding_mode_parameter;

        // calculate torque assist target current
        uint16_t ui16_adc_battery_current_target_torque_assist = ((uint16_t) ui16_adc_pedal_torque_delta
                * ui8_torque_assist_factor) / TORQUE_ASSIST_FACTOR_DENOMINATOR;

        // set motor acceleration / deceleration
		set_motor_ramp();
		
        // set battery current target
        if (ui16_adc_battery_current_target_torque_assist > ui8_adc_battery_current_max) {
            ui8_adc_battery_current_target = ui8_adc_battery_current_max;
        }
		else {
            ui8_adc_battery_current_target = ui16_adc_battery_current_target_torque_assist;
        }
		
		// set startup assist battery current target
		if (ui8_startup_assist_flag) {
			if (ui8_adc_battery_current_target > ui8_startup_assist_adc_battery_current_target) {
				ui8_startup_assist_adc_battery_current_target = ui8_adc_battery_current_target;
			}
			ui8_adc_battery_current_target = ui8_startup_assist_adc_battery_current_target;
		}
		else {
			ui8_startup_assist_adc_battery_current_target = 0;
		}
		
		// set duty cycle target
        if (ui8_adc_battery_current_target) {
            ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
        }
		else {
            ui8_duty_cycle_target = 0;
        }
    }
}// at this point, ui8_adc_battery_current_target is set and ui8_duty_cycle_target is 255 (or 0)

static void apply_cadence_assist(void)
{
    if (ui8_pedal_cadence_RPM) {
		// simulated pedal torque delta
		ui16_adc_pedal_torque_delta = ((uint16_t)ui8_riding_mode_parameter + (uint16_t)ui8_pedal_cadence_RPM) >> 2;
		
		// smooth start
		if (ui8_smooth_start_counter_set < SMOOTH_START_RAMP_DEFAULT) {
			 ui8_smooth_start_counter_set = SMOOTH_START_RAMP_DEFAULT;
		}
		apply_smooth_start();
		ui8_smooth_start_counter_set = ui8_smooth_start_counter_set_temp;
		
        // set cadence assist current target
		uint16_t ui16_adc_battery_current_target_cadence_assist = ui16_adc_pedal_torque_delta;
		
		// restore pedal torque delta
		ui16_adc_pedal_torque_delta = ui16_adc_pedal_torque_delta_temp;
		
		// set motor acceleration / deceleration
		set_motor_ramp();
		
        // set battery current target
        if (ui16_adc_battery_current_target_cadence_assist > ui8_adc_battery_current_max) {
            ui8_adc_battery_current_target = ui8_adc_battery_current_max;
        }
		else {
            ui8_adc_battery_current_target = ui16_adc_battery_current_target_cadence_assist;
        }
		
		// set duty cycle target
        if (ui8_adc_battery_current_target) {
            ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
        }
		else {
            ui8_duty_cycle_target = 0;
        }
    }
}

static void apply_emtb_assist(void)
{
#define eMTB_ASSIST_DENOMINATOR_MIN			10
	
	// check for assist without pedal rotation when there is no pedal rotation
	if (ui8_assist_without_pedal_rotation_enabled) {
		if ((!ui8_pedal_cadence_RPM)&&
			(ui16_adc_pedal_torque_delta > (120 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 1;
		}
	}
	
	if (ui8_assist_with_error_enabled) {
		ui8_pedal_cadence_RPM = 1;
	}
	
	if (((ui16_adc_pedal_torque_delta)&&(ui8_pedal_cadence_RPM))
	  ||(ui8_startup_assist_adc_battery_current_target)) {
		
		// for compatibility with v20.1C-4.4 display
		if (ui8_riding_mode_parameter < 21) {
			uint16_t ui16_temp = (uint16_t)(ui8_riding_mode_parameter * 127) / 10;
			ui8_riding_mode_parameter = (uint8_t) ui16_temp;
		}
		// get the eMTB assist denominator torque based
		uint16_t ui16_eMTB_assist_denominator = (508 - (ui8_riding_mode_parameter << 1));
		// get the eMTB assist denominator power based
		if (ui8_eMTB_based_on_power) {
			if (ui16_eMTB_assist_denominator >= ui8_pedal_cadence_RPM) {
				ui16_eMTB_assist_denominator -= ui8_pedal_cadence_RPM;
			}
			else {
				ui16_eMTB_assist_denominator = 0;
			}
		}
		ui16_eMTB_assist_denominator += eMTB_ASSIST_DENOMINATOR_MIN;
		
		// eMTB pedal torque delta calculation (progressive)
		uint16_t ui16_eMTB_adc_pedal_torque_delta = (uint16_t)((uint32_t)((ui16_adc_pedal_torque_delta * ui16_adc_pedal_torque_delta) + ui16_eMTB_assist_denominator)
			/ ui16_eMTB_assist_denominator);
		
		// set eMTB assist target current
		uint16_t ui16_adc_battery_current_target_eMTB_assist = ui16_eMTB_adc_pedal_torque_delta;
		
        // set motor acceleration / deceleration
		set_motor_ramp();
		
        // set battery current target
        if (ui16_adc_battery_current_target_eMTB_assist > ui8_adc_battery_current_max) {
            ui8_adc_battery_current_target = ui8_adc_battery_current_max;
        }
		else {
            ui8_adc_battery_current_target = ui16_adc_battery_current_target_eMTB_assist;
        }
		
		// set startup assist battery current target
		if (ui8_startup_assist_flag) {
			if (ui8_adc_battery_current_target > ui8_startup_assist_adc_battery_current_target) {
				ui8_startup_assist_adc_battery_current_target = ui8_adc_battery_current_target;
			}
			ui8_adc_battery_current_target = ui8_startup_assist_adc_battery_current_target;
		}
		else {
			ui8_startup_assist_adc_battery_current_target = 0;
		}
		
        // set duty cycle target
        if (ui8_adc_battery_current_target) {
            ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
        }
		else {
            ui8_duty_cycle_target = 0;
        }
    }
}


static void apply_hybrid_assist(void)
{
	uint16_t ui16_adc_battery_current_target_power_assist;
	uint16_t ui16_adc_battery_current_target_torque_assist;
	uint16_t ui16_adc_battery_current_target;
	
	// smooth start
	if (ui8_smooth_start_enabled) {
		apply_smooth_start();
	}
	
	// check for assist without pedal rotation when there is no pedal rotation
	if (ui8_assist_without_pedal_rotation_enabled) {
		if ((!ui8_pedal_cadence_RPM)&&
			(ui16_adc_pedal_torque_delta > (120 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 1;
		}
	}
	
	if (ui8_assist_with_error_enabled) {
		ui8_pedal_cadence_RPM = 1;
	}
	
	if ((ui8_pedal_cadence_RPM)||(ui8_startup_assist_adc_battery_current_target)) {
		// calculate torque assistance
		if (ui16_adc_pedal_torque_delta) {
			// get the torque assist factor
			uint8_t ui8_torque_assist_factor = ui8_hybrid_torque_parameter;
		
			// calculate torque assist target current
			ui16_adc_battery_current_target_torque_assist = ((uint16_t) ui16_adc_pedal_torque_delta * ui8_torque_assist_factor) / TORQUE_ASSIST_FACTOR_DENOMINATOR;
		}
		else {
			ui16_adc_battery_current_target_torque_assist = 0;
		}
	
		// calculate power assistance
		// get the power assist multiplier
		uint8_t ui8_power_assist_multiplier_x50 = ui8_riding_mode_parameter;

		// calculate power assist by multiplying human power with the power assist multiplier
		uint32_t ui32_power_assist_x100 = (((uint32_t)(ui8_pedal_cadence_RPM * ui8_power_assist_multiplier_x50))
				* ui16_pedal_torque_x100) >> 9; // see note below
	
		// calculate power assist target current x100
		uint32_t ui32_battery_current_target_x100 = (ui32_power_assist_x100 * 1000) / ui16_battery_voltage_filtered_x1000;
	
		// calculate power assist target current
		ui16_adc_battery_current_target_power_assist = (uint16_t)ui32_battery_current_target_x100 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;
	
		// set battery current target in ADC steps
		if (ui16_adc_battery_current_target_power_assist > ui16_adc_battery_current_target_torque_assist) {
			ui16_adc_battery_current_target = ui16_adc_battery_current_target_power_assist;
		}
		else {
			ui16_adc_battery_current_target = ui16_adc_battery_current_target_torque_assist;
		}
		// set motor acceleration / deceleration
		set_motor_ramp();
	
		// set battery current target
		if (ui16_adc_battery_current_target > ui8_adc_battery_current_max) {
			ui8_adc_battery_current_target = ui8_adc_battery_current_max;
		}
		else {
			ui8_adc_battery_current_target = ui16_adc_battery_current_target;
		}
	
		// set startup assist battery current target
		if (ui8_startup_assist_flag) {
			if (ui8_adc_battery_current_target > ui8_startup_assist_adc_battery_current_target) {
				ui8_startup_assist_adc_battery_current_target = ui8_adc_battery_current_target;
			}
			ui8_adc_battery_current_target = ui8_startup_assist_adc_battery_current_target;
		}
		else {
			ui8_startup_assist_adc_battery_current_target = 0;
		}
	
		// set duty cycle target
		if (ui8_adc_battery_current_target) {
			ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
		}
		else {
			ui8_duty_cycle_target = 0;
		}
	}
}

static void apply_walk_assist(void)
{
	if (ui8_assist_with_error_enabled) {
		// get walk assist duty cycle target
		ui8_walk_assist_duty_cycle_target = ui8_walk_assist_parameter + 20;
	}
	else {
		// get walk assist speed target x10
		ui8_walk_assist_speed_target_x10 = ui8_walk_assist_parameter;
		
		// set walk assist duty cycle target
		if ((!ui8_walk_assist_speed_flag)&&(!ui16_motor_speed_erps)) {
			ui8_walk_assist_duty_cycle_target = WALK_ASSIST_DUTY_CYCLE_STARTUP;
			ui8_walk_assist_duty_cycle_max = WALK_ASSIST_DUTY_CYCLE_STARTUP;
			ui16_walk_assist_wheel_speed_counter = 0;
			ui16_walk_assist_erps_target = 0;
		}
		else if (ui8_walk_assist_speed_flag) {
			if (ui16_motor_speed_erps < ui16_walk_assist_erps_min) {
				ui8_walk_assist_adj_delay = WALK_ASSIST_ADJ_DELAY_MIN;
				
				if (ui8_walk_assist_duty_cycle_counter++ > ui8_walk_assist_adj_delay) {
					if (ui8_walk_assist_duty_cycle_target < ui8_walk_assist_duty_cycle_max) {
						ui8_walk_assist_duty_cycle_target++;
					}
					else {
						ui8_walk_assist_duty_cycle_max++;
					}
					ui8_walk_assist_duty_cycle_counter = 0;
				}
			}
			else if (ui16_motor_speed_erps < ui16_walk_assist_erps_target) {
				ui8_walk_assist_adj_delay = (ui16_motor_speed_erps - ui16_walk_assist_erps_min) * WALK_ASSIST_ADJ_DELAY_MIN; // 4
				
				if (ui8_walk_assist_duty_cycle_counter++ > ui8_walk_assist_adj_delay) {
					if (ui8_walk_assist_duty_cycle_target < ui8_walk_assist_duty_cycle_max) {
						ui8_walk_assist_duty_cycle_target++;
					}
					
					ui8_walk_assist_duty_cycle_counter = 0;
				}
			}
			else if (ui16_motor_speed_erps < ui16_walk_assist_erps_max) {
				ui8_walk_assist_adj_delay = (ui16_walk_assist_erps_max - ui16_motor_speed_erps) * WALK_ASSIST_ADJ_DELAY_MIN;  // 4
			
				if (ui8_walk_assist_duty_cycle_counter++ > ui8_walk_assist_adj_delay) {
					if (ui8_walk_assist_duty_cycle_target > WALK_ASSIST_DUTY_CYCLE_MIN) {
						ui8_walk_assist_duty_cycle_target--; 
					}
					ui8_walk_assist_duty_cycle_counter = 0;
				}
			}
			else if (ui16_motor_speed_erps >= ui16_walk_assist_erps_max) {
				ui8_walk_assist_adj_delay = WALK_ASSIST_ADJ_DELAY_MIN;
			
				if (ui8_walk_assist_duty_cycle_counter++ > ui8_walk_assist_adj_delay) {
					if (ui8_walk_assist_duty_cycle_target > WALK_ASSIST_DUTY_CYCLE_MIN) {
						ui8_walk_assist_duty_cycle_target--;
						ui8_walk_assist_duty_cycle_max--;
					}
					ui8_walk_assist_duty_cycle_counter = 0;
				}
			}
		}
		else {
			ui8_walk_assist_adj_delay = WALK_ASSIST_ADJ_DELAY_STARTUP;
			
			if (ui8_walk_assist_duty_cycle_counter++ > ui8_walk_assist_adj_delay) {
				if (ui16_wheel_speed_x10) {
					if (ui16_wheel_speed_x10 > WALK_ASSIST_WHEEL_SPEED_MIN_DETECT_X10) {
						ui8_walk_assist_duty_cycle_target--;
					}
					
					if (ui16_walk_assist_wheel_speed_counter++ >= 10) {
						ui8_walk_assist_duty_cycle_max += 10;
					
						// set walk assist erps target
						ui16_walk_assist_erps_target = ((ui16_motor_speed_erps * ui8_walk_assist_speed_target_x10) / ui16_wheel_speed_x10);
						ui16_walk_assist_erps_min = ui16_walk_assist_erps_target - WALK_ASSIST_ERPS_THRESHOLD;
						ui16_walk_assist_erps_max = ui16_walk_assist_erps_target + WALK_ASSIST_ERPS_THRESHOLD;
					
						// set walk assist speed flag
						ui8_walk_assist_speed_flag = 1;
					}
				}
				else {
					if ((ui8_walk_assist_duty_cycle_max + 10) < WALK_ASSIST_DUTY_CYCLE_MAX) {
						ui8_walk_assist_duty_cycle_target++;
						ui8_walk_assist_duty_cycle_max++;
					}
				}
				ui8_walk_assist_duty_cycle_counter = 0;
			}
		}
	}

	if (ui8_walk_assist_duty_cycle_target > WALK_ASSIST_DUTY_CYCLE_MAX) {
		ui8_walk_assist_duty_cycle_target = WALK_ASSIST_DUTY_CYCLE_MAX;
	}
	
	// set motor acceleration / deceleration
	ui8_duty_cycle_ramp_up_inverse_step = WALK_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP;	
	ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
	
	// set battery current target
	ui8_adc_battery_current_target = ui8_min(WALK_ASSIST_ADC_BATTERY_CURRENT_MAX, ui8_adc_battery_current_max);
	
	// set duty cycle target
	ui8_duty_cycle_target = ui8_walk_assist_duty_cycle_target;
}

static void apply_cruise(void)
{
#define CRUISE_PID_KP							4
#define CRUISE_PID_KD							6
#define CRUISE_PID_KI 							0.6
#define CRUISE_PID_INTEGRAL_LIMIT				1000
#define CRUISE_PID_OUTPUT_LIMIT					1000
	static int16_t i16_error;
	static int16_t i16_last_error;
	static int16_t i16_integral;
	static int16_t i16_derivative;
	static int16_t i16_control_output;
	static uint16_t ui16_wheel_speed_target_x10;
	
    if ((ui16_wheel_speed_x10 > CRUISE_THRESHOLD_SPEED_X10)
	  ||((ui16_wheel_speed_x10 > WALK_ASSIST_THRESHOLD_SPEED_X10)&&(!ui8_cruise_PID_initialize))) {
		// initialize cruise PID controller
		if (ui8_cruise_PID_initialize) {
			ui8_cruise_PID_initialize = 0;
			
			// reset PID variables
			i16_error = 0;
			i16_last_error = 0;
			i16_integral = 500; // initialize integral to a value so the motor does not start from zero
			i16_derivative = 0;
			i16_control_output = 0;
			
            // check what target wheel speed to use (received or current)
            //uint16_t ui16_wheel_speed_target_received_x10 = ui8_riding_mode_parameter * (uint8_t)10;
			
			if (ui16_wheel_speed_target_received_x10) {
                // set received target wheel speed to target wheel speed
				ui16_wheel_speed_target_x10 = ui16_wheel_speed_target_received_x10;
			}
			else {
                // set current wheel speed to maintain
				ui16_wheel_speed_target_x10 = ui16_wheel_speed_x10;
			}
		}
		
		// calculate error
		i16_error = (ui16_wheel_speed_target_x10 - ui16_wheel_speed_x10);
		
		// calculate integral
		i16_integral = i16_integral + i16_error;
		
		// limit integral
		if (i16_integral > CRUISE_PID_INTEGRAL_LIMIT) {
			i16_integral = CRUISE_PID_INTEGRAL_LIMIT;
		}
		else if (i16_integral < 0) {
			i16_integral = 0;
		}
		
		// calculate derivative
		i16_derivative = i16_error - i16_last_error;
		
		// save error to last error
		i16_last_error = i16_error;

		// calculate control output ( output =  P I D )
		i16_control_output = (CRUISE_PID_KP * i16_error)
							+ (CRUISE_PID_KI * i16_integral)
							+ (CRUISE_PID_KD * i16_derivative);
		
		// limit control output to just positive values
		if (i16_control_output < 0) {
			i16_control_output = 0;
		}
		
		// limit control output to the maximum value
		if (i16_control_output > CRUISE_PID_OUTPUT_LIMIT) {
			i16_control_output = CRUISE_PID_OUTPUT_LIMIT;
		}
		
		// set motor acceleration / deceleration
        ui8_duty_cycle_ramp_up_inverse_step = CRUISE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP;
        ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
		
		// set battery current target
		ui8_adc_battery_current_target = ui8_adc_battery_current_max;
		
		// set duty cycle target  |  map the control output to an appropriate target PWM value
		ui8_duty_cycle_target = map_ui8((uint8_t) (i16_control_output >> 2),
			(uint8_t) 0,					// minimum control output from PID
			(uint8_t) 250,					// maximum control output from PID
			(uint8_t) 0,					// minimum duty cycle
			(uint8_t) PWM_DUTY_CYCLE_MAX);	// maximum duty cycle
	}
}


static void apply_calibration_assist(void)
{
    // ui8_riding_mode_parameter contains the target duty cycle
    uint8_t ui8_calibration_assist_duty_cycle_target = ui8_riding_mode_parameter;

    // limit cadence assist duty cycle target
    if (ui8_calibration_assist_duty_cycle_target >= PWM_DUTY_CYCLE_MAX) {
        ui8_calibration_assist_duty_cycle_target = (uint8_t)(PWM_DUTY_CYCLE_MAX-1);
    }

    // set motor acceleration / deceleration
    ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
    ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;

    // set battery current target
    ui8_adc_battery_current_target = ui8_adc_battery_current_max;

    // set duty cycle target
    ui8_duty_cycle_target = ui8_calibration_assist_duty_cycle_target;
}



static void apply_throttle(void)
{
	//Next line has been moved from motor.c to here to save time in irq 1 ; >>2 because we use 10 bits instead of 12 bits
	ui16_adc_throttle = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 5 ) & 0x0FFF) >> 2; // throttle gr1 ch7 result 5  in bg  p2.5
    if (ui8_throttle_feature_enabled) {
		// map adc value from 0 to 255
		ui8_throttle_adc_in = map_ui8((uint8_t)(ui16_adc_throttle >> 2),
            (uint8_t) ADC_THROTTLE_MIN_VALUE,
            (uint8_t) ADC_THROTTLE_MAX_VALUE,
            (uint8_t) 0,
            (uint8_t) 255);
			
		// set throttle assist, virtual or adc
		if (ui8_throttle_virtual) {
			ui8_adc_throttle_assist = ui8_throttle_virtual;
		}
		else if ((ui8_optional_ADC_function == THROTTLE_CONTROL)&&(ui8_throttle_adc_in)) {
			ui8_adc_throttle_assist = ui8_throttle_adc_in;
		}
		else {
			ui8_adc_throttle_assist = 0;
		}
		
		// throttle with pedaling
		if ((ui8_throttle_legal)&&(!ui8_pedal_cadence_RPM)) {
			ui8_adc_throttle_assist = 0;
		}
	
		if (ui8_adc_throttle_assist) {
			// map ADC throttle value from 0 to max battery current
			uint8_t ui8_adc_battery_current_target_throttle = map_ui8(ui8_adc_throttle_assist,
				(uint8_t) 0,
				(uint8_t) 255,
				(uint8_t) 0,
				(uint8_t) ui8_adc_battery_current_max);
			
			if (ui8_adc_battery_current_target_throttle > ui8_adc_battery_current_target) {
				// set motor acceleration / deceleration
				if (ui16_wheel_speed_x10 >= 255) {
					ui8_duty_cycle_ramp_up_inverse_step = THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
					ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
				}
				else {
					ui8_duty_cycle_ramp_up_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
						(uint8_t) 40,
						(uint8_t) 255,
						(uint8_t) THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT,
						(uint8_t) THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);

					ui8_duty_cycle_ramp_down_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
						(uint8_t) 40,
						(uint8_t) 255,
						(uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,
						(uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
				}
				
				// set battery current target
				if (ui8_adc_battery_current_target_throttle > ui8_adc_battery_current_max) {
					ui8_adc_battery_current_target = ui8_adc_battery_current_max;
				}
				else {
					ui8_adc_battery_current_target = ui8_adc_battery_current_target_throttle;
				}

				// set duty cycle target
				ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
			}
		}
		ui8_adc_throttle_assist = 0;
	}
}

static void apply_temperature_limiting(void)
{
	static uint16_t ui16_adc_motor_temperature_filtered;
	
    // get ADC measurement
    uint16_t ui16_temp = ui16_adc_throttle;

    // filter ADC measurement to motor temperature variable
    ui16_adc_motor_temperature_filtered = filter(ui16_temp, ui16_adc_motor_temperature_filtered, 8);

    // convert ADC value
    ui16_motor_temperature_filtered_x10 = (uint16_t)(((uint32_t) ui16_adc_motor_temperature_filtered * 10000) / 2048);

    // min temperature value can not be equal or higher than max temperature value
    if (ui8_motor_temperature_min_value_to_limit >= ui8_motor_temperature_max_value_to_limit) {
        ui8_adc_battery_current_target = 0;
    }
	else {
        // adjust target current if motor over temperature limit
        ui8_adc_battery_current_target = map_ui16((uint16_t) ui16_motor_temperature_filtered_x10,
				(uint16_t) ((uint8_t)ui8_motor_temperature_min_value_to_limit * (uint8_t)10U),
				(uint16_t) ((uint8_t)ui8_motor_temperature_max_value_to_limit * (uint8_t)10U),
				ui8_adc_battery_current_target,
				0);
	}
}


static void apply_speed_limit(void)
{
    if (ui8_wheel_speed_max > 0U) {
		uint16_t speed_limit_low  = (uint16_t)((uint8_t)(ui8_wheel_speed_max - 2U) * (uint8_t)10U); // casting literal to uint8_t ensures usage of MUL X,A
		uint16_t speed_limit_high = (uint16_t)((uint8_t)(ui8_wheel_speed_max + 2U) * (uint8_t)10U);
		
		ui8_speed_limit_high_exceeded = 0;
		if (ui16_wheel_speed_x10 > speed_limit_high) {
			if (ui8_adc_battery_current_target > 0U) {
				ui8_speed_limit_high_exceeded = 1;
			}
			ui8_duty_cycle_target = 0;
		}
		
        // set battery current target
        ui8_adc_battery_current_target = (uint8_t) map_ui16(ui16_wheel_speed_x10,
                speed_limit_low,
                speed_limit_high,
                ui8_adc_battery_current_target,
                0U);
    }
}


static void calc_wheel_speed(void)
{
    // calc wheel speed (km/h x10)
    if (ui16_wheel_speed_sensor_ticks) {
        uint16_t ui16_tmp = ui16_wheel_speed_sensor_ticks;
        // rps = PWM_CYCLES_SECOND / ui16_wheel_speed_sensor_ticks (rev/sec)
        // km/h*10 = rps * ui16_wheel_perimeter * ((3600 / (1000 * 1000)) * 10)
        // !!!warning if PWM_CYCLES_SECOND is not a multiple of 1000
        ui16_wheel_speed_x10 = (uint16_t)(((uint32_t) ui16_wheel_perimeter * ((PWM_CYCLES_SECOND/1000)*36U)) / ui16_tmp);
    }
	else {
		#if WHEEL_SPEED_SENSOR_SIMULATION
		if(ui16_motor_speed_erps > 2) {
			ui16_wheel_speed_x10 = (ui16_motor_speed_erps * 8) / 14;
		} else {
			ui16_wheel_speed_x10 = 0;
		}
		#else
		ui16_wheel_speed_x10 = 0;
		#endif
	}
}

static void calc_cadence(void)
{
    // get the cadence sensor ticks
    uint16_t ui16_cadence_sensor_ticks_temp = ui16_cadence_sensor_ticks;

    // adjust cadence sensor ticks counter min depending on wheel speed
    ui16_cadence_ticks_count_min_speed_adj = map_ui16(ui16_wheel_speed_x10,
            40,
            400,
            CADENCE_SENSOR_CALC_COUNTER_MIN,
            CADENCE_SENSOR_TICKS_COUNTER_MIN_AT_SPEED);

    // calculate cadence in RPM and avoid zero division
    // !!!warning if PWM_CYCLES_SECOND > 21845
	// * 3 because 60 sec in 1 min and 20 magnetic in one pedal rotation
    if (ui16_cadence_sensor_ticks_temp) {
        ui8_pedal_cadence_RPM = (uint8_t)((PWM_CYCLES_SECOND * 3U) / ui16_cadence_sensor_ticks_temp);
		
		if (ui8_pedal_cadence_RPM > 120) {
			ui8_pedal_cadence_RPM = 120;
		}
	}
	else {
        ui8_pedal_cadence_RPM = 0;
	}
	
	/*-------------------------------------------------------------------------------------------------

     NOTE: regarding the cadence calculation

     Cadence is calculated by counting how many ticks there are between two LOW to HIGH transitions.

     Formula for calculating the cadence in RPM:

     (1) Cadence in RPM = (60 * PWM_CYCLES_SECOND) / CADENCE_SENSOR_NUMBER_MAGNETS) / ticks

     (2) Cadence in RPM = (PWM_CYCLES_SECOND * 3) / ticks

     -------------------------------------------------------------------------------------------------*/
}



void get_battery_voltage(void) 
{
#define READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT   2

    /*---------------------------------------------------------
     NOTE: regarding filter coefficients

     Possible values: 0, 1, 2, 3, 4, 5, 6
     0 equals to no filtering and no delay, higher values
     will increase filtering but will also add a bigger delay.
     ---------------------------------------------------------*/

    static uint16_t ui16_adc_battery_voltage_accumulated;
	
    // low pass filter the voltage readed value, to avoid possible fast spikes/noise
    ui16_adc_battery_voltage_accumulated -= ui16_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
    ui16_adc_battery_voltage_accumulated += ui16_adc_voltage;
	ui16_adc_battery_voltage_filtered = ui16_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
    ui16_battery_voltage_filtered_x1000 = ui16_adc_battery_voltage_filtered * BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000;
}


#define TOFFSET_CYCLES 120 // 3sec (25ms*120)
static uint8_t toffset_cycle_counter = 0;

static void get_pedal_torque(void)
{
	uint16_t ui16_temp = 0;
	// this has been moved from motor.c to here in order to save time in the irq; >>2 is to go from ADC 12 bits to 10 bits like TSDZ2
	ui16_adc_torque   = (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 2 ) & 0xFFF) >> 2; // torque gr0 ch7 result 2 in bg p2.2
    if (toffset_cycle_counter < TOFFSET_CYCLES) {
        uint16_t ui16_tmp = ui16_adc_torque;
        ui16_adc_pedal_torque_offset_init = filter(ui16_tmp, ui16_adc_pedal_torque_offset_init, 2);
        toffset_cycle_counter++;
		
		// check the offset calibration
		if ((toffset_cycle_counter == TOFFSET_CYCLES)&&(ui8_torque_sensor_calibration_enabled)) {
			if ((ui16_adc_pedal_torque_offset_init > ui16_adc_pedal_torque_offset_min)&& 
			  (ui16_adc_pedal_torque_offset_init < ui16_adc_pedal_torque_offset_max)) {
				ui8_adc_pedal_torque_offset_error = 0;
			}
			else {
				ui8_adc_pedal_torque_offset_error = 1;
			}
		}
		
        ui16_adc_pedal_torque = ui16_adc_pedal_torque_offset_init;
		ui16_adc_pedal_torque_offset_cal = ui16_adc_pedal_torque_offset_init + ui8_adc_torque_calibration_offset;
	}
	else {
		// torque sensor offset adjustment
		ui16_adc_pedal_torque_offset = ui16_adc_pedal_torque_offset_cal;
		if (ui8_pedal_cadence_RPM) {
			ui16_adc_pedal_torque_offset -= ui8_adc_torque_middle_offset_adj;
			ui16_adc_pedal_torque_offset += ui8_adc_pedal_torque_offset_adj;
		}
		
		if ((ui8_coaster_brake_enabled)&&(ui16_adc_pedal_torque_offset > ui8_coaster_brake_torque_threshold)) {
			//ui16_adc_coaster_brake_threshold = ui16_adc_pedal_torque_offset - ui8_coaster_brake_torque_threshold;
			ui16_adc_coaster_brake_threshold = ui16_adc_pedal_torque_offset_cal - ui8_coaster_brake_torque_threshold;
		}
		else {
			ui16_adc_coaster_brake_threshold = 0;
		}
		
        // get adc pedal torque
        ui16_adc_pedal_torque = ui16_adc_torque;
    }
	
    // calculate the delta value from calibration
    if (ui16_adc_pedal_torque > ui16_adc_pedal_torque_offset) {
		// adc pedal torque delta remapping
		if (ui8_torque_sensor_calibration_enabled) {
			// adc pedal torque delta adjustment
			ui16_temp = ui16_adc_pedal_torque - ui16_adc_pedal_torque_offset_init;
			ui16_adc_pedal_torque_delta = ui16_adc_pedal_torque - ui16_adc_pedal_torque_offset
				- ((ui8_adc_pedal_torque_delta_adj	* ui16_temp) / ADC_TORQUE_SENSOR_RANGE_TARGET);
		
			// adc pedal torque range adjusment
			ui16_temp = (ui16_adc_pedal_torque_delta * ui16_adc_pedal_torque_range_ingrease_x100) / 10;
			ui16_adc_pedal_torque_delta = (((ui16_temp
				* ((ui16_temp / ADC_TORQUE_SENSOR_ANGLE_COEFF + ADC_TORQUE_SENSOR_ANGLE_COEFF_X10) / ADC_TORQUE_SENSOR_ANGLE_COEFF)) / 50) // 100
				* (100 + ui8_adc_pedal_torque_range_adj)) / 200; // 100
			
			// adc pedal torque angle adjusment
			if (ui16_adc_pedal_torque_delta < ui16_adc_pedal_torque_range_target_max) {
				ui16_temp = (ui16_adc_pedal_torque_delta * ui16_adc_pedal_torque_range_target_max)
					/ (ui16_adc_pedal_torque_range_target_max - (((ui16_adc_pedal_torque_range_target_max - ui16_adc_pedal_torque_delta) * 10)
					/ ui8_adc_pedal_torque_angle_adj));
				
				ui16_adc_pedal_torque_delta = ui16_temp;
			}
		}
		else {
			ui16_adc_pedal_torque_delta = ui16_adc_pedal_torque - ui16_adc_pedal_torque_offset;
		}
		ui16_adc_pedal_torque_delta = (ui16_adc_pedal_torque_delta + ui16_adc_pedal_torque_delta_temp) >> 1;
	}
	else {
		ui16_adc_pedal_torque_delta = 0;
    }
	ui16_adc_pedal_torque_delta_temp = ui16_adc_pedal_torque_delta;
	
	// for cadence sensor check
	ui16_adc_pedal_torque_delta_no_boost = ui16_adc_pedal_torque_delta;
	
    // calculate torque on pedals
    ui16_pedal_torque_x100 = ui16_adc_pedal_torque_delta * ui8_pedal_torque_per_10_bit_ADC_step_x100;
	
	/*------------------------------------------------------------------------

    NOTE: regarding the human power calculation
    
    (1) Formula: power = torque * rotations per second * 2 * pi
    (2) Formula: power = torque * rotations per minute * 2 * pi / 60
    (3) Formula: power = torque * rotations per minute * 0.1047
    (4) Formula: power = torque * 100 * rotations per minute * 0.001047
    (5) Formula: power = torque * 100 * rotations per minute / 955
    (6) Formula: power * 10  =  torque * 100 * rotations per minute / 96
    
	------------------------------------------------------------------------*/
}

static void check_system(void)
{
// voltage cut-off warning msg
	if(ui16_adc_voltage < ui16_adc_voltage_cut_off) {
		ui8_voltage_cut_off_flag = 1;
	}
	else {
		ui8_voltage_cut_off_flag = 0;
	}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E02 ERROR_TORQUE_SENSOR
	static uint8_t ui8_riding_torque_mode = 0;
	
	// set riding torque mode
	if (((ui8_riding_mode == POWER_ASSIST_MODE)
	  ||(ui8_riding_mode == TORQUE_ASSIST_MODE)
	  ||(ui8_riding_mode == HYBRID_ASSIST_MODE)
	  ||(ui8_riding_mode == eMTB_ASSIST_MODE))
	  && (ui8_adc_throttle_assist == 0U)) {
		ui8_riding_torque_mode = 1;
	}
	else {
		ui8_riding_torque_mode = 0;
	}
    // check torque sensor
    if (ui8_riding_torque_mode) {
		if ((ui16_adc_pedal_torque_offset > 300)
		  ||(ui16_adc_pedal_torque_offset < 10)
		  ||(ui16_adc_pedal_torque > 500)
		  ||(ui8_adc_pedal_torque_offset_error)) {
			// set error code
			ui8_m_system_state |= ERROR_TORQUE_SENSOR;
		}
	}
	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E03 ERROR_CADENCE_SENSOR
#define CHECK_CADENCE_SENSOR_COUNTER_THRESHOLD          250 // 250 * 100ms = 25 seconds
#define ADC_TORQUE_SENSOR_DELTA_THRESHOLD				(uint16_t)((ADC_TORQUE_SENSOR_RANGE_TARGET >> 1) + 20)
	static uint8_t ui8_check_cadence_sensor_counter;
	
	// check cadence sensor
	if ((ui16_adc_pedal_torque_delta_no_boost > ADC_TORQUE_SENSOR_DELTA_THRESHOLD)
	  &&(!ui8_startup_assist_flag)&&(ui8_riding_torque_mode)
	  &&((ui8_pedal_cadence_RPM > 130)||(!ui8_pedal_cadence_RPM))) {
		ui8_check_cadence_sensor_counter++;
	}
	else {
		ui8_check_cadence_sensor_counter = 0;
	}
	
	if (ui8_check_cadence_sensor_counter > CHECK_CADENCE_SENSOR_COUNTER_THRESHOLD) {
		// set cadence sensor error code
		ui8_m_system_state |= ERROR_CADENCE_SENSOR;
	}
	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E08 ERROR_SPEED_SENSOR
#define CHECK_SPEED_SENSOR_COUNTER_THRESHOLD          125 // 125 * 100ms = 12.5 seconds
#define MOTOR_ERPS_SPEED_THRESHOLD	                  90 // 180 for TSDZ2; should be 2 X less for TSDZ8 (4 poles instead of 8)
if (ui8_test_mode_flag == NORMAL_RUNNING_MODE) {  // check only in normal running mode ; not when
	static uint16_t ui16_check_speed_sensor_counter;
	
	// check speed sensor
	if ((ui16_motor_speed_erps > MOTOR_ERPS_SPEED_THRESHOLD)
	  &&(ui8_riding_mode != WALK_ASSIST_MODE)
	  &&(ui8_riding_mode != CRUISE_MODE)) {
		ui16_check_speed_sensor_counter++;
	}
	else {
		ui16_check_speed_sensor_counter = 0;
	}
	
	if (ui16_wheel_speed_x10) {
		ui16_check_speed_sensor_counter = 0;
	}
	
	if (ui16_check_speed_sensor_counter > CHECK_SPEED_SENSOR_COUNTER_THRESHOLD) {
		// set speed sensor error code
		ui8_m_system_state |= ERROR_SPEED_SENSOR;
	}
} // end speed sensor
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E04 ERROR_MOTOR_BLOCKED
#define MOTOR_BLOCKED_COUNTER_THRESHOLD               	10  // 10 * 100ms = 1.0 seconds
#define MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X10   	30 // 30 = 3.0 amps
#define MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X5   	(uint8_t)(MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X10 / 2)
#define MOTOR_BLOCKED_ERPS_THRESHOLD                  	10 // 20 ERPS for TSDZ2; so 10 for TSDZ8 (4 poles instead of 8 poles)

    static uint8_t ui8_motor_blocked_counter;
    
	// if battery current is over the current threshold and the motor ERPS is below threshold start setting motor blocked error code
	if ((ui8_battery_current_filtered_x5 > MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X5)
	  && (ui16_motor_speed_erps < MOTOR_BLOCKED_ERPS_THRESHOLD)) {
		// increment motor blocked counter with 100 milliseconds
		++ui8_motor_blocked_counter;

		// check if motor is blocked for more than some safe threshold
		if (ui8_motor_blocked_counter > MOTOR_BLOCKED_COUNTER_THRESHOLD) {
			// set error code
			ui8_m_system_state |= ERROR_MOTOR_BLOCKED;
		}
	}
	else {
		// current is below the threshold and/or motor ERPS is above the threshold so reset the counter
		ui8_motor_blocked_counter = 0;
	}
	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E05 ERROR_THROTTLE
#define THROTTLE_CHECK_COUNTER_THRESHOLD		 20 // 20 * 100ms = 2.0 seconds
#define ADC_THROTTLE_MIN_VALUE_THRESHOLD		(uint8_t)(ADC_THROTTLE_MIN_VALUE + 5)

    static uint8_t ui8_throttle_check_counter;
	
	if ((ui8_throttle_feature_enabled)&&(ui8_optional_ADC_function == THROTTLE_CONTROL)) {
		if (ui8_throttle_check_counter < THROTTLE_CHECK_COUNTER_THRESHOLD) {
			ui8_throttle_check_counter++;
		
			if ((ui16_adc_throttle >> 2) > ADC_THROTTLE_MIN_VALUE_THRESHOLD) {
				ui8_m_system_state |= ERROR_THROTTLE;
			}
		}
	}
}


void ebike_control_lights(void)
{
#define DEFAULT_FLASH_ON_COUNTER_MAX      3
#define DEFAULT_FLASH_OFF_COUNTER_MAX     2
#define BRAKING_FLASH_ON_COUNTER_MAX      1
#define BRAKING_FLASH_OFF_COUNTER_MAX     1

    static uint8_t ui8_default_flash_state;
    static uint8_t ui8_default_flash_state_counter; // increments every function call -> 100 ms
    static uint8_t ui8_braking_flash_state;
    static uint8_t ui8_braking_flash_state_counter; // increments every function call -> 100 ms

    /****************************************************************************/

    // increment flash counters
    ++ui8_default_flash_state_counter;
    ++ui8_braking_flash_state_counter;

    /****************************************************************************/

    // set default flash state
    if ((ui8_default_flash_state) && (ui8_default_flash_state_counter > DEFAULT_FLASH_ON_COUNTER_MAX)) {
        // reset flash state counter
        ui8_default_flash_state_counter = 0;

        // toggle flash state
        ui8_default_flash_state = 0;
    }
	else if ((!ui8_default_flash_state) && (ui8_default_flash_state_counter > DEFAULT_FLASH_OFF_COUNTER_MAX)) {
        // reset flash state counter
        ui8_default_flash_state_counter = 0;

        // toggle flash state
        ui8_default_flash_state = 1;
    }

    /****************************************************************************/

    // set braking flash state
    if ((ui8_braking_flash_state) && (ui8_braking_flash_state_counter > BRAKING_FLASH_ON_COUNTER_MAX)) {
        // reset flash state counter
        ui8_braking_flash_state_counter = 0;

        // toggle flash state
        ui8_braking_flash_state = 0;
    }
	else if ((!ui8_braking_flash_state) && (ui8_braking_flash_state_counter > BRAKING_FLASH_OFF_COUNTER_MAX)) {
        // reset flash state counter
        ui8_braking_flash_state_counter = 0;

        // toggle flash state
        ui8_braking_flash_state = 1;
    }

    /****************************************************************************/

    // select lights configuration
    switch (ui8_lights_configuration) {
      case 0:
        // set lights
        lights_set_state(ui8_lights_state);
        break;
      case 1:
        // check lights state
        if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 2:
        // check light and brake state
        if (ui8_lights_state && ui8_brake_state) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 3:
        // check light and brake state
        if (ui8_lights_state && ui8_brake_state) {
            // set lights
            lights_set_state(ui8_brake_state);
        }
		else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 4:
        // check light and brake state
        if (ui8_lights_state && ui8_brake_state) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        }
		else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 5:
        // check brake state
        if (ui8_brake_state) {
            // set lights
            lights_set_state(ui8_brake_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 6:
        // check brake state
        if (ui8_brake_state) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 7:
        // check brake state
        if (ui8_brake_state) {
            // set lights
            lights_set_state(ui8_brake_state);
        }
		else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 8:
        // check brake state
        if (ui8_brake_state) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        }
		else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
	  default:
        // set lights
        lights_set_state(ui8_lights_state);
        break;
    }

    /*------------------------------------------------------------------------------------------------------------------

     NOTE: regarding the various light modes

     (0) lights ON when enabled
     (1) lights FLASHING when enabled

     (2) lights ON when enabled and BRAKE-FLASHING when braking
     (3) lights FLASHING when enabled and ON when braking
     (4) lights FLASHING when enabled and BRAKE-FLASHING when braking

     (5) lights ON when enabled, but ON when braking regardless if lights are enabled
     (6) lights ON when enabled, but BRAKE-FLASHING when braking regardless if lights are enabled

     (7) lights FLASHING when enabled, but ON when braking regardless if lights are enabled
     (8) lights FLASHING when enabled, but BRAKE-FLASHING when braking regardless if lights are enabled

     ------------------------------------------------------------------------------------------------------------------*/
}


void fillRxBuffer(void)
{
	// when there is a byte received and we are not yet processing the frame
	while ((XMC_USIC_CH_RXFIFO_IsEmpty(	CYBSP_DEBUG_UART_HW	) == false) && (ui8_received_package_flag == 0)){
		// get the byte
		uint8_t	ui8_byte_received = (uint8_t) XMC_USIC_CH_RXFIFO_GetData(CYBSP_DEBUG_UART_HW);
 		switch (ui8_state_machine) {
			case 0:
				if (ui8_byte_received == 0x59) { // see if we get start package byte
					ui8_rx_buffer[0] = ui8_byte_received;
					ui8_state_machine = 1;
				}
				else {
					ui8_state_machine = 0;
				}
				break;

			case 1:
				if (ui8_byte_received > (UART_NUMBER_DATA_BYTES_TO_RECEIVE - 2)) {
					ui8_state_machine = 0;
				}
				else {
					ui8_rx_buffer[1] = ui8_byte_received;
					ui8_rx_len = ui8_byte_received;
					ui8_state_machine = 2;
				}
				break;

			case 2:
				ui8_rx_buffer[ui8_rx_cnt + 2] = ui8_byte_received;
				++ui8_rx_cnt;

				if (ui8_rx_cnt >= ui8_rx_len) {
					ui8_rx_cnt = 0;
					ui8_state_machine = 0;
					ui8_received_package_flag = 1; // signal that we have a full package to be processed
				}
				break;

			default:
				break;
		}
	}
}

static void communications_controller(void)
{
	uint8_t ui8_frame_type_to_send = 0;
	uint8_t ui8_len;

	if (ui8_received_package_flag) {
		// just to make easy next calculations
		ui16_crc_rx = 0xffff;
		ui8_len = ui8_rx_buffer[1];
		for (ui8_i = 0; ui8_i < ui8_len; ui8_i++) {
			crc16(ui8_rx_buffer[ui8_i], &ui16_crc_rx);
		}

		// if CRC is correct read the package
		if (((((uint16_t) ui8_rx_buffer[ui8_len + 1]) << 8) +
           ((uint16_t) ui8_rx_buffer[ui8_len])) == ui16_crc_rx) {
			ui8_comm_error_counter = 0;

			if (ui8_m_motor_init_state == MOTOR_INIT_STATE_RESET) {
				ui8_m_motor_init_state = MOTOR_INIT_STATE_NO_INIT;
			}
			ui8_frame_type_to_send = ui8_rx_buffer[2];
			communications_process_packages(ui8_frame_type_to_send);
		}
		else {
			ui8_received_package_flag = 0;
			ui8_comm_error_counter++;
		}
	}
	else {
		ui8_comm_error_counter++;
	}

	// check for communications fail or display master fail
	// can't fail more then 1000ms ??? 20 * 50ms
	if (ui8_comm_error_counter > 20) {
		motor_disable_pwm();
		ui8_motor_enabled = 0;
		ui8_m_system_state |= ERROR_FATAL;// Comms failed
	}

	if (ui8_m_motor_init_state == MOTOR_INIT_STATE_RESET) {
		communications_process_packages(COMM_FRAME_TYPE_ALIVE); 
	}
}


static void communications_process_packages(uint8_t ui8_frame_type)
{
	uint8_t ui8_temp;
	uint16_t ui16_temp;
	//uint32_t ui32_temp;
	uint8_t ui8_len = 3; // 3 bytes: 1 type of frame + 2 CRC bytes

	// start up byte
	ui8_tx_buffer[0] = 0x43;
	ui8_tx_buffer[2] = ui8_frame_type;

	// prepare payload
	switch (ui8_frame_type) {
	  // periodic data
	  case COMM_FRAME_TYPE_PERIODIC:
		// display will send periodic command after motor init ok, now reset so the state machine will be ready for next time
		ui8_m_motor_init_status = MOTOR_INIT_STATUS_RESET;
	  
		//m_config_vars.ui16_assist_level_factor_x1000 = (((uint16_t) ui8_rx_buffer[4]) << 8) + ((uint16_t) ui8_rx_buffer[3]);
		// riding mode parameter
		ui8_riding_mode_parameter = ui8_rx_buffer[3];
		
		// hybrid torque parameter
		ui8_hybrid_torque_parameter = ui8_rx_buffer[4];
		
		ui8_temp = ui8_rx_buffer[5];
		// lights state
		ui8_lights_state = ui8_temp & 1;
		// walk assist
		uint8_t ui8_walk_assist = (ui8_temp & 2) >> 1;
		// assist level flag
		ui8_assist_level_flag = (ui8_temp & 4) >> 2;
		// cruise enabled
		uint8_t ui8_cruise_enabled = (ui8_temp & 8) >> 3;
		// startup assist
		ui8_startup_assist_flag = (ui8_temp & 16) >> 4;
		// throttle enabled
		ui8_throttle_feature_enabled = (ui8_temp & 32) >> 5;
		// throttle legal
		ui8_throttle_legal = (ui8_temp & 64) >> 6;
		// cruise legal
		ui8_cruise_legal = (ui8_temp & 128) >> 7;
		
		// battery max power target
		ui8_target_battery_max_power_div25 = ui8_rx_buffer[6];
		
		// calculate max battery current in ADC steps
		// from the received battery current limit & power limit
		if (ui8_target_battery_max_power_div25 != ui8_target_battery_max_power_div25_temp) {
			ui8_target_battery_max_power_div25_temp = ui8_target_battery_max_power_div25;
			
			uint8_t ui8_adc_battery_current_max_temp_1 = (uint16_t)(ui8_battery_current_max * (uint8_t)100)
					/ (uint16_t)BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;

			// calculate max battery current in ADC steps from the received power limit
			uint32_t ui32_battery_current_max_x100 = ((uint32_t) ui8_target_battery_max_power_div25 * 2500000)
					/ ui16_battery_voltage_filtered_x1000;
			uint8_t ui8_adc_battery_current_max_temp_2 = ui32_battery_current_max_x100 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;

			// set max battery current
			ui8_adc_battery_current_max = ui8_min(ui8_adc_battery_current_max_temp_1, ui8_adc_battery_current_max_temp_2);
			// set max motor phase current
			ui16_temp = (uint16_t)(ui8_adc_battery_current_max * ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX);
			ui16_adc_motor_phase_current_max = (uint8_t)(ui16_temp / ADC_10_BIT_BATTERY_CURRENT_MAX);
			// limit max motor phase current if higher than configured hardware limit (safety)
			if (ui16_adc_motor_phase_current_max > ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX) {
			ui16_adc_motor_phase_current_max = ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX;
			}
			// set limit battery overcurrent
			ui8_adc_battery_overcurrent = ui8_adc_battery_current_max + ADC_10_BIT_BATTERY_EXTRACURRENT;
		}
		
		// walk assist parameter
		ui8_walk_assist_parameter = ui8_rx_buffer[7];
		
		// riding mode
		if ((ui8_walk_assist)&&(ui16_wheel_speed_x10 < WALK_ASSIST_THRESHOLD_SPEED_X10)) {
			// enable walk assist depending on speed
			ui8_riding_mode = WALK_ASSIST_MODE;
		}
		else if ((ui8_cruise_enabled)&& (ui16_wheel_speed_x10 > CRUISE_THRESHOLD_SPEED_X10)) {
			if (((ui8_cruise_legal)&&(ui8_pedal_cadence_RPM))||(!ui8_cruise_legal)) {
				// enable cruise function depending on speed
				ui8_riding_mode = CRUISE_MODE;
			}
		}
		else {
			ui8_riding_mode = ui8_rx_buffer[8];
		}
		if (!ui8_walk_assist) {
			// reset walk assist speed flag
			ui8_walk_assist_speed_flag = 0;
		}
		// wheel max speed
		ui8_wheel_speed_max = ui8_rx_buffer[9];

		// optional ADC function, temperature sensor or throttle or not in use
		ui8_optional_ADC_function = ui8_rx_buffer[10];

		// virtual throttle
		ui8_throttle_virtual = ui8_rx_buffer[11];

		///////////////////////////////////////////////////////////////////////////////////////
		
		// now send data back
		// ADC 10 bits battery voltage
		ui8_tx_buffer[3] = (ui16_adc_battery_voltage_filtered & 0xff);
		ui8_tx_buffer[4] = ((uint8_t) (ui16_adc_battery_voltage_filtered >> 4)) & 0x30;

		// send battery_current_x5
		ui8_tx_buffer[5] = ui8_battery_current_filtered_x5;

		// wheel speed
		ui8_tx_buffer[6] = (uint8_t) (ui16_wheel_speed_x10 & 0xff);
		ui8_tx_buffer[7] = ((uint8_t) (ui16_wheel_speed_x10 >> 8)) & 0x07;

		// brake state
		ui8_tx_buffer[8] = (ui8_brake_state & 1);
		// add the hall sensors state, that should be 3 bits only, value from 0 to 7
		ui8_tx_buffer[8] |= (ui8_hall_sensors_state << 1);
		ui8_tx_buffer[8] |= ((ui8_speed_limit_high_exceeded & 1) << 4);
		ui8_tx_buffer[8] |= ((ui8_voltage_cut_off_flag & 1) << 5);
		ui8_tx_buffer[8] |= ((ui8_voltage_shutdown_flag & 1) << 6);
		// ui8_tx_buffer[8] |= ((available & 1) << 7);

		// throttle value from ADC
		ui8_tx_buffer[9] = (uint8_t)(ui16_adc_throttle >> 2);
		// adjusted throttle value or temperature limit depending on user setup
		if (ui8_optional_ADC_function == TEMPERATURE_CONTROL) {
			// temperature value
			ui8_tx_buffer[10] = (uint8_t) (ui16_motor_temperature_filtered_x10 / 10);
		}
		else {
			// throttle value with offset removed and mapped to 255
			ui8_tx_buffer[10] = ui8_throttle_adc_in;
		}
		
		// ADC torque_sensor
		ui8_tx_buffer[11] = (uint8_t) (ui16_adc_torque & 0xff);
		// ADC torque_sensor (higher bits), this bits are shared with wheel speed bits
		ui8_tx_buffer[7] |= (uint8_t) ((ui16_adc_torque & 0x300) >> 2); //xx00 0000

		// pedal torque delta no boost
		ui8_tx_buffer[12] = (uint8_t) (ui16_adc_pedal_torque_delta_no_boost & 0xff);
		ui8_tx_buffer[13] = (uint8_t) (ui16_adc_pedal_torque_delta_no_boost >> 8);
		
		// PAS cadence
		ui8_tx_buffer[14] = ui8_pedal_cadence_RPM;
		
		// PWM duty_cycle
		// convert duty-cycle to 0 - 100 %
		ui16_temp = (uint16_t) ui8_g_duty_cycle;
		ui16_temp = (ui16_temp * 100) / PWM_DUTY_CYCLE_MAX;
		ui8_tx_buffer[15] = (uint8_t) ui16_temp;
		
		// motor speed in ERPS 
		ui8_tx_buffer[16] = (uint8_t) (ui16_motor_speed_erps & 0xff);
		ui8_tx_buffer[17] = (uint8_t) (ui16_motor_speed_erps >> 8);
		
		// FOC angle
		ui8_tx_buffer[18] = ui8_g_foc_angle;

		// system state
		ui8_tx_buffer[19] = ui8_m_system_state;

		// send motor_current_x5
		ui8_tx_buffer[20] = ui8_motor_current_filtered_x5;
		
		// wheel_speed_sensor_tick_counter
		ui8_tx_buffer[21] = (uint8_t) (ui32_wheel_speed_sensor_ticks_total & 0xff);
		ui8_tx_buffer[22] = (uint8_t) ((ui32_wheel_speed_sensor_ticks_total >> 8) & 0xff);
		ui8_tx_buffer[23] = (uint8_t) ((ui32_wheel_speed_sensor_ticks_total >> 16) & 0xff);

		// pedal torque delta boost
		ui8_tx_buffer[24] = (uint8_t) (ui16_adc_pedal_torque_delta & 0xff);
		ui8_tx_buffer[25] = (uint8_t) (ui16_adc_pedal_torque_delta >> 8);

		// first 8 bits of adc_motor_current
		//ui8_tx_buffer[26] = (uint8_t) (ui16_adc_battery_current & 0xff);
		ui8_tx_buffer[26] = ui8_adc_battery_current_filtered;
	  
		ui8_len += 24;
		break;

	  // set configurations
	  case COMM_FRAME_TYPE_CONFIGURATIONS:
		// disable the motor to avoid a quick of the motor while configurations are changed
		// disable the motor, lets hope this is safe to do here, in this way
		// the motor shold be enabled again on the ebike_control_motor()
		motor_disable_pwm();
		ui8_motor_enabled = 0;
		ui8_m_system_state |= ERROR_NOT_INIT;
		ui8_m_motor_init_state = MOTOR_INIT_STATE_INIT_START_DELAY;
		ui8_m_motor_init_status = MOTOR_INIT_STATUS_GOT_CONFIG;

		// battery low voltage cut-off x10
		ui16_battery_low_voltage_cut_off_x10 = (((uint16_t) ui8_rx_buffer[4]) << 8) + ((uint16_t) ui8_rx_buffer[3]);

		// set low voltage cutoff (10 bit)
		ui16_adc_voltage_cut_off = (ui16_battery_low_voltage_cut_off_x10 * 100U) / BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000;
		
		// set low voltage shutdown (10 bit)
		ui16_adc_voltage_shutdown = ui16_adc_voltage_cut_off - DIFFERENCE_CUT_OFF_SHUTDOWN_10_BIT;
		
		// wheel perimeter
		ui16_wheel_perimeter = (((uint16_t) ui8_rx_buffer[6]) << 8) + ((uint16_t) ui8_rx_buffer[5]);

		// battery max current
		//ebike_app_set_battery_max_current(ui8_rx_buffer[7]);
		ui8_battery_current_max = ui8_rx_buffer[7];
		ui8_target_battery_max_power_div25_temp = 0;

		ui8_temp = ui8_rx_buffer[8];
		ui8_startup_boost_enabled = ui8_temp & 1;
		ui8_startup_boost_at_zero = (ui8_temp & 2) >> 1;
		ui8_smooth_start_enabled = (ui8_temp & 4) >> 2;
		ui8_torque_sensor_calibration_enabled = (ui8_temp & 8) >> 3;
		ui8_assist_with_error_enabled = (ui8_temp & 16) >> 4;
		ui8_assist_without_pedal_rotation_enabled = (ui8_temp & 32) >> 5;
		uint8_t ui8_motor_type = (ui8_temp & 64) >> 6;
		ui8_eMTB_based_on_power = (ui8_temp & 128) >> 7;
		
		//ui8_motor_inductance_x1048576
		// motor inductance & cruise pid parameter
		if (ui8_motor_type == 0) {
			// 48 V motor
			ui8_foc_angle_multiplicator = FOC_ANGLE_MULTIPLIER; // mstrens : for TSDZ8, we do not take care of the motor type
		}
		else {
			// 36 V motor
			ui8_foc_angle_multiplicator = FOC_ANGLE_MULTIPLIER; // mstrens : for TSDZ8, we do not take care of the motor type
		}
		
		// startup boost
		ui16_startup_boost_factor_array[0] = (uint16_t) ui8_rx_buffer[10] << 1;
		ui8_startup_boost_cadence_step = ui8_rx_buffer[11];

		for (ui8_i = 1; ui8_i < 120; ui8_i++) {
			ui16_temp = (ui16_startup_boost_factor_array[ui8_i - 1] * (uint16_t)ui8_startup_boost_cadence_step) >> 8;
			ui16_startup_boost_factor_array[ui8_i] = ui16_startup_boost_factor_array[ui8_i - 1] - ui16_temp;	
		}

		// motor over temperature min value limit
		ui8_motor_temperature_min_value_to_limit = ui8_rx_buffer[12];
		// motor over temperature max value limit
		ui8_motor_temperature_max_value_to_limit = ui8_rx_buffer[13];
		
		// motor acceleration adjustment
		uint8_t ui8_motor_acceleration_adjustment = ui8_rx_buffer[14];
	  
		// set duty cycle ramp up inverse step default
		ui8_duty_cycle_ramp_up_inverse_step_default = map_ui8((uint8_t)ui8_motor_acceleration_adjustment,
				(uint8_t) 0,
                (uint8_t) 100,
                (uint8_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT,
                (uint8_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);
		
		// received target speed for cruise
		//ui16_wheel_speed_target_received_x10 = (uint16_t) (ui8_rx_buffer[15] * 10);
		// motor deceleration adjustment
		uint8_t ui8_motor_deceleration_adjustment = ui8_rx_buffer[15];
	  
		// set duty cycle ramp down inverse step default
		ui8_duty_cycle_ramp_down_inverse_step_default = map_ui8((uint8_t)ui8_motor_deceleration_adjustment,
				(uint8_t) 0,
                (uint8_t) 100,
                (uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,
                (uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
		
		// Torque ADC offset adjustment (0 / 34)
		ui8_adc_pedal_torque_offset_adj = ui8_rx_buffer[50];
		
		// Torque ADC range adjustment (0 / 40)
		ui8_adc_pedal_torque_range_adj = ui8_rx_buffer[51];
		
		// Torque ADC angle adjustment (0 / 40)
		ui8_adc_pedal_torque_angle_adj = ui8_rx_buffer[52];
		
		// Parameters for torque ADC offset adjustment
		ui8_adc_torque_calibration_offset = ui8_rx_buffer[53];
		ui8_adc_torque_middle_offset_adj = ui8_rx_buffer[54];
		
		// Torque ADC delta adjustment
		ui8_adc_pedal_torque_delta_adj = (ui8_adc_torque_middle_offset_adj * 2) - ui8_adc_torque_calibration_offset - ui8_adc_pedal_torque_offset_adj;
		
		// Smooth start counter set
		ui8_temp = ui8_rx_buffer[55];
		ui8_smooth_start_counter_set = map_ui8((uint8_t)ui8_temp,
				(uint8_t) 0,
                (uint8_t) 100,
                (uint8_t) 255,
                (uint8_t) SMOOTH_START_RAMP_MIN);
		
		// overcurrent check delay 
		ui8_battery_overcurrent_delay = ui8_rx_buffer[57];
		
		// for old display version
		uint8_t ui8_display_eeprom_version = ui8_rx_buffer[56];
		if (ui8_display_eeprom_version < 0x50) {
			ui8_smooth_start_counter_set = SMOOTH_START_RAMP_DEFAULT;
			ui8_battery_overcurrent_delay = 2; // * 25ms
		}
		
		// pedal torque adc offset min, max, for check calibration
		ui16_adc_pedal_torque_offset_set = (((uint16_t) ui8_rx_buffer[77]) << 8) + ((uint16_t) ui8_rx_buffer[76]);
		ui16_adc_pedal_torque_offset_min = ui16_adc_pedal_torque_offset_set - ADC_TORQUE_SENSOR_OFFSET_THRESHOLD;
		ui16_adc_pedal_torque_offset_max = ui16_adc_pedal_torque_offset_set + ADC_TORQUE_SENSOR_OFFSET_THRESHOLD;

		// pedal torque range (Right ADC8 - Right ADC1, weight=max)
		ui16_adc_pedal_torque_range = (((uint16_t) ui8_rx_buffer[79]) << 8) + ((uint16_t) ui8_rx_buffer[78]);
		ui16_adc_pedal_torque_range_ingrease_x100 = (ADC_TORQUE_SENSOR_RANGE_TARGET * 50) / ui16_adc_pedal_torque_range; //  / 2 * 100
		
		// pedal torque range target max
		ui16_adc_pedal_torque_range_target_max = (ADC_TORQUE_SENSOR_RANGE_TARGET_MIN
			* (100 + ui8_adc_pedal_torque_range_adj)) / 100;

		ui8_temp = ui8_rx_buffer[80];
		//uint8_t ui8_pedal_cadence_fast_stop = ui8_temp & 1; // not used
		ui8_field_weakening_feature_enabled = (ui8_temp & 2) >> 1;
		ui8_coaster_brake_enabled = (ui8_temp & 4) >> 2;
		// free for future use

		// if coast brake enabled, smooth srart min/default
		if (ui8_coaster_brake_enabled) {
			if (ui8_smooth_start_counter_set < SMOOTH_START_RAMP_DEFAULT) {
				ui8_smooth_start_counter_set = SMOOTH_START_RAMP_DEFAULT;
			}
		}
		ui8_smooth_start_counter_set_temp = ui8_smooth_start_counter_set;
		
		// coast brake threshold
		ui8_coaster_brake_torque_threshold = ui8_rx_buffer[81];
			
		//ui8_m_adc_lights_current_offset = (uint16_t) ui8_rx_buffer[82];
		// lights configuration
		ui8_lights_configuration = ui8_rx_buffer[82];
	  
		// torque sensor filter value
		//m_config_vars.ui8_torque_sensor_filter = ui8_rx_buffer[83];
		// torque sensor adc step (default 67) calibration disabled
		// torque sensor adc step (default 34) calibration enabled
		ui8_pedal_torque_per_10_bit_ADC_step_x100 = ui8_rx_buffer[83];

		// torque sensor ADC threshold
		if (ui8_assist_without_pedal_rotation_enabled) {
			ui8_assist_without_pedal_rotation_threshold = ui8_rx_buffer[84];
			if (ui8_assist_without_pedal_rotation_threshold > 100)
				{ ui8_assist_without_pedal_rotation_threshold = 100; }
		}
		else {
			ui8_assist_without_pedal_rotation_threshold = 0;
		}
		break;

      // firmware version
      case COMM_FRAME_TYPE_FIRMWARE_VERSION:
		ui8_tx_buffer[3] = ui8_m_system_state;
		ui8_tx_buffer[4] = 0;
		ui8_tx_buffer[5] = 21;
		ui8_tx_buffer[6] = 51;
		ui8_len += 4;
		break;

      case COMM_FRAME_TYPE_ALIVE:
		// nothing to add
		break;

      case COMM_FRAME_TYPE_STATUS:
		ui8_tx_buffer[3] = ui8_m_motor_init_status;
		ui8_len += 1;
		break;

      default:
		break;
	}

	ui8_tx_buffer[1] = ui8_len;

	// prepare crc of the package
	ui16_crc_tx = 0xffff;
	for (ui8_i = 0; ui8_i < ui8_len; ui8_i++) {
		crc16(ui8_tx_buffer[ui8_i], &ui16_crc_tx);
	}
	ui8_tx_buffer[ui8_len] = (uint8_t) (ui16_crc_tx & 0xff);
	ui8_tx_buffer[ui8_len + 1] = (uint8_t) (ui16_crc_tx >> 8) & 0xff;

	ui8_m_tx_buffer_index = 0;
	// start transmition
	// send the buffer on uart
	if ((XMC_USIC_CH_TXFIFO_GetLevel(CYBSP_DEBUG_UART_HW) ) >= (ui8_len + 2)){ // check if there is enough free space in Txfifo
		for(uint8_t i = 0; i < (ui8_len + 2); i++)  {
			XMC_USIC_CH_TXFIFO_PutData(CYBSP_DEBUG_UART_HW, (uint16_t) ui8_tx_buffer[i]);
			//XMC_UART_CH_Transmit(CYBSP_DEBUG_UART_HW , ui8_tx_buffer[i]);
		}
	}	 

	// get ready to get next package
	ui8_received_package_flag = 0;
}

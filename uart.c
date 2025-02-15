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

#include "uart.h"
#include "main.h"
#include "ebike_app.h"
#include "common.h"
#include "motor.h"

#ifdef USE_TSDZ6_VLCD5_VERSION
// function to get data from rxfifo and set them in rxBuffer; set a flag when a valid frame is received
// is called only when at least one byte is present in fifo
// UART
#define UART_RX_BUFFER_LEN   			7
#define RX_CHECK_CODE					(UART_RX_BUFFER_LEN - 1)															
#define UART_TX_BUFFER_LEN				9
#define TX_CHECK_CODE					(UART_TX_BUFFER_LEN - 1)
#define TX_STX							0x43
#define RX_STX							0x59
//uint8_t ui8_state_machine = 0; // 0= not yet a start byte , 1 = accumulate 
uint32_t rxIdx = 0;
//uint32_t ui8_rx_buffer[UART_RX_BUFFER_LEN];
//uint8_t ui8_rx_counter = 0;
//uint8_t ui8_received_package_flag = 0; // become 1 when buffer is filled with a frame
//uint8_t ui8_tx_buffer[UART_TX_BUFFER_LEN];

// UART
volatile uint8_t ui8_received_package_flag = 0;
volatile uint8_t ui8_rx_buffer[UART_RX_BUFFER_LEN];
volatile uint8_t ui8_rx_counter = 0;
volatile uint8_t ui8_tx_buffer[UART_TX_BUFFER_LEN];
volatile uint8_t ui8_byte_received;
volatile uint8_t ui8_state_machine = 0;

// uart send
volatile uint8_t ui8_working_status = 0;
volatile uint8_t ui8_display_fault_code = 0;


void fillRxBuffer() {
	if (XMC_USIC_CH_RXFIFO_IsEmpty(	CYBSP_DEBUG_UART_HW	))	return; // Skip when there is no data in rxfifo 
	
	// when there are data:

	// Read data : Reading data clear UART2_FLAG_RXNE flag
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






    switch (ui8_state_machine) {
        case 0:
            // see if we get start package byte
            if (ui8_byte_received == RX_STX)  {
                ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
                ui8_rx_counter++;
                ui8_state_machine = 1;
            }
            else {
                ui8_rx_counter = 0;
                ui8_state_machine = 0;
            }
        break;

        case 1:
            ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
            ui8_rx_counter++;

            // see if is the last byte of the package
            if (ui8_rx_counter >= UART_RX_BUFFER_LEN) {
                ui8_rx_counter = 0;
                ui8_state_machine = 0;
                ui8_received_package_flag = 1; // signal that we have a full package to be processed
            }
        break;

        default:
        break;

    }
}



// assist level 
#define OFF											0
#define ECO											1
#define TOUR										2
#define SPORT										3
#define TURBO										4

// assist pedal level mask
#define ASSIST_PEDAL_LEVEL0							0x10
#define ASSIST_PEDAL_LEVEL01						0x80
#define ASSIST_PEDAL_LEVEL1							0x40
#define ASSIST_PEDAL_LEVEL2							0x02
#define ASSIST_PEDAL_LEVEL3							0x04
#define ASSIST_PEDAL_LEVEL4							0x08

// oem display fault & function code
#define CLEAR_DISPLAY								0
#define NO_FUNCTION									0
#define NO_FAULT									0
#define NO_ERROR                                  	0 


void uart_receive_package() {  // once every 4*25 msec = 100 msec
	uint8_t ui8_i;
	uint8_t ui8_rx_check_code;
	uint8_t ui8_assist_level_mask;
	static uint8_t no_rx_counter = 0;
	static uint8_t ui8_lights_counter = 0;
	static uint8_t ui8_walk_assist_debounce_flag = 0;
	static uint8_t ui8_walk_assist_debounce_counter = 0;
	static uint8_t ui8_walk_assist_button_pressed = 0;
	
	// increment the comms safety counter
    no_rx_counter++;
	// increment walk assist counter
	ui8_walk_assist_debounce_counter++;
	// increment lights_counter
	ui8_lights_counter++;
	// increment display menu counter
	ui8_menu_counter++;

	if (ui8_received_package_flag) {
		// verify check code of the package
		ui8_rx_check_code = 0x00;
							 
		for(ui8_i = 0; ui8_i < RX_CHECK_CODE; ui8_i++)
		{
			ui8_rx_check_code += ui8_rx_buffer[ui8_i];
		}

		// see if check code is ok...
		if (ui8_rx_check_code == ui8_rx_buffer[RX_CHECK_CODE]) {
			// Reset the safety counter when a valid message from the LCD is received
            no_rx_counter = 0;
			
			// mask lights button from display
			ui8_lights_button_flag = ui8_rx_buffer[1] & 0x01;
			
			// mask walk assist button from display
			ui8_walk_assist_button_pressed = ui8_rx_buffer[1] & 0x20;
			
			// mask assist level from display
			ui8_assist_level_mask = ui8_rx_buffer[1] & 0xDE; // mask: 11011110
			ui8_assist_level_01_flag = 0;
			
			// set assist level
			switch (ui8_assist_level_mask) {
				case ASSIST_PEDAL_LEVEL0: ui8_assist_level = OFF; break;
				case ASSIST_PEDAL_LEVEL01:
					ui8_assist_level = ECO;
					ui8_assist_level_01_flag = 1;
					break;
				case ASSIST_PEDAL_LEVEL1: ui8_assist_level = ECO; break;
				case ASSIST_PEDAL_LEVEL2: ui8_assist_level = TOUR; break;
				case ASSIST_PEDAL_LEVEL3: ui8_assist_level = SPORT; break;
				case ASSIST_PEDAL_LEVEL4: ui8_assist_level = TURBO; break;
			}
			
			if (!ui8_display_ready_flag) {
				// assist level temp at power on
				ui8_assist_level_temp = ui8_assist_level;
			}
			// display ready
			ui8_display_ready_flag = 1;
			
			// display lights button pressed:
			if (ui8_lights_button_flag) {
				// lights off:
				if ((!ui8_lights_flag)&&
				  ((m_configuration_variables.ui8_set_parameter_enabled)||
				  (ui8_assist_level == OFF)||
				  ((ui8_startup_counter < m_config.delay_menu_on ) && (ui8_assist_level == TURBO)))) { // delay menu on = 50 => 5sec
					// lights 5s on
					ui8_lights_on_5s = 1;
					
					// menu flag
					if (!ui8_menu_flag) 
					{
						// set menu flag
						ui8_menu_flag = 1;
							
						// set menu index
						if (++ui8_menu_index > 3) { 
							ui8_menu_index = 3;
						}
						
						// display status alternative lights configuration
						ui8_display_alternative_lights_configuration = 0;
						
						// restore previous parameter
						switch (ui8_assist_level) {	
							case OFF:
								switch (ui8_menu_index) {
									case 2:
										// restore previous set parameter
										m_configuration_variables.ui8_set_parameter_enabled = ui8_set_parameter_enabled_temp;
										ui8_display_function_status[0][OFF] = m_configuration_variables.ui8_set_parameter_enabled;
										break;
									case 3:
										// restore previous auto display data
										m_configuration_variables.ui8_auto_display_data_enabled = ui8_auto_display_data_enabled_temp;
										ui8_display_function_status[1][OFF] = m_configuration_variables.ui8_auto_display_data_enabled;
										break;
								}
								break;
								
							case ECO:
								switch (ui8_menu_index) {
									case 2:
										// restore previous street mode
										m_configuration_variables.ui8_street_mode_enabled = ui8_street_mode_enabled_temp;
										ui8_display_function_status[0][ECO] = m_configuration_variables.ui8_street_mode_enabled;
										break;
									case 3:
										// restore previous startup boost
										m_configuration_variables.ui8_startup_boost_enabled = ui8_startup_boost_enabled_temp;
										ui8_display_function_status[1][ECO] = m_configuration_variables.ui8_startup_boost_enabled;
									
										if (ui8_display_torque_sensor_flag_2) {
											// set display torque sensor step for calibration
											ui8_display_torque_sensor_step = 1;
											// delay display torque sensor step for calibration
											ui8_delay_display_function = DELAY_DISPLAY_TORQUE_CALIBRATION; //250 = 25 sec
										}
										else if (ui8_display_torque_sensor_flag_1) {
											// restore torque sensor advanced
											m_configuration_variables.ui8_torque_sensor_adv_enabled = ui8_torque_sensor_adv_enabled_temp;
											ui8_display_function_status[2][ECO] = m_configuration_variables.ui8_torque_sensor_adv_enabled;
											
											// set display torque sensor value for calibration
											ui8_display_torque_sensor_value = 1;
											// delay display torque sensor value for calibration
											ui8_delay_display_function = DELAY_DISPLAY_TORQUE_CALIBRATION; // 25 sec
										}
										break;
								}
								break;
							
							case TURBO:	
								switch (ui8_menu_index) {
									case 2:
										if (ui8_lights_configuration_2 == 9) {
											// restore previous lights configuration
											m_configuration_variables.ui8_lights_configuration = ui8_lights_configuration_temp;
											ui8_display_lights_configuration = m_configuration_variables.ui8_lights_configuration;
											// display status
											ui8_display_alternative_lights_configuration = 1;
										}
										break;
									case 3:
										if (ui8_lights_configuration_2 == 9) {
											// restore previous assist without pedal rotation
											m_configuration_variables.ui8_assist_without_pedal_rotation_enabled = ui8_assist_without_pedal_rotation_temp;
											ui8_display_function_status[1][TURBO] = m_configuration_variables.ui8_assist_without_pedal_rotation_enabled;
										}
										else {
											// restore previous lights configuration
											m_configuration_variables.ui8_lights_configuration = ui8_lights_configuration_temp;
											ui8_display_lights_configuration = m_configuration_variables.ui8_lights_configuration;
										}
										
										if (ui8_lights_configuration_3 == 10) {
											// display status
											ui8_display_alternative_lights_configuration = 1;
										}
										break;
								}							
								break;
						} // end of switch
						
						// display function code enabled (E02, E03,E04)
						ui8_display_function_code = ui8_menu_index + 1;
						// display function code temp (for display function status VLCD5/6)
						ui8_display_function_code_temp = ui8_display_function_code;
						// display data function enabled
						ui8_display_data_enabled = 1;
						
						// restart lights counter
						ui8_lights_counter = 0;	
						// restart menu counter
						ui8_menu_counter = 0;					
					}
					
					// after some seconds: switch on lights (if enabled) and abort function
					if ((ui8_lights_counter >=  c_DELAY_LIGHTS_ON)||
					  ((ui8_assist_level != ui8_assist_level_temp)&&(!ui8_torque_sensor_calibration_flag)&&(!ui8_auto_display_data_flag)))
					{
						// set lights flag
						ui8_lights_flag = 1;
						// lights 5s off		
						ui8_lights_on_5s = 0;
						// clear menu flag
						ui8_menu_flag = 0;
						// clear menu index
						ui8_menu_index = 0;
						// clear menu counter
						ui8_menu_counter = ui8_delay_display_function;
						// clear lights counter
						ui8_lights_counter = c_DELAY_LIGHTS_ON;
						// display function code disabled
						ui8_display_function_code = NO_FUNCTION;
					}
				}
				else {
					// set lights flag
					ui8_lights_flag = 1;
				}
			
			}
			else {
				// lights off:
				if (!ui8_lights_flag)
				{
					// menu flag active:
					if (ui8_menu_flag) {
						// clear menu flag
						ui8_menu_flag = 0;
							
						// lights 5s off		
						ui8_lights_on_5s = 0;	
							
						// restart menu counter
						ui8_menu_counter = 0;
								
						// menu function enabled
						ui8_menu_function_enabled = 1;
					}
				}
				else {
					// clear lights flag
					ui8_lights_flag = 0;
				}
			}

			// restart menu display function (after some delay or (if assist level changed and not torque calib and not auto display)
			if ((ui8_menu_counter >= ui8_delay_display_function)||
			  ((ui8_assist_level != ui8_assist_level_temp)&&(!ui8_torque_sensor_calibration_flag)&&(!ui8_auto_display_data_flag)))
			{					
				// clear menu flag
				ui8_menu_flag = 0;
				// clear menu index
				ui8_menu_index = 0;
				// clear menu counter
				ui8_menu_counter = ui8_delay_display_function;
				// menu function disabled
				ui8_menu_function_enabled = 0;
				// display data function disabled
				ui8_display_data_enabled = 0;
				// display function code disabled
				ui8_display_function_code = NO_FUNCTION;
				// display torque flag 1 disabled
				ui8_display_torque_sensor_flag_1 = 0;
				// display torque flag 2 disabled
				ui8_display_torque_sensor_flag_2 = 0;
				// display torque value disabled
				ui8_display_torque_sensor_value = 0;
				// display torque step disabled
				ui8_display_torque_sensor_step = 0;
				// clear display soc %
				ui8_display_battery_soc = 0;
			}

			// display menu function
			if (ui8_menu_function_enabled) {
				// display status lights configuration
				ui8_display_alternative_lights_configuration = 0;
				// set display parameter
				if ((m_configuration_variables.ui8_set_parameter_enabled)||
				  (ui8_assist_level == OFF)||
				  ((ui8_startup_counter < m_config.delay_menu_on) && (ui8_assist_level == TURBO)))
				{
					switch (ui8_assist_level) {	
						case OFF:
							// set parameter
							switch (ui8_menu_index) {
								case 1:
									// for restore set parameter
									ui8_set_parameter_enabled_temp = m_configuration_variables.ui8_set_parameter_enabled;
									
									// set parameter enabled  // invert the parameter
									m_configuration_variables.ui8_set_parameter_enabled = !m_configuration_variables.ui8_set_parameter_enabled;
									ui8_display_function_status[0][OFF] = m_configuration_variables.ui8_set_parameter_enabled;
									break;
								case 2:		   
									// for restore auto display data
									ui8_auto_display_data_enabled_temp = m_configuration_variables.ui8_auto_display_data_enabled;
									
									// set auto display data
									m_configuration_variables.ui8_auto_display_data_enabled = !m_configuration_variables.ui8_auto_display_data_enabled;
									ui8_display_function_status[1][OFF] = m_configuration_variables.ui8_auto_display_data_enabled;
									break;
								case 3:
									// save current configuration
	// mstrens todo	!!!!!!!!						//EEPROM_controller(WRITE_TO_MEMORY, EEPROM_BYTES_INIT_OEM_DISPLAY);
									break;
							}
							break;
						
						case ECO:
							// set street/offroad mode
							switch (ui8_menu_index) {
								case 1:
									// for restore street mode
									ui8_street_mode_enabled_temp = m_configuration_variables.ui8_street_mode_enabled;
									
									// change street mode
									m_configuration_variables.ui8_street_mode_enabled = !m_configuration_variables.ui8_street_mode_enabled;
									ui8_display_function_status[0][ECO] = m_configuration_variables.ui8_street_mode_enabled;
									break;
								case 2:																		 
									// for restore startup boost
									ui8_startup_boost_enabled_temp = m_configuration_variables.ui8_startup_boost_enabled;	
									
									// change startup boost
									m_configuration_variables.ui8_startup_boost_enabled = !m_configuration_variables.ui8_startup_boost_enabled;
									ui8_display_function_status[1][ECO] = m_configuration_variables.ui8_startup_boost_enabled;
									break;
								case 3: // is used for 2 functions (torque sensor adv ON/OFF or for torque sensor calibration)
									if (!ui8_display_torque_sensor_value)
									{
										// for restore torque sensor advanced
										ui8_torque_sensor_adv_enabled_temp = m_configuration_variables.ui8_torque_sensor_adv_enabled;
									
										// change torque sensor advanced mode
										m_configuration_variables.ui8_torque_sensor_adv_enabled = !m_configuration_variables.ui8_torque_sensor_adv_enabled;
										ui8_display_function_status[2][ECO] = m_configuration_variables.ui8_torque_sensor_adv_enabled;
										
										// set display torque sensor flag 1 for value calibration
										ui8_display_torque_sensor_flag_1 = 1;
									}
									else {
										// for recovery actual riding mode
										if (m_configuration_variables.ui8_riding_mode != TORQUE_SENSOR_CALIBRATION_MODE) {
											ui8_riding_mode_temp = m_configuration_variables.ui8_riding_mode;
										}
										// special riding mode (torque sensor calibration)
										m_configuration_variables.ui8_riding_mode = TORQUE_SENSOR_CALIBRATION_MODE; // 8 = special mode for calibration
										
										// set display torque sensor flag 2 for step calibration
										ui8_display_torque_sensor_flag_2 = 1;
									}
									break;
							}
							break;

						case TOUR:
							// set riding mode 1
							switch (ui8_menu_index) {
								case 1: m_configuration_variables.ui8_riding_mode = POWER_ASSIST_MODE; break; // 1
								case 2: m_configuration_variables.ui8_riding_mode = TORQUE_ASSIST_MODE; break; // 2
								case 3: m_configuration_variables.ui8_riding_mode = CADENCE_ASSIST_MODE; break; // 3
							}
							ui8_display_riding_mode = m_configuration_variables.ui8_riding_mode;
							break;
						
						case SPORT:
							// set riding mode 2
							switch (ui8_menu_index) {
								case 1:	m_configuration_variables.ui8_riding_mode = eMTB_ASSIST_MODE; break; // 4
								case 2: m_configuration_variables.ui8_riding_mode = HYBRID_ASSIST_MODE; break;  // 5        
								case 3: m_configuration_variables.ui8_riding_mode = CRUISE_MODE; break; // 6
							}
							ui8_display_riding_mode = m_configuration_variables.ui8_riding_mode;
							break;
						
						case TURBO:
							// set lights mode
							switch (ui8_menu_index) {
								case 1:  
									if (ui8_startup_counter < m_config.delay_menu_on) {  // 5 sec
										// manually setting battery percentage x10 (actual charge) within 5 seconds of power on
										ui16_battery_SOC_percentage_x10 = read_battery_soc();
										// calculate watt-hours x10
										ui32_wh_x10_offset = ((uint32_t) (1000 - ui16_battery_SOC_percentage_x10) * ui16_actual_battery_capacity) / 100;
										// for display soc %
										ui8_display_battery_soc = 1;
									}
									else {
										// for restore lights configuration (Light ON (0) - Light flashing (1))
										ui8_lights_configuration_temp = m_configuration_variables.ui8_lights_configuration;
									
										if (m_configuration_variables.ui8_lights_configuration != m_config.lights_configuration_on_startup) {
											m_configuration_variables.ui8_lights_configuration = m_config.lights_configuration_on_startup; // 0
										}
										else {
											m_configuration_variables.ui8_lights_configuration = m_config.lights_configuration_1; // 1
										}
									}
									break;
								case 2: // - LIGHTS ON & BRAKE FLASHING or ASSIST WITHOUT PEDALIG ROTATION (0 / 1)
									if (ui8_lights_configuration_2 == 9) {
										// for restore assist without pedal rotation
										ui8_assist_without_pedal_rotation_temp = m_configuration_variables.ui8_assist_without_pedal_rotation_enabled;
									
										// change assist without pedal rotation
										m_configuration_variables.ui8_assist_without_pedal_rotation_enabled = !m_configuration_variables.ui8_assist_without_pedal_rotation_enabled;
										ui8_display_function_status[1][TURBO] = m_configuration_variables.ui8_assist_without_pedal_rotation_enabled;
										// display status
										ui8_display_alternative_lights_configuration = 1;
									}
									else {
										m_configuration_variables.ui8_lights_configuration = m_config.lights_configuration_2;  // 9
										// for restore lights configuration
										ui8_lights_configuration_temp = m_configuration_variables.ui8_lights_configuration;
									
									}
									break;
								case 3: // LIGHTS FLASHING & BRAKE ON or ASSIST WITH SENSORS ERROR (0 / 1)
									if (ui8_lights_configuration_3 == 10) {
										// change system error enabled
										m_configuration_variables.ui8_assist_with_error_enabled = !m_configuration_variables.ui8_assist_with_error_enabled;
										ui8_display_function_status[2][TURBO] = m_configuration_variables.ui8_assist_with_error_enabled;
										// display status
										ui8_display_alternative_lights_configuration = 1;
									}
									else {
										m_configuration_variables.ui8_lights_configuration = m_config.lights_configuration_3;
									}
									break;
							}
							// display lights configuration
							ui8_display_lights_configuration = m_configuration_variables.ui8_lights_configuration;
							break;
					}
					
					// delay display menu function 
					if (!ui8_display_torque_sensor_value)
						ui8_delay_display_function = m_config.delay_menu_on;  // 5 sec
					
					// display data value enabled
					ui8_display_data_enabled = 1;
				}
			}
			
			// display function status VLCD5/6
			//#if ENABLE_VLCD5 || ENABLE_VLCD6
			if ((m_config.enable_vlcd5) || (m_config.enable_vlcd6)){
				if (ui8_menu_flag) {
					if (ui8_menu_counter >= c_DELAY_FUNCTION_STATUS)  // 2,5 sec
						// display function code disabled
						ui8_display_function_code = NO_FUNCTION;
				}
				else {
					if ((ui8_menu_counter > (m_config.delay_menu_on - c_DELAY_FUNCTION_STATUS))&&
						(ui8_menu_counter < m_config.delay_menu_on)&&(ui8_menu_index)) {
						// restore display function code
						ui8_display_function_code = ui8_display_function_code_temp;
					}
					else {
						// display function code disabled
						ui8_display_function_code = NO_FUNCTION;
					}
				}
			}
			
			// menu function disabled
			ui8_menu_function_enabled = 0;

			// special riding modes with walk assist button
			switch (m_configuration_variables.ui8_riding_mode) {	
				case TORQUE_SENSOR_CALIBRATION_MODE:
					bool flag1 = false;
					//#if ENABLE_XH18 || ENABLE_VLCD5 || ENABLE_850C
					if ((m_config.enable_xh18) || (m_config.enable_vlcd5) || (m_config.enablec850) ){
						if (((ui8_assist_level != OFF)&&(ui8_assist_level != ECO))||
							(ui8_menu_counter >= ui8_delay_display_function))              flag1 = true;
					} else { //#else // ENABLE VLCD6
						if ((ui8_assist_level != ECO)||(ui8_menu_counter >= ui8_delay_display_function)) flag1 = true;
					}
					if (flag1)
					{	
						// riding mode recovery at level change
						m_configuration_variables.ui8_riding_mode = ui8_riding_mode_temp;
						// clear torque sensor calibration flag
						ui8_torque_sensor_calibration_flag = 0;
						// display data function disabled
						ui8_display_data_enabled = 0;
						// clear menu counter
						ui8_menu_counter = ui8_delay_display_function;
					}
					else {
						if (!ui8_torque_sensor_calibration_flag) {
							// set torque sensor calibration flag
							ui8_torque_sensor_calibration_flag = 1;
							// restart menu counter
							ui8_menu_counter = 0;
						}

						// walk assist button pressed
						if ((ui8_walk_assist_button_pressed)&&(ui8_display_ready_flag)) {
							ui8_torque_sensor_calibration_started = 1;
						}
						else {
							ui8_torque_sensor_calibration_started = 0;
						}
						// display data function enabled
						//ui8_display_data_enabled = 1;
						
						//#if ENABLE_VLCD5 || ENABLE_VLCD6
						if ((m_config.enable_vlcd5) || (m_config.enable_vlcd6)){
							// display function code disabled
							ui8_display_function_code = NO_FUNCTION;
						}
					}
					break;
					
				case CRUISE_MODE:
					// walk assist button pressed
					if ((ui8_walk_assist_button_pressed)&&(ui8_display_ready_flag)) {
						ui8_cruise_button_flag = 1;
					}
					else {
						ui8_cruise_button_flag = 0;
					}
					break;
				
				default:
					// startup assist mode
					//#if STARTUP_ASSIST_ENABLED
					if (m_config.startup_assist_enabled) {
						// walk assist button pressed
						if ((ui8_walk_assist_button_pressed)&&(ui8_display_ready_flag)
							&&(!ui8_walk_assist_flag)&&(ui8_lights_flag)) {
							ui8_startup_assist_flag = 1;
						}
						else {
							ui8_startup_assist_flag = 0;
						}
					}
					
					// walk assist mode
					//#if ENABLE_WALK_ASSIST
					if (m_config.enable_walk_assist){
						// walk assist button pressed
						if ((ui8_walk_assist_button_pressed)&&(ui8_display_ready_flag)&&(!ui8_startup_assist_flag)
						&&(ui8_walk_assist_enabled_array[m_configuration_variables.ui8_street_mode_enabled])) {
							if (!ui8_walk_assist_flag) {
								// set walk assist flag
								ui8_walk_assist_flag = 1;
								// for restore riding mode
								ui8_riding_mode_temp = m_configuration_variables.ui8_riding_mode;
								// set walk assist mode
								m_configuration_variables.ui8_riding_mode = WALK_ASSIST_MODE;
							}
						}
						else {
							//#if WALK_ASSIST_DEBOUNCE_ENABLED && ENABLE_BRAKE_SENSOR
							if ((m_config.walk_assist_debounce_enabled) && (m_config.enable_brake_sensor)){
								if (ui8_walk_assist_flag) {
									if (!ui8_walk_assist_debounce_flag) {
										// set walk assist debounce flag
										ui8_walk_assist_debounce_flag = 1;
										// restart walk assist counter
										ui8_walk_assist_debounce_counter = 0;
										// walk assist level during debounce time
										ui8_walk_assist_level = ui8_assist_level;
									}
								
									if (ui8_walk_assist_debounce_counter < m_config.walk_assist_debounce_time) {
										// stop walk assist during debounce time
										if ((ui8_assist_level != ui8_walk_assist_level)||(ui8_brake_state)
										||(m_configuration_variables.ui8_street_mode_enabled)) {
											// restore previous riding mode
											m_configuration_variables.ui8_riding_mode = ui8_riding_mode_temp;
											// reset walk assist flag
											ui8_walk_assist_flag = 0;
											// reset walk assist debounce flag
											ui8_walk_assist_debounce_flag = 0;
											// reset walk assist speed flag
											ui8_walk_assist_speed_flag = 0;
										}
									}	
									else {
										// restore previous riding mode
										if (ui8_walk_assist_flag) {
											m_configuration_variables.ui8_riding_mode = ui8_riding_mode_temp;
										}
										// reset walk assist flag
										ui8_walk_assist_flag = 0;
										// reset walk assist debounce flag
										ui8_walk_assist_debounce_flag = 0;
										// reset walk assist speed flag
										ui8_walk_assist_speed_flag = 0;
									}
								}
							} else { //	#else
								// restore previous riding mode
								if (ui8_walk_assist_flag) {
									m_configuration_variables.ui8_riding_mode = ui8_riding_mode_temp;
								}
								// reset walk assist flag
								ui8_walk_assist_flag = 0;
								// reset walk assist debounce flag
								ui8_walk_assist_debounce_flag = 0;
								// reset walk assist speed flag
								ui8_walk_assist_speed_flag = 0;
							} //	#endif
						}
					}	//#endif
					break;
			}
			
			// set assist parameter
			ui8_riding_mode_parameter = ui8_riding_mode_parameter_array[m_configuration_variables.ui8_riding_mode - 1][ui8_assist_level];
			if (ui8_assist_level_01_flag) {
				ui8_riding_mode_parameter = (uint8_t)(((uint16_t) ui8_riding_mode_parameter * m_config.assist_level_1_of_5_percent) / 100); // ASSIST_LEVEL_1_OF_5_PERCENT
			}
			
			// automatic data display at lights on
			if (m_configuration_variables.ui8_auto_display_data_enabled) {	
				if ((ui8_lights_flag) && (!ui8_menu_index) && (!ui8_display_battery_soc) &&
						(!ui8_display_torque_sensor_value) && (!ui8_display_torque_sensor_step)) {
					if (!ui8_auto_display_data_flag) {	
						// set auto display data flag
						ui8_auto_display_data_flag = 1;
						ui8_auto_display_data_status = 1;
						// display data function enabled
						ui8_display_data_enabled = 1;
						// restart menu counter
						ui8_menu_counter = 0;
						// set data index
						ui8_data_index = 0;
						// assist level temp, ignore first change
						ui8_assist_level_temp = ui8_assist_level;
						// delay data function
						if (ui8_delay_display_array[ui8_data_index]) {
							ui8_delay_display_function  = ui8_delay_display_array[ui8_data_index]; // ecpected delay if not 0
						}
						else {
							ui8_delay_display_function  = m_config.delay_menu_on; // 5 sec
						}
					}
					
					// restart menu counter if data delay is zero
					if (!ui8_delay_display_array[ui8_data_index]) {
						ui8_menu_counter = 0;
					}
					if ((ui8_data_index + 1) < ui8_auto_data_number_display) {
						if ((ui8_menu_counter >= (ui8_delay_display_function - 4))||(ui8_assist_level != ui8_assist_level_temp)) {
							// restart menu counter
							ui8_menu_counter = 0;
							// increment data index
							ui8_data_index++;
							// delay data function
							if (ui8_delay_display_array[ui8_data_index]) {
								ui8_delay_display_function  = ui8_delay_display_array[ui8_data_index];
							}
							else {
								ui8_delay_display_function  = m_config.delay_menu_on; //DELAY_MENU_ON
							}
						}
					}
					else if (ui8_menu_counter >= ui8_delay_display_function) {
						ui8_auto_display_data_status = 0;
					}
				}
				else {
					if (ui8_auto_display_data_flag) {
						// reset auto display data flag
						ui8_auto_display_data_flag = 0;
						ui8_auto_display_data_status = 0;
						// display data function disabled
						ui8_display_data_enabled = 0;
					}
				}
			}
			
			// assist level temp, to change or stop operation at change of level
			ui8_assist_level_temp = ui8_assist_level;
			
			// set lights
			//#if ENABLE_LIGHTS
			if (m_config.enable_lights) {
				// switch on/switch off lights
				if ((ui8_lights_flag)||(ui8_lights_on_5s)) {
					ui8_lights_state = 1;
				}
				else {
					ui8_lights_state = 0;
				}
			}//#endif
			
			// ui8_rx_buffer[2] current max?
			
			// get wheel diameter from display
			ui8_oem_wheel_diameter = ui8_rx_buffer[3];
			
			// factor to calculate the value of the data to be displayed
			if (m_config.units_type) {// 1 mph and miles
				ui16_display_data_factor = 900 * ui8_oem_wheel_diameter; // OEM_WHEEL_FACTOR = 1435
			} else {                  // 0 = km/h and km
				ui16_display_data_factor = 1435 * ui8_oem_wheel_diameter; // OEM_WHEEL_FACTOR = 1435	
			}

			// ui8_rx_buffer[4] test?
			
			//#if ENABLE_WHEEL_MAX_SPEED_FROM_DISPLAY
			if (m_config.enable_wheel_max_speed_from_display){
				// set wheel max speed from display
				ui8_wheel_speed_max_array[OFFROAD_MODE] = ui8_rx_buffer[5];
				if (ui8_wheel_speed_max_array[STREET_MODE] > ui8_wheel_speed_max_array[OFFROAD_MODE]) {
					ui8_wheel_speed_max_array[STREET_MODE] = ui8_wheel_speed_max_array[OFFROAD_MODE];
				}
			}//#endif
			
			// set speed limit in street, offroad, walk assist, startup assist, throttle 6km/h mode
			if ((m_configuration_variables.ui8_riding_mode == WALK_ASSIST_MODE)
				||(ui8_startup_assist_flag)) {
				m_configuration_variables.ui8_wheel_speed_max = m_config.walk_assist_threshold_speed_x10 / 10; // 6
			}
			else if ((ui8_throttle_mode_array[m_configuration_variables.ui8_street_mode_enabled] == W_O_P_6KM_H_ONLY)
				&& (ui8_throttle_adc_in)) {
					m_configuration_variables.ui8_wheel_speed_max = m_config.walk_assist_threshold_speed_x10 / 10;  // 6
			}
			else {
				m_configuration_variables.ui8_wheel_speed_max = ui8_wheel_speed_max_array[m_configuration_variables.ui8_street_mode_enabled];
			}
			/* moved to ebike_app.c because it is called more often and this occurs only if messages are received from display
			// current limit with power limit
			ui8_adc_battery_current_max_temp_2 = (uint8_t)((uint32_t)(ui32_adc_battery_power_max_x1000_array[m_configuration_variables.ui8_street_mode_enabled]
				/ ui16_battery_voltage_filtered_x1000));
			
			// set max battery current
			ui8_adc_battery_current_max = ui8_min(ui8_adc_battery_current_max_temp_1, ui8_adc_battery_current_max_temp_2);
			*/
			// set pedal torque per 10_bit DC_step x100 advanced
			ui8_pedal_torque_per_10_bit_ADC_step_x100 = ui8_pedal_torque_per_10_bit_ADC_step_x100_array[m_configuration_variables.ui8_torque_sensor_adv_enabled];
		}
		
		// signal that we processed the full package
		ui8_received_package_flag = 0;
		
		// assist level = OFF if connection with the LCD is lost for more than 0,3 sec (safety)
		if (no_rx_counter > 3) {
			ui8_assist_level = OFF;
		}
	}
}

void uart_send_package(){
	uint8_t ui8_i;
	uint8_t ui8_tx_check_code;
	
	// display ready
	if (ui8_display_ready_flag) {
		// send the data to the LCD
		// start up byte
		ui8_tx_buffer[0] = TX_STX;

		// clear fault code
		ui8_display_fault_code = NO_FAULT;

		// initialize working status
		ui8_working_status &= 0xFE; // bit0 = 0 (battery normal)

		//#if ENABLE_VLCD6 || ENABLE_XH18
		if ((m_config.enable_vlcd6) || (m_config.enable_xh18)){
			switch (ui8_battery_state_of_charge) {
				case 0:
					ui8_working_status |= 0x01; // bit0 = 1 (battery undervoltage)
					ui8_tx_buffer[1] = 0x00;
					break;
				case 1:
					ui8_tx_buffer[1] = 0x00; // Battery 0/4 (empty and blinking)
					break;
				case 2:
					ui8_tx_buffer[1] = 0x02; // Battery 1/4
					break;
				case 3:
					ui8_tx_buffer[1] = 0x06; // Battery 2/4
					break;
				case 4:
					ui8_tx_buffer[1] = 0x09; // Battery 3/4
					break;
				case 5:
					ui8_tx_buffer[1] = 0x0C; // Battery 4/4 (full)
					break;
				case 6:
					ui8_tx_buffer[1] = 0x0C; // Battery 4/4 (soc reset)
					break;
				case 7:
					ui8_tx_buffer[1] = 0x0C; // Battery 4/4 (full)
					// E01 (E06 blinking for XH18) ERROR_OVERVOLTAGE
					ui8_display_fault_code = ERROR_OVERVOLTAGE; // Fault overvoltage
					break;
			}
		} else {//}	#else // ENABLE_VLCD5 or 850C
			switch (ui8_battery_state_of_charge) {
				case 0:
					ui8_working_status |= 0x01; // bit0 = 1 (battery undervoltage)
					ui8_tx_buffer[1] = 0x00;
					break;
				case 1:
					ui8_tx_buffer[1] = 0x00; // Battery 0/6 (empty and blinking)
					break;
				case 2:
					ui8_tx_buffer[1] = 0x02; // Battery 1/6
					break;
				case 3:
					ui8_tx_buffer[1] = 0x04; // Battery 2/6
					break;
				case 4:
					ui8_tx_buffer[1] = 0x06; // Battery 3/6
					break;
				case 5:
					ui8_tx_buffer[1] = 0x08; // Battery 4/6
					break;
				case 6:
					ui8_tx_buffer[1] = 0x0A; // Battery 5/6
					break;
				case 7:
					ui8_tx_buffer[1] = 0x0C; // Battery 6/6 (full)
					break;
				case 8:
					ui8_tx_buffer[1] = 0x0C; // Battery 6/6 (soc reset)
					break;
				case 9:
					ui8_tx_buffer[1] = 0x0C; // Battery 6/6 (full)
					// E01 ERROR_OVERVOLTAGE
					ui8_display_fault_code = ERROR_OVERVOLTAGE; // Fault overvoltage
					break;
			}
		} //#endif
		
		// reserved for VLCD5, torque sensor value TE and TE1
		//#if ENABLE_VLCD5
		if ( m_config.enable_vlcd5) {
			ui8_tx_buffer[3] = ui16_adc_pedal_torque_offset_init;
			if (ui16_adc_pedal_torque > ui16_adc_pedal_torque_offset_init) {
				ui8_tx_buffer[4] = ui16_adc_pedal_torque - ui16_adc_pedal_torque_offset_init;
			}
			else {
				ui8_tx_buffer[4] = 0;
			}
		} else if (m_config.enablec850){ //} #elif ENABLE_850C
			ui8_tx_buffer[3] = ui8_battery_current_filtered_x10;
			// battery power filtered x 10 for display data
			ui16_battery_power_filtered_x10 = filter(ui16_battery_power_x10, ui16_battery_power_filtered_x10, 8);
			ui8_tx_buffer[4] = (uint8_t) (ui16_battery_power_filtered_x10 / 100);
		} else { //#else
			ui8_tx_buffer[3] = 0x46;
			ui8_tx_buffer[4] = 0x46;
		} //#endif
		
		// fault temperature limit
		// E06 ERROR_OVERTEMPERATURE
		#if (OPTIONAL_ADC_FUNCTION == TEMPERATURE_CONTROL) && ENABLE_TEMPERATURE_ERROR_MIN_LIMIT
		// temperature error at min limit value
		if (((uint8_t) (ui16_motor_temperature_filtered_x10 / 10)) >= ui8_motor_temperature_min_value_to_limit_array[TEMPERATURE_SENSOR_TYPE])
		{
			ui8_display_fault_code = ERROR_OVERTEMPERATURE;
		}
		#elif (OPTIONAL_ADC_FUNCTION == TEMPERATURE_CONTROL) && !ENABLE_TEMPERATURE_ERROR_MIN_LIMIT
		// temperature error at max limit value
		if (((uint8_t) (ui16_motor_temperature_filtered_x10 / 10)) >= ui8_motor_temperature_max_value_to_limit_array[TEMPERATURE_SENSOR_TYPE])
		{
			ui8_display_fault_code = ERROR_OVERTEMPERATURE;
		}
		#elif BRAKE_TEMPERATURE_SWITCH
		if (ui8_brake_state) {
			ui8_display_fault_code = ERROR_OVERTEMPERATURE;
		}
		#endif

// here new code added by mstrens
    #define MAX_TEMPERATURE 80
	/* Calculate temperature of the chip in Kelvin */
        uint32_t temp_k = XMC_SCU_CalcTemperature();

        /* Convert temperature to Celsius */
        if ((temp_k - 273) > MAX_TEMPERATURE){
			ui8_display_fault_code = ERROR_OVERTEMPERATURE;
		}


		// blocked motor error has priority
		if (ui8_system_state == ERROR_MOTOR_BLOCKED) {	
			ui8_display_fault_code = ERROR_MOTOR_BLOCKED;
		} else {
			if ((ui8_system_state != NO_ERROR)&&(ui8_display_fault_code == NO_FAULT)) {
				ui8_display_fault_code = ui8_system_state;
			}
		}

		// send to display function code or fault code
		if ((ui8_display_fault_code != NO_FAULT)&&(ui8_display_function_code == NO_FUNCTION)) {
			//#if ENABLE_XH18
			if (m_config.enable_xh18){
				if (ui8_display_fault_code == ERROR_WRITE_EEPROM) {
					// shared with ERROR_MOTOR_CHECK
					// instead of E09, display blinking E08
					if (ui8_default_flash_state) {
						ui8_tx_buffer[5] = 8;
					}
				}
				else if (ui8_display_fault_code == ERROR_OVERVOLTAGE) {	
					// instead of E01, display blinking E06
					if (ui8_default_flash_state) {
						ui8_tx_buffer[5] = 6;
					}
				}
				else if (ui8_display_fault_code == ERROR_THROTTLE) {	
					// instead of E05, display blinking E03
					if (ui8_default_flash_state) {
						ui8_tx_buffer[5] = 3;
					}
				}
				else if (ui8_display_fault_code == ERROR_BATTERY_OVERCURRENT) {	
					// instead of E07, display blinking E04
					if (ui8_default_flash_state) {
						ui8_tx_buffer[5] = 4;
					}
				}
				else {
					// fault code
					ui8_tx_buffer[5] = ui8_display_fault_code;
				}
			} else if ((m_config.enable_vlcd5) || (m_config.enable_vlcd6) ){ //#elif ENABLE_VLCD5 || ENABLE_VLCD6
				if ((ui8_auto_display_data_status)
				|| (m_configuration_variables.ui8_assist_with_error_enabled)) {
					// display data
					ui8_tx_buffer[5] = CLEAR_DISPLAY; // 0
				}
				else {
					// fault code
					ui8_tx_buffer[5] = ui8_display_fault_code;
				}
			} else { //#else // 850C
				// fault code
				ui8_tx_buffer[5] = ui8_display_fault_code;
			} // #endif
		}
		else if (ui8_display_function_code != NO_FUNCTION) {
			// function code
			if ((!ui8_menu_flag)&&(ui8_menu_index)&&
			   ((m_configuration_variables.ui8_set_parameter_enabled)||
			   (ui8_assist_level == OFF)||
			   ((ui8_startup_counter < m_config.delay_menu_on)&&(ui8_assist_level == TURBO)))) {
				// display blinking function code
				if (ui8_default_flash_state) {
					ui8_tx_buffer[5] = ui8_display_function_code;
				}
				else {
					// clear code
					ui8_tx_buffer[5] = CLEAR_DISPLAY;
				}
			}
			else {
				// display data function code
				ui8_tx_buffer[5] = ui8_display_function_code;
			}
		}
		else {
			// clear code
			ui8_tx_buffer[5] = CLEAR_DISPLAY;
		}
		
		// send to display data value or wheel speed
		if (ui8_display_data_enabled) {
			// display data
			// The maximum value displayable on the display is 99.9, and is always sent in km/h.
			// By setting mph, it is the display that converts it, so the maximum displayable value becomes 62.4 (99.9/1.6),
			// Data that can exceed this value is best always divided by 10.
			if ((ui8_display_battery_soc)||((ui8_startup_counter < m_config.delay_menu_on)&&(ui8_assist_level == TURBO))) {
				if (m_config.units_type == MILES) {
					ui16_display_data = (uint16_t) ui16_display_data_factor / ui16_battery_SOC_percentage_x10 * 10;
				} else {
				ui16_display_data = (uint16_t) ui16_display_data_factor / ui16_battery_SOC_percentage_x10;
				}
			}
			else if ((ui8_display_data_on_startup)&&(ui8_startup_counter < m_config.delay_menu_on)&&(ui8_assist_level != TURBO)&&(!ui8_menu_index)) {
			  switch (ui8_display_data_on_startup) {
				case 1:
					if (m_config.units_type == MILES) {
					ui16_display_data = (uint16_t) ui16_display_data_factor / ui16_battery_SOC_percentage_x10 * 10;
					} else {
					ui16_display_data = (uint16_t) ui16_display_data_factor / ui16_battery_SOC_percentage_x10;
					}
				  break;
				case 2:
					// battery voltage calibrated x10 for display data
					ui16_battery_voltage_calibrated_x10 = (ui16_battery_voltage_filtered_x10 * m_config.actual_battery_voltage_percent) / 100; //  ACTUAL_BATTERY_VOLTAGE_PERCENT
					ui16_display_data = (uint16_t) ui16_display_data_factor / ui16_battery_voltage_calibrated_x10;
				  break;
				default:
					ui16_display_data = 0;
				  break;
			  }
			}
			else if (ui8_torque_sensor_calibration_started) {
				ui16_display_data = (uint16_t) ui16_display_data_factor / (ui16_pedal_weight_x100 / 10);
			}
			else if (ui8_display_torque_sensor_step) {
				ui16_display_data = (uint16_t) ui16_display_data_factor / (ui8_pedal_torque_per_10_bit_ADC_step_x100 * 10);
			}
			else if (ui8_display_torque_sensor_value) {
				ui16_display_data = (uint16_t) ui16_display_data_factor / ui16_adc_torque;
			}
			else if ((ui8_menu_counter <= ui8_delay_display_function) && 
					(ui8_menu_index) && 
					((ui8_assist_level < 2)||(ui8_display_alternative_lights_configuration))) { // OFF & ECO & alternative lights configuration
			  uint8_t index_temp = (ui8_display_function_status[ui8_menu_index - 1][ui8_assist_level]);
			  switch (index_temp) {
				case 0:
					ui16_display_data = (uint16_t) ui16_display_data_factor / FUNCTION_STATUS_OFF; // 1
				  break;
				case 1:
					ui16_display_data = (uint16_t) ui16_display_data_factor / FUNCTION_STATUS_ON; //105
				  break;
				default:
				  break;
			  }
			}
			else if ((ui8_menu_counter <= ui8_delay_display_function) &&
						(ui8_menu_index) && 
						(ui8_assist_level == TURBO)) {
				ui16_display_data = (uint16_t) ui16_display_data_factor / (ui8_display_lights_configuration * 100 + DISPLAY_STATUS_OFFSET);
			}
			else if ((ui8_menu_counter <= ui8_delay_display_function)&&(ui8_menu_index)) {
				ui16_display_data = (uint16_t) ui16_display_data_factor / (ui8_display_riding_mode * 100 + DISPLAY_STATUS_OFFSET);
			}
			else {
			  switch (ui8_data_index_array[ui8_data_index]) {
				case 0:
					if (m_config.units_type == MILES) {
					ui16_display_data = (uint16_t) ui16_display_data_factor / ui16_motor_temperature_filtered_x10 * 10;
					} else {
					ui16_display_data = (uint16_t) ui16_display_data_factor / ui16_motor_temperature_filtered_x10;
					}
				  break;
				case 1:
					if (m_config.units_type == MILES) {
					ui16_display_data = (uint16_t) ui16_display_data_factor / ui16_battery_SOC_percentage_x10 * 10;
					} else {
					ui16_display_data = (uint16_t) ui16_display_data_factor / ui16_battery_SOC_percentage_x10;
					}
				  break;
				case 2:
					// battery voltage calibrated x10 for display data
					ui16_battery_voltage_calibrated_x10 = (ui16_battery_voltage_filtered_x10 * m_config.actual_battery_voltage_percent) / 100; //ACTUAL_BATTERY_VOLTAGE_PERCENT
					ui16_display_data = (uint16_t) ui16_display_data_factor / ui16_battery_voltage_calibrated_x10;
				  break;
				case 3:
					//ui16_display_data = (uint16_t) ui16_display_data_factor / ui8_battery_current_filtered_x10;
					ui16_display_data = 62;
				  break;
				case 4:
					// battery power filtered x 10 for display data
					ui16_battery_power_filtered_x10 = filter(ui16_battery_power_x10, ui16_battery_power_filtered_x10, 8);
					if (m_config.units_type == MILES) {
					ui16_display_data = (uint16_t) ui16_display_data_factor / (ui16_battery_power_filtered_x10);
					} else {
					ui16_display_data = (uint16_t) ui16_display_data_factor / (ui16_battery_power_filtered_x10 / 10);
					}
				  break;
				case 5:
					ui16_display_data = (uint16_t) ui16_display_data_factor / (ui16_adc_throttle >> 2);
				  break;
				case 6:
					ui16_display_data = (uint16_t) ui16_display_data_factor / ui16_adc_torque;
				  break;
				case 7:
					if (m_config.units_type == MILES) {
						ui16_display_data = (uint16_t) ui16_display_data_factor / (ui8_pedal_cadence_RPM * 10);
					} else {
						if (ui8_pedal_cadence_RPM > 99) {
							ui16_display_data = (uint16_t) ui16_display_data_factor / ui8_pedal_cadence_RPM;
						}
						else {
							ui16_display_data = (uint16_t) ui16_display_data_factor / (ui8_pedal_cadence_RPM * 10);
						}
					}
				  break;
				case 8:
					// human power filtered x 10 for display data
					ui16_human_power_filtered_x10 = filter(ui16_human_power_x10, ui16_human_power_filtered_x10, 8);
					ui16_display_data = (uint16_t) ui16_display_data_factor / (ui16_human_power_filtered_x10 / 10);
				  break;
				case 9:
					ui16_display_data = (uint16_t) ui16_display_data_factor / ui16_adc_pedal_torque_delta;
				  break;
				case 10:
					if (m_config.units_type == MILES) {
						ui16_display_data = (uint16_t) ui16_display_data_factor / (ui32_wh_x10);
					} else {
						ui16_display_data = (uint16_t) ui16_display_data_factor / (ui32_wh_x10 / 10);
					}
				  break;
				case 11:
					ui16_display_data = (uint16_t) ui16_display_data_factor / ui16_motor_speed_erps;
				  break;
				case 12:
					ui16_duty_cycle_percent = (uint16_t) ((ui8_g_duty_cycle * 100) / ui8_pwm_duty_cycle_max) - 1;
					ui16_display_data = (uint16_t) ui16_display_data_factor / (ui16_duty_cycle_percent * 10);
				  break;
				default:
				  break;
			  }
			}
			
			// todo, filter for 500C display
			
			// valid value
			if (ui16_display_data) {
				ui8_tx_buffer[6] = (uint8_t) (ui16_display_data & 0xFF);
				ui8_tx_buffer[7] = (uint8_t) (ui16_display_data >> 8);
			}
			else {
				ui8_tx_buffer[6] = 0x07;
				ui8_tx_buffer[7] = 0x07;
			}
		}
		else {
			// wheel speed
			if (ui16_oem_wheel_speed_time) {
				//#if ALTERNATIVE_MILES
				if (m_config.alternative_miles){
					// in VLCD6 display the km/miles conversion is not present.
					// alternative mph for VLCD6 converts the sent speed time
					// applicable to other displays type, setting km/h on diplay
					// odometer history would remain in km, only those added would be in miles.
					ui16_data_value = (ui16_oem_wheel_speed_time * 16) / 10;
					ui8_tx_buffer[6] = (uint8_t) (ui16_data_value & 0xFF);
					ui8_tx_buffer[7] = (uint8_t) (ui16_data_value >> 8);
				} else { //	#else
					// km/h or mph
					ui8_tx_buffer[6] = (uint8_t) (ui16_oem_wheel_speed_time & 0xFF);
					ui8_tx_buffer[7] = (uint8_t) (ui16_oem_wheel_speed_time >> 8);
				} //#endif
			}
			else {
				ui8_tx_buffer[6] = 0x07;
				ui8_tx_buffer[7] = 0x07;
			}
		}
				
		// set working flag
		//#if ENABLE_DISPLAY_ALWAYS_ON
		if (m_config.enable_display_always_on){
			// set working flag used to hold display always on
			ui8_working_status |= 0x04;
		} //#endif
		
		//#if ENABLE_DISPLAY_WORKING_FLAG
		if (m_config.enable_display_working_flag) {
			// wheel turning
			if (ui16_oem_wheel_speed_time) {
				// bit7 = 1 (wheel turning)
				ui8_working_status |= 0x80;
			}
			else {
				// bit7 = 0 (wheel not turning)
					ui8_working_status &= 0x7F;
			}
			// motor working
			if (ui8_g_duty_cycle > 10) {
				// bit6 = 1 (motor working)
				ui8_working_status |= 0x40;
			}
			else {
				// bit6 = 0 (motor not working)
				ui8_working_status &= 0xBF;
			}
			// motor working or wheel turning?
			if (ui8_working_status & 0xC0)
			{
				// set working flag used by display
				ui8_working_status |= 0x04;
			}
			else {
				// clear working flag used by display
				ui8_working_status &= 0xFB;
			}
		} //#endif

		// working status
		ui8_tx_buffer[2] = (ui8_working_status & 0x1F);
		
		// clear motor working, wheel turning and working flags
		ui8_working_status &= 0x3B;	
		
		// prepare check code of the package
		ui8_tx_check_code = 0x00;
		for(ui8_i = 0; ui8_i < TX_CHECK_CODE; ui8_i++)
		{
			ui8_tx_check_code += ui8_tx_buffer[ui8_i];
		}
		ui8_tx_buffer[TX_CHECK_CODE] = ui8_tx_check_code;
		// send the buffer on uart
		if ((30 - XMC_USIC_CH_TXFIFO_GetLevel(CYBSP_DEBUG_UART_HW) ) >= 10){ // check if there is enough free space in Txfifo
			for(uint8_t i = 0; i < 9; i++)  {
				XMC_USIC_CH_TXFIFO_PutData(CYBSP_DEBUG_UART_HW, (uint16_t) ui8_tx_buffer[i]);
				//XMC_UART_CH_Transmit(CYBSP_DEBUG_UART_HW , ui8_tx_buffer[i]);
			}
		}	 
	}
}
#endif
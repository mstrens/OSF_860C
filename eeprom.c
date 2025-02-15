#include "eeprom.h"

#include "main.h"
#include "config.h"
#include "eeprom.h"
#include "ebike_app.h"

#ifdef USE_TSD28_VLCD5_VERSION

#define EEPROM_BYTES_STORED                             20
#define EEPROM_BYTES_STORED_OEM_DISPLAY					11
#define EEPROM_BYTES_INIT_OEM_DISPLAY					EEPROM_BYTES_STORED - EEPROM_BYTES_STORED_OEM_DISPLAY

extern struct_configuration_variables m_configuration_variables ; 

/*
static const uint8_t ui8_default_array[EEPROM_BYTES_STORED] = 
{
  DEFAULT_VALUE_KEY,							// 0 + EEPROM_BASE_ADDRESS (Array index)
  BATTERY_CURRENT_MAX,							// 1 + EEPROM_BASE_ADDRESS    It was 13A for TSDZ2, reduced to 5 for test of TSDZ8
  BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0,			// 2 + EEPROM_BASE_ADDRESS
  BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1,			// 3 + EEPROM_BASE_ADDRESS
  WHEEL_PERIMETER_0,							// 4 + EEPROM_BASE_ADDRESS
  WHEEL_PERIMETER_1,							// 5 + EEPROM_BASE_ADDRESS
  WHEEL_MAX_SPEED,								// 6 + EEPROM_BASE_ADDRESS
  MOTOR_TYPE,									// 7 + EEPROM_BASE_ADDRESS
  AVAIABLE_FOR_FUTURE_USE,						// 8 + EEPROM_BASE_ADDRESS
  // for oem display
  MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION,		// 9 + EEPROM_BASE_ADDRESS 0 or 1 ; 1 allows to get assist just pressing on the pedal
  ASSISTANCE_WITH_ERROR_ENABLED,				// 10 + EEPROM_BASE_ADDRESS
  BATTERY_SOC,									// 11 + EEPROM_BASE_ADDRESS    State of charge
  ENABLE_SET_PARAMETER_ON_STARTUP,				// 12 + EEPROM_BASE_ADDRESS
  ENABLE_STREET_MODE_ON_STARTUP,				// 13 + EEPROM_BASE_ADDRESS
  RIDING_MODE_ON_STARTUP,						// 14 + EEPROM_BASE_ADDRESS   ; see list of code here below    
  LIGHTS_CONFIGURATION_ON_STARTUP,				// 15 + EEPROM_BASE_ADDRESS
  STARTUP_BOOST_ON_STARTUP,						// 16 + EEPROM_BASE_ADDRESS
  ENABLE_AUTO_DATA_DISPLAY,						// 17 + EEPROM_BASE_ADDRESS
  SOC_PERCENT_CALC,								// 18 + EEPROM_BASE_ADDRESS
  TORQUE_SENSOR_ADV_ON_STARTUP					// 19 + EEPROM_BASE_ADDRESS
};
*/

// riding modes for documentation (define is done in common.h)
//#define OFF_MODE                                  0 // not used
//#define POWER_ASSIST_MODE                         1
//#define TORQUE_ASSIST_MODE                        2
//#define CADENCE_ASSIST_MODE                       3
//#define eMTB_ASSIST_MODE                          4
//#define HYBRID_ASSIST_MODE						            5
//#define CRUISE_MODE                               6
//#define WALK_ASSIST_MODE                          7
//#define TORQUE_SENSOR_CALIBRATION_MODE            8								   

/*
void m_configuration_init(void){
 // added by mstrens in order to fill m-configuration_variables with default before having eeprom functions
    //The default values are in an array 
    uint16_t ui16_temp ;
    uint8_t ui8_temp;
    m_configuration_variables.ui8_battery_current_max = ui8_default_array[1];
	  
    ui16_temp = ui8_default_array[2]; //FLASH_ReadByte(ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0);
    ui8_temp = ui8_default_array[3];  //FLASH_ReadByte(ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1);
    ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
    m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 = ui16_temp;
    
    ui16_temp = ui8_default_array[4]; //FLASH_ReadByte(ADDRESS_WHEEL_PERIMETER_0);
    ui8_temp = ui8_default_array[5];  //FLASH_ReadByte(ADDRESS_WHEEL_PERIMETER_1);
    ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
    m_configuration_variables.ui16_wheel_perimeter = ui16_temp;

    m_configuration_variables.ui8_wheel_speed_max = ui8_default_array[6]; //FLASH_ReadByte(ADDRESS_WHEEL_SPEED_MAX);

    m_configuration_variables.ui8_motor_type = ui8_default_array[7]; //FLASH_ReadByte(ADDRESS_MOTOR_TYPE);
    
    m_configuration_variables.ui8_avaiable_for_future_use = ui8_default_array[8]; //FLASH_ReadByte(ADDRESS_AVAIABLE_FOR_FUTURE_USE);
    // for oem display
    m_configuration_variables.ui8_assist_without_pedal_rotation_enabled = ui8_default_array[9]; //FLASH_ReadByte(ADDRESS_MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION);
    
    m_configuration_variables.ui8_assist_with_error_enabled = ui8_default_array[10]; //FLASH_ReadByte(ADDRESS_MOTOR_ASSISTANCE_WITH_ERROR_ENABLED);
    m_configuration_variables.ui8_battery_SOC_percentage_8b = ui8_default_array[11]; //FLASH_ReadByte(ADDRESS_BATTERY_SOC);
    m_configuration_variables.ui8_set_parameter_enabled = ui8_default_array[12]; //FLASH_ReadByte(ADDRESS_SET_PARAMETER_ON_STARTUP);
    m_configuration_variables.ui8_street_mode_enabled = ui8_default_array[13]; //FLASH_ReadByte(ADDRESS_STREET_MODE_ON_STARTUP);
    m_configuration_variables.ui8_riding_mode = ui8_default_array[14]; //FLASH_ReadByte(ADDRESS_RIDING_MODE_ON_STARTUP);
    m_configuration_variables.ui8_lights_configuration = ui8_default_array[15]; //FLASH_ReadByte(ADDRESS_LIGHTS_CONFIGURATION_ON_STARTUP);
    m_configuration_variables.ui8_startup_boost_enabled = ui8_default_array[16]; //FLASH_ReadByte(ADDRESS_STARTUP_BOOST_ON_STARTUP);
    m_configuration_variables.ui8_auto_display_data_enabled = ui8_default_array[17]; //FLASH_ReadByte(ADDRESS_ENABLE_AUTO_DATA_DISPLAY);
    m_configuration_variables.ui8_soc_percent_calculation = ui8_default_array[18]; //FLASH_ReadByte(ADDRESS_SOC_PERCENT_CALC);
    
    m_configuration_variables.ui8_torque_sensor_adv_enabled = ui8_default_array[19]; //FLASH_ReadByte(ADDRESS_TORQUE_SENSOR_ADV_ON_STARTUP);
}
*/

void m_configuration_init(void){
 // added by mstrens in order to fill m-configuration_variables with the variables from m_config before having eeprom functions
 // This has to be called AFTER that m_config fas been filled   
    m_configuration_variables.ui8_battery_current_max = m_config.battery_current_max;
    m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 = m_config.battery_low_voltage_cut_off * 10;
    m_configuration_variables.ui16_wheel_perimeter = m_config.wheel_perimeter;
    m_configuration_variables.ui8_wheel_speed_max = m_config.wheel_max_speed; 
    m_configuration_variables.ui8_motor_type = m_config.motor_type;     
    m_configuration_variables.ui8_avaiable_for_future_use = 0; 
    // for oem display
    m_configuration_variables.ui8_assist_without_pedal_rotation_enabled = m_config.motor_assistance_without_pedal_rotation;
    m_configuration_variables.ui8_assist_with_error_enabled = ASSISTANCE_WITH_ERROR_ENABLED; // is always disabled; must be enabled with the display
    m_configuration_variables.ui8_battery_SOC_percentage_8b = 0; // mstrens : Should be calculated and strored in eeprom (not defined in m_config) 
    m_configuration_variables.ui8_set_parameter_enabled = m_config.enable_set_parameter_on_startup;
    m_configuration_variables.ui8_street_mode_enabled = m_config.enable_street_mode_on_startup;
    m_configuration_variables.ui8_riding_mode = m_config.riding_mode_on_startup;
    m_configuration_variables.ui8_lights_configuration = m_config.lights_configuration_on_startup;
    m_configuration_variables.ui8_startup_boost_enabled = m_config.startup_boost_on_startup;
    m_configuration_variables.ui8_auto_display_data_enabled = m_config.enable_auto_data_display;
    m_configuration_variables.ui8_soc_percent_calculation = m_config.soc_percent_calc;
    m_configuration_variables.ui8_torque_sensor_adv_enabled = m_config.torque_sensor_adv_on_startup;
}

#endif
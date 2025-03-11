/*
 * TongSheng TSDZ2 motor controller firmware
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */
#include "main.h"
#include "cybsp.h"
#include "cy_utils.h"
#include "motor.h"
#include "ebike_app.h"
#include "common.h"

#include "cy_retarget_io.h"
//#include "cy_utils.h"

// **************  to test slow rotation without using the hall sensor and so discover pattern sequence
// just to test rotation at a low speed and low power to verify the the hall sequence is OK
#define SPEED_COUNTER_MAX 19000 /360  // one electrical rotation per sec ; so 1 mecanical rotation takes 4 sec ; so 15 rpm
#define DUTY_CYCLE_TEST 30// 256 = 100% ; 40 gives a current = 1A from ADC on pin 2.8 with a 12V battery
#define ANGLE_INIT 0
// end of those test parameters


// pattern for hall sensor is 1,3,2,6,4, 5
// with full use of posif possibilities, this table should be read with expected pattern and so upload in shadow register for the next expected
// when current pattern is 1 and expected = 3 , the sadow register should be prepare for the next transition with current = 3 and exp=6 
// in current version, it is used only to detect if a transition is valid in irq0
const uint8_t expected_pattern_table[8] = {
    3, // 0 should not happen
    3, // after 1 => 3 
    6, // after 2 => 6
    2, // after 3 => 2
    5, // after 4 => 5
    1, // after 5 => 1
    4, // after 6 => 4
    1 // 7 should not happen 
};


// copied from tsdz2
#define SVM_TABLE_LEN   256
// for tsdz8 using a timer counting up to 1680 (instead of 420); values have been calculated in an XLS sheet (on google drive)
// svm table 19 Khz
static const uint16_t ui16_svm_table[SVM_TABLE_LEN] = {
1566,1576,1586,1595,1604,1612,1620,1627,1634,1640,1646,1651,1656,1661,1665,1668,1671,1674,1676,
1677,1678,1678,1678,1678,1677,1675,1673,1670,1667,1664,1659,1655,1650,1644,1638,1632,1625,1617,
1609,1601,1592,1583,1573,1556,1524,1493,1461,1428,1396,1363,1329,1295,1261,1227,1193,1158,1123,
1088,1053,1018,982,947,911,876,840,804,769,733,698,662,627,592,557,522,487,453,419,385,351,317,
284,252,219,187,156,124,107,97,88,79,71,63,55,48,42,36,30,25,21,16,13,10,7,5,3,2,2,2,2,3,4,6,9,
12,15,19,24,29,34,40,46,53,60,68,76,85,94,104,114,104,94,85,76,68,60,53,46,40,34,29,24,19,15,12,
9,6,4,3,2,2,2,2,3,5,7,10,13,16,21,25,30,36,42,48,55,63,71,79,88,97,107,124,156,187,219,252,284,
317,351,385,419,453,487,522,557,592,627,662,698,733,769,804,840,876,911,947,982,1018,1053,1088,
1123,1158,1193,1227,1261,1295,1329,1363,1396,1428,1461,1493,1524,1556,1573,1583,1592,1601,1609,
1617,1625,1632,1638,1644,1650,1655,1659,1664,1667,1670,1673,1675,1677,1678,1678,1678,1678,1677,
1676,1674,1671,1668,1665,1661,1656,1651,1646,1640,1634,1627,1620,1612,1604,1595,1586,1576};


// motor variables
uint8_t ui8_hall_360_ref_valid = 0; // fill with a hall pattern to check sequence is correct
uint8_t ui8_motor_commutation_type = BLOCK_COMMUTATION;
static uint8_t ui8_motor_phase_absolute_angle;
volatile uint16_t ui16_hall_counter_total = 0xffff; // number of tim3 ticks between 2 rotations// inTSDZ2 it was a u16
volatile uint8_t ui8_hall_sensors_state = 0;
static uint16_t ui16_hall_counter_total_previous = 0;  // used to check if erps is stable
// power variables
volatile uint8_t ui8_controller_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT; // 194
volatile uint8_t ui8_controller_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT; // 73
volatile uint16_t ui16_adc_voltage_cut_off = 300*100/BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000; // 30Volt default value =  300*100/87 in TSDZ2
volatile uint8_t ui8_adc_battery_current_filtered = 0;
volatile uint32_t ui32_adc_battery_current_filtered_15b = 0; // value in 12 +2 +1 = 15 bits (ADC + IIR + average)
volatile uint8_t ui8_controller_adc_battery_current_target = 0;
volatile uint8_t ui8_g_duty_cycle = 0;
volatile uint8_t ui8_controller_duty_cycle_target = 0;

// Field Weakening Hall offset (added during interpolation)
volatile uint8_t ui8_fw_hall_counter_offset = 0;
volatile uint8_t ui8_fw_hall_counter_offset_max = 0;
volatile uint8_t ui8_field_weakening_enabled = 0;

// Duty cycle ramp up
static uint8_t ui8_counter_duty_cycle_ramp_up = 0;
static uint8_t ui8_counter_duty_cycle_ramp_down = 0;

// FOC angle
static uint8_t ui8_foc_angle_accumulated;
static uint8_t ui8_foc_flag;
volatile uint8_t ui8_g_foc_angle = 0;
uint8_t ui8_foc_angle_multiplicator = FOC_ANGLE_MULTIPLIER; //39 for 48V motor
static uint8_t ui8_adc_foc_angle_current = 0;

// battery current variables
uint16_t ui16_adc_battery_current_acc_X4 = 0;
uint16_t ui16_adc_battery_current_filtered_X4 = 0;
volatile uint16_t ui16_adc_motor_phase_current = 0; // to check for ui16 or ui8 mstrens: it was uint8 in original code

// ADC Values
volatile uint16_t ui16_adc_voltage;
volatile uint16_t ui16_adc_torque;
volatile uint16_t ui16_adc_throttle;

// brakes
volatile uint8_t ui8_brake_state = 0;

// cadence sensor
#define NO_PAS_REF 5
volatile uint16_t ui16_cadence_sensor_ticks = 0;
static uint16_t ui16_cadence_sensor_ticks_counter_min = CADENCE_SENSOR_CALC_COUNTER_MIN; // initialiszed at 4270 , then varies with wheelSpeed
static uint8_t ui8_pas_state_old = 4;
static uint16_t ui16_cadence_calc_counter, ui16_cadence_stop_counter;
static uint8_t ui8_cadence_calc_ref_state = NO_PAS_REF;
const static uint8_t ui8_pas_old_valid_state[4] = { 0x01, 0x03, 0x00, 0x02 };

// wheel speed sensor
volatile uint16_t ui16_wheel_speed_sensor_ticks = 0;
volatile uint16_t ui16_wheel_speed_sensor_ticks_counter_min = 0;
volatile uint32_t ui32_wheel_speed_sensor_ticks_total = 0;


// battery soc
volatile uint8_t ui8_battery_SOC_saved_flag = 0;
volatile uint8_t ui8_battery_SOC_reset_flag = 0;

// Hall sensor state
uint8_t current_hall_pattern = 0;
uint8_t previous_hall_pattern = 7; // Invalid value, force execution of Hall code at the first run

// Hall counter value of last Hall transition 
uint16_t previous_360_ref_ticks ; 

// ----------   end of copy from tsdz2 -------------------------

uint8_t ui8_temp;
uint16_t ui16_temp;

volatile uint16_t ui16_a = PWM_COUNTER_MAX / 2 ;//   840 in tsdz8   // 4*210 from tsdz2
volatile uint16_t ui16_b = PWM_COUNTER_MAX / 2 ;//   840 in tsdz8   // 4*210 from tsdz2
volatile uint16_t ui16_c = PWM_COUNTER_MAX / 2 ;//   840 in tsdz8   // 4*210 from tsdz2

uint8_t hall_reference_angle  ; // This value is initialised in ebike_app.c with DEFAULT_HALL_REFERENCE_ANGLE and m_config.global_offset_angle 

// to debug time spent in irq0 and irq1
//volatile uint16_t debug_time_ccu8_irq0 = 0;
//volatile uint16_t debug_time_ccu8_irq1 = 0;
//volatile uint16_t debug_time_ccu8_irq1b = 0;
//volatile uint16_t debug_time_ccu8_irq1c = 0;
//volatile uint16_t debug_time_ccu8_irq1d = 0;
//volatile uint16_t debug_time_ccu8_irq1e = 0;
uint16_t hall_ref_angles_counter = 0;

extern uint8_t ui8_pwm_duty_cycle_max;


/*
// those values result from running the test to find hall sensor positions
const uint8_t ui8_hall_ref_angles[8] = { // Sequence is 1, 3, 2 , 6, 4, 6; so angle are 39, 86, 127, 167, 216, 0 (256=360°)
        0,                     // error ; index must be between 1 and 6
        20 , //   test gives angle = 39   for hall pattern 1
        113, //   test gives angle = 127  for hall pattern 2  
        67, //    test gives angle = 86   for hall pattern 3
        197, //   test gives angle = 216  for hall pattern 4 
        242 , //    test gives angle = 0    for hall pattern 5
        147, //   test gives angle = 167  for hall pattern 6  
        0                     // error ; index must be between 1 and 6
};
*/
// those values result from running the test to find hall sensor positions with a 36V battery 
// and with a duty cycle of 50 for 5 msec and 30 for 10 msec
// it consumes about 2.5A
// first test with find best offset : 14, 117, 73, 203, 246, 141 with 36V and dutycycle = 150
// test with one hall gives +8, +3 , -4 , -5 , -5 , 7
// So new values for a second run is 22, 120, 69 , 198, 241, 148 
// second test for one hall patterns gives 2,-3,-2,2,0,2
// So new values becomes 24 , 117, 67, 200, 241, 150
// Third test strating with Hall pattern 3 give X, -3 , 0 ,
// so change patter 2 to 114
// then test pattern 6 give +2
/*
const uint8_t ui8_hall_ref_angles[8] = { // Sequence is 1, 3, 2 , 6, 4, 5; so angle are in theory e.g. 39, 86, 127, 167, 216, 0 (256=360°)
        0,                     // error ; index must be between 1 and 6
        24 , //   test gives angle = 39   for hall pattern 1
        114, //   test gives angle = 127  for hall pattern 2  
        67, //    test gives angle = 86   for hall pattern 3
        200, //   test gives angle = 216  for hall pattern 4 
        241 , //    test gives angle = 0    for hall pattern 5
        154, //   test gives angle = 167  for hall pattern 6  
        0                     // error ; index must be between 1 and 6
};
*/
uint8_t ui8_hall_ref_angles[8] = { // Sequence is 1, 3, 2 , 6, 4, 5; so angle are in theory e.g. 39, 86, 127, 167, 216, 0 (256=360°)
        0,                     // error ; index must be between 1 and 6
        24 , //   test gives angle = 39   for hall pattern 1
        106, //   test gives angle = 127  for hall pattern 2  
        65, //    test gives angle = 86   for hall pattern 3
        194, //   test gives angle = 216  for hall pattern 4 
        234 , //    test gives angle = 0    for hall pattern 5
        151, //   test gives angle = 167  for hall pattern 6  
        0                     // error ; index must be between 1 and 6
};


volatile uint8_t ui8_best_ref_angles[8] ; // this table is prefilled in main.c at start up


uint32_t best_ref_angles_X16bits[8] ;  // same as ui8_best_ref_angles but with 8 more bits for better filtering
uint32_t ui32_angle_per_tick_X16shift; // 

/*
int8_t i8_hall_add_angles[8] = { // Sequence is 1, 3, 2 , 6, 4, 5; so angle are 39, 86, 127, 167, 216, 0 (256=360°)
        0,                     // error ; index must be between 1 and 6
        0 , //    for hall pattern 1
        0, //     for hall pattern 2  
        0, //    for hall pattern 3
        0, //     for hall pattern 4 
        0 , //      for hall pattern 5
        0, //     for hall pattern 6  
        0                     // error ; index must be between 1 and 6
};
*/

// Hall offset for current Hall state; This offset is added in the interpolation process (so based also on the erps)
// the value is in ticks (1 ticks = 4 usec); we need  55usec/4 : 55 = 39 + 16 (39 = 3/4 of 55usec = delay between measuring and PWM change); 16=delay hall sensor
static uint8_t ui8_hall_counter_offset = 14; 


// ************************************** begin of IRQ *************************
// *************** irq 0 of ccu8
__RAM_FUNC void CCU80_0_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting UP (= 1/4 of 19mhz cycles with 1680 ticks at 64mHz and centered aligned)
// here we just calculate the new compare values used for the 3 slices (0,1,2) that generates the 3 PWM

    // get the current ticks
    uint16_t current_speed_timer_ticks = (uint16_t) (XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW) );
    // get the capture register = last changed pattern = current pattern
    uint16_t last_hall_pattern_change_ticks = (uint16_t) XMC_CCU4_SLICE_GetCaptureRegisterValue(HALL_SPEED_TIMER_HW , 1);
    // get the current hall pattern
    current_hall_pattern = XMC_POSIF_HSC_GetLastSampledPattern(HALL_POSIF_HW) ;
    // elapsed time between now and last pattern
    uint16_t enlapsed_time =  current_speed_timer_ticks - last_hall_pattern_change_ticks ; // ticks between now and last change
   
    // to debug
    //uint16_t start_ticks = current_speed_timer_ticks; // save to calculate enlased time inside the irq // just for debug could be removed
		
    // when pattern change
    if ( current_hall_pattern != previous_hall_pattern) {
        if (current_hall_pattern != expected_pattern_table[previous_hall_pattern]){ // new pattern is not the expected one
            ui8_motor_commutation_type = BLOCK_COMMUTATION; // 0x00
            ui8_hall_360_ref_valid = 0;  // reset the indicator saying no error for a 360° electric rotation 
            ui32_angle_per_tick_X16shift = 0; // 0 means unvalid value
            //hall_pattern_error_counter++; // for debuging
        } else {   // valid transition
            if (current_hall_pattern ==  0x01) {  // rotor at 210°
                if (ui8_hall_360_ref_valid) { // check that we have a full rotation without pattern sequence error
                    ui16_hall_counter_total = last_hall_pattern_change_ticks - previous_360_ref_ticks; // save the total number of tick for one electric rotation
                    ui32_angle_per_tick_X16shift = ( 1 << 24) / ui16_hall_counter_total; // new value for interpolation and updating table with reference angle
                    ui8_motor_commutation_type = SINEWAVE_INTERPOLATION_60_DEGREES; // 0x80 ; it says that we can interpolate because speed is known
                }
                ui8_hall_360_ref_valid = 0x01;
                previous_360_ref_ticks = last_hall_pattern_change_ticks ;    
            } else if (current_hall_pattern ==  0x03) {  // rotor at 150°){
                // update ui8_g_foc_angle once every ERPS (used for g_foc_angle calculation) ;
                // I do not know why this is done when hall pattern = 0X03 and not with 0X01 to avoid a test
                            ui8_foc_flag = 1;
            }
            #define MIN_ANGLE_PER_TICK_X16SHIFT ((1 << 24) / 5000u ) // update of hall position is done only when erps > 50 = about rpm > 750)
            // 720 rpm = 12 rps = 12*4 erps = 48 erps => 250000 tick/sec /50 = 5000 ticks per electric rotation
            if ((current_hall_pattern >1) // no need to calculate for hall pattern == 1
                    && (ui32_angle_per_tick_X16shift > MIN_ANGLE_PER_TICK_X16SHIFT) // speed must be high (no enough inertia)
                    && (ui32_angle_per_tick_X16shift)  // angle per tick must be calculated
                    && (ui16_hall_counter_total == ui16_hall_counter_total_previous)){  // speed must be stable between 2 electric rotation
                
                uint16_t ui16_measured_angle_X16bits = ( (uint32_t) ((uint16_t) (last_hall_pattern_change_ticks - previous_360_ref_ticks))
                         * ui32_angle_per_tick_X16shift) >> 8;  // here we get an angle * 256 (to get better rounding for filtering)
                ui16_measured_angle_X16bits += ((uint16_t) ui8_hall_ref_angles[1]) << 8 ;   // add the angle for hall pattern 1 *256 (with overrunning)
                //ui8_best_ref_angles[current_hall_pattern] = filtering >> 8; // set angle back in 8bits
                uint8_t ui8_measured_angle = ui16_measured_angle_X16bits >> 8;
                uint8_t delta =  ui8_measured_angle - ui8_hall_ref_angles[current_hall_pattern];
                if ((delta < 5 ) || (delta > 251 )) { // when there is no big difference 
                    // apply filter on values X16bits
                    uint32_t filtering = best_ref_angles_X16bits[current_hall_pattern];
                    #define FILTER_HALL_POSITIONS 3
                    filtering = ((filtering << FILTER_HALL_POSITIONS) - filtering + (uint32_t) ui16_measured_angle_X16bits) >> FILTER_HALL_POSITIONS ;
                    best_ref_angles_X16bits[current_hall_pattern] = filtering ; // apply new filtered value
                    ui8_best_ref_angles[current_hall_pattern] = (filtering + 128 )>> 8; // +128 for rounding
                    // update the table with reference angles
                    //ui8_hall_ref_angles[current_hall_pattern] = ui8_best_ref_angles[current_hall_pattern] ;
                    hall_ref_angles_counter++;  // just to debug to see if table is updated at regular intervals
                }
            } 
            if (current_hall_pattern == 1 ){
                ui16_hall_counter_total_previous = ui16_hall_counter_total; // save previous counter (to check if erps is stable)
            }
        }
        previous_hall_pattern = current_hall_pattern; // saved to detect future change and check for valid transition
        // set rotor angle based on hall patern
        ui8_motor_phase_absolute_angle = ui8_best_ref_angles[current_hall_pattern]; // use best ref instead of hall_ref_angles[]
    } else { // no hall patern change
        // Verify if rotor stopped (< 10 ERPS)
        if (enlapsed_time > (HALL_COUNTER_FREQ/MOTOR_ROTOR_INTERPOLATION_MIN_ERPS/6)) { //  for TSDZ2: 250000/10 /6 = 4166 ; for TSDZ8 = 8332
            ui8_motor_commutation_type = BLOCK_COMMUTATION; // 0
            ui8_g_foc_angle = 0;
            ui8_hall_360_ref_valid = 0;
            ui32_angle_per_tick_X16shift = 0; // 0 means unvalid value
            ui16_hall_counter_total = 0xffff;
        }
    }
    /****************************************************************************/
    // - calculate interpolation angle and sine wave table index when speed is known
    uint8_t ui8_interpolation_angle = 0; // interpolation angle
    uint32_t compensated_enlapsed_time = 0; 
    if (ui8_motor_commutation_type != BLOCK_COMMUTATION) {  // as long as hall patern are OK and motor is running 
        // ---------
        // uint8_t ui16_temp = ((uint32_t)ui16_a << 8) / ui16_hall_counter_total; // ui16_hall_counter_total is the number of ticks for a full cycle
        // temp is here related to 256 (because 256 represent 360°); So 36° is here 25
        compensated_enlapsed_time = enlapsed_time + ui8_fw_hall_counter_offset + ui8_hall_counter_offset;
        // convert time tick to angle (256 = 360°)
           //ui8_interpolation_angle = (((uint32_t) compensated_enlapsed_time) << 8) /  ui16_hall_counter_total; // <<8 = 256 = 360 electric angle
        // convert time tick to angle (256 = 360°) using the already calculated angle per tick (avoid a division)
        ui8_interpolation_angle = (((uint32_t) compensated_enlapsed_time) *  ui32_angle_per_tick_X16shift) >> 16 ; 
        //if (ui8_interpolation_angle > 90){  // added by mstrens because interpolation should not exceed 60°
        //    ui8_interpolation_angle = 21; // 21 is about 30° so mid position between 2 hall pattern changes
        //}
    }
    // ------------ Calculate the rotor angle and use it as index in the table----------------- 
    // hall_reference_angle is the sum of the DEFAULT_HALL_REFERENCE_ANGLE and the fine tune with m_config.global_offset_angle
    uint8_t ui8_svm_table_index = ui8_interpolation_angle + ui8_motor_phase_absolute_angle + ui8_g_foc_angle + hall_reference_angle ;
    //                            + i8_hall_add_angles[current_hall_pattern]; // add the value found during each hall pattern optimisation
    
    // Phase A is advanced 240 degrees over phase B
    ui16_temp = ui16_svm_table[(uint8_t) (ui8_svm_table_index + 171)]; // 171 = 240 deg when 360° is coded as 256
    if (ui16_temp > MIDDLE_SVM_TABLE) { // 214 at 19 khz
        ui16_a = MIDDLE_SVM_TABLE + (((ui16_temp - MIDDLE_SVM_TABLE) * (uint16_t) ui8_g_duty_cycle)>>8); // >>8 because duty_cycle 100% is 256
    } else {
        ui16_a = MIDDLE_SVM_TABLE - (((MIDDLE_SVM_TABLE - ui16_temp) * (uint16_t) ui8_g_duty_cycle)>>8);
    }
    // phase B as reference phase
    ui16_temp = ui16_svm_table[ui8_svm_table_index] ;
    if (ui16_temp > MIDDLE_SVM_TABLE) {
        ui16_b = MIDDLE_SVM_TABLE + (((ui16_temp - MIDDLE_SVM_TABLE) * (uint16_t) ui8_g_duty_cycle)>>8);
    } else {
        ui16_b = MIDDLE_SVM_TABLE - (((MIDDLE_SVM_TABLE - ui16_temp) * (uint16_t) ui8_g_duty_cycle)>>8);
    }
    // phase C is advanced 120 degrees over phase B
    ui16_temp = ui16_svm_table[(uint8_t) (ui8_svm_table_index + 85 )] ; // 85 = 120 deg
    if (ui16_temp > MIDDLE_SVM_TABLE) {
        ui16_c = MIDDLE_SVM_TABLE + (((ui16_temp - MIDDLE_SVM_TABLE) * (uint16_t) ui8_g_duty_cycle)>>8);
    } else {
        ui16_c = MIDDLE_SVM_TABLE - (((MIDDLE_SVM_TABLE - ui16_temp) * (uint16_t) ui8_g_duty_cycle)>>8);
    }
    // get the voltage ; done in irq0 because used in irq1 and irq0 takes less time
    ui16_adc_voltage  = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 4 ) & 0x0FFF) >> 2; // battery gr1 ch6 result 4
        
    //uint16_t temp  = XMC_CCU4_SLICE_GetTimerValue(RUNNING_250KH_TIMER_HW) ;
    //temp = temp - start_ticks;
    //if (temp > debug_time_ccu8_irq0) debug_time_ccu8_irq0 = temp; // store the max enlapsed time in the irq
} // end of CCU80_0_IRQHandler


// ************* irq handler 
__RAM_FUNC void CCU80_1_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting DOWN (= 1/4 of 19mhz cycles)   __RAM_FUNC 
    // fill the PWM parameters with the values calculated in the other CCU8 interrupt
    //XMC_CCU8_SLICE_SetTimerCompareMatch(PHASE_U_TIMER_HW, XMC_CCU8_SLICE_COMPARE_CHANNEL_1 , ui16_a);
    //XMC_CCU8_SLICE_SetTimerCompareMatch(PHASE_V_TIMER_HW, XMC_CCU8_SLICE_COMPARE_CHANNEL_1 , ui16_b);
    //XMC_CCU8_SLICE_SetTimerCompareMatch(PHASE_W_TIMER_HW, XMC_CCU8_SLICE_COMPARE_CHANNEL_1 , ui16_c);
    PHASE_U_TIMER_HW->CR1S = (uint32_t) ui16_a;
    PHASE_V_TIMER_HW->CR1S = (uint32_t) ui16_b;
    PHASE_W_TIMER_HW->CR1S = (uint32_t) ui16_c;
    /* Enable shadow transfer for slice 0,1,2 for CCU80 Kernel */
	//XMC_CCU8_EnableShadowTransfer(ccu8_0_HW, ((uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |
	//                                            (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |
	//                                            (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_2 ));
    ccu8_0_HW->GCSS = ((uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |
	                                            (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |
	                                            (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_2 );
    // to debug
    //uint16_t temp1b  =  XMC_CCU4_SLICE_GetTimerValue(RUNNING_250KH_TIMER_HW);
    //uint16_t temp1b  = (uint16_t) RUNNING_250KH_TIMER_HW->TIMER;
    //temp1b = temp1b - start_ticks;
    //if (temp1b > debug_time_ccu8_irq1b) debug_time_ccu8_irq1b = temp1b; // store the max enlapsed time in the irq
    
    /****************************************************************************/
        // Read all ADC values (right aligned values).
       // adc values are reduced to 10 bits instead of 12 bits to use the same resolution as tsdz2
       // note: per vadc group, the result number is the same as the pin number (except for group 1 current sensor)
        // next line has been moved in irq 0 to save time here
        //ui16_adc_voltage  = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 4 ) & 0xFFF) >> 2; // battery gr1 ch6 result 4   in bg
        // next line has been moved to ebike_app.c to save time here
        //ui16_adc_torque   = (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 2 ) & 0xFFF) >> 2; // torque gr0 ch7 result 2 in bg p2.2
        // next line has been moved to ebike_app.c to save time in this irq
        //ui16_adc_throttle = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 5 ) & 0xFFF) >> 2; // throttle gr1 ch7 result 5  in bg  p2.5
        
        //the resistance/gain in TSDZ8 is 4X smaller than in TSDZ2; still ADC is 12 bits instead of 10; so ADC 12bits TSDZ8 = ADC 10 bits TSDZ2
        // in TSDZ2, we used only the 8 lowest bits of adc; 1 adc step = 0,16A
        // In tsdz8, the resistance is (I expect) 0.003 Ohm ; So 1A => 0,003V => 0,03V (gain aop is 10)*4096/5Vcc = 24,576 steps
        //      SO 1 adc step 12bits = 1/24,576 = 0,040A
        // For 10 A, TSDZ2 should gives 10/0,16 = 62 steps
        // For 10 A, TSDZ8 shoud give 10*24,576 steps = 246 steps
        // to convert TSDZ8 steps 12bits  in the same units as TSDZ2, we shoud take ADC12bits *62/245,76 = 0,25 and divide by 4 (or >>2)
        // current is available in gr0 result 15 in queue 0 p2.8 and/or in gr1 result 152 (p2.8)
        // both results use IIR filters and so results are in 14 bits instead of 12 bits
        uint32_t ui32_temp_current_15b = ((XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 15 ) & 0xFFFF) +
                                    (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 15 ) & 0xFFFF)) ;  // So here result is in 15 bits (averaging
        ui32_adc_battery_current_filtered_15b =  ((( ui32_adc_battery_current_filtered_15b << 2) - ui32_adc_battery_current_filtered_15b) +
             (ui32_temp_current_15b)) >> 2;
        ui8_adc_battery_current_filtered  = ui32_adc_battery_current_filtered_15b >> 5 ; // from 15 bits to 10 bits like TSDZ2 
   // to debug
    //uint16_t temp1c  =  XMC_CCU4_SLICE_GetTimerValue(RUNNING_250KH_TIMER_HW);
    //uint16_t temp1c  = (uint16_t) RUNNING_250KH_TIMER_HW->TIMER;
    //temp1c = temp1c - start_ticks;
    //if (temp1c > debug_time_ccu8_irq1c) debug_time_ccu8_irq1c = temp1c; // store the max enlapsed time in the irq
        
        // update foc_angle once per electric rotation (based on fog_flag
        // foc_angle is added to the position given by hall sensor + interpolation )
        if (ui8_g_duty_cycle > 0) {
            // calculate phase current.
            if (ui8_g_duty_cycle > 10) {
                ui16_adc_motor_phase_current = (uint16_t)((uint16_t)((uint16_t)ui8_adc_battery_current_filtered << 8)) / ui8_g_duty_cycle;
            } else {
                ui16_adc_motor_phase_current = (uint16_t)ui8_adc_battery_current_filtered;
            }
            if (ui8_foc_flag) { // is set on 1 when rotor is at 150° so once per electric rotation
				ui8_adc_foc_angle_current = (ui8_adc_battery_current_filtered >> 1) + (ui16_adc_motor_phase_current >> 1);
                ui8_foc_flag = (uint16_t)(ui8_adc_foc_angle_current * ui8_foc_angle_multiplicator) >> 8 ; // multiplier = 39 for 48V tsdz2, 
                if (ui8_foc_flag > 13)
                    ui8_foc_flag = 13;
                ui8_foc_angle_accumulated = ui8_foc_angle_accumulated - (ui8_foc_angle_accumulated >> 4) + ui8_foc_flag;
                ui8_g_foc_angle = ui8_foc_angle_accumulated >> 4;
                ui8_foc_flag = 0;
            }
        } else {
            ui16_adc_motor_phase_current = 0;
            if (ui8_foc_flag) {
                ui8_foc_angle_accumulated = ui8_foc_angle_accumulated - (ui8_foc_angle_accumulated >> 4);
                ui8_g_foc_angle = ui8_foc_angle_accumulated >> 4;
                ui8_foc_flag = 0;
            }
        }
        // get brake state-
        ui8_brake_state = XMC_GPIO_GetInput(IN_BRAKE_PORT, IN_BRAKE_PIN) == 0; // Low level means that brake is on
    
    // to debug
    //uint16_t temp1d  = (uint16_t) RUNNING_250KH_TIMER_HW->TIMER; //uint16_t temp1d  =  XMC_CCU4_SLICE_GetTimerValue(RUNNING_250KH_TIMER_HW);
    //temp1d = temp1d - start_ticks;
    //if (temp1d > debug_time_ccu8_irq1d) debug_time_ccu8_irq1d = temp1d; // store the max enlapsed time in the irq
    
                /****************************************************************************/
        // PWM duty_cycle controller:
        // - limit battery undervolt
        // - limit battery max current
        // - limit motor max phase current
        // - limit motor max ERPS
        // - ramp up/down PWM duty_cycle and/or field weakening angle value

        // check if to decrease, increase or maintain duty cycle
        //note:
        // ui8_adc_battery_current_filtered is calculated just here above
        // ui8_adc_motor_phase_current_max = 135 per default for TSDZ2 (13A *100/16) *187/112 = battery_current convert to ADC10bits *and ratio between adc max for phase and for battery
        //        is initiaded in void ebike_app_init(void) in ebyke_app.c
        
        
        // every 25ms ebike_app_controller fills
        //  - ui8_controller_adc_battery_current_target
        //  - ui8_controller_duty_cycle_target // is usually filled with 255 (= 100%)
        //  - ui8_controller_duty_cycle_ramp_up_inverse_step
        //  - ui8_controller_duty_cycle_ramp_down_inverse_step
        // Furthermore,  hen ebyke_app_controller start pwm, g_duty_cycle is first set on 30 (= 12%)
        if ((ui8_controller_duty_cycle_target < ui8_g_duty_cycle)                     // requested duty cycle is lower than actual
          || (ui8_controller_adc_battery_current_target < ui8_adc_battery_current_filtered)  // requested current is lower than actual
		  || (ui16_adc_motor_phase_current > (uint16_t) ui16_adc_motor_phase_current_max)               // motor phase is to high
          || (ui16_hall_counter_total < (HALL_COUNTER_FREQ / MOTOR_OVER_SPEED_ERPS))        // Erps is to high
          || (ui16_adc_voltage < ui16_adc_voltage_cut_off)                                  // voltage is to low
          || (ui8_brake_state)) {                                                           // brake is ON
	  // reset duty cycle ramp up counter (filter)
            ui8_counter_duty_cycle_ramp_up = 0;
            // ramp down duty cycle ;  after N iterations at 19 khz 
            if (++ui8_counter_duty_cycle_ramp_down > ui8_controller_duty_cycle_ramp_down_inverse_step) {
                ui8_counter_duty_cycle_ramp_down = 0;
                //  first decrement field weakening angle if set or duty cycle if not
                if (ui8_fw_hall_counter_offset > 0) {
                    ui8_fw_hall_counter_offset--;
                }
				else if (ui8_g_duty_cycle > 0) {
                    ui8_g_duty_cycle--;
				}
            }
        }
		else if ((ui8_controller_duty_cycle_target > ui8_g_duty_cycle)                     // requested duty cycle is higher than actual
          && (ui8_controller_adc_battery_current_target > ui8_adc_battery_current_filtered)) { //Requested current is higher than actual
			// reset duty cycle ramp down counter (filter)
            ui8_counter_duty_cycle_ramp_down = 0;
            // ramp up duty cycle
            if (++ui8_counter_duty_cycle_ramp_up > ui8_controller_duty_cycle_ramp_up_inverse_step) {
                ui8_counter_duty_cycle_ramp_up = 0;
                // increment duty cycle
                if (ui8_g_duty_cycle < ui8_pwm_duty_cycle_max) {
                    ui8_g_duty_cycle++;
                }
            }
        }
		else if ((ui8_field_weakening_enabled)
				&& (ui8_g_duty_cycle == ui8_pwm_duty_cycle_max)) {
            // reset duty cycle ramp down counter (filter)
            ui8_counter_duty_cycle_ramp_down = 0;
            if (++ui8_counter_duty_cycle_ramp_up > ui8_controller_duty_cycle_ramp_up_inverse_step) {
               ui8_counter_duty_cycle_ramp_up = 0;               
               // increment field weakening angle
               if (ui8_fw_hall_counter_offset < ui8_fw_hall_counter_offset_max) {
                   ui8_fw_hall_counter_offset++;
			   }
            }
        }
		else {
            // duty cycle is where it needs to be so reset ramp counters (filter)
            ui8_counter_duty_cycle_ramp_up = 0;
            ui8_counter_duty_cycle_ramp_down = 0;
        }
    // to debug
    //uint16_t temp1e  =  XMC_CCU4_SLICE_GetTimerValue(RUNNING_250KH_TIMER_HW);
    //temp1e = temp1e - start_ticks;
    //if (temp1e > debug_time_ccu8_irq1e) debug_time_ccu8_irq1e = temp1e; // store the max enlapsed time in the irq

        /****************************************************************************/
        // Wheel speed sensor detection
        // 
        
        static uint16_t ui16_wheel_speed_sensor_ticks_counter;
        static uint8_t ui8_wheel_speed_sensor_ticks_counter_started;
        static uint8_t ui8_wheel_speed_sensor_pin_state_old;

        // check wheel speed sensor pin state
        //ui8_temp = WHEEL_SPEED_SENSOR__PORT->IDR & WHEEL_SPEED_SENSOR__PIN; // in tsdz2
        uint8_t ui8_in_speed_pin_state = (uint8_t) XMC_GPIO_GetInput(IN_SPEED_PORT, IN_SPEED_PIN);
        // check wheel speed sensor ticks counter min value
		if (ui16_wheel_speed_sensor_ticks) { 
            ui16_wheel_speed_sensor_ticks_counter_min = ui16_wheel_speed_sensor_ticks >> 3; 
        } else {
            ui16_wheel_speed_sensor_ticks_counter_min = WHEEL_SPEED_SENSOR_TICKS_COUNTER_MIN >> 3; // =39932/8= 4991
        } 
		if (!ui8_wheel_speed_sensor_ticks_counter_started ||
		  (ui16_wheel_speed_sensor_ticks_counter > ui16_wheel_speed_sensor_ticks_counter_min)) { 
			// check if wheel speed sensor pin state has changed
			if (ui8_in_speed_pin_state != ui8_wheel_speed_sensor_pin_state_old) {
				// update old wheel speed sensor pin state
				ui8_wheel_speed_sensor_pin_state_old = ui8_in_speed_pin_state;
				// only consider the 0 -> 1 transition
				if (ui8_in_speed_pin_state) {
					// check if first transition
					if (!ui8_wheel_speed_sensor_ticks_counter_started) {
						// start wheel speed sensor ticks counter as this is the first transition
						ui8_wheel_speed_sensor_ticks_counter_started = 1;
					} else {
						// check if wheel speed sensor ticks counter is out of bounds
						if (ui16_wheel_speed_sensor_ticks_counter < WHEEL_SPEED_SENSOR_TICKS_COUNTER_MAX) { // 164
							ui16_wheel_speed_sensor_ticks = 0;
							ui16_wheel_speed_sensor_ticks_counter = 0;
							ui8_wheel_speed_sensor_ticks_counter_started = 0;
						} else {
                            // a valid time occured : save the counter with the enlapse time * 55usec
							ui16_wheel_speed_sensor_ticks = ui16_wheel_speed_sensor_ticks_counter; 
							ui16_wheel_speed_sensor_ticks_counter = 0;
                            ++ui32_wheel_speed_sensor_ticks_total;
						}
					}
				}
			}
		}

        // increment and also limit the ticks counter
        if (ui8_wheel_speed_sensor_ticks_counter_started) {
            if (ui16_wheel_speed_sensor_ticks_counter < WHEEL_SPEED_SENSOR_TICKS_COUNTER_MIN) { // 39932 ; so when speed is more than a min
                ++ui16_wheel_speed_sensor_ticks_counter; // increase counter
            } else {
                // reset variables
                ui16_wheel_speed_sensor_ticks = 0;
                ui16_wheel_speed_sensor_ticks_counter = 0;
                ui8_wheel_speed_sensor_ticks_counter_started = 0;
            }
        }
        //end wheel speed

        /****************************************************************************/
        /*
         * - New pedal start/stop detection Algorithm (by MSpider65) -
         *
         * Pedal start/stop detection uses both transitions of both PAS sensors
         * ui8_temp stores the PAS1 and PAS2 state: bit0=PAS1,  bit1=PAS2
         * Pedal forward ui8_temp sequence is: 0x01 -> 0x00 -> 0x02 -> 0x03 -> 0x01
         * After a stop, the first forward transition is taken as reference transition
         * Following forward transition sets the cadence to 7RPM for immediate startup
         * Then, starting from the second reference transition, the cadence is calculated based on counter value
         * All transitions are a reference for the stop detection counter (4 time faster stop detection):
         */
        
        uint8_t ui8_temp_cadence = 0;
        //if (PAS1__PORT->IDR & PAS1__PIN) {    // this was the code in TSDZ2
        //    ui8_temp |= (unsigned char)0x01;
		//}
        //if (PAS2__PORT->IDR & PAS2__PIN) {
        //    ui8_temp |= (unsigned char)0x02;
		//}
        ui8_temp_cadence = (uint8_t) (XMC_GPIO_GetInput(IN_PAS1_PORT, IN_PAS1_PIN ) | ( XMC_GPIO_GetInput(IN_PAS2_PORT, IN_PAS2_PIN ) <<1 ));
        if (ui8_temp_cadence != ui8_pas_state_old) {
            if (ui8_pas_state_old != ui8_pas_old_valid_state[ui8_temp_cadence]) {
                // wrong state sequence: backward rotation
                ui16_cadence_sensor_ticks = 0;
                ui8_cadence_calc_ref_state = NO_PAS_REF; // 5
                goto skip_cadence;
            }
			ui16_cadence_sensor_ticks_counter_min = ui16_cadence_ticks_count_min_speed_adj; // 4270 at 4km/h ... 341 at 40 km/h
            if (ui8_temp_cadence == ui8_cadence_calc_ref_state) { // pattern is valid and represent 1 tour
                // ui16_cadence_calc_counter is valid for cadence calculation
                ui16_cadence_sensor_ticks = ui16_cadence_calc_counter; // use the counter as cadence for ebike_app.c
                ui16_cadence_calc_counter = 0;
                // software based Schmitt trigger to stop motor jitter when at resolution limits
                ui16_cadence_sensor_ticks_counter_min += CADENCE_SENSOR_STANDARD_MODE_SCHMITT_TRIGGER_THRESHOLD; // 427 at 19 khz
            } else if (ui8_cadence_calc_ref_state == NO_PAS_REF) {  // 5
                // this is the new reference state for cadence calculation
                ui8_cadence_calc_ref_state = ui8_temp_cadence;
                ui16_cadence_calc_counter = 0;
            } else if (ui16_cadence_sensor_ticks == 0) {
                // Waiting the second reference transition: set the cadence to 7 RPM for immediate start
                ui16_cadence_sensor_ticks = CADENCE_TICKS_STARTUP; // 7619
            }
            skip_cadence:
            ui16_cadence_stop_counter = 0; // reset the counter used to detect pedal stop
            ui8_pas_state_old = ui8_temp_cadence; // save current PAS state to detect a change
        } // end of change in PAS pattern
        if (++ui16_cadence_stop_counter > ui16_cadence_sensor_ticks_counter_min) {// pedals stop detected
            ui16_cadence_sensor_ticks = 0;
            ui16_cadence_stop_counter = 0;
            ui8_cadence_calc_ref_state = NO_PAS_REF;
        } else if (ui8_cadence_calc_ref_state != NO_PAS_REF) { // 5
            // increment cadence tick counter
            ++ui16_cadence_calc_counter;
        }
        // end cadence

        // original perform also a save of some parameters (battery consumption) // to do 
    // to debug    
    //uint16_t temp1  =  XMC_CCU4_SLICE_GetTimerValue(RUNNING_250KH_TIMER_HW);
    //temp1 = temp1 - start_ticks;
    //if (temp1 > debug_time_ccu8_irq1) debug_time_ccu8_irq1 = temp1; // store the max enlapsed time in the irq
}  // end of CCU8_1_IRQ


void motor_enable_pwm(void) { //set posif with current position & restart the timers
    get_hall_pattern(); // refresh hall pattern in current_hall_pattern
    
    // one solution to activate is to generate an event that starts all timers in a synchronized way
    // Enable Global Start Control CCU80  in a synchronized way*/
    XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC80_Msk);
    XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk);
    uint32_t retry_start_counter = 10;
    while ((!XMC_CCU8_SLICE_IsTimerRunning(PHASE_U_TIMER_HW)) && (retry_start_counter > 0)){ // to be sure it is running
        XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC80_Msk);
        XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk);
    }
}

void motor_disable_pwm(void) {
    // we stop and clear the 3 timers that control motor PWM
    XMC_CCU8_SLICE_StopClearTimer(PHASE_U_TIMER_HW);
    XMC_CCU8_SLICE_StopClearTimer(PHASE_V_TIMER_HW);
    XMC_CCU8_SLICE_StopClearTimer(PHASE_W_TIMER_HW);
    // slice CCU8_3 is not stopped becauses it is required to manage some tasks (speed, torque,...) 
}

void get_hall_pattern(){
    uint32_t critical_section_value = XMC_EnterCriticalSection();
    current_hall_pattern = XMC_GPIO_GetInput(IN_HALL0_PORT, IN_HALL0_PIN);// hall 0
    current_hall_pattern |=  XMC_GPIO_GetInput(IN_HALL1_PORT, IN_HALL1_PIN) << 1;
    current_hall_pattern |=  XMC_GPIO_GetInput(IN_HALL2_PORT, IN_HALL2_PIN) << 2;
    XMC_ExitCriticalSection(critical_section_value);
}


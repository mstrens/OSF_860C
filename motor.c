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
#include "adc.h"

#include "cy_retarget_io.h"
//#include "cy_utils.h"
#if(uCPROBE_GUI_OSCILLOSCOPE == MY_ENABLED)
#include "ProbeScope/probe_scope.h"
#endif



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

#if (USE_INT_LUT == (1))
static const int16_t i16_LUT_SINUS[256] = {
    770,770,770,771,772,773,775,776,778,780,782,784,786,789,791,793,
    795,796,798,799,800,800,800,799,798,796,794,791,787,782,776,770,
    762,754,744,733,722,709,695,680,664,647,629,609,589,567,544,521,
    496,470,443,416,387,358,328,297,266,234,201,168,135,102,68,34,
    0,-34,-68,-102,-135,-168,-201,-234,-266,-297,-328,-358,-387,-416,-443,-470,
    -496,-521,-544,-567,-589,-609,-629,-647,-664,-680,-695,-709,-722,-733,-744,-754,
    -762,-770,-776,-782,-787,-791,-794,-796,-798,-799,-800,-800,-800,-799,-798,-796,
    -795,-793,-791,-789,-786,-784,-782,-780,-778,-776,-775,-773,-772,-771,-770,-770,
    -770,-770,-770,-771,-772,-773,-775,-776,-778,-780,-782,-784,-786,-789,-791,-793,
    -795,-796,-798,-799,-800,-800,-800,-799,-798,-796,-794,-791,-787,-782,-776,-770,
    -762,-754,-744,-733,-722,-709,-695,-680,-664,-647,-629,-609,-589,-567,-544,-521,
    -496,-470,-443,-416,-387,-358,-328,-297,-266,-234,-201,-168,-135,-102,-68,-34,
    0,34,68,102,135,168,201,234,266,297,328,358,387,416,443,470,
    496,521,544,567,589,609,629,647,664,680,695,709,722,733,744,754,
    762,770,776,782,787,791,794,796,798,799,800,800,800,799,798,796,
    795,793,791,789,786,784,782,780,778,776,775,773,772,771,770,770
};

// Hall positions in Q8.8
// Position rotorique in Q8.8 et vitesse en Q16.16
typedef int32_t q16_16_t; // (signed) (16 bits for decimal, 16bits for unit, 1 unit = 360/256 = 1.4°)
/* Q8.8 typedefs */
typedef int16_t q8_8_t;     // valeur signée Q8.8 (−128..+127.996) si on veut signed
typedef uint16_t uq8_8_t;   // valeur non signée Q8.8 (0..255.996) pour index / LUT

#define Q8_8_SHIFT      8
#define Q8_8_ONE        (1 << Q8_8_SHIFT)
#define Q8_8_HALF_TURN  (128 << Q8_8_SHIFT) // 180° = 128 units (Q8.8)
#define Q8_8_FULL       (256 << Q8_8_SHIFT) // wrap value in Q8.8 arithmetic
// --- Définition d'un offset de 30° en Q8.8 ---
#define HALL_ANGLE_OFFSET_30_DEG_Q8_8  ((uint16_t)((30UL * 65536UL) / 360UL))  // ≈ 5461
// --- Définition d'un offset de 60° en Q8.8 to limit interpolation---
#define HALL_INTERP_MAX_DELTA_Q8_8   ((uint16_t)((60 * 65536UL) / 360))   // 60° = 10922 en Q8.8 (~0x2AAA)
#define Q16_16_SHIFT              16     // position Q16.16

uq8_8_t ui16_motor_phase_absolute_angle_q8_8 = 0;
// Angles mesurés pour les patterns Hall valides (Q8.8 = angle° * 256); sequence is 1,3,2,6,4, 5
uq8_8_t u16_hall_angle_table_Q8_8[8] = {
    0,      // pattern 0 invalide
    24<<8,  // 1 -> 24 * 360 / 256 degré
    107<<8, // 2 -> 107 * 360 / 256 degré
    66<<8,  // 3 -> 66 * 360 / 256 degré
    195<<8, // 4 -> 195 * 360 / 256 degré
    235<<8, // 5 -> 235 * 360 / 256 degré
    152<<8, // 6 -> 152 * 360 / 256 degré
    0       // 7 invalide
};

// for hall position & hall velocity 
q8_8_t i16_hall_position_q8_8 = 0;           // position rotorique absolue (Q18.8) based on hall (+ interpolation)
//int32_t i32_hall_velocity_q8_8X256;  // vitesse mesurée directement entre deux fronts Hall
uint32_t u32_hall_velocity_q8_8X256;  // vitesse mesurée directement entre deux fronts Hall

uint16_t ui16_angle_for_id_q8_8;   // position without taking care of lead angle; updated at the end of ISR 0 

//o debug
uint8_t ui8_error_interpolation_q8_8 = 0; // to debug
int8_t diff_lut_index = 0 ;
uint8_t ui8_signed_index_debug =0;
uint8_t ui8_unsigned_index_debug = 0;
int16_t diff_interpol = 0;
uint16_t diff_interpol_max = 0;
int16_t diff_abs_pos =  0;
uint8_t u8_debug_negative_velocity ;    
uint32_t velocity_max =0;
uint16_t enlapsed_debug;
uint16_t debug_current_speed_timer_ticks;
uint16_t debug_last_hall_pattern_change_ticks;
uint16_t debug_diff_interval ;
uint16_t debug_prev_front_interval ;
uint16_t debug_diff_interval_max;
uint16_t debug_diff_interval_cnt = 0;
uint16_t ui16_hall_counter_total_max;
uint16_t debug_prev_front_interval_max;
uint16_t elapse_buffer[12];
uint8_t elapse_buffer_idx = 0;
uint16_t elapse_buffer_bu[13];
uint16_t previous_hall_pattern_change_ticks ;
uint16_t prev_pattern_change_ticks;
uint16_t debug_cnt1 = 0;
uint16_t debug_cnt2 = 0;
#endif

// motor variables
uint8_t ui8_hall_360_ref_valid = 0; // fill with a hall pattern to check sequence is correct
uint8_t ui8_motor_commutation_type = BLOCK_COMMUTATION;
uint8_t ui8_motor_phase_absolute_angle = 0;
volatile uint16_t ui16_hall_counter_total = 0xffff; // number of tim3 ticks between 2 rotations// inTSDZ2 it was a u16
//static uint16_t ui16_hall_counter_total_previous = 0;  // used to check if erps is stable
uint8_t ui8_interpolation_angle = 0; // interpolation angle

// power variables
volatile uint8_t ui8_controller_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT; // 194
volatile uint8_t ui8_controller_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT; // 73
volatile uint16_t ui16_adc_voltage_cut_off = 300*100/BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000; // 30Volt default value =  300*100/87 in TSDZ2
volatile uint8_t ui8_adc_battery_current_filtered = 0; // current in adc10 bits units (average on 1 rotation)
volatile uint32_t ui32_adc_battery_current_1_rotation_15b = 0; // value in 12 +2 +1 = 15 bits (ADC + IIR + average)
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
//static uint8_t ui8_foc_angle_accumulated = 0;
static uint8_t ui8_foc_flag = 0;
volatile uint8_t ui8_g_foc_angle = 0;
uint8_t ui8_foc_angle_multiplicator = 0;
//static uint8_t ui8_foc_angle_multiplier = FOC_ANGLE_MULTIPLIER; //39 for 48V motor
//static uint8_t ui8_adc_foc_angle_current = 0; // use a ui16 inside the irq

// battery current variables
uint16_t ui16_adc_battery_current_acc_X4 = 0;
uint16_t ui16_adc_battery_current_filtered_X4 = 0;
volatile uint16_t ui16_adc_motor_phase_current = 0; // mstrens: it was uint8 in original code

// ADC Values
volatile uint16_t ui16_adc_voltage = 0;
volatile uint16_t ui16_adc_torque = 0;
//volatile uint16_t ui16_adc_throttle = 0; // moved to ebike_app.c
//added by mstrens
volatile uint16_t ui16_adc_torque_filtered = 0 ; // filtered adc torque
volatile uint16_t ui16_adc_torque_actual_rotation = 0;
volatile uint16_t ui16_adc_torque_previous_rotation = 0;
volatile uint8_t ui8_adc_torque_rotation_reset = 0;
    
// brakes
volatile uint8_t ui8_brake_state = 0;

// cadence sensor
#define NO_PAS_REF 5
volatile uint16_t ui16_cadence_sensor_ticks = 0;
static uint16_t ui16_cadence_sensor_ticks_counter_min = CADENCE_SENSOR_CALC_COUNTER_MIN; // initialiszed at 4270 , then varies with wheelSpeed
static uint8_t ui8_pas_state_old = 4;
static uint16_t ui16_cadence_calc_counter = 0;
static uint16_t ui16_cadence_stop_counter = 0;
static uint8_t ui8_cadence_calc_ref_state = NO_PAS_REF;
const static uint8_t ui8_pas_old_valid_state[4] = { 0x01, 0x03, 0x00, 0x02 };
//added by mstrens
uint8_t ui8_pas_counter = 0; // counter to detect a full pedal rotation (after 20 valid transitions)

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

volatile uint8_t ui8_hall_sensors_state = 0; // name used by ebike_app.c to identify current_hall_pattern; added here for compatibility

// Hall counter value of last Hall transition 
uint16_t previous_360_ref_ticks = 0 ; 

// ----------   end of copy from tsdz2 -------------------------

uint8_t ui8_temp = 0;
uint16_t ui16_temp = 0;

volatile uint16_t ui16_a = PWM_COUNTER_MAX / 2 ;//   840 in tsdz8   // 4*210 from tsdz2
volatile uint16_t ui16_b = PWM_COUNTER_MAX / 2 ;//   840 in tsdz8   // 4*210 from tsdz2
volatile uint16_t ui16_c = PWM_COUNTER_MAX / 2 ;//   840 in tsdz8   // 4*210 from tsdz2

uint8_t hall_reference_angle = 0 ; // This value is initialised in ebike_app.c with DEFAULT_HALL_REFERENCE_ANGLE and m_config.global_offset_angle 

// to debug time spent in irq0 and irq1
volatile uint16_t debug_time_ccu8_irq0 = 0;
//volatile uint16_t debug_time_ccu8_irq1 = 0;
//volatile uint16_t debug_time_ccu8_irq1b = 0;
//volatile uint16_t debug_time_ccu8_irq1c = 0;
//volatile uint16_t debug_time_ccu8_irq1d = 0;
//volatile uint16_t debug_time_ccu8_irq1e = 0;
uint16_t hall_ref_angles_counter = 0;

//extern uint8_t ui8_pwm_duty_cycle_max;

uint8_t ui8_hall_ref_angles[8] = { // Sequence is 1, 3, 2 , 6, 4, 5; so angle are in theory e.g. 39, 86, 127, 167, 216, 0 (256=360°)
        0,                     // error ; index must be between 1 and 6
        24 , //    for hall pattern 1
        107, //     for hall pattern 2  106
        66, //       for hall pattern 3 65
        195, //    for hall pattern 4 
        235 , //    for hall pattern 5
        152, //     for hall pattern 6  
        0                     // error ; index must be between 1 and 6
};

// this could probably be removed
volatile uint8_t ui8_best_ref_angles[8] ; // this table is prefilled in main.c at start up
uint32_t best_ref_angles_X16bits[8] ;  // same as ui8_best_ref_angles but with 8 more bits for better filtering
uint32_t ui32_angle_per_tick_X16shift = 0 ; // 

// Hall offset for current Hall state; This offset is added in the interpolation process (so based also on the erps)
// the value is in ticks (1 ticks = 4 usec); we need  55usec/4 : 55 = 39 + 16 (39 = 3/4 of 55usec = delay between measuring and PWM change); 16=delay hall sensor
// based on the regression tests, there should probably be a correction of about 2 depending it is a rising or a falling edge of hall pattern
// still this should have only a small impact
uint8_t ui8_hall_counter_offset = 14; 
uint8_t ui8_angle_for_id;   // position without taking care of lead angle; updated at the end of ISR 0 


#if (DYNAMIC_LEAD_ANGLE == (1) ) //1 dynamic based on Id and a PID + optimiser 
// to calculate Id
uint8_t ui8_angle_for_id_prev; // position; saved at begin of ISR 0 to match with current iu,Iv,iw measured at begin of ISR 1
uint16_t ADC_Bias_Iu = 1 << 11; // ADC is 12 bits, 0 = mid point 
uint16_t ADC_Bias_Iv = 1 << 11; // ADC is 12 bits, 0 = mid point 
uint16_t ADC_Bias_Iw = 1 << 11; // ADC is 12 bits, 0 = mid point 
int32_t i32_id_filtr = 0;       // Id filtered (calculated in calculate_id_part1 and 2 ; used to adapt Q31_lead_angle with a pid)
volatile int32_t i32_id_pid_acc = 0 ;    // accumulate the Id value to be able to calculate the avg
volatile int32_t i32_id_pid_cnt = 0 ;    // count the Id value in acc to be able to calculate the avg

int32_t q31_lead_angle = 0 ; // lead angle in Q31
int32_t foc_pid_I_term = 0;  // integral term of foc pid

#define SQRT3                                       (1.732050807569F)       /* √3 */
#define DIV_SQRT3                                   (591)                  /* ((int16_t)((1/SQRT3) * (1<<SCALE_SQRT3))) */
#define DIV_SQRT3_Q14                               (9459U)
#define SCALE_DIV_3                                 (14U)                   /* For 1/3 scaling. */
#define DIV_3                                       (5461U)                 /* ((int16_t)((1/3) * (1<<SCALE_DIV_3))) */

#define DEGREE_90                                   (4194304U << 8U)        /* 90° angle (0 ~ 2^23 represent electrical angle 0° ~ 180° in CORDIC) */
#define DEGREE_X                                    (DEGREE_90 * 1U)        /* X = 0°, 90°, 180°, or 270° */
#define DEGREE_SHIFT                                (652448U << 8U)         /* 14° angle shift */

#define CORDIC_VECTORING_MODE                       (0x62)                  /* CORDIC: Circular Vectoring Mode (default). MPS: Divide by 2 (default).*/
#define CORDIC_ROTATION_MODE                        (0x6A)                  /*  CORDIC: Circular Rotation Mode. MPS: Divide by 2 (default).*/
#define CORDIC_SHIFT                                (14U)             /* 8 ~ 16. Shift for CORDIC input / output registers, whose [7:0] are 0x00. Normally no need change.*/
#endif // DYNAMIC_LEAD_ANGLE == 1 dynamic based on Id and a PID + optimiser 

// to debug
int16_t I_u; // to check current in each phase
int16_t I_v;
int16_t I_w;
int16_t I_t;
uint16_t prev_ticks = 0;
uint16_t interval_ticks = 0;
uint8_t first_ticks = 1; // says that interval has not yet been calculated
uint16_t error_ticks_counter = 0;
uint16_t error_ticks_value;
uint16_t error_ticks_prev;
uint16_t interval_ticks_min = 0xFFFF; 
uint16_t interval_ticks_max = 0; 

uint16_t irq0_min = 0xFFFF;
uint16_t irq0_max = 0;
uint16_t irq1_min = 0xFFFF;
uint16_t irq1_max = 0;


uint16_t hall_pattern_error_counter = 0;
// used to calculate hall angles based of linear regression of all ticks intervals
// are filled in irq0 and transmitted in ebike_app.c using segger_rtt_print 
#if ( GENERATE_DATA_FOR_REGRESSION_ANGLES == (1) )
uint16_t ticks_intervals[8]; // ticks intervals between 2 pattern changes;
uint8_t ticks_intervals_status; // 0 =  new data can be written; 1 data being written; 2 all data written, must be transmitted
uint16_t previous_hall_pattern_change_ticks;  // save the ticks of last pattern change
#endif

volatile uint16_t ticks_hall_pattern_irq_last = 0;
volatile uint8_t current_hall_pattern_irq = 0;

//uint32_t ui32_angle_per_tick_X16shift_new;
//uint32_t ui32_ref_angle; 
//uint32_t ui32_ref_angle_new ; // temporary calculation
//uint8_t ui8_motor_phase_absolute_angle_new; // reference for extrapolation with new algorithm 
//uint8_t ui8_svm_old = 0;
//uint8_t ui8_svm_new = 0;

//uint16_t ui16_duty_cycle_count_down = 0;
//uint16_t ui16_duty_cycle_count_up = 0;

// for current calculation
uint32_t ui32_adc_battery_current_15b = 0; // value from adc

#if (TYPE_OF_FILTER_FOR_CURRENT == (0)) // moving average
uint32_t ui32_adc_battery_current_15b_moving_average = 0;
int battery_current_moving_avg_index = 0;
int battery_current_moving_avg_sum = 0;
int battery_current_moving_avg_buffer[64] = {0};
#else                                  // average current on 1 eletric rotation
uint32_t ui32_adc_battery_current_15b_accum = 0; 
uint32_t ui32_adc_battery_current_15b_counter = 0;
#endif



// to manage torque sensor using the logic of mspider in https://github.com/TSDZ2-ESP32/TSDZ2-Smart-EBike
// 1 = PAS state value changed
// 0x80  = PAS state invalid -> reset
volatile uint8_t ui8_pas_new_transition = 0;

#if (TYPE_OF_FILTER_FOR_CURRENT == (0)) // when we calulate the average of last 64 values
inline uint32_t update_moving_average(uint32_t new_value){
    battery_current_moving_avg_sum -= battery_current_moving_avg_buffer[battery_current_moving_avg_index];
    battery_current_moving_avg_buffer[battery_current_moving_avg_index] = new_value;
    battery_current_moving_avg_sum += new_value;
    battery_current_moving_avg_index = (battery_current_moving_avg_index + 1) & 0x3F; 
    // Retourne la moyenne actuelle
    return (battery_current_moving_avg_sum + 32) >> 6; // divide by 64; add 32 for better rounding
}
#endif

inline uint32_t filtering_function(uint32_t ui32_temp_15b , uint32_t ui32_filtered_15b , uint32_t alpha){
    uint32_t ui32_temp_new = ui32_temp_15b * (16U - alpha);
    uint32_t ui32_temp_old =  ui32_filtered_15b * alpha;
    uint32_t ui32_filtered_value = ((ui32_temp_new + ui32_temp_old + (8)) >> 4);                    
    if (ui32_filtered_value == ui32_filtered_15b) {
        if (ui32_filtered_value < ui32_temp_15b)
            ui32_filtered_value++;
        else if (ui32_filtered_value > ui32_temp_15b)
            ui32_filtered_value--;
    }
    return ui32_filtered_value ;                  
}



void VADC0_G0_0_IRQHandler() {  // VADC is configured to compare the total current (12bits) with "1000" and generate an irq
    ui8_m_system_state |= ERROR_BATTERY_OVERCURRENT; // set the error to avoid that motor starts again
    // disable the motor
    ui8_motor_enabled = 0;
    motor_disable_pwm();
}

#if (USE_IRQ_FOR_HALL == (1))
// this irq callback occurs when posif detects a new pattern 
__RAM_FUNC void POSIF0_0_IRQHandler(){
//void POSIF0_0_IRQHandler(){
        // Capture time stamp 
    ticks_hall_pattern_irq_last = XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
    // capture hall pattern
    current_hall_pattern_irq = XMC_GPIO_GetInput(IN_HALL0_PORT, IN_HALL0_PIN);// hall 0
    current_hall_pattern_irq |=  XMC_GPIO_GetInput(IN_HALL1_PORT, IN_HALL1_PIN) << 1;
    current_hall_pattern_irq |=  XMC_GPIO_GetInput(IN_HALL2_PORT, IN_HALL2_PIN) << 2;
}
#endif

#if (DYNAMIC_LEAD_ANGLE == (1)) // (1) dynamic based on Id and a PID + optimiser
__RAM_FUNC static inline void calculate_id_part1(){  // to be called in begin of ISR 1 when rotor position has been updated and current are measured
    // it measure actual currents but angle must be one one that was apply for PWM and so it is the angle from isr 0 before update.
    //static inline void calculate_id_part1(){  // to be called in first ISR when rotor position has been updated  
        // read the 3 ADC
        // substact the ADC bias
        // calculate i_alpha and i_beta (clark transform)
        // fill cordic to get IQ ID (park transform)
        int16_t i16_raw_Iu = XMC_VADC_GROUP_GetResult(VADC_I1_GROUP , VADC_I1_RESULT_REG ) & 0x0FFF;
        int16_t i16_raw_Iw = XMC_VADC_GROUP_GetResult(VADC_I3_GROUP , VADC_I3_RESULT_REG ) & 0x0FFF;
        int16_t i16_raw_Iv = XMC_VADC_GROUP_GetResult(VADC_I2_GROUP , VADC_I2_RESULT_REG ) & 0x0FFF;
    
        // note :  in infineon version the sign are reversed ; this is strange
        int32_t i32_Iu = (i16_raw_Iu - ADC_Bias_Iu) << 3; // change from 12 bits to 15 bits to use Q15
        int32_t i32_Iv = (i16_raw_Iv - ADC_Bias_Iv) << 3;
        int32_t i32_Iw = (i16_raw_Iw - ADC_Bias_Iw) << 3;
        
        // calculate I alpha and I beta
        /* I_Alpha = (2 * I_U - (I_V + I_W))/3 */  // ou Ialpha = (2/3) * (Ia - 0.5*Ib - 0.5*Ic)
        //HandlePtr->I_Alpha_1Q31 = ((CurrentPhaseU << 1) - (CurrentPhaseV + CurrentPhaseW)) * (DIV_3 << (CORDIC_SHIFT-14));
        int32_t I_Alpha_1Q31 = ((i32_Iu << 1) - (i32_Iv + i32_Iw)) * (DIV_3 << (CORDIC_SHIFT-14));
    
        /*  I_Beta = (I_V - I_W)/√3 in 1Q31 */
        //HandlePtr->I_Beta_1Q31 = (CurrentPhaseV - CurrentPhaseW) * (DIV_SQRT3_Q14 << (CORDIC_SHIFT-14));
        int32_t I_Beta_1Q31 = (i32_Iv - i32_Iw) * (DIV_SQRT3_Q14 << (CORDIC_SHIFT-14));
    
        // prepare parktransform with cordic
            /* General control of CORDIC Control Register */
        MATH->CON = CORDIC_ROTATION_MODE;
    
        /* Z = φ, Hall rotor angle, or estimated rotor angle of last PWM cycle from PLL */
        //MATH->CORDZ = RotorAngleQ31;
        // to convert an angle from ui8 to Q31, we must first do a cast of uint8 to int8 and then a shift left by 24 
        MATH->CORDZ = ((int8_t) ui8_angle_for_id_prev) << 24; // we convert angle in 0/255 to Q31 
    
        /* Y = I_Alpha */
        MATH->CORDY = I_Alpha_1Q31;
    
        /* X = I_Beta. Input CORDX data, and auto start of CORDIC calculation (~62 kernel clock cycles) */
        MATH->CORDX = I_Beta_1Q31;
    }
    
    
    #define ALPHA_Q15   172     // ~0.005263 * 32768
    #define Q15_SHIFT   15
    
    __RAM_FUNC inline void calculate_id_part2(){ // to be called at the end of ISR1 (so cordic has time to finish)
        // get the result of cordic for id and iq
        // apply a filter on id.
    
        /* Wait if CORDIC is still running calculation */
        while (MATH->STATC & 0x01)
        {
            continue;
        }
        /* Read CORDIC results Iq and Id - 32-bit. CORDIC Result Register [7:0] are 0x00 */
        int32_t i32_iq = MATH->CORRX;
        i32_iq >>= CORDIC_SHIFT; // shift 14
        i32_iq = (i32_iq * 311) >> 8;   // x MPS/K.;
        
        //Idem for Id
        int32_t i32_id = MATH->CORRY;
        i32_id >>= CORDIC_SHIFT;
        i32_id = (i32_id * 311) >> 8;   // x MPS/K.;
        // here id should be in the same units as original current (so as with ADC 15 bits because we used ADC12 << 3)
        // 1 step ADC10 = 0,16A
        // 1 step ADC15 = 0,16A / 32 = 0,005 A = 5 mA
        // Current does not exceed 50A, so ADC 15 bit should not exceed 50000 / 5 = 10000 

        // apply IIR on id
        // IIR: Id_filt += (alpha * (Id_raw - Id_filt)) >> 15
        //int32_t diff = i32_id - i32_id_filtr;
        //i32_id_filtr += (diff * ALPHA_Q15) >> Q15_SHIFT;
        
        // save data to calculate AVG at 100hz (PID) : cnt max = 19000 /100= 190; 190*10000 fit in i32 (so OK)
        i32_id_pid_acc += i32_id; // accumulate
        i32_id_pid_cnt++;         // count
        // update of foc angle occurs in 100 hz and not in ISR
    }
#endif // end (1) dynamic based on Id and a PID + optimiser    

// ************************************** begin of IRQ *************************
// *************** irq 0 of ccu8
__RAM_FUNC void CCU80_0_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting UP (= 1/4 of 19mhz cycles with 1680 ticks at 64mHz and centered aligned)
//void CCU80_0_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting UP (= 1/4 of 19mhz cycles with 1680 ticks at 64mHz and centered aligned)

//void CCU80_0_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting UP (= 1/4 of 19mhz cycles with 1680 ticks at 64mHz and centered aligned)
// here we just calculate the new compare values used for the 3 slices (0,1,2) that generates the 3 PWM
#if (USE_IRQ_FOR_HALL == (1))
    uint32_t critical_section_value = XMC_EnterCriticalSection();
    // get the current ticks
    uint16_t current_speed_timer_ticks = (uint16_t) (XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW) );
    // get the last changed pattern ticks (from posif irq)
    uint16_t last_hall_pattern_change_ticks = ticks_hall_pattern_irq_last;
    // get the current hall pattern as saved duting the posif irq
    current_hall_pattern = current_hall_pattern_irq;
    XMC_ExitCriticalSection(critical_section_value);
#else // irq0 when using a XMC_CCU4_SLICE_CAPTURE
    // get the current ticks
    //uint16_t current_speed_timer_ticks = (uint16_t) (XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW) );
    uint16_t current_speed_timer_ticks = (uint16_t) HALL_SPEED_TIMER_HW->TIMER;
    // get the capture register = last changed pattern = current pattern
    uint16_t last_hall_pattern_change_ticks = (uint16_t) XMC_CCU4_SLICE_GetCaptureRegisterValue(HALL_SPEED_TIMER_HW , 1);
    // get the current hall pattern
    current_hall_pattern = XMC_POSIF_HSC_GetLastSampledPattern(HALL_POSIF_HW) ;
#endif
    ui8_hall_sensors_state = current_hall_pattern; // duplicate just for easier maintenance of ebike_app.c for 860c (sent to display)
 
    // to debug glitch when ccu4 slice 0 rising st (at compare value) trigger a capture on ccu4 slice 1 as event 0
    if (ui16_hall_counter_total<2000) {
        if ((current_hall_pattern !=  previous_hall_pattern) && (last_hall_pattern_change_ticks == prev_pattern_change_ticks)){
            debug_cnt1++;
        } else if ((current_hall_pattern ==  previous_hall_pattern) && (last_hall_pattern_change_ticks != prev_pattern_change_ticks)){
            debug_cnt2++;
        }
    }
    prev_pattern_change_ticks = last_hall_pattern_change_ticks;


    // elapsed time between now and last pattern change (used for interpolation)
    uint16_t enlapsed_time =  current_speed_timer_ticks - last_hall_pattern_change_ticks ; // ticks between now and last pattern change
    //debug
    //debug_current_speed_timer_ticks = current_speed_timer_ticks;
    //debug_last_hall_pattern_change_ticks = last_hall_pattern_change_ticks;
    
    // elapse time since last pattern 1
//    uint16_t elapsed_ticks_since_pattern_1 = current_speed_timer_ticks - previous_360_ref_ticks;

//    uint8_t new_hall_pattern = 0; // filled only when a new pattern is detected

    // to debug time in irq
    //uint16_t start_ticks = current_speed_timer_ticks; // save to calculate enlased time inside the irq // just for debug could be removed
    #define DEBUG_IRQ0_INTERVALS (0) // 1 = calculate min and max intervals between 2 irq0
    #if (DEBUG_IRQ0_INTERVALS == (1))
    interval_ticks = current_speed_timer_ticks - prev_ticks;
    if (first_ticks == 0){
        if ( (interval_ticks <=13) || (interval_ticks >= 13)) {
            error_ticks_counter++;
            //error_ticks_value = current_speed_timer_ticks;
            //error_ticks_prev = prev_ticks;
            if (interval_ticks_min > interval_ticks) interval_ticks_min = interval_ticks;
            if (interval_ticks_max < interval_ticks) interval_ticks_max = interval_ticks;
             
        }
    } else {
        first_ticks = 0; 
    }
    prev_ticks = current_speed_timer_ticks ;
    #endif
    #if (DYNAMIC_LEAD_ANGLE == (1)) // 1 dynamic based on Id and a PID + optimiser 
    // added by mstrens to calculate Id with position used for PWM beeing currently applied (so when timer reached 0 match)
    ui8_angle_for_id_prev = ui8_angle_for_id;  // save angle to use it in ISR 1 (when current iu, iv, iw are measured for actual pwm)
    #endif

//the resistance/gain in TSDZ8 is 4X smaller than in TSDZ2; still ADC is 12 bits instead of 10; so ADC 12bits TSDZ8 = ADC 10 bits TSDZ2
        // in TSDZ2, we used only the 8 lowest bits of adc; 1 adc step = 0,16A
        // In tsdz8, the resistance is (I expect) 0.003 Ohm ; So 1A => 0,003V => 0,03V (gain aop is 10)*4096/5Vcc = 24,576 steps
        //      SO 1 adc step 12bits = 1/24,576 = 0,040A
        // For 10 A, TSDZ2 should gives 10/0,16 = 62 steps
        // For 10 A, TSDZ8 shoud give 10*24,576 steps = 246 steps
        // to convert TSDZ8 steps 12bits  in the same units as TSDZ2, we shoud take ADC12bits *62/245,76 = 0,25 and divide by 4 (or >>2)
        // current is available in gr0 result 15 in queue 0 p2.8 and/or in gr1 result 152 (p2.8)
        // both results use IIR filters and so results are in 14 bits instead of 12 bits
        // use measurement from the 2 groups
    //ui32_adc_battery_current_15b = ((XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 15 ) & 0xFFFF) +
    //                                (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 15 ) & 0xFFFF)) ;  // So here result is in 15 bits (averaging
    // changed when using infineon init for vadc (result in 12bits and in ch 1)
    ui32_adc_battery_current_15b = (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , VADC_I4_RESULT_REG ) & 0xFFFF) <<3; // change from 12 to 15 digits 
    #if (TYPE_OF_FILTER_FOR_CURRENT == (0)) // when using a moving average on 64 values    
    ui32_adc_battery_current_15b_moving_average = update_moving_average(ui32_adc_battery_current_15b);
    #else
    //accumulate the current to calculate an average on 1 rotation (there are quite big variations inside each 60° sector)
    ui32_adc_battery_current_15b_accum += ui32_adc_battery_current_15b;
    ui32_adc_battery_current_15b_counter++;
    // the average is then calculated only once per rotation later on
    #endif
    
    // to see on prove scope oscillo
    //I_t = ui32_adc_battery_current_15b >> 3; 
    // when pattern change
    if ( current_hall_pattern != previous_hall_pattern) {
        if (current_hall_pattern != expected_pattern_table[previous_hall_pattern]){ // new pattern is not the expected one
            ui8_motor_commutation_type = BLOCK_COMMUTATION; // 0x00
            ui8_hall_360_ref_valid = 0;  // reset the indicator saying no error for a 360° electric rotation 
            ui32_angle_per_tick_X16shift = 0; // 0 means unvalid value
            #if (USE_INT_LUT == (1))
            //i32_hall_velocity_q8_8X256 =  0;  // reset the speed for interpolation in block commutation
            u32_hall_velocity_q8_8X256 =  0;  // reset the speed for interpolation in block commutation
            #endif
            if (ui16_hall_counter_total < 2000) hall_pattern_error_counter++; // for debuging when speed is high enough
    //        SEGGER_RTT_printf(0, "error %u\r\n", hall_pattern_error_counter);
//            ui32_ref_angle = 0 ; // reset the position at pattern 1
            
        } else {   // valid transition
            if (current_hall_pattern ==  0x01) {  // rotor at 210°
                if (ui8_hall_360_ref_valid) { // check that we have a full rotation without pattern sequence error
                    ui16_hall_counter_total = last_hall_pattern_change_ticks - previous_360_ref_ticks; // save the total number of tick for one electric rotation
                    // when speed is still zero, calculate the first value without correction
//                    if (ui32_angle_per_tick_X16shift_new == 0) {
//                        ui32_angle_per_tick_X16shift_new = ( 1 << 24) / (ui16_interval_second_180_ticks << 1);
//                        ui32_ref_angle = 0; // reset the reference on 0 
//                    }
                    // calculate new reference angle for next tour = theoretical position when pattern change occured
                    // it is previous reference angle + enlapsed time between 2 pattern 1 * previous speed
                    // previous reference angle is ui32_angle_pattern_1_x16shift
                    // enlapsed time = ui16_hall_counter_total
                    // previous speed = ui32_angle_per_tick_X16shift
                    // ref angle should ideally be 0 (does not include the real angle of hall at pattern 1)
                    // new ref angle = ref angle * ticks for previous tour * speed used for previous turn
//                    uint32_t ui32_ref_angle_new = ui32_ref_angle + ui16_hall_counter_total * ui32_angle_per_tick_X16shift_new;
                    // when previous ref angle > 180° (1<<23), substract 360° (1<<24) (because e.g. 350° = in fact -10°)
//                    if (ui32_ref_angle > (1<<23)) ui32_ref_angle_new -= (1<<24);
                    // check that new reference is valid (must be between 180 and 270°)
                    // when unvalid, we reset the reference to 0 and calculate the speed based on second 180° tick (so without correction) 
//                    if (( ui32_ref_angle_new < (1<<23)) || ( ui32_ref_angle_new > ((1<<24) + (1<<23)))){
//                        ui32_ref_angle = 0;
//                        ui32_angle_per_tick_X16shift_new = ( 1 << 24) / (ui16_interval_second_180_ticks << 1);
//                    } else {
                        // new speed is calculated based on ((2*360) - ref_angle_new) / "expected tick interval for next 360°"
                        // expected tick interval for next 360° = 2 X second 180 ticks
//                        ui32_angle_per_tick_X16shift_new = ( (1 << 25) - ui32_ref_angle_new ) /  (ui16_interval_second_180_ticks << 1);
                        // for next turn we start with then new ref
//                        ui32_ref_angle = ui32_ref_angle_new ; // used as reference for all following extrapolation.
                        // still when ref_angle exceed 360, we have to substract 360°
//                        if (ui32_ref_angle > (1<<24) ) ui32_ref_angle -= (1<<24);
//                    }
                    // in all cases, set the angle reference in 8 bits for next calculation 
//                    ui8_motor_phase_absolute_angle_new = (uint8_t) (ui32_ref_angle >> 16) ; // reference for new algorithm
                    if (ui16_hall_counter_total > 100 ) { // avoid division by 0 and error in uint if counter would ve to low
                        ui32_angle_per_tick_X16shift = ((uint32_t) ( 1 << 24)) / ui16_hall_counter_total; // new value for interpolation and updating table with reference angle
                        //i32_hall_velocity_q8_8X256 = ui32_angle_per_tick_X16shift; //in 256 * Q8.8 per tick
                        u32_hall_velocity_q8_8X256 = ui32_angle_per_tick_X16shift; //in 256 * Q8.8 per tick
                    } else {
                        ui32_angle_per_tick_X16shift = 0;
                        //i32_hall_velocity_q8_8X256 = 0; 
                        u32_hall_velocity_q8_8X256 = 0; 
                    }
                    //if (i32_hall_velocity_q8_8X256 < 0) u8_debug_negative_velocity = 1;
    
                    ui8_motor_commutation_type = SINEWAVE_INTERPOLATION_60_DEGREES; // 0x80 ; it says that we can interpolate because speed is known
    

                }
                ui8_hall_360_ref_valid = 0x01;
                previous_360_ref_ticks = last_hall_pattern_change_ticks ;    
            } else if (current_hall_pattern ==  0x03) {  // rotor at 150°)
                // update ui8_g_foc_angle once every ERPS (used for g_foc_angle calculation) ;
                // I do not know why this is done when hall pattern = 0X03 and not with 0X01 to avoid a test
                ui8_foc_flag = 1;
                #if (TYPE_OF_FILTER_FOR_CURRENT == (1)) // when we calculate once per rotation and not an a moving avg
                // when we have had at least one full rotation
                if (ui8_motor_commutation_type == SINEWAVE_INTERPOLATION_60_DEGREES){
                    // Calculate avg current per rotation once per rotation
                    ui32_adc_battery_current_1_rotation_15b = ui32_adc_battery_current_15b_accum / ui32_adc_battery_current_15b_counter;
                    ui32_adc_battery_current_15b_accum =0;
                    ui32_adc_battery_current_15b_counter = 0;
                }
                #endif     
            }    
//            new_hall_pattern = current_hall_pattern;
            /*
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
                    // apply filter on values X16bits and update best_ref_angles
                //    best_ref_angles_X16bits[current_hall_pattern] = filtering_function((uint32_t)ui16_measured_angle_X16bits , best_ref_angles_X16bits[current_hall_pattern] ,7 );
                //    ui8_best_ref_angles[current_hall_pattern] = (best_ref_angles_X16bits[current_hall_pattern] + 128 )>> 8; // +128 for rounding
                    
                    uint32_t filtering = best_ref_angles_X16bits[current_hall_pattern];
                    #define FILTER_HALL_POSITIONS 3
                    filtering = ((filtering << FILTER_HALL_POSITIONS) - filtering + (uint32_t) ui16_measured_angle_X16bits) >> FILTER_HALL_POSITIONS ;
                    best_ref_angles_X16bits[current_hall_pattern] = filtering ; // apply new filtered value
                    ui8_best_ref_angles[current_hall_pattern] = (filtering + 128 )>> 8; // +128 for rounding
                    
                    // update the table with reference angles
                    //ui8_hall_ref_angles[current_hall_pattern] = ui8_best_ref_angles[current_hall_pattern] ;
                    hall_ref_angles_counter++;  // just to debug to see if table is/should updated at regular intervals
                }
            } 
            */
            /*
            if (current_hall_pattern == 1 ){
                ui16_hall_counter_total_previous = ui16_hall_counter_total; // save previous counter (to check if erps is stable)
            }
            */
           
           debug_diff_interval = abs( ui16_hall_counter_total - debug_prev_front_interval);
           debug_prev_front_interval =  ui16_hall_counter_total;
           if (ui16_hall_counter_total<2000){
                if (debug_diff_interval_max < debug_diff_interval) {
                    debug_diff_interval_max = debug_diff_interval;
                    ui16_hall_counter_total_max = ui16_hall_counter_total;
                    debug_prev_front_interval_max = debug_prev_front_interval;
                }    
                if (debug_diff_interval > 200) {
                    debug_diff_interval_cnt++;
                    // copy to buffer to display
                    elapse_buffer_bu[0] = elapse_buffer[0];  
                    elapse_buffer_bu[1] = elapse_buffer[1];  
                    elapse_buffer_bu[2] = elapse_buffer[2];  
                    elapse_buffer_bu[3] = elapse_buffer[3];  
                    elapse_buffer_bu[4] = elapse_buffer[4];  
                    elapse_buffer_bu[5] = elapse_buffer[5];  
                    elapse_buffer_bu[6] = elapse_buffer[6];  
                    elapse_buffer_bu[7] = elapse_buffer[7];  
                    elapse_buffer_bu[8] = elapse_buffer[8];  
                    elapse_buffer_bu[9] = elapse_buffer[9];  
                    elapse_buffer_bu[10] = elapse_buffer[10];  
                    elapse_buffer_bu[11] = elapse_buffer[11];  
                    elapse_buffer_bu[11] = last_hall_pattern_change_ticks - previous_hall_pattern_change_ticks;          
                }    
           }
        }
        previous_hall_pattern = current_hall_pattern; // saved to detect future change and check for valid transition
        // set rotor angle based on best ref angles
        //ui8_motor_phase_absolute_angle = ui8_best_ref_angles[current_hall_pattern]; // use best ref instead of ui8_hall_ref_angles[]
        // set rotor angle based on fixed hall ref angles
        ui8_motor_phase_absolute_angle = ui8_hall_ref_angles[current_hall_pattern]; // use  hall_ref_angles[]
        #if (USE_INT_LUT)
        ui16_motor_phase_absolute_angle_q8_8 = u16_hall_angle_table_Q8_8[current_hall_pattern]; 
        #endif
        #if ( GENERATE_DATA_FOR_REGRESSION_ANGLES == (1) )
        if ((ticks_intervals_status == 0) && (current_hall_pattern == 1) ) {
            ticks_intervals[0] = ui16_hall_counter_total ; // save the total ticks for previous 360°
            ticks_intervals[1] = last_hall_pattern_change_ticks - previous_hall_pattern_change_ticks ;
            ticks_intervals[7] = ui8_g_duty_cycle; // save the duty cycle
            ticks_intervals_status = 1;
        } else if (ticks_intervals_status == 1)  {
            ticks_intervals[current_hall_pattern] = last_hall_pattern_change_ticks - previous_hall_pattern_change_ticks ;
            if (current_hall_pattern == 5) {
                ticks_intervals_status = 2; // stop filling the intervals (wait the values are sent)
            }    
        } 
        previous_hall_pattern_change_ticks = last_hall_pattern_change_ticks;
        #endif
        elapse_buffer[elapse_buffer_idx] = last_hall_pattern_change_ticks - previous_hall_pattern_change_ticks ;
        previous_hall_pattern_change_ticks = last_hall_pattern_change_ticks; 
        elapse_buffer_idx++;
        if (elapse_buffer_idx >= 12) elapse_buffer_idx = 0;


    } else { // no hall patern change
        // Verify if rotor stopped (< 10 ERPS)
        if (enlapsed_time > (HALL_COUNTER_FREQ/MOTOR_ROTOR_INTERPOLATION_MIN_ERPS/6)) { //  for TSDZ2: 250000/10 /6 = 4166 ; for TSDZ8 = 8332
            // value must be choosen also to avoid that number of ticks on 360° exceeds uint16 max 
            ui8_motor_commutation_type = BLOCK_COMMUTATION; // 0
            ui8_g_foc_angle = 0;
            ui8_hall_360_ref_valid = 0;
            ui32_angle_per_tick_X16shift = 0; // 0 means unvalid value
            #if (USE_INT_LUT == (1))
            //i32_hall_velocity_q8_8X256 = 0;  
            u32_hall_velocity_q8_8X256 = 0;  
            #endif
            ui16_hall_counter_total = 0xffff;
            #if (TYPE_OF_FILTER_FOR_CURRENT == (1))
            // when we can not use accumulated value, use the latest on and reset accum
            ui32_adc_battery_current_1_rotation_15b = ui32_adc_battery_current_15b;
            ui32_adc_battery_current_15b_accum = 0;
            ui32_adc_battery_current_15b_counter = 0; 
            #endif
        }
    }

    // mstrens : moved from irq1 to irq0 to use average current over 1 rotation for regulation
    #if (TYPE_OF_FILTER_FOR_CURRENT == (0))
    if (ui32_adc_battery_current_15b_moving_average > (255 << 5)) { // clamp for safety
        ui32_adc_battery_current_15b_moving_average = 255 << 5;
    }    
    ui8_adc_battery_current_filtered = ui32_adc_battery_current_15b_moving_average  >> 5;
    #else
    ui8_adc_battery_current_filtered  = ui32_adc_battery_current_1_rotation_15b >> 5 ; // from 15 bits to 10 bits like TSDZ2 
    #endif
    /****************************************************************************/
    // - calculate interpolation angle and sine wave table index when speed is known
    ui8_interpolation_angle = 0; // interpolation angle
    uint32_t compensated_enlapsed_time = 0; 
    
    int32_t i32_interpolation_angle_q8_8 = 0;
    uint32_t u32_interpolation_angle_q8_8 = 0;
    

    if (ui8_motor_commutation_type != BLOCK_COMMUTATION) {  // as long as hall patern are OK and motor is running
        // hall counter offset take care of the delay between entering this ISR and applying the new PWM and also the delay of the hall sensors      
        //compensated_enlapsed_time = enlapsed_time + ui8_fw_hall_counter_offset + ui8_hall_counter_offset;
        compensated_enlapsed_time = enlapsed_time ; // to debug difference with q8_8
        
        // convert time tick to angle (256 = 360°)
           //ui8_interpolation_angle = (((uint32_t) compensated_enlapsed_time) << 8) /  ui16_hall_counter_total; // <<8 = 256 = 360 electric angle
        // convert time tick to angle (256 = 360°) using the already calculated angle per tick (avoid a division)
        // add 1<<15 for better rounding
        ui8_interpolation_angle = ((((uint32_t) compensated_enlapsed_time) *  ui32_angle_per_tick_X16shift) + 0 )>> 16 ; 
        //if (ui8_interpolation_angle > 90){  // added by mstrens because interpolation should not exceed 60°
        //    ui8_interpolation_angle = 21; // 21 is about 30° so mid position between 2 hall pattern changes
        //}
//        compensated_enlapsed_time_new = elapsed_ticks_since_pattern_1 + ui8_fw_hall_counter_offset + ui8_hall_counter_offset;
//        ui8_interpolation_angle_new = (((uint32_t) compensated_enlapsed_time_new) *  ui32_angle_per_tick_X16shift_new) >> 16 ; 
        #if (USE_INT_LUT == (1))
        // update hall rotor position based only on hall 
        //i32_interpolation_angle_q8_8 = ((int32_t)i32_hall_velocity_q8_8X256 * (int32_t)enlapsed_time) >> 8;
        u32_interpolation_angle_q8_8 = ( (((uint32_t) enlapsed_time) *  (uint32_t)u32_hall_velocity_q8_8X256 ) + 0) >> 8;
        
        // Saturation à ±60°
        //if (i32_interpolation_angle_q8_8 > HALL_INTERP_MAX_DELTA_Q8_8)  i32_interpolation_angle_q8_8 = HALL_INTERP_MAX_DELTA_Q8_8;
        
        //if (i32_interpolation_angle_q8_8 < 0) ui8_error_interpolation_q8_8 = 1;
        #endif
    }
    

    // ------------ Calculate the rotor angle and use it as index in the table----------------- 
    // hall_reference_angle is set on 66 based on tests with my motor. (note : 64 = 90°)
    ui8_angle_for_id = ui8_interpolation_angle + ui8_motor_phase_absolute_angle + hall_reference_angle + FINE_TUNE_ANGLE_OFFSET ;
    uint8_t ui8_svm_table_index = ui8_angle_for_id + ui8_g_foc_angle; // add lead angle (that is updated by a PID at 100hz)

    

    #if ( USE_INT_LUT == (1) )
    //ui16_angle_for_id_q8_8 = (uint16_t)i32_interpolation_angle_q8_8;
    ui16_angle_for_id_q8_8 = (uint16_t)u32_interpolation_angle_q8_8;
    ui16_angle_for_id_q8_8 += (uint16_t) ui16_motor_phase_absolute_angle_q8_8;
    ui16_angle_for_id_q8_8 += (uint16_t) hall_reference_angle << 8;

    uint16_t u16_SVM_table_index_q8_8 = ui16_angle_for_id_q8_8 + (uint16_t)(ui8_g_foc_angle<<8);

    //utiliser ui16__motor_phase_absolute_angle_q8_8
    // use integer LUT table 
    uint8_t u8_lut_index = (uint8_t)(u16_SVM_table_index_q8_8 >> 8);

    diff_lut_index =  (int16_t) u8_lut_index - (int16_t) ui8_svm_table_index; 
    ui8_signed_index_debug = u8_lut_index;
    ui8_unsigned_index_debug = ui8_svm_table_index;
    //diff_interpol = (int16_t)(((uint16_t)i32_interpolation_angle_q8_8) >> 8) - (int16_t)ui8_interpolation_angle ;
    diff_interpol = (int16_t)(((int16_t)u32_interpolation_angle_q8_8) >> 8) - (int16_t)ui8_interpolation_angle ;
    diff_abs_pos =  (int16_t) (ui16_motor_phase_absolute_angle_q8_8 >> 8) - (int16_t) ui8_motor_phase_absolute_angle;
    if (velocity_max < u32_hall_velocity_q8_8X256) velocity_max = u32_hall_velocity_q8_8X256;
    enlapsed_debug = enlapsed_time ;
    //uint8_t u8_lut_index = (uint8_t) (u16_SVM_table_index_q8_8 >> 8);
    uint8_t u8_lut_index_A = (u8_lut_index + 171) & 0xFF; // -120° = 256*2/3 ≈ 171
    uint8_t u8_lut_index_B = u8_lut_index ;
    uint8_t u8_lut_index_C = (u8_lut_index + 85) & 0xFF; // + 120°

    int16_t svm_A = i16_LUT_SINUS[u8_lut_index_A];
    int16_t svm_B = i16_LUT_SINUS[u8_lut_index_B];
    int16_t svm_C = i16_LUT_SINUS[u8_lut_index_C];
    
    ui16_a = (uint16_t) (MIDDLE_SVM_TABLE + (( svm_A * (int16_t) ui8_g_duty_cycle)>>8)); // >>8 because duty_cycle 100% is 256
    ui16_b = (uint16_t) (MIDDLE_SVM_TABLE + (( svm_B * (int16_t) ui8_g_duty_cycle)>>8)); // >>8 because duty_cycle 100% is 256
    ui16_c = (uint16_t) (MIDDLE_SVM_TABLE + (( svm_C * (int16_t) ui8_g_duty_cycle)>>8)); // >>8 because duty_cycle 100% is 256  
 
    #else
    
    // to debug
//    #else
//    uint8_t ui8_svm_table_index = ui8_interpolation_angle_new + (ui32_ref_angle >> 16) +  ui8_best_ref_angles[1] + 
//        ui8_g_foc_angle + hall_reference_angle ;
    
//    #endif
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
    #endif

    #define DEBUG_IRQO_TIME (0) // 1 = calculate the time spent in irq0
    #if (DEBUG_IRQO_TIME == (1))
    uint16_t temp  = XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW) ;
    temp = temp - current_speed_timer_ticks;
    if (irq0_min > temp) irq0_min = temp; // store the in enlapsed time in the irq
    if (irq0_max < temp) irq0_max = temp; // store the max enlapsed time in the irq
    #endif


    // get the voltage ; done in irq0 because it is used in irq1 and irq0 takes less time
        //ui16_adc_voltage  = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 4 ) & 0x0FFF) >> 2; // battery gr1 ch6 result 4
    // changed to take care of infineon VADC init (result in reg 6)
    ui16_adc_voltage  = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , VADC_VDC_RESULT_REG ) & 0x0FFF) >> 2; // battery gr1 ch6 result 6
          
    
    #if (uCPROBE_GUI_OSCILLOSCOPE == MY_ENABLED)
    //I_u = XMC_VADC_GROUP_GetResult(VADC_I1_GROUP , VADC_I1_RESULT_REG ) & 0x0FFF;
    //I_w = XMC_VADC_GROUP_GetResult(VADC_I3_GROUP , VADC_I3_RESULT_REG ) & 0x0FFF;
    //I_v = XMC_VADC_GROUP_GetResult(VADC_I2_GROUP , VADC_I2_RESULT_REG ) & 0x0FFF;
    
    // ProbeScope_Sampling must be called to update data displayed on PC in graph
    // if we do not require a high refresh rate, this could be set in another loop 
    ProbeScope_Sampling(); // this is here in a interrupt that run fast
    #endif



} // end of CCU80_0_IRQHandler

#define DEBUG_IRQ1_TIME (0) // 1 = calculate time spent in irq1
// ************* irq handler 
__RAM_FUNC void CCU80_1_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting DOWN (= 1/4 of 19mhz cycles)    
//void CCU80_1_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting DOWN (= 1/4 of 19mhz cycles)    
    #if (DEBUG_IRQ1_TIME == (1))
    // to debug max time in this iSR
    uint16_t start_ticks  =  XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
    #endif

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
    // update of PWM will occur later on when timer reach O match 
    
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
        
        #if (DYNAMIC_LEAD_ANGLE == (1))
        // read current iu,iv,iw and start calculating Id with cordic (result will be get at the end of ISR 1 to avoid wait time)       
        calculate_id_part1();
        #endif

        // update foc_angle once per electric rotation (based on fog_flag
        // foc_angle is added to the position given by hall sensor + interpolation )
        if (ui8_g_duty_cycle > 0) {
            // calculate phase current.
            if (ui8_g_duty_cycle > 10) {
                ui16_adc_motor_phase_current = (uint16_t)((uint16_t)(((uint16_t)ui8_adc_battery_current_filtered) << 8)) / ui8_g_duty_cycle;
            } else {
                ui16_adc_motor_phase_current = (uint16_t)ui8_adc_battery_current_filtered;
            }
            if (ui8_foc_flag) { // is set on 1 when rotor is at 150° so once per electric rotation
				//uint16_t ui16_adc_foc_angle_current = ((uint16_t)(ui8_adc_battery_current_filtered ) + (ui16_adc_motor_phase_current )) >> 1;
                // mstrens : added 128 for better rounding
                //ui8_foc_flag = ((ui16_adc_foc_angle_current * ui8_foc_angle_multiplicator) + 128) >> 8 ; // multiplier = 39 for 48V tsdz2, 
                ui8_foc_flag = (((uint16_t) ui8_adc_battery_current_filtered * (uint16_t) ui8_foc_angle_multiplicator) + 128) >> 8 ; // multiplier = 39 for 48V tsdz2, 
                // max = 23 *100 / 16 * 40 = 22
                if (ui8_foc_flag > 25)
                    ui8_foc_flag = 25;
                // removed by mstrens because current is already based on an average on 1 rotation
                //ui8_foc_angle_accumulated = ui8_foc_angle_accumulated - (ui8_foc_angle_accumulated >> 4) + ui8_foc_flag;
                //ui8_g_foc_angle = ui8_foc_angle_accumulated >> 4;
                //ui8_foc_flag = 0;
                // added by mstrens
                ui8_g_foc_angle = ui8_foc_flag ;
            }
        } else { // duty cycle = 0
            ui16_adc_motor_phase_current = 0;
            // removed by mstrens
            //if (ui8_foc_flag) {
            //    ui8_foc_angle_accumulated = ui8_foc_angle_accumulated - (ui8_foc_angle_accumulated >> 4);
            //    ui8_g_foc_angle = ui8_foc_angle_accumulated >> 4;
            //    ui8_foc_flag = 0;
            //}
            // added by mstrens
            ui8_g_foc_angle = 0; 
            
        }
        ui8_foc_flag = 0;

        // get brake state-
        ui8_brake_state = XMC_GPIO_GetInput(IN_BRAKE_PORT, IN_BRAKE_PIN) == 0; // Low level means that brake is on
        
        // added by mstrens to detect overcurrent and to decrase immediatelu the duty cycle
        //uint8_t ui8_temp_adc_current = ((XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 15 ) & 0xFFFF) +
	    //								(XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 15 ) & 0xFFFF)) >>5  ;  // >>2 for IIR, >>2 for ADC12 to ADC10 , >>1 for averaging		
	    // changed by mstrens to take care of infineon init for vadc (result 12bits and in reg 1)
	    uint8_t ui8_temp_adc_current = (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , VADC_I4_RESULT_REG ) & 0xFFFF) >> 2;// from 12 to 10bits 
	    if ( ui8_temp_adc_current > ui8_adc_battery_overcurrent){ // 112+50 in tsdz2 (*0,16A) => 26A
            ui8_g_duty_cycle -= (ui8_g_duty_cycle >> 2); // reduce immediately dutycycle by 25% to avoid overcurrent in next pwm 
        }    
		

    // to debug
    //uint16_t temp1d  =  XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
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
        // ui16_adc_motor_phase_current_max = 135 per default for TSDZ2 (13A *100/16) *187/112 = battery_current convert to ADC10bits *and ratio between adc max for phase and for battery
        //        is initiaded in void ebike_app_init(void) in ebyke_app.c
        
        
        // every 25ms ebike_app_controller fills
        //  - ui8_controller_adc_battery_current_target
        //  - ui8_controller_duty_cycle_target // is usually filled with 255 (= 100%)
        //  - ui8_controller_duty_cycle_ramp_up_inverse_step
        //  - ui8_controller_duty_cycle_ramp_down_inverse_step
        // Furthermore,  when ebyke_app_controller start pwm, g_duty_cycle is first set on 30 (= 12%)
        if ((ui8_controller_duty_cycle_target < ui8_g_duty_cycle)                     // requested duty cycle is lower than actual
          || (ui8_controller_adc_battery_current_target < ui8_adc_battery_current_filtered)  // requested current is lower than actual
		  || (ui16_adc_motor_phase_current >  ui16_adc_motor_phase_current_max)               // motor phase is to high
//          || (ui16_hall_counter_total < (HALL_COUNTER_FREQ / MOTOR_OVER_SPEED_ERPS))        // Erps is to high
          || (ui16_adc_voltage < ui16_adc_voltage_cut_off)                                  // voltage is to low
          || (ui8_brake_state)
        ) {                                                           // brake is ON
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
                if (ui8_g_duty_cycle < PWM_DUTY_CYCLE_STARTUP) {
                    ui8_g_duty_cycle = PWM_DUTY_CYCLE_STARTUP;
                }	
                else if (ui8_g_duty_cycle < PWM_DUTY_CYCLE_MAX) {
                    if (ui8_g_duty_cycle < PWM_DUTY_CYCLE_MAX) {
                        ui8_g_duty_cycle++;
                    }
                }    
            }
        }
		else if ((ui8_field_weakening_enabled)
				&& (ui8_g_duty_cycle == PWM_DUTY_CYCLE_MAX)) {
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
    //uint16_t temp1e  =  XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
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
                            ++ui32_wheel_speed_sensor_ticks_total; // used only in 860C version
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

        // added by mstrens
        // get raw adc torque sensor (in 10 bits) 
        ui16_adc_torque   = (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , VADC_TORQUE_RESULT_REG ) & 0xFFF) >> 2; // torque gr0 ch7 result 7 in bg p2.2
        //filter it (3 X previous + 1 X new)
        uint16_t ui16_adc_torque_new_filtered = ( ui16_adc_torque + (ui16_adc_torque_filtered<<1) + ui16_adc_torque_filtered) >> 2;
        if (ui16_adc_torque_new_filtered == ui16_adc_torque_filtered){ // code to ensure it reaches the limits
            if ( ui16_adc_torque_new_filtered < ui16_adc_torque) 
                ui16_adc_torque_new_filtered++; 
            else if (ui16_adc_torque_new_filtered > ui16_adc_torque) 
                ui16_adc_torque_new_filtered--;
        }
        ui16_adc_torque_filtered = ui16_adc_torque_new_filtered;
        
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
                ui8_pas_new_transition = 0x80; // used in mspider logic for torque sensor
                goto skip_cadence;
            }
			ui16_cadence_sensor_ticks_counter_min = ui16_cadence_ticks_count_min_speed_adj; // 4270 at 4km/h ... 341 at 40 km/h
            if (ui8_temp_cadence == ui8_cadence_calc_ref_state) { // pattern is valid and represent 1 tour
                ui8_pas_new_transition = 1; // mspider logic for torque sensor;mark for one of the 20 transitions per rotation
            
                // ui16_cadence_calc_counter is valid for cadence calculation
                ui16_cadence_sensor_ticks = ui16_cadence_calc_counter; // use the counter as cadence for ebike_app.c
                ui16_cadence_calc_counter = 0;
                // software based Schmitt trigger to stop motor jitter when at resolution limits
                ui16_cadence_sensor_ticks_counter_min += CADENCE_SENSOR_STANDARD_MODE_SCHMITT_TRIGGER_THRESHOLD; // 427 at 19 khz
                ui8_pas_counter++; // mstrens : increment the counter when the transition is valid
            } else if (ui8_cadence_calc_ref_state == NO_PAS_REF) {  // 5
                // this is the new reference state for cadence calculation
                ui8_cadence_calc_ref_state = ui8_temp_cadence;
                ui16_cadence_calc_counter = 0;
                ui8_pas_counter = 0; // mstrens :  reset the counter for full rotation
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
            ui8_pas_new_transition = 0x80; // for mspider logic for torque sensor
            ui8_pas_counter = 0; // mstrens :  reset the counter for full rotation
        } else if (ui8_cadence_calc_ref_state != NO_PAS_REF) { // 5
            // increment cadence tick counter
            ++ui16_cadence_calc_counter;
        }
        // end cadence

        // original perform also a save of some parameters (battery consumption) // to do 
    #if (DEBUG_IRQ1_TIME == (1))
    uint16_t temp1  =  XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
    temp1 = temp1 - start_ticks;
    if (irq1_min > temp1) irq1_min = temp1; // store the min enlapsed time in the irq
    if (irq1_max < temp1) irq1_max = temp1; // store the min enlapsed time in the irq
    #endif
    
    // added by mstrens to calculate torque sensor without cyclic effect using the max per current and previous rotation
    // note this code is used only when we do not use SPIDER or katana(1or 2) logic.
    // So, it could probably be removed as this logic does not seems the best one.
    // we have several data
    // ui16_adc_torque_filtered is the actual filtered ADC torque
    // ui16_adc_torque_actual_rotation is the max during current rotation
    // ui16_adc_torque_previous_rotation is the max during previous rotation
    // ui8_pas_counter count the number of transition to detect a 360° pedal rotation

    // first reset the values per rotation when requested by ebike_app.c (because cadence is lower than a threshold)
    if (ui8_adc_torque_rotation_reset) {
        ui8_adc_torque_rotation_reset = 0; //reset the flag
        ui16_adc_torque_actual_rotation = 0;  
        ui16_adc_torque_previous_rotation = 0;
        ui8_pas_counter = 0; // reset the counter also
    }
    if (ui16_cadence_sensor_ticks > 0) { // when we have a cadence, we update data over rotation
        // actual_rotation is the max
        if (ui16_adc_torque_actual_rotation < ui16_adc_torque_filtered) ui16_adc_torque_actual_rotation = ui16_adc_torque_filtered;
        if (ui8_pas_counter >= 20) { // if we have had a full rotation
            ui8_pas_counter = 0; // reset the counter
            ui16_adc_torque_previous_rotation = ui16_adc_torque_actual_rotation;  // save the actual rotation value
            ui16_adc_torque_actual_rotation =  0; // reset the actual rotation
        }
    } else {
        ui8_pas_counter = 0 ;
        ui16_adc_torque_previous_rotation = 0;
        ui16_adc_torque_actual_rotation = 0;
    }
    
    #if (DYNAMIC_LEAD_ANGLE == (1))
    // update data to get an avg of Id
    calculate_id_part2();
    #endif
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

void get_hall_pattern(){  // use to initialise at power on and in motor_enable()
    uint32_t critical_section_value = XMC_EnterCriticalSection();
    current_hall_pattern = XMC_GPIO_GetInput(IN_HALL0_PORT, IN_HALL0_PIN);// hall 0
    current_hall_pattern |=  XMC_GPIO_GetInput(IN_HALL1_PORT, IN_HALL1_PIN) << 1;
    current_hall_pattern |=  XMC_GPIO_GetInput(IN_HALL2_PORT, IN_HALL2_PIN) << 2;
    XMC_ExitCriticalSection(critical_section_value);
}

#if (DYNAMIC_LEAD_ANGLE == (1))
// +++++++++++++++++ from here the code to apply a pid+optimiser for lead angle using id +++++++++++++++++++++

//int32_t apply_PID_on_lead_angle(int32_t Id_filt,int32_t q31_lead_angle);
void apply_PID_on_lead_angle(); //prototype

#define SHIFT_BIAS_LPF 3
void update_foc_pid() { // this is called from main() every 10 msec, it supposes that Id is calculated and filtered in ISR
    // when motor is blocked since some time, we update first the ADC bias for Iu, iv, iW
    // when motor is not running (based on ui8_motor_enabled) we reset foc and foc PID
    // when motor is running we use a PI based on ID (calculated and filtered in ISR) to update FOC angle
    // in a second step we can calculate a value for foc angle based on rpm and current and apply pid as a correction.

    // first when motor is not running, update adc bias
    if (ui8_motor_enabled == 0) {
        //	/* Init ADC bias */
        // for THREE_SHUNT_SYNC_CONV)
        uint16_t Iu;
        uint16_t Iv;
        uint16_t Iw;

        Iu = XMC_VADC_GROUP_GetResult(VADC_I1_GROUP , VADC_I1_RESULT_REG ) & 0x0FFF;
        Iw = XMC_VADC_GROUP_GetResult(VADC_I3_GROUP , VADC_I3_RESULT_REG ) & 0x0FFF;
        Iv = XMC_VADC_GROUP_GetResult(VADC_I2_GROUP , VADC_I2_RESULT_REG ) & 0x0FFF;
               /* Read Iu ADC bias */
        ADC_Bias_Iu = (uint32_t) ((ADC_Bias_Iu * (((uint32_t) 1 << SHIFT_BIAS_LPF) - 1U)) + Iu) >> SHIFT_BIAS_LPF;
        /* Read Iv ADC bias */
        ADC_Bias_Iv = (uint32_t) ((ADC_Bias_Iv * (((uint32_t) 1 << SHIFT_BIAS_LPF) - 1U)) + Iv) >> SHIFT_BIAS_LPF;
        /* Read Iw ADC bias */
        ADC_Bias_Iw = (uint32_t) ((ADC_Bias_Iw * (((uint32_t) 1 << SHIFT_BIAS_LPF) - 1U)) + Iw) >> SHIFT_BIAS_LPF;

        // reset lead angle to 0 and integral term of pid
        q31_lead_angle = 0; 
        foc_pid_I_term = 0;
    }
    else {
        // apply PI on id
        //q31_lead_angle = apply_PID_on_lead_angle(i32_id_filtr , q31_lead_angle);
        //q31_lead_angle = apply_PID_on_lead_angle();
        apply_PID_on_lead_angle();
    }
}    

// ---------------------- CONFIG for PID and optimiser----------------------
#define PWM_FREQ        19000      // Hz
#define PID_FREQ        100        // Hz (10 ms)
#define OPTIM_FREQ      5          // Hz (200 ms)

#define SAMPLES_PER_PID (PWM_FREQ / PID_FREQ)  // ~190
#define PID_PERIOD_MS   (1000 / PID_FREQ)      // 10 ms
#define OPTIM_PERIOD_MS (1000 / OPTIM_FREQ)    // 200 ms
// Q16 : 1 tour = 360° = 65536 unités
// we use a convention Q16 -180°/180°, So 16 bits = 360° 
#define Q16_ONE         (1 << 16)

// bornes lead angle en Q16 (signé)
#define LEAD_MIN_Q16   ( (-(15) * Q16_ONE) / 360 )    // -15°
#define LEAD_MAX_Q16   ( ((30) * Q16_ONE) / 360 )     // +30°
#define PID_MAX_Q16    ( ((5)  * Q16_ONE) / 360 )     // 5°
#define LEAD_STEP_Q16  ( ((2)  * Q16_ONE) / 360 /10 )    // 0.2° = 2/3600 of full turn

// buffer optimiser
#define OPTIM_BUF_LEN 20                 // 20 échantillons = 200 ms

// slew rate pour lead_angle_final
#define MAX_FINAL_STEP_Q16  ( ((5) * Q16_ONE) / 3600 )  // 0.05° par step (~10ms)


// ============================================================================
// VARIABLES GLOBALES
// ============================================================================

// intégrateur PID (int64 pour éviter overflow)
// unité : mA·s approximatif
static int64_t pid_integrator = 0;

// composantes lead angle (Q16 signé, -180°..+180° environ)
static int32_t lead_angle_pid   = 0;
static int32_t lead_angle_optim = 0;
static int32_t lead_angle_final = 0;  // utilisé par la génération PWM
static uint8_t lead_angle_LUT_256 = 0;     // to read a LUT of 256 items (0 360°)

// optimiser
static int optim_dir = 1;                // direction hill-climbing
static int32_t last_Id_avg = 0;          // in mA
static int32_t optim_buffer[OPTIM_BUF_LEN];
static int optim_index = 0;
static int optim_count = 0;

// ============================================================================
// CONSTANTES PID
// ============================================================================
// Kp choisi : 10 A → 5°
// 5° = 5 * 65536 / 360 ≈ 910 units
// real gain = 910 / 10000 = 0.091
// KP_Q16 = 0.091 * 65536 ≈ 5964
// => kp ≈ 0.091 → Q16 = 5964
const int32_t KP_Q16 = 5964;

// Ki ≈ Kp / 10
const int32_t KI_Q16 = 600;

// période d’échantillonnage en Q16
#define DT_Q16  ((int32_t)((((int64_t)Q16_ONE) + (PID_FREQ/2)) / PID_FREQ))

// limit for intégrator (in 64 bits)
static const int64_t INTEGRATOR_MAX =
    (((int64_t)PID_MAX_Q16 << 16) / (KI_Q16 > 0 ? KI_Q16 : 1));

// ---------------------- UTILS ----------------------

// clamp entier Q16
static inline int32_t clamp_q16(int32_t x, int32_t xmin, int32_t xmax) {
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

// valeur absolue int32
static inline int32_t abs32(int32_t x) { return x < 0 ? -x : x; }

// ---------------------- PID UPDATE       (100 Hz)              ----------------------
// Entrées externes (mises à jour à 19 kHz par ISR) :
//   - i32_id_pid_acc (accumulateur Id ADC)
//   - i32_id_pid_cnt (nb d’échantillons)

// ++++++++ lead angle is supposed to be Q16 (0 1 for 0 - 360°)
void apply_PID_on_lead_angle(void) { // return the new lead angle
    
    // Save state, disable irq
    uint32_t prim = __get_PRIMASK();   // sauvegarde l'état des interruptions
    __disable_irq();                   // bloque toutes les IRQ (PRIMASK = 1)

    int32_t i32_acc = i32_id_pid_acc ;  // get accumulator and cnt
    int32_t i32_cnt = i32_id_pid_cnt ; 
    i32_id_pid_acc = 0; // Reset accumulator
    i32_id_pid_cnt = 0;

    // restaure irq
    __set_PRIMASK(prim); 

    // Moyenne bloc en Q15
    // 1 step ADC10 = 0,16A
    // 1 step ADC15 = 0,16A / 32 = 0,005 A = 5 mA
    // to get in mA, we multiply by 5.
    if (i32_cnt == 0) return;
    // convert to mA :  1 LSB ADC15 = 5 mA
    int32_t Id_avg = (int32_t)(((int64_t)i32_acc * 5) / i32_cnt);  //Id_avg in mA

    // --- PID ---
    int32_t error = -Id_avg;  // objectif Id=0mA
    
    // proportional (Q16)
    int64_t P_tmp = (int64_t)KP_Q16 * (int64_t)error;
    int32_t P_q16 = (int32_t)(P_tmp >> 16);

// anti-windup optionnel : n'intégrer que si on n'est pas saturé dans le sens erreur
    // anti-windup simple : si PID saturé et erreur renforce la saturation, skip integrate
    int64_t tentative_full = (int64_t)P_q16 + (((int64_t)KI_Q16 * pid_integrator) >> 16);
    if (!((tentative_full > PID_MAX_Q16 && error > 0) ||
          (tentative_full < -PID_MAX_Q16 && error < 0))) {
        // safe to integrate
        pid_integrator += ((int64_t)error * (int64_t)DT_Q16) >> 16;
    }

    if (pid_integrator > INTEGRATOR_MAX)  pid_integrator = INTEGRATOR_MAX;
    if (pid_integrator < -INTEGRATOR_MAX) pid_integrator = -INTEGRATOR_MAX;

    // I term (Q16)
    int64_t I_tmp = (int64_t)KI_Q16 * pid_integrator;
    int32_t I_q16 = (int32_t)(I_tmp >> 16);
    
    // sum and saturation
    int64_t out_tmp = (int64_t)P_q16 + (int64_t)I_q16;
    if (out_tmp > PID_MAX_Q16)      lead_angle_pid = PID_MAX_Q16;
    else if (out_tmp < -PID_MAX_Q16) lead_angle_pid = -PID_MAX_Q16;
    else                             lead_angle_pid = (int32_t)out_tmp;
    
    // --- buffer optimiser---
    optim_buffer[optim_index] = abs32(Id_avg);
    optim_index = (optim_index + 1) % OPTIM_BUF_LEN;
    if (optim_count < OPTIM_BUF_LEN) optim_count++;

    // --- Lead angle final ---
    int32_t tmp = lead_angle_pid + lead_angle_optim;
    tmp = clamp_q16(tmp, LEAD_MIN_Q16, LEAD_MAX_Q16);

    // ---------------------- SLEW RATE ----------------------
    int32_t delta = tmp - lead_angle_final;
    if (delta > MAX_FINAL_STEP_Q16) delta = MAX_FINAL_STEP_Q16;
    else if (delta < -MAX_FINAL_STEP_Q16) delta = -MAX_FINAL_STEP_Q16;
    lead_angle_final += delta;

    uint16_t pwm_angle16;
    if (lead_angle_final < 0) pwm_angle16 = (uint16_t)(lead_angle_final + Q16_ONE);
    else                      pwm_angle16 = (uint16_t)lead_angle_final;

    // Ré-échantillonner 65536->256 en conservant la correspondance angulaire (MSB)
    lead_angle_LUT_256 = (uint8_t)(pwm_angle16 >> 8); // correct mapping 0..255
    
    return ; 
}

// ---------------------- OPTIMISER UPDATE (5 Hz) ----------------------
static int32_t Id_filtered = 0;    // filtre low-pass pour Id_avg
static int32_t sigma_filtered = 0;   // sigma filtré
static int32_t step_filtered     = 0;      // step filtré pour lead_angle_optim
#define LPF_ALPHA  4  // 1..255, plus grand = plus lent, valeur typique ~4
void update_foc_optimiser(void) {
    if (optim_count == 0) return;

    // Calcul avg
    int64_t sum = 0;
    for (int i = 0; i < optim_count; i++) sum += optim_buffer[i];
    int32_t avg = (int32_t)(sum / optim_count); // mA

    // filtrage low-pass (exponentiel) : Id_filtered = α*prev + (1-α)*avg
    // approximation entier : Id_filtered = (prev*(255-α) + avg*α)/255
    Id_filtered = ( (Id_filtered*(255-LPF_ALPHA) + avg*LPF_ALPHA) ) / 255;

    // variance (mA^2)
    int64_t var_sum = 0;
    for (int i = 0; i < optim_count; i++) {
        int32_t diff = optim_buffer[i] - avg;
        var_sum += ((int64_t)diff * diff) ;  // >>0 car déjà 64 bits
    }
    int32_t variance = (int32_t)(var_sum / optim_count); // in mA^2

    // --- filtrage low-pass de sigma (écart type) ---
    int32_t sigma = (int32_t)sqrt((double)variance);
    sigma_filtered = (sigma_filtered*(255-LPF_ALPHA) + sigma*LPF_ALPHA)/255;

    // --- seuils adaptatifs en fonction de sigma filtré ---
    int32_t VAR_LOW  = (int32_t)(4 * sigma_filtered * sigma_filtered);   // 2*sigma
    int32_t VAR_HIGH = (int32_t)(36 * sigma_filtered * sigma_filtered);  // 6*sigma

    // Garder des bornes min/max pour éviter extrêmes
    const int32_t VAR_LOW_MIN  = 25;    // équivalent σ≈2.5 mA -> VAR_LOW min
    const int32_t VAR_HIGH_MIN = 400;   // équivalent σ≈20 mA
    if (VAR_LOW < VAR_LOW_MIN) VAR_LOW = VAR_LOW_MIN;
    if (VAR_HIGH < VAR_HIGH_MIN) VAR_HIGH = VAR_HIGH_MIN;

    // Pas adaptatif selon variance
    int32_t step = LEAD_STEP_Q16;
        if (variance < VAR_LOW)       step = LEAD_STEP_Q16;
    else if (variance < VAR_HIGH) step = LEAD_STEP_Q16 / 2;
    else                          step = LEAD_STEP_Q16 / 4;

    // Décision de direction
    if (Id_filtered > last_Id_avg) optim_dir = -optim_dir;

    // --- filtrage du step appliqué ---
    step_filtered = (step_filtered*(255-LPF_ALPHA) + step*LPF_ALPHA)/255;

    // update optimiser
    int64_t new_opt = (int64_t)lead_angle_optim + (int64_t)optim_dir * step_filtered;
    if (new_opt > LEAD_MAX_Q16) new_opt = LEAD_MAX_Q16;
    if (new_opt < LEAD_MIN_Q16) new_opt = LEAD_MIN_Q16;
    lead_angle_optim = (int32_t)new_opt;

    last_Id_avg = Id_filtered;
    // optional: debug logs (décommenter si tu as UART)
    // printf("optim: avg=%d mA var=%d mA2 sigma=%.2f VAR_LOW=%d VAR_HIGH=%d step=%d\n",
    //       avg, variance, sigma, VAR_LOW, VAR_HIGH, step);
}


/*
Valeurs pour PID et optimiser pour adapter lead angle en fonction de Id
This apply to C code here above.


Terme / Variable           Plage réelle           Q16 Value / Calcul                  Commentaire
--------------------------  --------------------  ---------------------------------  -------------------------------------------------
Q16_ONE                     360°                  65536                                1 tour = 2^16 unités Q16
LEAD_MIN_Q16                -15°                  (-15*65536)/360 = -2730             Limite basse lead angle
LEAD_MAX_Q16                +30°                  (30*65536)/360 = 5461               Limite haute lead angle
PID_MAX_Q16                 ±5°                   (5*65536)/360 = 910                  Saturation PID ±5°
LEAD_STEP_Q16               0.2°                  ((2*65536)/360)/10 = 36             Pas optimiseur = 0.2°
KP_Q16                      -                     5964                                  Gain proportionnel PID
KI_Q16                      -                     600                                   Gain intégral PID
DT_Q16                      10 ms                 ((65536 + PID_FREQ/2)/PID_FREQ) ≈ 656  Période d’échantillonnage Q16
INTEGRATOR_MAX              ±910                  ((PID_MAX_Q16 << 16)/KI_Q16) ≈ 99277 Limite intégrateur 64 bits
lead_angle_pid              ±5°                   ±910                                  Sortie PID seule
lead_angle_optim            -15…+30°              -2730…+5461                           Valeur hill-climbing
lead_angle_final            -15…+30°              -2730…+5461                           PID + optimiseur, borné
lead_angle_LUT_256          0…255                 lead_angle_final >> 8                 Pour LUT 256 entrées
i32_id_pid_acc               0…?                  -                                     Accumulateur ADC Id
i32_id_pid_cnt               1…?                  -                                     Nombre d’échantillons ADC
Id_avg                       0…20 A               0…20000                               Courant en mA
error                        -20…+20 A            -20000…+20000                         Objectif Id=0
P_q16                        -2.77…+2.77°         ±910                                  Terme proportionnel PID
pid_integrator               ±5°                   ±99277                               Terme intégral PID
I_q16                        ±5°                   ±910                                  Terme intégral PID appliqué
out_tmp                       ±5°                   ±910                                  Somme P+I, saturée ±PID_MAX_Q16
optim_buffer[]                0…20 A               0…20000                               Buffer pour optimiser Id_abs
Id_filtered                   0…20 A               0…20000                               Low-pass sur Id_avg
variance                       0…?                  0…?                                   Variance brute Id
sigma                          0…?                  0…?                                   Ecart type Id
sigma_filtered                 0…?                  0…?                                   Filtrage low-pass sigma
step                           0…0.2°               0…36                                  Pas PID adaptatif
step_filtered                  0…0.2°               0…36                                  Pas filtré appliqué
optim_dir                      -1 / +1             -1 / +1                               Direction hill-climbing
last_Id_avg                    0…20 A               0…20000                               Dernier Id filtré




Signal / Terme            Plage réelle (° / A)       Q16 value    Commentaire
-------------------------  -------------------------  ---------   -------------------------------------------------
Id_avg                     0…20 A                     0…20000     Courant moyen en mA
error (PID)                -20…+20 A                  -20000…+20000  -Id_avg
P_q16                      -2.77…+2.77°               ±910        Gain proportionnel, saturé ±PID_MAX_Q16
pid_integrator             ±5°                         ±99277      Borné par INTEGRATOR_MAX
I_q16                      ±5°                         ±910        Terme intégral appliqué
out_tmp                     ±5°                        ±910        P+I, saturé ±PID_MAX_Q16
lead_angle_pid             ±5°                         ±910        Sortie PID uniquement
lead_angle_optim           -15…+30°                    -2730…+5461  Optimiseur hill-climbing
lead_angle_final           -15…+30°                    -2730…+5461  PID + optimiseur, borné LEAD_MIN…LEAD_MAX
LEAD_STEP_Q16              0.2°                        36          Pas de l’optimiseur (0.2° = 2/3600 tour)
Id_filtered                0…20 A                      0…20000     Low-pass sur Id_avg
sigma_filtered             0…?                         0…?         Low-pass sur écart type de Id
step_filtered              0…0.2°                       0…36       Low-pass sur step appliqué à lead_angle_optim
lead_angle_LUT_256         0…255                        0…255      Conversion Q16 → 8 bits pour LUT


+-----------------------------------------+----------------+-----------------+-----------------+
| Variable / Terme                         | Plage réelle   | Q16 Value       | Commentaire     |
+-----------------------------------------+----------------+-----------------+-----------------+
| Courant Id                               | 0 … 20 A       | 0 … 20000 mA    | Mesure en mA    |
| Erreur PID                               | -20 … +20 A    | -20000 … +20000 | Objectif Id=0   |
| lead_angle_pid                           | -5° … +5°      | -910 … +910     | Terme proportionnel PID (saturé) |
| pid_integrator                            | ±5°            | ±99277          | Limite intégrateur Q16 |
| I_q16                                    | ±5°            | ±910            | Terme intégral PID appliqué |
| out_tmp                                  | ±5°            | ±910            | Somme P+I, saturée ±PID_MAX_Q16 |
| lead_angle_optim                          | -15° … +30°    | -2730 … +5461   | Valeur hill-climbing |
| lead_angle_final                           | -15° … +30°    | -2730 … +5461   | PID + optimiseur, borné |
| lead_angle_LUT_256                         | 0 … 360°       | 0 … 255         | Pour LUT 256 entrées (MSB Q16) |
| LEAD_STEP_Q16                              | 0.2°           | 36              | Pas optimiseur |
| Id_filtered                                | 0 … 20 A       | 0 … 20000       | Low-pass sur Id_avg |
| variance                                   | 0 … ?          | 0 … ?           | Variance Id (mA^2) |
| sigma                                      | 0 … ?          | 0 … ?           | Ecart-type Id (mA) |
| sigma_filtered                             | 0 … ?          | 0 … ?           | Filtrage low-pass sigma |
| step                                       | 0 … 0.2°       | 0 … 36          | Pas PID adaptatif |
| step_filtered                              | 0 … 0.2°       | 0 … 36          | Pas filtré appliqué |
| optim_dir                                  | -1 / +1       | -1 / +1         | Direction hill-climbing |
| last_Id_avg                                | 0 … 20 A       | 0 … 20000       | Dernier Id filtré |
+-----------------------------------------+----------------+-----------------+-----------------+

===========================================================================
                        SCHÉMA DES PLAGES Q16
===========================================================================

Moteur : courant max 20A
Unités : Q16 (1 tour = 65536 unités)

--------------------------------------------------------------------------
PID
--------------------------------------------------------------------------
KP_Q16         : 5964            // Kp ≈ 0.091 en Q16
KI_Q16         : 600             // Ki ≈ Kp / 10
PID_MAX_Q16    : 910             // 5° max ≈ 5*65536/360
Integrateur    : ±INTEGRATOR_MAX
DT_Q16         : 65536 / 100 ≈ 656

lead_angle_pid : [-PID_MAX_Q16, +PID_MAX_Q16] ≈ [-910, +910]

--------------------------------------------------------------------------
Optimiseur
--------------------------------------------------------------------------
lead_angle_optim : [LEAD_MIN_Q16, LEAD_MAX_Q16]
                  ≈ [-2731, +5461]   // -15° à +30° en Q16

STEP adaptatif (LEAD_STEP_Q16) :
    - Variance faible     : 2/3600 ≈ 364 (0.2°)
    - Variance moyenne    : 0.1° (LEAD_STEP_Q16 /2)
    - Variance élevée     : 0.05° (LEAD_STEP_Q16 /4)
Direction : +1 / -1 selon Id_filtered

--------------------------------------------------------------------------
Lead angle final (avant PWM LUT)
--------------------------------------------------------------------------
lead_angle_final = lead_angle_pid + lead_angle_optim
borne : [LEAD_MIN_Q16, LEAD_MAX_Q16] ≈ [-2731, +5461]

Slew rate : MAX_F_


*/
#endif // #if (DYNAMIC_LEAD_ANGLE == (1))


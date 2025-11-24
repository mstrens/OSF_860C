//branch test 4---



// for cadence, adapt ui16_cadence_sensor_ticks_counter_min in ebike_app.c in order to take care that counter runs at 1kHz instead of 19kHz
//             then remove the division by 19 in motor ; this avoid a division in the ISR
// for cadence activate     ui8_pas_new_transition = 0x80; // used in mspider logic for torque sensor
// for cadence activate     ui8_pas_new_transition = 1; // mspider logic for torque sensor;mark for one of the 20 transitions per rotation
// !!!! quand on change la fréquence du timer hall_speed de 250000 à 1mHz, il y a aussi des changements dans main 
// !!! aussi à uint16_t last_clock_ticks = 0;  // used to call a function every 25 ms (ebbike controller at 40Hz)
//uint16_t last_foc_pid_ticks = 0;    // used to call a function every 10 msec (update foc pid angle at 100hz)
//uint16_t last_foc_optimiser_ticks = 0 ; // used to call a function every 200 msec (update of optimizer at 5 hz)
//uint16_t last_system_ticks = 0;
//volatile uint32_t system_ticks2 = 0;
// il faut faire un search général sur HALL_SPEED_TIMER_HW pour voir tous les impacts (notamment pour les fonctions dans common)


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
#include <math.h>
#include "systick.h"

#include "cy_retarget_io.h"
//#include "cy_utils.h"
#if(uCPROBE_GUI_OSCILLOSCOPE == MY_ENABLED)
#include "ProbeScope/probe_scope.h"
#endif

#define RPM_FOR_MOTOR_STOP        100UL    // par exemple 100 tr/min

// --- Dépendances utilisées ---
#define PWM_FREQUENCY             19000UL  // fréquence ISR PWM (Hz)
#define MOTOR_POLE_PAIRS          4UL      // nombre de paires de pôles moteur

// --- Dérivé : temps max (en ticks PWM) entre deux fronts hall avant détection stop ---
#define RPM_FOR_MOTOR_STOP_PWM_TICKS \
    ((uint32_t)((60UL * PWM_FREQUENCY) / (RPM_FOR_MOTOR_STOP * 6UL * MOTOR_POLE_PAIRS)))

// **************  to test slow rotation without using the hall sensor and so discover pattern sequence
// just to test rotation at a low speed and low power to verify the the hall sequence is OK
#define SPEED_COUNTER_MAX 19000 /360  // one electrical rotation per sec ; so 1 mecanical rotation takes 4 sec ; so 15 rpm
#define DUTY_CYCLE_TEST 30// 256 = 100% ; 40 gives a current = 1A from ADC on pin 2.8 with a 12V battery
#define ANGLE_INIT 0
// end of those test parameters


// pattern sequence for hall sensor is 1,3,2,6,4, 5
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
// === Mapping des états Hall vers secteurs === Sequence is 1, 3, 2, 6, 4, 5 
//                                                 for sect 0, 1, 2, 3, 4, 5
const int8_t hall_to_sector[8] = {
    0, 0, 2, 1, 4, 5, 3, 0 // when hall pattern is invalid (0 or 7) we use sector 0 to avoid further checs
};

// table has to be updated if PWM frequency change !!!!!!!!!!!!!!

// table generated with sin(x) + 1/6*sin(3*x) scaled to -800/+800 to avoid being to close of the limits (-840/+840 for 19 kHz)
// first value in the table is for x = 90° (to be similar to TSDZ2)
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

// this table says which phases are best read by ADC to have a larger window at mid point
// 1 = phase U and V ; 2 = phase U and W ; 3 = phase V and W
// this table is specific for lut sinus with sin(x) + 1/6 sin(3x)
static const uint8_t ui8_LUT_SECTOR_CASE[256] = {
    2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
    2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
    2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,
    2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
    2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2
};

/*
// table generated with sin(x) scaled to -800/+800 to avoid being to close of the limits (-840/+840 for 19 kHz)
// first value in the table is for x = 90° (to be similar to TSDZ2)
static const int16_t i16_LUT_SINUS[256] = {
    800,800,799,798,796,794,791,788,785,781,776,771,766,760,753,746,
    739,731,723,715,706,696,686,676,665,654,643,631,618,606,593,579,
    566,552,537,523,508,492,477,461,444,428,411,394,377,360,342,324,
    306,288,270,251,232,213,194,175,156,137,117,98,78,59,39,20,
    0,-20,-39,-59,-78,-98,-117,-137,-156,-175,-194,-213,-232,-251,-270,-288,
    -306,-324,-342,-360,-377,-394,-411,-428,-444,-461,-477,-492,-508,-523,-537,-552,
    -566,-579,-593,-606,-618,-631,-643,-654,-665,-676,-686,-696,-706,-715,-723,-731,
    -739,-746,-753,-760,-766,-771,-776,-781,-785,-788,-791,-794,-796,-798,-799,-800,
    -800,-800,-799,-798,-796,-794,-791,-788,-785,-781,-776,-771,-766,-760,-753,-746,
    -739,-731,-723,-715,-706,-696,-686,-676,-665,-654,-643,-631,-618,-606,-593,-579,
    -566,-552,-537,-523,-508,-492,-477,-461,-444,-428,-411,-394,-377,-360,-342,-324,
    -306,-288,-270,-251,-232,-213,-194,-175,-156,-137,-117,-98,-78,-59,-39,-20,
    0,20,39,59,78,98,117,137,156,175,194,213,232,251,270,288,
    306,324,342,360,377,394,411,428,444,461,477,492,508,523,537,552,
    566,579,593,606,618,631,643,654,665,676,686,696,706,715,723,731,
    739,746,753,760,766,771,776,781,785,788,791,794,796,798,799,800
};
*/
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
// --- Définition d'un offset de 80° en Q8.8 to limit interpolation---
#define HALL_INTERP_MAX_DELTA_Q8_8   ((uint16_t)((80 * 65536UL) / 360))   // it was first 60° = 10922 en Q8.8 (~0x2AAA)
#define Q16_16_SHIFT              16     // position Q16.16

#define NB_SECTORS      6
#define SHIFT_CORR      3            // correction sur 8 ISR PWM
uint16_t ui16_base_sector_q8_8[NB_SECTORS]   = { // values are ovewritten at the end of hall calibration
    24<<8,  // 1 -> 24 * 360 / 256 degré
    66<<8,  // 3 -> 66 * 360 / 256 degré
    107<<8, // 2 -> 107 * 360 / 256 degré
    152<<8, // 6 -> 152 * 360 / 256 degré    
    195<<8, // 4 -> 195 * 360 / 256 degré
    235<<8, // 5 -> 235 * 360 / 256 degré
};
uint16_t ui16_sector_angle_q8_8[NB_SECTORS] = {  // values are ovewritten at the end of hall calibration
    (66-24)<<8,  // 1 -> 24 * 360 / 256 degré
    (107-66)<<8, // 2 -> 107 * 360 / 256 degré
    (152-107)<<8,  // 3 -> 66 * 360 / 256 degré
    (195-152)<<8, // 4 -> 195 * 360 / 256 degré
    (235-195)<<8, // 5 -> 235 * 360 / 256 degré
    (24+256-235)<<8, // 6 -> 152 * 360 / 256 degré
};

// for hall position & hall velocity 
uq8_8_t ui16_curr_base_angle_q8_8 = 0;  // position of hall at the begin of the current sector
uint8_t ui8_curr_sector = 0; //
uint8_t ui8_prev_sector = 0; 

//uint16_t ui16_curr_hall_ticks;
uint16_t ui16_prev_hall_ticks = 0; // used only for one read and one write (no need to copy local)

uint16_t ui16_hall_angle_position_q8_8; // hall position (abs + interpol) to compare with pll position
uint32_t ui32_hall_velocity_q8_8X1024;  // vitesse filtrée (ou mesurée sur un tour dans certaines versions)
//uint32_t ui32_raw_velocity_q8_8X1024;   // vitesse brutte (non filtrée) entre 2 fronts

// +++++++++++++++  for hybrid hall positioning ++++++++++++++++++++
// === Vitesse seuil pour transition Hall→ hybrid
#define HALL_TO_HYBRID_VELOCITY  ((uint32_t) (1000 * 4474 / 1000))  // velocity is rpm * '4,474
#define HYBRID_TO_HALL_VELOCITY  ((uint32_t) (500 * 4474 / 1000))
uint8_t ui8_hybrid_position_valid = 0;  // 0 = Hall-only, 1 = hybrid
uint16_t ui16_hyb_angle_no_ref_no_lead_q8_8 = 0;       // position estimée
uint16_t ui16_prev_base_angle_q8_8 = 0;              // position base du front précédent
uint16_t ui16_prev_sector_angle_q8_8 = 0;            // angle between presious base and current base
uint32_t ui32_hyb_velocity_q8_8X1024 = 0;              // vitesse secteur précédent (angle/tick X256)
int16_t i16_correction_q8_8 = 0;       // correction progressive restante
int16_t i16_step_q8_8 = 0;             // step par ISR
int8_t i8_cnt_steps = 0;             // nombre d’ISR restant pour la correction
int16_t i16_residual_q8_8 = 0;         // correction résiduelle pour ajustement exact



// for dynamic lead angle 
//uint16_t ui16_angle_for_id_q8_8;   // position including reference without taking care of lead angle; updated at the end of ISR 0 
uint8_t ui8_angle_for_id;   // position without taking care of lead angle; updated at the end of ISR 0 


//for debug 
uint8_t ui8_signed_index_debug =0;
uint32_t velocity_max =0;
uint16_t enlapsed_debug;
int16_t i16_debug_diff_pos_hall_hyb = 0;
int32_t i32_debug_diff_velocity_hall_hyb;


// motor variables
uint8_t ui8_hall_360_ref_valid = 0; // fill with a hall pattern to check sequence is correct
uint8_t ui8_motor_commutation_type = BLOCK_COMMUTATION;
volatile uint16_t ui16_hall_counter_total = 0xffff; // number of tim3 ticks between 2 rotations// inTSDZ2 it was a u16

// power variables
volatile uint8_t ui8_controller_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT; // 194
volatile uint8_t ui8_controller_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT; // 73
volatile uint16_t ui16_adc_voltage_cut_off = 300*100/BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000; // 30Volt default value =  300*100/87 in TSDZ2
volatile uint8_t ui8_adc_battery_current_filtered = 0; // current in adc10 bits units (= moving average on 64 PWM cycles)
volatile uint32_t ui32_adc_battery_current_1_rotation_15b = 0; // value in 12 +2 +1 = 15 bits (ADC + IIR + average)
volatile uint8_t ui8_controller_adc_battery_current_target = 0;
volatile uint16_t ui16_g_duty_cycle = 0;
volatile uint8_t ui8_controller_duty_cycle_target = 0;
// Field Weakening Hall offset (added during interpolation)
volatile uint8_t ui8_fw_hall_counter_offset = 0;
volatile uint8_t ui8_fw_hall_counter_offset_max = 0;
volatile uint8_t ui8_field_weakening_enabled = 0;

// Duty cycle ramp up
//static uint8_t ui8_counter_duty_cycle_ramp_up = 0; // replaced by step up and down in systick
//static uint8_t ui8_counter_duty_cycle_ramp_down = 0;

// FOC angle
//static uint8_t ui8_foc_angle_accumulated = 0;
//uint8_t ui8_foc_flag = 0; // not used anymore with optimised lead angle in systick.c
//volatile uint8_t ui8_g_foc_angle = 0; // not used anymore with optimised lead angle in systick.c
//uint8_t ui8_foc_angle_multiplicator = 0; // not used anymore with optimised lead angle in systick.c
//volatile uint16_t ui16_g_foc_angle_q8_8 = 0; // not used anymore with optimised lead angle in systick.c
//static uint8_t ui8_foc_angle_multiplier = FOC_ANGLE_MULTIPLIER; //39 for 48V motor
//static uint8_t ui8_adc_foc_angle_current = 0; // use a ui16 inside the irq

// battery current variables
uint16_t ui16_adc_battery_current_acc_X4 = 0;
uint16_t ui16_adc_battery_current_filtered_X4 = 0;
volatile uint16_t ui16_adc_motor_phase_current = 0; // mstrens: it was uint8 in original code

// ADC Values
volatile uint16_t ui16_adc_voltage = 0;
//volatile uint16_t ui16_adc_torque = 0;
//volatile uint16_t ui16_adc_throttle = 0; // moved to ebike_app.c
//added by mstrens
volatile uint16_t ui16_adc_torque_filtered = 0 ; // filtered adc torque
//volatile uint16_t ui16_adc_torque_actual_rotation = 0;
//volatile uint16_t ui16_adc_torque_previous_rotation = 0;
//volatile uint8_t ui8_adc_torque_rotation_reset = 0;
    
// brakes
volatile uint8_t ui8_brake_state = 0;

// cadence sensor
#define NO_PAS_REF 5
volatile uint16_t ui16_cadence_sensor_ticks = 0;
//static uint16_t ui16_cadence_sensor_ticks_counter_min = CADENCE_SENSOR_CALC_COUNTER_MIN; // initialiszed at 4270 , then varies with wheelSpeed
//static uint8_t ui8_pas_state_old = 4;
//static uint16_t ui16_cadence_calc_counter = 0;
//static uint16_t ui16_cadence_stop_counter = 0;
//static uint8_t ui8_cadence_calc_ref_state = NO_PAS_REF;
//const static uint8_t ui8_pas_old_valid_state[4] = { 0x01, 0x03, 0x00, 0x02 };
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
uint8_t ui8_curr_hall_pattern = 0;
uint8_t ui8_prev_hall_pattern = 7; // Invalid value, force execution of Hall code at the first run; use only for one 1 and 1 write

volatile uint8_t ui8_hall_sensors_state = 0; // name used by ebike_app.c to identify ui8_curr_hall_pattern; added here for compatibility

// Hall counter value of last Hall transition 
uint16_t previous_360_ref_ticks = 0 ; 

// ----------   end of copy from tsdz2 -------------------------

uint8_t ui8_temp = 0;
uint16_t ui16_temp = 0;


uint8_t hall_reference_angle = 0 ; // !! Is not in Q8_8 but only in uint8 ;This value is initialised in ebike_app.c with DEFAULT_HALL_REFERENCE_ANGLE and m_config.global_offset_angle 

// to debug time spent in irq0 and irq1
volatile uint16_t debug_time_ccu8_irq0 = 0;
//volatile uint16_t debug_time_ccu8_irq1 = 0;
//volatile uint16_t debug_time_ccu8_irq1b = 0;
//volatile uint16_t debug_time_ccu8_irq1c = 0;
//volatile uint16_t debug_time_ccu8_irq1d = 0;
//volatile uint16_t debug_time_ccu8_irq1e = 0;
uint16_t hall_ref_angles_counter = 0;

// Hall offset for current Hall state; This offset is added in the interpolation process (so based also on the erps)
// the value is in ticks = usec ; we need  about 60 usec : 
//     55usec = delay between measuring at begin of ISR0 and applying PWM change at end of PWM cycle;
//     There is also some delay in hall sensor but it is probably included in hall calibration process
// based on the regression tests, there should probably be a correction of about 2 depending it is a rising or a falling edge of hall pattern
// still this should have only a small impact
uint8_t ui8_hall_counter_offset = 60;

//#if (DYNAMIC_LEAD_ANGLE == (1) ) //1 dynamic based on Id and a PID + optimiser 
// to calculate Id
uint16_t ui16_angle_for_id_prev_q8_8; // position; saved at begin of ISR 0 to match with current iu,Iv,iw measured at begin of ISR 1
volatile uint16_t ADC_Bias_Iu = 1 << 11; // ADC is 12 bits, 0 = mid point 
volatile uint16_t ADC_Bias_Iv = 1 << 11; // ADC is 12 bits, 0 = mid point 
volatile uint16_t ADC_Bias_Iw = 1 << 11; // ADC is 12 bits, 0 = mid point 
//int32_t i32_id_filtr = 0;       // Id filtered (calculated in calculate_id_part1 and 2 ; used to adapt Q31_lead_angle with a pid)
//volatile int32_t i32_id_pid_acc = 0 ;    // accumulate the Id value to be able to calculate the avg
//volatile int32_t i32_id_pid_cnt = 0 ;    // count the Id value in acc to be able to calculate the avg

//int32_t q31_lead_angle = 0 ; // lead angle in Q31
//int32_t foc_pid_I_term = 0;  // integral term of foc pid

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
//#endif // DYNAMIC_LEAD_ANGLE == 1 dynamic based on Id and a PID + optimiser 

// to debug
int16_t I_u; // to check current in each phase
int16_t I_v;
int16_t I_w;
int16_t I_t;
// to measure tick intervals in isr0
uint16_t ui16_prev_ISR0_ticks = 0;
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

uint16_t debug_error_div = 0;

uint16_t hall_pattern_error_counter = 0; // to debug only

    
volatile int32_t debug_id = 0;
volatile int32_t debug_iq = 0;
volatile int32_t debug_I1 = 0;
volatile int32_t debug_I2 = 0;
volatile int32_t debug_I3 = 0;
int32_t volatile debug_Iu;
int32_t volatile debug_Iv;
int32_t volatile debug_Iw;
int32_t volatile debug_Iuvw;
int32_t volatile debug_Ialpha;
int32_t volatile debug_Ibeta;
int32_t volatile debug_angle;
int32_t volatile debug_va ; // to debug
int32_t volatile debug_vb ; // to debug
int32_t volatile debug_vc ;  // to debug
uint8_t cordic_offset =128;
int32_t debug_id_accum = 0;
int32_t debug_iq_accum = 0;
uint8_t debug_id_filter = 6;
int32_t volatile debug_foc = 0; 
int32_t debug_cordic_angle = 0;
int32_t debug_cordic_offset = 0;   
int32_t debug_raw_id = 0;
int32_t debug_raw_iq = 0;
int32_t debug_i32_Iu1 = 0;
int32_t debug_i32_Iv1 = 0;
int32_t debug_i32_Iw1 = 0;
int32_t debug_i_avg = 0;


// new wheel and cadence variables : moved to systick.c
// =============== VARIABLES PARTAGÉES =============== 
//volatile uint32_t ui32_pwm_ticks = 0;          // compteur soft 19kHz
//volatile uint32_t ui32_cadence_last_ticks[6] = {0};   // timestamps pédalage (codes 0..5)
//volatile uint32_t ui32_wheel_last_pwm_ticks = 0; // dernier front roue (ui32_pwm_ticks)

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


//  -------- TABLE DE TRANSITION QUADRATURE 16→CODE -------------
//   index = (prev<<2) | curr
//   prev,curr ∈ [0..3] → 16 combinaisons possibles
//   mapping :
//     reverse : 00->01 (1), 01->11 (7), 11->10 (14), 10->00 (8)  ; table filled with 4
//     forward : 00->10 (2), 10->11 (11), 11->01 (13), 01->00 (4) ; table filled with 0...3
//     no-change / invalid : autres cas                           ; table filled with 5
const uint8_t ui8_cadence_transpose[16] = {
    /*0*/ 5,  /*1*/ 4,  /*2*/ 0,  /*3*/ 5,
    /*4*/ 3,  /*5*/ 5,  /*6*/ 5,  /*7*/ 4,
    /*8*/ 4,  /*9*/ 5,  /*10*/5,  /*11*/1,
    /*12*/5,  /*13*/2,  /*14*/4,  /*15*/5
};


uint8_t ui8_prev_cadence_state = 0;   // 2 bits combinés prev A/B
uint8_t ui8_prev_wheel_state = 0;
        
// this function has to be called in ISR0 or ISR1 (at 19kHz) to collect the data that are processed in a systick irq at 1kHz
static inline __attribute__((always_inline))  void collect_wheel_cadence_data(){
        
        ui32_pwm_ticks++; // incrément soft timer 32 bits
        // --- wheel sensor ---
        uint8_t ui8_wheel_state = (uint8_t) XMC_GPIO_GetInput(IN_SPEED_PORT, IN_SPEED_PIN);
        if (!ui8_prev_wheel_state && ui8_wheel_state) {
            ui32_wheel_last_pwm_ticks = ui32_pwm_ticks; // rising edge
        }
        ui8_prev_wheel_state = ui8_wheel_state;
    
        // --- cadence sensor  (2 bits) ---
        uint8_t ui8_cadence_state = (uint8_t) (XMC_GPIO_GetInput(IN_PAS1_PORT, IN_PAS1_PIN ) | 
                                        ( XMC_GPIO_GetInput(IN_PAS2_PORT, IN_PAS2_PIN ) <<1 ));
        if ( ui8_cadence_state != ui8_prev_cadence_state) {
            uint8_t ui8_cadence_idx = ((ui8_prev_cadence_state << 2) | ui8_cadence_state) & 0x0F;
            uint8_t ui8_cadence_code = ui8_cadence_transpose[ui8_cadence_idx];
            ui32_cadence_last_ticks[ui8_cadence_code] = ui32_pwm_ticks; // enregistre l’instant du code
            ui8_prev_cadence_state = ui8_cadence_state;
        }    
}



// used to calculate hall angles based of linear regression of all ticks intervals
// are filled in irq0 and transmitted in ebike_app.c using segger_rtt_print 
#if ( GENERATE_DATA_FOR_REGRESSION_ANGLES == (1) )
uint16_t ticks_intervals[8]; // ticks intervals between 2 pattern changes;
uint8_t ticks_intervals_status; // 0 =  new data can be written; 1 data being written; 2 all data written, must be transmitted
#endif


// use in hall irq to capture pattern and timestamp
typedef union __attribute__((aligned(4))) {
    struct {
        uint16_t ticks;     // timestamp (16 bits)
        uint8_t  pattern;   // hall pattern (3 bits utiles)
        uint8_t  flags;     // réservé (ex: sens, erreur, etc.)
    };
    uint32_t raw;           // accès 32 bits atomique
} hall_sample_t;
volatile hall_sample_t hall_irq_sample;   // mis à jour dans ISR HALL
volatile bool hall_event_pending = false; // flag lu dans ISR PWM

// for current calculation
uint32_t ui32_adc_battery_current_15b = 0; // value from adc

//uint32_t ui32_adc_battery_current_15b_moving_average = 0;
int battery_current_moving_avg_index = 0;
int battery_current_moving_avg_sum = 0;
int battery_current_moving_avg_buffer[64] = {0};

// for security checks ; shared with systick.c
volatile uint32_t ui32_Iu_rms_2_filt = 0;
volatile uint32_t ui32_Iv_rms_2_filt = 0;
volatile uint32_t ui32_Iw_rms_2_filt = 0;
volatile uint32_t ui32_Imotor_rms_2_filt = 0;

// For security checks : Flags fault shared with ebike.c
volatile bool fault_phase_current_peak = false;
volatile bool fault_idc_fast = false;

// check which permutation are valid
uint8_t debug_permutation = 0; // this field was used to test (with ucProbe) different permutation of I1,I2,I3 with IU, Iv,IW
                            // using this requires to uncomment some lines in ISR1

// used in systick to optimise lead angle based on average Id, Iq 
int32_t i32_id_sum = 0;     // accumulate Id to use an average in systick based on 64 values
int32_t i32_iq_sum = 0;     // idem for Iq
uint8_t ui8_id_iq_counter = ID_IQ_COUNTER; // 64 ; used to filter id & iq ; pwm at 19kHz and systick at 200Hz => 19000/200 = 95 measurements


// to manage torque sensor using the logic of mspider in https://github.com/TSDZ2-ESP32/TSDZ2-Smart-EBike
// 1 = one of 1/20 of a rotation occured (= 4 state transitions )
// 0x80  = reverse rotation  or timeout detected (stop)-> reset
volatile uint8_t ui8_pas_new_transition = 0;

inline uint32_t update_moving_average(uint32_t new_value){
    battery_current_moving_avg_sum -= battery_current_moving_avg_buffer[battery_current_moving_avg_index];
    battery_current_moving_avg_buffer[battery_current_moving_avg_index] = new_value;
    battery_current_moving_avg_sum += new_value;
    battery_current_moving_avg_index = (battery_current_moving_avg_index + 1) & 0x3F; 
    // Retourne la moyenne actuelle
    return (battery_current_moving_avg_sum + 32) >> 6; // divide by 64; add 32 for better rounding
}

inline __attribute__((always_inline)) uint32_t filtering_function(uint32_t ui32_temp_15b , uint32_t ui32_filtered_15b , uint32_t alpha){
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

// Lecture atomique et rapide des Halls
__STATIC_INLINE uint8_t read_hall_pattern(void)
{
    // Supposons que HALL0, HALL1, HALL2 sont sur le même port
    uint32_t portval = IN_HALL0_PORT->IN;  // lecture unique du port
    // Extraction et positionnement exact des bits
    uint8_t pattern = 0;
    pattern |= (uint8_t)((portval >> IN_HALL0_PIN) & 1);      // Hall0 -> bit 0
    pattern |= (uint8_t)((portval >> IN_HALL1_PIN) & 1) << 1; // Hall1 -> bit 1
    pattern |= (uint8_t)((portval >> IN_HALL2_PIN) & 1) << 2; // Hall2 -> bit 2
    return pattern;
}

// this irq callback occurs when posif detects a new pattern 
__RAM_FUNC void POSIF0_0_IRQHandler(){
    hall_sample_t s;

    s.ticks   =  XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW); // Capture time stamp 
    s.pattern = read_hall_pattern() ;// capture hall pattern
    // Écriture atomique unique sur 32 bits
    hall_irq_sample.raw = s.raw;
    hall_event_pending = true; // juste un flag
}

// +++++++++++++    to calibrate ++++++++++++++++
uint8_t hall_calib_state = HALL_TO_CALIBRATE;
uint32_t hall_cal_sum[6];
uint16_t hall_cal_count[6];
uint16_t hall_cal_total_count;

// to debug
uint16_t base_angle_0;
uint16_t base_angle_1;
uint16_t base_angle_2;
uint16_t base_angle_3;
uint16_t base_angle_4;
uint16_t base_angle_5;
uint16_t sector_angle_0;
uint16_t sector_angle_1;
uint16_t sector_angle_2;
uint16_t sector_angle_3;
uint16_t sector_angle_4;
uint16_t sector_angle_5;

// function to call in main loop or every 25msec
void hall_calibrate(){
    if (hall_calib_state == HALL_MEASURED){
        uint32_t avg_duration[6];
        uint32_t total_duration = 0;
        uint16_t sector_angle[6] ; // angle in q8.8 (65356=360°) in this sector (begin with this index)
        uint16_t base_angle[6];    // angle at the begin of the sector

        // --- Moyenne par secteur ---
        for (int i = 0; i < 6; i++) {
            if (hall_cal_count[i] == 0) avg_duration[i] = 1; // éviter div0
            else avg_duration[i] = hall_cal_sum[i] / hall_cal_count[i];
            total_duration += avg_duration[i];
        }
    
        if (total_duration == 0) {
            hall_calib_state = HALL_CALIBRATION_ERROR; // sécurité
            return;
        }    
        // --- Calcul des angles Q16 (0..65536 = 360°) ---
        int32_t offset =  24<<8;
        int16_t accum_angle = offset; // 24 to match current tabel 
        for (int i = 0; i < 6; i++) {
            // sector_angle[i] proportionnel à avg_duration[i] / total_duration
            sector_angle[i] = (uint16_t)((avg_duration[i] * 65536u) / total_duration);
            base_angle[i] = (uint16_t)accum_angle;
            accum_angle += sector_angle[i];
        }
        // --- Corriger le dernier secteur pour compenser arrondi ---
        if (accum_angle != (65536u + offset))  {
            int32_t diff = 65536u + offset - accum_angle;
            sector_angle[5] = (uint16_t)((int32_t) sector_angle[5] + diff);
        }
        // fast copy in table used by iSR
        for (int i = 0; i < 6; i++) {
            ui16_sector_angle_q8_8[i] = sector_angle[i];
            ui16_base_sector_q8_8[i] = base_angle[i];
        }
        hall_calib_state = HALL_CALIBRATED;
        
        // to debug
        base_angle_0 = (base_angle[0] + 128) >> 8;
        base_angle_1 = (base_angle[1] + 128) >> 8;
        base_angle_2 = (base_angle[2] + 128) >> 8;
        base_angle_3 = (base_angle[3] + 128) >> 8;
        base_angle_4 = (base_angle[4] + 128) >> 8;
        base_angle_5 = (base_angle[5] + 128) >> 8;
        sector_angle_0 = sector_angle[0];
        sector_angle_1 = sector_angle[1];
        sector_angle_2 = sector_angle[2];
        sector_angle_3 = sector_angle[3];
        sector_angle_4 = sector_angle[4];
        sector_angle_5 = sector_angle[5];
    }
}
// ++++++++++++++ end for calibrate ++++++++++++++




// +++++++++++++++  for hybrid hall positioning ++++++++++++++++++++
// ============================================================
// === Fonctions utilitaires pour angles et ticks ===
int16_t debug_angle_diff = 0;

inline int16_t angle_diff(uint16_t a, uint16_t b) {
    int32_t d = (int32_t)a - (int32_t)b;
    if (d > 32767) d -= 65536;
    if (d < -32768) d += 65536;
    return (int16_t)d;
}

// =========  filtrage sans reliquat du au calcul en entier =========
//int32_t diff = omega_mech - i32_omega_est X256;
//int32_t delta = diff >> OMEGA_ALPHA_SHIFT;
//if (delta == 0 && diff != 0)  delta = (diff > 0) ? 1 : -1;
//i32_omega_est X256 += delta;

// called when hall pattern change and hybrid is valid (because rpm is high enough) so it makes sense to calculate Hybrid position
// note : initialisation when switching from Hall to hybrid mode is done in code from ISR0
inline __attribute__((always_inline)) void synchronise_hall_hybrid(uint16_t ui16_ticks_between_2_hall_fronts,
                uint32_t ui32_raw_velocity_q8_8X1024  ) {         
    // Interpolation au front précédent avec vitesse précédente et ticks entre 2 fronts
    uint16_t ui16_theta_est_at_T1_q8_8 = ui16_prev_base_angle_q8_8 + (uint16_t)((ui32_hyb_velocity_q8_8X1024 * (uint32_t)ui16_ticks_between_2_hall_fronts) >> 10);// speed is in x1024 to keep accuracy
    // Erreur vs base nouveau secteur
    int16_t i16_err_q8_8 = angle_diff(ui16_curr_base_angle_q8_8 , ui16_theta_est_at_T1_q8_8);
    debug_angle_diff = i16_err_q8_8 ;
    // === Vérifier si l'erreur est excessive (supérieure à ±30°) ===
    if ((i16_err_q8_8 > (int16_t)HALL_ANGLE_OFFSET_30_DEG_Q8_8) || (i16_err_q8_8 < -(int16_t)HALL_ANGLE_OFFSET_30_DEG_Q8_8))  {
        // --- Réalignement partiel (application immédiate de 75% de la correction) ---
        int16_t i16_err_immediate_q8_8 = (i16_err_q8_8 * 3) >>2;  // 75%
        int16_t i16_err_remaining_q8_8 = i16_err_q8_8 - i16_err_immediate_q8_8;  // reste 25%

        // Appliquer 75% tout de suite (on décale la position estimée)
        ui16_hyb_angle_no_ref_no_lead_q8_8 = ui16_curr_base_angle_q8_8 - ((uint16_t)i16_err_remaining_q8_8);

        // Corriger progressivement le reste sur quelques ISR
        i16_step_q8_8 = i16_err_remaining_q8_8 >> SHIFT_CORR;
        i8_cnt_steps = (int8_t)(1 << SHIFT_CORR);
        i16_correction_q8_8 = i16_err_remaining_q8_8;
        i16_residual_q8_8 = (int16_t)((int32_t)i16_err_remaining_q8_8 -
                                    (int32_t)i16_step_q8_8 * (int32_t)i8_cnt_steps);
    } else        
    {
        // Préparer correction progressive
        i16_step_q8_8 = i16_err_q8_8 >> SHIFT_CORR;
        //if (i16_step_q8_8 == 0 && i16_err_q8_8 != 0) i16_step_q8_8 = (i16_err_q8_8 > 0) ? 1 : -1;
        i8_cnt_steps = 1 << SHIFT_CORR;                  // nombre d’ISR pour lisser l’erreur
        i16_correction_q8_8 = i16_err_q8_8;     // correction à appliquer immédiatement
        i16_residual_q8_8 = i16_err_q8_8 - (i16_step_q8_8 * (int32_t)i8_cnt_steps);
    }
    // Vitesse secteur précédent ui32_raw_velocity_q8_8X1024 déjà calculée est utilisée pour les prochaines interpolations après filtrage
    //uint32_t ui32_hyb_velocity_q8_8X1024_new = (((uint32_t)ui16_sector_angle_q8_8[ui8_prev_sector]) << 8) / (uint32_t) ui16_ticks_between_2_hall_fronts;
    int32_t diff = ui32_raw_velocity_q8_8X1024 - ui32_hyb_velocity_q8_8X1024;
    int32_t delta = diff >> 2;
    if (delta == 0 && diff != 0)  delta = (diff > 0) ? 1 : -1;
    ui32_hyb_velocity_q8_8X1024 += delta;

    // --- Repli Hall-only si vitesse trop faible ---
    if (ui32_raw_velocity_q8_8X1024 < HYBRID_TO_HALL_VELOCITY) ui8_hybrid_position_valid = 0;
}

#define SHIFT_BIAS_CURRENT_LPF 7


//

void capture_3_phase_current_offset(){  // called by main
    // when motor is blocked since some time, we update first the ADC bias for Iu, iv, iW
    // when motor is not running (based on ui8_motor_enabled) we reset foc and foc PID
    // when motor is running we use a PI based on ID (calculated and filtered in ISR) to update FOC angle
    // in a second step we can calculate a value for foc angle based on rpm and current and apply pid as a correction.
    #define ADC_BIAS_SHIFT (9)
    static uint32_t ui32_ADC_Bias_Iu = 1<<(11+ADC_BIAS_SHIFT); // variable are shifted to increase accuracy
    static uint32_t ui32_ADC_Bias_Iv = 1<<(11+ADC_BIAS_SHIFT);
    static uint32_t ui32_ADC_Bias_Iw = 1<<(11+ADC_BIAS_SHIFT);
    // first when motor is not running, update adc bias
    if (ui8_motor_enabled == 0) {
            //ADC sequences - Iw -> Iv -> Iu  = default sequence set in ISR0
            //    VADC_G1->ALIAS = (((uint32_t)VADC_IU_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G1_CHANNEL);
            //    VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL); 
            // tests on Id Iq shows that Iu=I1, Iv= I2, Iw=I3 for this setting of alias !!!!!
        uint32_t I1 = VADC_I1_GROUP->RESD[VADC_I1_RESULT_REG]&0x0FFF; // IW is first measured current based on set up in ISR0
        uint32_t I2 = VADC_I2_GROUP->RESD[VADC_I2_RESULT_REG]&0x0FFF; // IV is second one
        uint32_t I3 = VADC_I3_GROUP->RESD[VADC_I3_RESULT_REG]&0x0FFF; // IU is third one 
        debug_I1 = I1; debug_I2 = I2; debug_I3 = I3;   
        // Read Iu ADC bias and apply filter
        //uint32_t Iu = ((uint32_t)(XMC_VADC_GROUP_GetResult(VADC_I1_GROUP , VADC_I1_RESULT_REG ) & 0x0FFF)) << 10 ; // << 10 to increase accuracy
        uint32_t Iu = I1 << ADC_BIAS_SHIFT;
        ui32_ADC_Bias_Iu =  (uint32_t) ((ui32_ADC_Bias_Iu * (((uint32_t) 1 << SHIFT_BIAS_CURRENT_LPF) - 1U)) + Iu) >> SHIFT_BIAS_CURRENT_LPF;
        ADC_Bias_Iu = (uint16_t) (ui32_ADC_Bias_Iu >> ADC_BIAS_SHIFT) ;
        /* Read Iv ADC bias */
        //uint32_t Iv = ((uint32_t)XMC_VADC_GROUP_GetResult(VADC_I2_GROUP , VADC_I2_RESULT_REG ) & 0x0FFF) << 10;
        uint32_t Iv = I2 <<ADC_BIAS_SHIFT;
        ui32_ADC_Bias_Iv = (uint32_t) ((ui32_ADC_Bias_Iv * (((uint32_t) 1 << SHIFT_BIAS_CURRENT_LPF) - 1U)) + Iv) >> SHIFT_BIAS_CURRENT_LPF;
        ADC_Bias_Iv = (uint16_t) (ui32_ADC_Bias_Iv >> ADC_BIAS_SHIFT) ;
        /* Read Iw ADC bias */
        //uint32_t Iw = ((uint32_t) XMC_VADC_GROUP_GetResult(VADC_I3_GROUP , VADC_I3_RESULT_REG ) & 0x0FFF) << 10;
        uint32_t Iw = I3 << ADC_BIAS_SHIFT;
        ui32_ADC_Bias_Iw = (uint32_t) ((ui32_ADC_Bias_Iw * (((uint32_t) 1 << SHIFT_BIAS_CURRENT_LPF) - 1U)) + Iw) >> SHIFT_BIAS_CURRENT_LPF;
        ADC_Bias_Iw = (uint16_t) (ui32_ADC_Bias_Iw >> ADC_BIAS_SHIFT) ;
    }
}

//uint8_t ui8_measured_phases; // register which 2 phases (from the 3) have to be used to calculate clarck transform

// for park transfom when sinus table is used
#define SIN_TABLE_SIZE      256
#define ANGLE_TO_INDEX_SHIFT 8  // 16 bits Q8.8 → 8 bits d'index (256)

__attribute__((aligned(4), section(".rodata"))) const int16_t sin_table[SIN_TABLE_SIZE] = {
    0,804,1608,2411,3212,4011,4808,5602,6393,7180,7962,8740,9512,10279,11039,11793,
12540,13279,14010,14733,15447,16151,16846,17531,18205,18868,19520,20160,20788,21403,22006,22595,
23170,23732,24279,24812,25330,25833,26320,26791,27246,27684,28106,28511,28899,29269,29622,29957,
30274,30572,30853,31114,31357,31581,31786,31972,32138,32286,32413,32522,32610,32679,32729,32758,
32767,32758,32729,32679,32610,32522,32413,32286,32138,31972,31786,31581,31357,31114,30853,30572,
30274,29957,29622,29269,28899,28511,28106,27684,27246,26791,26320,25833,25330,24812,24279,23732,
23170,22595,22006,21403,20788,20160,19520,18868,18205,17531,16846,16151,15447,14733,14010,13279,
12540,11793,11039,10279,9512,8740,7962,7180,6393,5602,4808,4011,3212,2411,1608,804,
0,-804,-1608,-2411,-3212,-4011,-4808,-5602,-6393,-7180,-7962,-8740,-9512,-10279,-11039,-11793,
-12540,-13279,-14010,-14733,-15447,-16151,-16846,-17531,-18205,-18868,-19520,-20160,-20788,-21403,-22006,-22595,
-23170,-23732,-24279,-24812,-25330,-25833,-26320,-26791,-27246,-27684,-28106,-28511,-28899,-29269,-29622,-29957,
-30274,-30572,-30853,-31114,-31357,-31581,-31786,-31972,-32138,-32286,-32413,-32522,-32610,-32679,-32729,-32758,
-32768,-32758,-32729,-32679,-32610,-32522,-32413,-32286,-32138,-31972,-31786,-31581,-31357,-31114,-30853,-30572,
-30274,-29957,-29622,-29269,-28899,-28511,-28106,-27684,-27246,-26791,-26320,-25833,-25330,-24812,-24279,-23732,
-23170,-22595,-22006,-21403,-20788,-20160,-19520,-18868,-18205,-17531,-16846,-16151,-15447,-14733,-14010,-13279,
-12540,-11793,-11039,-10279,-9512,-8740,-7962,-7180,-6393,-5602,-4808,-4011,-3212,-2411,-1608,-804
};

// Multiplication Q15
__RAM_FUNC static inline int16_t mult_q15(int16_t a, int16_t b)
{
    int32_t temp = (int32_t)a * (int32_t)b;
    temp += 0x4000;  // arrondi
    return (int16_t)(temp >> 15);
}

__RAM_FUNC static inline void park_transform_q15(int16_t Ialpha, int16_t Ibeta, uint16_t angle_q8_8,
                        int16_t *Id, int16_t *Iq)
{
    // Index dans la table
    uint16_t index = (angle_q8_8 >> ANGLE_TO_INDEX_SHIFT) & (SIN_TABLE_SIZE - 1);
    uint16_t cos_index = (index + (SIN_TABLE_SIZE / 4)) & (SIN_TABLE_SIZE - 1);

    int16_t sin_t = sin_table[index];
    int16_t cos_t = sin_table[cos_index];

    // Park transform
    int16_t Id_tmp = mult_q15(Ialpha, cos_t) + mult_q15(Ibeta, sin_t);
    int16_t Iq_tmp = mult_q15(Ibeta, cos_t) - mult_q15(Ialpha, sin_t);

    *Id = Id_tmp;
    *Iq = Iq_tmp;
}


// retrieve all parameters related to current sector (sector, previous sector, base angle, base angle previous sector, angle of previous sector)
__RAM_FUNC  inline __attribute__((always_inline)) void fill_sector_data(uint8_t ui8_curr_hall_pattern_local){
    ui8_curr_sector = hall_to_sector[ui8_curr_hall_pattern_local];               // get current sector
    ui16_curr_base_angle_q8_8 =  ui16_base_sector_q8_8[ui8_curr_sector] ;  // get current base angle
    // get previous sector
    ui8_prev_sector = 5;
    if (ui8_curr_sector > 0) {ui8_prev_sector = ui8_curr_sector - 1;} else {ui8_prev_sector = 5;}

    ui16_prev_base_angle_q8_8 = ui16_base_sector_q8_8[ui8_prev_sector];  // get base of previous sector
    ui16_prev_sector_angle_q8_8 = ui16_sector_angle_q8_8[ui8_prev_sector]; // get angle of previous sector
}

volatile uint32_t ui32_pwm_ticks_since_last_front = 0; // used to check motor stop or very low speed
                                                    // keep global because it could be updated in ebike_app
uint16_t ui16_angle_no_ref_no_lead_q8_8;
uint16_t ui16_angle_no_lead_q8_8;  // angle based on Hall or hybrid with ref angle but no lead angle
uint16_t ui16_hall_angle_no_ref_no_lead_q8_8;
uint16_t ui16_SVM_table_index_q8_8;
int16_t svm_A = 0;
int16_t svm_B = 0;
int16_t svm_C = 0;

// to debug
volatile uint32_t ui32_adc_conversion_gr0 = 0; //to check that adc conversion has been done when reading 
volatile uint32_t ui32_adc_conversion_gr1 = 0; //to check that adc conversion has been done when reading 
volatile uint32_t debug_isr1_timer_start = 0;
volatile uint32_t debug_isr1_timer_end = 0;
volatile uint32_t debug_isr0_timer_start = 0;
volatile uint32_t debug_isr0_timer_end = 0;


// ************************************** begin of IRQ *************************
// *************** irq 0 of ccu8   
//   it takes between 5 and 18 usec (so less than 26 usec)
__RAM_FUNC void CCU80_0_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting UP (= 1/4 of 19mhz cycles with 1680 ticks at 64mHz and centered aligned)
    static bool valid_curr_hall_ticks = false;
    static bool valid_prev_hall_ticks = false;
    static bool first_run_in_PWM_ISR = true;

    uint32_t ui32_raw_velocity_q8_8X1024 = 0;   // vitesse brutte (non filtrée) entre 2 fronts
    uint16_t ui16_elapsed_ticks = 0;
    uint16_t ui16_compensated_elapsed_ticks = 0;
    uint16_t ui16_ticks_between_2_hall_fronts; 

    static uint16_t ui16_curr_hall_ticks = 0;
    
    uint8_t ui8_curr_hall_pattern_local = ui8_curr_hall_pattern;   // local copy for faster processing
    uint32_t ui32_hall_velocity_q8_8X1024_local = ui32_hall_velocity_q8_8X1024;
    #define DEBUG_IRQ0_INTERVALS (0) // 1 = calculate min and max intervals between 2 irq0
    #if (DEBUG_IRQ0_INTERVALS == (1))
    interval_ticks = ui16_curr_ISR0_ticks - ui16_prev_ISR0_ticks;
    if (first_ticks == 0){
        if ( (interval_ticks <=13) || (interval_ticks >= 13)) {
            error_ticks_counter++;
            //error_ticks_value = ui16_curr_ISR0_ticks;
            //error_ticks_prev = ui16_prev_ISR0_ticks;
            if (interval_ticks_min > interval_ticks) interval_ticks_min = interval_ticks;
            if (interval_ticks_max < interval_ticks) interval_ticks_max = interval_ticks;
            
        }
    } else {
        first_ticks = 0; 
    }
    ui16_prev_ISR0_ticks = ui16_curr_ISR0_ticks ;
    #endif

    // read irq data before reading current time stamp (to be sure that time now follow the ISR timestamp)
    hall_sample_t hall_isr_sample_local;
    bool hall_event_pending_local = hall_event_pending;
    hall_isr_sample_local.raw = hall_irq_sample.raw;// Lecture atomique (32 bits)
        
    // get now timestamp
    uint16_t ui16_curr_ISR0_ticks = (uint16_t) (XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW) );
    
    debug_isr0_timer_start = XMC_CCU8_SLICE_GetTimerValue(PWM_IRQ_TIMER_HW);

    // first read for safety
    if (first_run_in_PWM_ISR){
        first_run_in_PWM_ISR = false;
        ui8_curr_hall_pattern_local = read_hall_pattern();
        fill_sector_data(ui8_curr_hall_pattern_local);
    }
    if (hall_event_pending_local) {  //set on true in hall ISR when a new hall pattern occured
        hall_event_pending = false; // reset flag localement
        valid_curr_hall_ticks = true;     
        
        ui16_curr_hall_ticks = hall_isr_sample_local.ticks; // Extraction from ISR
        ui8_curr_hall_pattern_local = hall_isr_sample_local.pattern;

        fill_sector_data(ui8_curr_hall_pattern_local);  // get current and previous sector (and base and angle)
        // start calculate hall velocity (based on elapsed time between 2 fronts and angle of previous sector
        if (valid_prev_hall_ticks) {
            ui16_ticks_between_2_hall_fronts = ui16_curr_hall_ticks - ui16_prev_hall_ticks;
            if (ui16_ticks_between_2_hall_fronts < 417) ui16_ticks_between_2_hall_fronts = 417; // 417 is for rpm = 6000 at clock =1Mhz
            //highiest speed (about 6000rpm); this avoid exceeding 32 bit.
            // start division (it take 35 cycles, result is read afterwards if it is used to update raw velocity)
            MATH->DIVCON = 0X4 ; // unsigned division, no shift, autostart
            MATH->DVD = (((uint32_t)ui16_prev_sector_angle_q8_8) << 10); // Load the dividend value (<<10 to increase accuracy for low speed)
            MATH->DVS = (uint32_t) ui16_ticks_between_2_hall_fronts; // Load the divisor value, the division begin immediately   

            if( ui8_curr_hall_pattern_local != expected_pattern_table[ui8_prev_hall_pattern]) 
            { // Erreur de séquence
                valid_prev_hall_ticks = false; 
                ui8_hall_360_ref_valid = 0;
                ui8_motor_commutation_type = BLOCK_COMMUTATION; // 0x00
                // Si on était en calibration, on la réinitialise
                if (hall_calib_state == HALL_CALIBRATING)  hall_calib_state = HALL_TO_CALIBRATE;
                // cancel hybrid — on conserve l’angle actuel et le dernier incrément
                ui8_hybrid_position_valid = false ;
            } else { // Séquence valide
                if (ui8_curr_hall_pattern_local == 0x01) {  // par exemple rotor à 210°
                    if (ui8_hall_360_ref_valid) {
                        ui16_hall_counter_total = angle_diff(ui16_curr_hall_ticks, previous_360_ref_ticks);
                        ui8_motor_commutation_type = SINEWAVE_INTERPOLATION_60_DEGREES; // 0x80 ; it says that we can interpolate because speed is known
                    }
                    ui8_hall_360_ref_valid = 1;
                    previous_360_ref_ticks = ui16_curr_hall_ticks;
                }
                //if (ui8_curr_hall_pattern_local == 0x02) {  // exemple rotor à 150°
                    //debug_id = i32_id_sum / i32_id_count;
                    //i32_id_sum  = 0;
                    //i32_id_count = 0;
                //    ui8_foc_flag = 1; // sert à mettre à jour le lead angle dans FOC
                //}
           
                // calculate hall velocity after at least 1 rotation
                if ((ui8_hall_360_ref_valid) && (ui16_ticks_between_2_hall_fronts > 80)){ // avoid division by 0 and error in uint if counter would ve to low
                    while(MATH->DIVST); // Wait until DIV is ready (Not busy)
                    ui32_raw_velocity_q8_8X1024 = MATH->QUOT;
                } else {
                    ui32_raw_velocity_q8_8X1024 = 0;
                }
                // apply filter for ui32_hall_velocity_q8_8X1024 (test to see if OK to replace average on 1 full rotation)
                // raw velocity is also used in hybrid (with another filter;  to save time we could use the same filter)
                int32_t diff = ui32_raw_velocity_q8_8X1024 - ui32_hall_velocity_q8_8X1024_local;
                int32_t delta = diff >> 3;
                if (delta == 0 && diff != 0)  delta = (diff > 0) ? 1 : -1;
                ui32_hall_velocity_q8_8X1024_local += delta;
            
                // -------  manage hybrid mode synchronisation -------
                // if in hybrid mode, synchronise with Hybrid and reset to hall mode if speed is too low
                if (ui8_hybrid_position_valid) {
                    synchronise_hall_hybrid(ui16_ticks_between_2_hall_fronts,  ui32_raw_velocity_q8_8X1024 );
                }
                else if (ui32_hall_velocity_q8_8X1024_local > HALL_TO_HYBRID_VELOCITY)
                { // if not yet in hybrid mode and speed high enough, initialise hybrid and switch to hybrid
                    ui16_elapsed_ticks =  ui16_curr_ISR0_ticks - ui16_curr_hall_ticks ; // ticks between now and last pattern change
                    ui16_compensated_elapsed_ticks = ui16_elapsed_ticks + ui8_hall_counter_offset;
                    uint32_t ui32_interpolation_temp = ( (((uint32_t) ui16_compensated_elapsed_ticks) *
                                                (uint32_t)ui32_hall_velocity_q8_8X1024_local ) + 0) >> 10;
                    ui16_hyb_angle_no_ref_no_lead_q8_8 = ui16_curr_base_angle_q8_8 + (uint16_t) ui32_interpolation_temp; // position estimée = secteur de base du secteur courant
                    ui32_hyb_velocity_q8_8X1024 = ui32_hall_velocity_q8_8X1024_local;              // vitesse from hall  (angle/tick X1024)
                    i16_correction_q8_8 = 0;       // correction progressive restante
                    i16_step_q8_8 = 0;             // step par ISR
                    i8_cnt_steps = 0;             // nombre d’ISR restant pour la correction
                    i16_residual_q8_8 = 0;         // correction résiduelle pour ajustement exact
                    ui8_hybrid_position_valid = 1;
                }
                
                // ++++++ +manage calibration 
                // start calibration if rpm is high enough // 1000 rpm = 4474; so 2237 = 500 rpm 
                if ((hall_calib_state == HALL_TO_CALIBRATE) &&
                         (ui32_hall_velocity_q8_8X1024_local > (uint32_t)(2237))){ 
                    hall_calib_state = HALL_CALIBRATING;
                    hall_cal_total_count = 6 * 200; // number of electric rotation
                    for (uint8_t i=0;i<6;i++){
                        hall_cal_sum[i] = 0;
                        hall_cal_count[i] = 0;
                    }
                } 
                if (hall_calib_state == HALL_CALIBRATING){
                    hall_cal_sum[ui8_prev_sector] += ui16_ticks_between_2_hall_fronts ;
                    hall_cal_count[ui8_prev_sector]++;
                    hall_cal_total_count--;
                    if (hall_cal_total_count == 0) {hall_calib_state = HALL_MEASURED; }
                }

            }   // end valid previous hall ticks           
        } // end valid/ or invalid sequence after an event pending
        ui16_prev_hall_ticks = ui16_curr_hall_ticks;
        valid_prev_hall_ticks = true;
        ui8_prev_hall_pattern = ui8_curr_hall_pattern_local;  // used to check the sequence
        ui32_pwm_ticks_since_last_front = 0;            // reset to avoid timeout    
    } // end hall event pending
    else  // +++++++++++++++  no hall transition  ++++++++++++++
    { // no change in hall patern since some delay (timeout count in PWM cycles)
        ui32_pwm_ticks_since_last_front++; 
        if (ui32_pwm_ticks_since_last_front > RPM_FOR_MOTOR_STOP_PWM_TICKS ){  
            ui32_pwm_ticks_since_last_front = RPM_FOR_MOTOR_STOP_PWM_TICKS ; // avoid that counter increases and wrap.
            // value must be choosen also to avoid that number of ticks on 360° exceeds uint16 max 
            ui8_motor_commutation_type = BLOCK_COMMUTATION; // 0
            //ui8_g_foc_angle = 0; // not used anymore with optimised lead angle in systick.c
            ui8_hall_360_ref_valid = 0;
            ui32_hall_velocity_q8_8X1024_local = 0;
            ui16_hall_counter_total = 0xffff;
            if (hall_calib_state == HALL_CALIBRATING) hall_calib_state = HALL_TO_CALIBRATE; // will restart calibration when speed increase again
            ui8_hybrid_position_valid = 0; // do not use hybrid anymore
            valid_curr_hall_ticks = false; // we can't trust anymore timestamps because they could wrap
            valid_prev_hall_ticks = false; // we can't trust anymore timestamps because they could wrap
        }
    } // end no change.

    // save previous hall or hybrid position with reference to calculate Id Iq in ISR1; this angle may not include lead angle
    ui16_angle_for_id_prev_q8_8 = ui16_angle_no_lead_q8_8; //ui16_angle_no_ref_no_lead_q8_8 ;
    //  +++++++++++ here in all cases, we calculate rotor position +++++++++++++
    if (valid_curr_hall_ticks)  {           
        // elapsed time between now and last pattern change (used for interpolation)
        ui16_elapsed_ticks =  ui16_curr_ISR0_ticks - ui16_curr_hall_ticks ; // ticks between now and last pattern change
        ui16_compensated_elapsed_ticks = ui16_elapsed_ticks + ui8_hall_counter_offset; // there are some ticks before applying this PWM
    } else {
        ui16_elapsed_ticks = 0;
        ui16_compensated_elapsed_ticks = 0;
    }

    // save local variables used to copy global variable while increasing cpu performance
    ui8_curr_hall_pattern = ui8_curr_hall_pattern_local; 
    ui32_hall_velocity_q8_8X1024 = ui32_hall_velocity_q8_8X1024_local;

    /****************************************************************************/
    // - calculate interpolation angle and sine wave table index (when speed is known
    uint32_t ui32_hall_interpolation_angle_q8_8 = ( (((uint32_t) ui16_compensated_elapsed_ticks) *
                                          (uint32_t)ui32_hall_velocity_q8_8X1024_local ) + 0) >> 10;
        // Saturation à ±80°
        //if (i32_interpolation_angle_q8_8 > HALL_INTERP_MAX_DELTA_Q8_8)  i32_interpolation_angle_q8_8 = HALL_INTERP_MAX_DELTA_Q8_8;        
        //if (i32_interpolation_angle_q8_8 < 0) ui8_error_interpolation_q8_8 = 1;
    
    // ------------ Calculate the rotor angle and use it as index in the table----------------- 
    // hall angle position (to be compared with hybrid angle position 
    ui16_hall_angle_no_ref_no_lead_q8_8 = ui16_curr_base_angle_q8_8 + (uint16_t)( ui32_hall_interpolation_angle_q8_8 & 0xFFFF);

    // all angle position calculated by hybrid method if hybrid is valid (velocity high enough) 
    if(ui8_hybrid_position_valid) {
        // Interpolation linéaire depuis le dernier changement front
        uint16_t ui16_theta_interp_q8_8 = ui16_curr_base_angle_q8_8 + ((ui32_hyb_velocity_q8_8X1024 * ui16_compensated_elapsed_ticks) >> 10); // >>10 because velocity is scaled by 1024
        if (i8_cnt_steps > 0) {         // Appliquer correction progressive avec signe correct (soustraction)
            ui16_hyb_angle_no_ref_no_lead_q8_8 = ui16_theta_interp_q8_8 - i16_correction_q8_8;
            // Décrémenter correction pour le prochain ISR
            i16_correction_q8_8 -= i16_step_q8_8;
            i8_cnt_steps--;
        }
        else if (i8_cnt_steps == 0) {        // Appliquer résiduel une seule fois pour convergence exacte
            ui16_hyb_angle_no_ref_no_lead_q8_8 = ui16_theta_interp_q8_8 - i16_residual_q8_8;
            i8_cnt_steps--;  // ne plus rentrer ici
        } else {        // Plus de correction à appliquer
            ui16_hyb_angle_no_ref_no_lead_q8_8 = ui16_theta_interp_q8_8;
        }
    }  // end calculating hybrid position    

    if (ui8_hybrid_position_valid == 0) { 
        ui16_angle_no_ref_no_lead_q8_8 = ui16_hall_angle_no_ref_no_lead_q8_8;
    } else {
        ui16_angle_no_ref_no_lead_q8_8 = ui16_hyb_angle_no_ref_no_lead_q8_8;
    }
    // add hall_reference_angle ; set on 66 based on tests with my motor. (note : 64 = 90°)
    ui16_angle_no_lead_q8_8 = ui16_angle_no_ref_no_lead_q8_8 + (uint16_t) (hall_reference_angle << 8);
    
    // add lead angle
//    uint16_t ui16_SVM_table_index_q8_8 = ui16_angle_no_lead_q8_8 + (uint16_t)(ui8_g_foc_angle<<8);
    // here ui16_g_foc_angle_q8_8 is just based on hall velocity and a multiplicator (see systick).
    //ui16_SVM_table_index_q8_8 = ui16_angle_no_lead_q8_8 + (ui16_g_foc_angle_q8_8);
    // here we use the lead angle based on a table on velocity and a correction to set Id around 0 
    ui16_SVM_table_index_q8_8 = ui16_angle_no_lead_q8_8 + ui16_lead_total_q8_8;
    uint8_t ui8_lut_index = (uint8_t)(ui16_SVM_table_index_q8_8 >> 8);

    /*
    if (ui8_motor_enabled) {
        ui8_measured_phases = ui8_LUT_SECTOR_CASE[ui8_lut_index];
    } else {
        // take care that this must be the same sequence as used to calibrate the ADC offset (done when motor is not enabled)
        ui8_measured_phases = 3; // use default config for bias when motor is off
    }
    */
    
    /*
    // in case 1, use phase u and v, 2 = phase u and w ,  3 = phase v and w
    switch (ui8_measured_phases){
        case 1:
            //ADC sequences - Iu -> Iv -> Iw 
            VADC_G1->ALIAS = (((uint32_t)VADC_IW_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G1_CHANNEL);
            VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL);
        break;
        case 2:
            //ADC sequences - Iw -> Iu -> Iv 
            VADC_G1->ALIAS = (((uint32_t)VADC_IV_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G1_CHANNEL);
            VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G0_CHANNEL);
        break;
        default:
            //ADC sequences - Iw -> Iv -> Iu
            VADC_G1->ALIAS = (((uint32_t)VADC_IU_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G1_CHANNEL);
            VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL); 
        break;
    }
    */

    //uint8_t ui8_lut_index = (uint8_t) (ui16_SVM_table_index_q8_8 >> 8);
    uint8_t ui8_lut_index_A = (ui8_lut_index + 171) & 0xFF; // -120° = 256*2/3 ≈ 171
    uint8_t ui8_lut_index_B = ui8_lut_index ;
    uint8_t ui8_lut_index_C = (ui8_lut_index + 85) & 0xFF; // + 120°

    svm_A = i16_LUT_SINUS[ui8_lut_index_A];
    svm_B = i16_LUT_SINUS[ui8_lut_index_B];
    svm_C = i16_LUT_SINUS[ui8_lut_index_C];
    // fill the PWM parameters with the values calculated in the other CCU8 interrupt (so always after mid point)
    uint16_t temp_duty_cycle = (ui16_g_duty_cycle + 0x80) >> 8; // rounding
    PHASE_U_TIMER_HW->CR1S = (uint32_t) (MIDDLE_SVM_TABLE + (( svm_A * temp_duty_cycle)>>8)); // >>8 because duty_cycle 100% is 256;
    PHASE_V_TIMER_HW->CR1S = (uint32_t) (MIDDLE_SVM_TABLE + (( svm_B * temp_duty_cycle)>>8)); // >>8 because duty_cycle 100% is 256
    PHASE_W_TIMER_HW->CR1S = (uint32_t) (MIDDLE_SVM_TABLE + (( svm_C * temp_duty_cycle)>>8)); // >>8 because duty_cycle 100% is 256  

    // updload of the shadow registers of PWM timers is done in ISR1 after the mid point
    

    #define DEBUG_IRQO_TIME (1) // 1 = calculate the time spent in irq0
    #if (DEBUG_IRQO_TIME == (1))
    if (hall_calib_state == HALL_CALIBRATED) {  // we measure only when hall are calibrated to get more realistic values
        uint16_t temp  = XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW) ;
        temp = temp - ui16_curr_ISR0_ticks;
        if (irq0_min > temp) irq0_min = temp; // store the in enlapsed time in the irq
        if (irq0_max < temp) irq0_max = temp; // store the max enlapsed time in the irq
    }
    #endif

    /*
    // get the voltage ; done in irq0 because it is used in irq1 and irq0 takes less time
        //ui16_adc_voltage  = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 4 ) & 0x0FFF) >> 2; // battery gr1 ch6 result 4
    // changed to take care of infineon VADC init (result in reg 6)
    ui16_adc_voltage  = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , VADC_VDC_RESULT_REG ) & 0x0FFF) >> 2; // battery gr1 ch6 result 6
    */      
    
    #if (uCPROBE_GUI_OSCILLOSCOPE == MY_ENABLED)
    //I_u = XMC_VADC_GROUP_GetResult(VADC_I1_GROUP , VADC_I1_RESULT_REG ) & 0x0FFF;
    //I_w = XMC_VADC_GROUP_GetResult(VADC_I3_GROUP , VADC_I3_RESULT_REG ) & 0x0FFF;
    //I_v = XMC_VADC_GROUP_GetResult(VADC_I2_GROUP , VADC_I2_RESULT_REG ) & 0x0FFF;
    
    // ProbeScope_Sampling must be called to update data displayed on PC in graph
    // if we do not require a high refresh rate, this could be set in another loop 
    ProbeScope_Sampling(); // this is here in a interrupt that run fast
    #endif
    debug_isr0_timer_end = XMC_CCU8_SLICE_GetTimerValue(PWM_IRQ_TIMER_HW);

} // end of CCU80_0_IRQHandler

// ************* irq handler ******************************


#define DEBUG_IRQ1_TIME (1) // 1 = calculate time spent in irq1

__RAM_FUNC void CCU80_1_IRQHandler(){ // called when ccu8 Slice 3 reaches 1300 counting DOWN (= about 5 usec after mid point = adc conversion)    
// this function takes between 12 and 16 usec; so it ends before end of pwm cycle  (5+16 < 26)   
    #if (DEBUG_IRQ1_TIME == (1))
    // to debug max time in this iSR
    uint16_t start_ticks  =  XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
    #endif

//    debug_isr1_timer_start = XMC_CCU8_SLICE_GetTimerValue(PWM_IRQ_TIMER_HW); // to know when ISR start running (at about timer 1200 when compare is set on 1300)
//    debug_isr1_timer_start++;

// collect wheel and cadence ticks to further process in 1 msec irq +++++++++++
    // this could be in isr0 or ISR1 (probably best in ISR0)
    collect_wheel_cadence_data();

// for debugging ; check when ADC conversion is done
//    ui32_adc_conversion_gr1 = ((VADC_G1->VFR) & 0xFF) ;
//    ui32_adc_conversion_gr0 = ((VADC_G0->VFR) & 0xFF) ;

// Enable shadow transfer for slice 0,1,2 for CCU80 Kernel
    ccu8_0_HW->GCSS = ((uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |
                                                (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |
                                                (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_2 );   
// get phases currents (when ADC conversion is done)
        // it measures actual currents but angle must be one one that was apply for PWM and so it is the angle from isr 0 before update.
    // read the 3 ADC and substact the ADC bias and avg
    // calculate i_alpha and i_beta (clark transform)
    // fill cordic to get IQ ID (park transform)

    // Read current ADC (ADC synchronous conversion) 
    uint16_t I1 = VADC_I1_GROUP->RES[VADC_I1_RESULT_REG]&0X0FFF; // first conversion = G1 ch 0
    uint16_t I2 = VADC_I2_GROUP->RES[VADC_I2_RESULT_REG]&0X0FFF; // second conversion = G0 ch 0
    uint16_t I3 = VADC_I3_GROUP->RES[VADC_I3_RESULT_REG]&0X0FFF; // third conversion = G1 ch 1
    
    // take care that result registers from ADC could contains different phase currents depending on rotor position (alias being used)
    // furthermore in some cases, only 2 currents from the 3 should be used (not in this version)
    // in this version, we just use a fix setting for alias
    // this is the alias being used and this code is normally not required because it is already in ADC init and here we use only 1 sequence for this debug
    
    //VADC_G1->ALIAS = (((uint32_t)VADC_IU_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G1_CHANNEL);
    //VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL);
    
    int32_t i32_raw_Iu; int32_t i32_raw_Iv; int32_t i32_raw_Iw;
    // Here we use permutation 0 (see below) so Iu = I1, Iv=I2 , Iw = I3(with cordic offset = 128)
    i32_raw_Iu = (int32_t)I1; i32_raw_Iv = (int32_t)I2; i32_raw_Iw = (int32_t)I3; // here ADC in 12 bits
    
    // Whe have also to take care that there are 6 permutations of I1, I2, I3 with Iu, Iv, IW
    // It seems that 3 could be used but each of them requires a different cordic offset to get valid Id, Iq (at park transform)
    /*
    // This code allows to test all 6 permutations changing debug_permutation and cordic_offset within ucProbe
    // take care that a function updates bias when motor is disabled and must use a fix setting for alias!!
    switch (debug_permutation) {
        case 0:
            i32_raw_Iu = (int32_t)I1; i32_raw_Iv = (int32_t)I2; i32_raw_Iw = (int32_t)I3; break; //cordic offset 128
        case 1:
            i32_raw_Iu = I1; i32_raw_Iv = I3; i32_raw_Iw = I2; break;
        case 2:
            i32_raw_Iu = I2; i32_raw_Iv = I1; i32_raw_Iw = I3; break;
        case 3:
            i32_raw_Iu = I2; i32_raw_Iv = I3; i32_raw_Iw = I1; break; // cordic offset 42
        case 4:
            i32_raw_Iu = I3; i32_raw_Iv = I1; i32_raw_Iw = I2; break; // cordic offset 213
        default:
            i32_raw_Iu = I3; i32_raw_Iv = I2; i32_raw_Iw = I1; break;
    }
    */
       
    /* 
    // here some code that was prepared to use different settings for alias in order to sample the 2 most signicant currents depending on rotor position
    // take care that this code has not been updated to use the good permutation between I12,3 and Iu,v,w.
    // the code from infineon did not use a good permutation and so can't be used as model
    switch (ui8_measured_phases){ // in case 1, use phase u and v, 2 = phase u and w ,  3 = phase v and w
        case 1:
            //ADC sequences - Iu -> Iv -> Iw 
            //VADC_G1->ALIAS = (((uint32_t)VADC_IW_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G1_CHANNEL);
            //VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL);
            i16_raw_Iu = I1;
            i16_raw_Iv = I2;
            i16_raw_Iw = I3;
        break;
        case 2:
            //ADC sequences - Iw -> Iu -> Iv 
            //VADC_G1->ALIAS = (((uint32_t)VADC_IV_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G1_CHANNEL);
            //VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G0_CHANNEL);
            i16_raw_Iu = I2;
            i16_raw_Iv = I3;
            i16_raw_Iw = I1;
        break;
        default:
            //ADC sequences - Iw -> Iv -> Iu
            //VADC_G1->ALIAS = (((uint32_t)VADC_IU_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G1_CHANNEL);
            //VADC_G0->ALIAS = (((uint32_t)VADC_IDC_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL);
            i16_raw_Iu = I3;
            i16_raw_Iv = I2;
            i16_raw_Iw = I1;
        break;
    }
    */
    
    int32_t i32_Iu = (i32_raw_Iu -(int32_t)ADC_Bias_Iu ) << 3; // change from 12 bits to 15 bits to use Q15 in cordic
    int32_t i32_Iv = (i32_raw_Iv - (int32_t)ADC_Bias_Iv ) << 3;
    int32_t i32_Iw = (i32_raw_Iw - (int32_t)ADC_Bias_Iw ) << 3;

    int32_t i_avg = (((i32_Iu + i32_Iv + i32_Iw) * (int32_t) DIV_3)) >>  SCALE_DIV_3 ; 
    i32_Iu -= i_avg;
    i32_Iv -= i_avg;
    i32_Iw -= i_avg;
    // here we have the 3 phase currents without offset in 15 bits

    debug_i32_Iu1 = i32_Iu;
    debug_i32_Iv1 = i32_Iv;
    debug_i32_Iw1 = i32_Iw;
    //debug_i_avg = i_avg;

// measure IDC
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
    // changed when using infineon init for vadc (result in 12bits and in ch 1)
    uint32_t ui32_temp_adc_battery_current_15b = (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , VADC_I4_RESULT_REG ) & 0xFFFF) <<3; // change from 12 to 15 digits 
    ui32_adc_battery_current_15b = ui32_temp_adc_battery_current_15b;

    uint32_t ui32_adc_battery_current_15b_moving_average = update_moving_average(ui32_temp_adc_battery_current_15b);
    if (ui32_adc_battery_current_15b_moving_average > (255 << 5)) { // clamp for safety ; << 5 because in TSDZ2 current is in ADC10 bits and max is 255
        ui32_adc_battery_current_15b_moving_average = 255 << 5;
    }  
    ui8_adc_battery_current_filtered = ui32_adc_battery_current_15b_moving_average  >> 5;

// perform security checks
    //security checks: could be moved after calculating Id, IQ if cordic is used
    // RMS IIR phase (32 bits suffisent)

    // carrés des phases
    uint32_t i32_Iu2 = (i32_Iu >> 3) * (i32_Iu >> 3); // reduce to 12 * 12 bits to avoid overflow in i32 in systicks
    uint32_t i32_Iv2 = (i32_Iv >> 3) * (i32_Iv >> 3);
    uint32_t i32_Iw2 = (i32_Iw >> 3) * (i32_Iw >> 3);

    // Phase current Peak protection
    if(i32_Iu2 > PHASE_PEAK_TRIP2 || i32_Iv2 > PHASE_PEAK_TRIP2 || i32_Iw2 > PHASE_PEAK_TRIP2) {
        fault_phase_current_peak = true;
        motor_disable_pwm();
        ui8_motor_enabled = 0;
    }

    // RMS IIR phase (assembleur-like)
    int32_t diff;
    diff = (int32_t)i32_Iu2 - (int32_t)ui32_Iu_rms_2_filt;
    ui32_Iu_rms_2_filt += diff >> PHASE_RMS_ALPHA;
    diff = (int32_t)i32_Iv2 - (int32_t)ui32_Iv_rms_2_filt;
    ui32_Iv_rms_2_filt += diff >> PHASE_RMS_ALPHA;
    diff = (int32_t)i32_Iw2 - (int32_t)ui32_Iw_rms_2_filt;
    ui32_Iw_rms_2_filt += diff >> PHASE_RMS_ALPHA;

    // RMS moteur (somme carrés)
    ui32_Imotor_rms_2_filt = ui32_Iu_rms_2_filt + ui32_Iv_rms_2_filt + ui32_Iw_rms_2_filt; 

    // Idc fast
    if(ui32_adc_battery_current_15b > IDC_FAST_TRIP) {
        fault_idc_fast = true;
        motor_disable_pwm();
        ui8_motor_enabled = 0;
    }

// calculate clack transform 
    int32_t I_Alpha_1Q31;
    int32_t I_Beta_1Q31;
    // when we use the 3 phase currents; I32_Ix is in 15 bits
    I_Alpha_1Q31 = (((i32_Iu << 1) - (i32_Iv + i32_Iw)) * (int32_t) DIV_3) >> SCALE_DIV_3 ; // !! here in 15 bits even if less accurate
    I_Beta_1Q31 = ((i32_Iv - i32_Iw) * (int32_t) DIV_SQRT3_Q14) >> SCALE_DIV_3;
    //debug_Ialpha = I_Alpha_1Q31  ; // to get same units as Iu,Iv,Iw
    //debug_Ibeta = I_Beta_1Q31 ;

    /* // if we want to use only 2 phases depending on rotor position; this was with cordic and so with 14 more bits!!!!!
    switch (ui8_measured_phases){
        case 1:
            //ADC sequences - Iu -> Iv -> Iw
            I_Alpha_1Q31 = i32_Iu << CORDIC_SHIFT;
            I_Beta_1Q31 = (i32_Iu + (i32_Iv << 1)) * (DIV_SQRT3_Q14 <<(CORDIC_SHIFT-14));
            break;
        case 2:
            //ADC sequences - Iw -> Iu -> Iv 
            I_Alpha_1Q31 =  i32_Iu << CORDIC_SHIFT;
            I_Beta_1Q31 =  (i32_Iu + (i32_Iw << 1)) * (-(DIV_SQRT3_Q14 <<(CORDIC_SHIFT-14)));
            break;
        default:
            //ADC sequences - Iw -> Iv -> Iu
            I_Alpha_1Q31 = (-(i32_Iv + i32_Iw)) << CORDIC_SHIFT;
            I_Beta_1Q31 = (i32_Iv - i32_Iw) * (DIV_SQRT3_Q14 << (CORDIC_SHIFT-14));
            break;
    }
    */                    
// apply park transform
    // 1) using table sinus
    // here we keep phases currents in 15 bits (it could be different for Cordic)
    // take care to use cordic_offset according to the used permutation (here 128 to be *256 for Q8_8) 
    uint16_t ui16_angle = ui16_angle_for_id_prev_q8_8 + (((uint16_t)cordic_offset) <<8);
    int16_t i16_id;
    int16_t i16_iq;
    // get Id, Iq with table
    park_transform_q15((int16_t) I_Alpha_1Q31, (int16_t) I_Beta_1Q31, ui16_angle, &i16_id, &i16_iq);
    if (ui8_id_iq_counter){ //used to filter id & iq ; pwm at 19kHz and systick at 200Hz => 19000/200 = 95 measurements; here we take 64 measurements
        i32_id_sum += i16_id;
        i32_iq_sum += i16_iq;
        ui8_id_iq_counter--;
    } 

    //debug_I1 = I1; debug_I2 = I2; debug_I3 = I3; // 12 bits  
    //debug_Iu = i32_Iu; debug_Iv = i32_Iv; debug_Iw = i32_Iw;// 15 bits
    //debug_Iuvw =  debug_Iu + debug_Iv +debug_Iw;  // so in 15 bits
    //debug_va = ui16_a; debug_vb = ui16_b; debug_vc = ui16_c; // to debug
    //debug_cordic_offset = (int32_t)(((uint16_t)cordic_offset) <<8);
    //debug_Ialpha = (I_Alpha_1Q31 );  debug_Ibeta = (I_Beta_1Q31 ) ;
    //debug_id += ((i16_id - debug_id) + ((i16_id - debug_id > 0) ? 1 : (i16_id - debug_id < 0 ? -1 : 0))) >> 4;
    //debug_iq += ((i16_iq - debug_iq) + ((i16_iq - debug_iq > 0) ? 1 : (i16_iq - debug_iq < 0 ? -1 : 0))) >> 4;
    debug_angle = (int32_t) ui16_angle_for_id_prev_q8_8 ;
    debug_raw_id = i16_id; debug_raw_iq = i16_iq; // in 15 bits

    /*
    // With cordic ; !!!!!!!!!! code has to be checked ; there are some schanges required
    // calculate I alpha and I beta
    // I_Alpha = (2 * I_U - (I_V + I_W))/3   // ou Ialpha = (2/3) * (Ia - 0.5*Ib - 0.5*Ic)
    //HandlePtr->I_Alpha_1Q31 = ((CurrentPhaseU << 1) - (CurrentPhaseV + CurrentPhaseW)) * (DIV_3 << (CORDIC_SHIFT-14));
    // DIV3 = 5461 ; 1/3 = 5461 / (1<<14).; we avoid dividing by 1<<14 in order to get the value from 12 bit to 12+14 = 28 bit
    //    I_Alpha_1Q31 = ((i32_Iu << 1) - (i32_Iv + i32_Iw)) * DIV_3 ;

    //  I_Beta = (I_V - I_W)/√3 in 1Q31
    //HandlePtr->I_Beta_1Q31 = (CurrentPhaseV - CurrentPhaseW) * (DIV_SQRT3_Q14 << (CORDIC_SHIFT-14));
    // here also we avoid >>14 in order to get the adc 12 bits +14 bits for better accuracy
    //    I_Beta_1Q31 = (i32_Iv - i32_Iw) * DIV_SQRT3_Q14 ;
    // prepare parktransform with cordic
    // General control of CORDIC Control Register 
    MATH->CON = CORDIC_ROTATION_MODE;
    uint16_t ui16_angle = ui16_angle_for_id_prev_q8_8 + (((uint16_t)cordic_offset) <<8);
    MATH->CORDZ = (( int32_t) ui16_angle) << 16; // we convert angle in 0/65536 to Q31 
    // Y = I_Alpha 
    MATH->CORDY = I_Alpha_1Q31;
    // X = I_Beta. Input CORDX data, and auto start of CORDIC calculation (~62 kernel clock cycles) 
    MATH->CORDX = I_Beta_1Q31;
    // Wait if CORDIC is still running calculation 
    while (MATH->STATC & 0x01)
    {
        continue;
    }
    // Read CORDIC results Iq and Id - 32-bit. CORDIC Result Register [7:0] are 0x00 
    int32_t i32_iq = MATH->CORRX;
    i32_iq >>= CORDIC_SHIFT; // shift 14
    i32_iq = (i32_iq * 311) >> 8;   // x MPS/K.;
    //Idem for Id
    int32_t i32_id = MATH->CORRY;
    i32_id >>= CORDIC_SHIFT;
    i32_id = (i32_id * 311) >> 8;   // x MPS/K.;    
    // here id and iq are equivalent to 15 bits
    debug_id = i32_id;
    debug_iq = i32_iq;
    */

    // to debug
    //uint16_t temp1e  =  XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
    //temp1e = temp1e - start_ticks;
    //if (temp1e > debug_time_ccu8_irq1e) debug_time_ccu8_irq1e = temp1e; // store the max enlapsed time in the irq


    #if (DEBUG_IRQ1_TIME == (1))
    uint16_t temp1  =  XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
    temp1 = temp1 - start_ticks;
    if (irq1_min > temp1) irq1_min = temp1; // store the min enlapsed time in the irq
    if (irq1_max < temp1) irq1_max = temp1; // store the min enlapsed time in the irq
    #endif
    debug_isr1_timer_end = XMC_CCU8_SLICE_GetTimerValue(PWM_IRQ_TIMER_HW);
}  // end of CCU8_1_IRQ


void motor_enable_pwm(void) { //set posif with current position & restart the timers
    get_hall_pattern(); // refresh hall pattern in ui8_curr_hall_pattern
    
    // one solution to activate is to generate an event that starts all timers in a synchronized way
    // Enable Global Start Control CCU80  in a synchronized way*/
    XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC80_Msk);
    XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk);
    uint32_t retry_start_counter = 10;
    while ((!XMC_CCU8_SLICE_IsTimerRunning(PHASE_U_TIMER_HW)) && (retry_start_counter > 0)){ // to be sure it is running
        XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC80_Msk);
        XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk);
    }
    // Note if we want to use one slice U, V or W to trigger VADC, we should activate the GPIO; see note for disable
}

void motor_disable_pwm(void) {
    // we stop and clear the 3 timers that control motor PWM
    XMC_CCU8_SLICE_StopClearTimer(PHASE_U_TIMER_HW);
    XMC_CCU8_SLICE_StopClearTimer(PHASE_V_TIMER_HW);
    XMC_CCU8_SLICE_StopClearTimer(PHASE_W_TIMER_HW);
    // slice CCU8_3 is not stopped becauses it is required to manage some tasks (speed, torque,...) 
    // Note: if we want to use slice 1 to manage a VADC trigger based on the channel 2 compare value, we should not stop the timer.
    //       we should then set all PWM gpio on Thri-state; this is still perhaps less secure
    // currently, when PWM timers are stopped, levels are set to passive LOW  
}

void get_hall_pattern(){  // use to initialise at power on and in motor_enable()
    uint32_t critical_section_value = XMC_EnterCriticalSection();
    ui8_curr_hall_pattern = XMC_GPIO_GetInput(IN_HALL0_PORT, IN_HALL0_PIN);// hall 0
    ui8_curr_hall_pattern |=  XMC_GPIO_GetInput(IN_HALL1_PORT, IN_HALL1_PIN) << 1;
    ui8_curr_hall_pattern |=  XMC_GPIO_GetInput(IN_HALL2_PORT, IN_HALL2_PIN) << 2;
    XMC_ExitCriticalSection(critical_section_value);
}

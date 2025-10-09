// Motor.c adapté pour PLL simplifié et mise à jour dynamique de la lut
// to do, !!!!!!!!
// add a basic way to calculate lead angle (even if not optimized)
// retirer les lectures ADC de l'irq 0 pour les remettre dans l'irq 1
// mettre une partie du code pour wheel speed et cadence dans une boucle lente
// activer le code pour calculer lead angle avec un optimiser sur base de Idc
// retirer le code pour calculer lead angle avec un pid et un optimiser sur Id
// retirer des codes inutilisés ne garder qu'une méthode pour calculer le courant IDC filtrer
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

// This table cover 360° (in 256 steps)
// in this table values are signed (to avoid to adapt to mid point before multiplying by amplitude)
// values are based on sinus(x) + sinus(3x); 3rd harmonic injection to increase use of Vbus
// timer has a clock of 64mHz and is centered aligned. Freq = 19kHz.
// So max value in timer is 64000000 / 19000 /2 = 1680
// mid point is 840 in pwm timer but 0 in this table.  So values must be between -840 and 840 max.
// to avoid issues when closed to the limits, the range has been limitted to -800...800
// the table is calculated in xls (file = Calcule look up table for FOC in google doc)
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

/* Gains in Q8.8 */
const int16_t Kp_q8_8 = 5;   // par exemple 0.02*256 = 5 ; // Kp = 0.02 → Kp_q8_8 = round(0.02 * 256) = 5
const int16_t Ki_q8_8 = 0;   // si utilisé

q8_8_t i16_hall_position_q8_8 = 0;           // position rotorique absolue (Q18.8)
q8_8_t i16_last_hall_position_q8_8 = 0;      // position rotorique au dernier front Hall
q8_8_t i16_hall_velocity_q8_8 = 0;           // vitesse rotorique (Q8.8 / tick)
q8_8_t i16_hall_phase_error_q8_8 = 0;        // erreur de phase pour PLL
q8_8_t i16_hall_velocity_raw_q8_8;  // vitesse mesurée directement entre deux fronts Hall

// Lead angle Q8.8 (signed, -180°..+180°) when dynamic = 1
int16_t i16_lead_angle_pid_Id_Q8_8 = 0;

// use only when DYNAMIC_LEAD_ANGLE == 2 
volatile int16_t i16_lead_final_esc_q_8_8 = 0; // utilisé pour la position rotor when dynamic = 2
 
#define Q16_16_SHIFT              16     // position Q16.16

// Intégrateur (Q16.16)
int32_t i32_pll_integrator_q8_8 = 0; // to rename later in Q24_8

// Anti-windup limits (Q16.16)
const int32_t PLL_INT_MAX_Q8_8 = (1 << 23); // to do check if 23 is OK
const int32_t PLL_INT_MIN_Q8_8 = -(1 << 23);

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

volatile uint16_t ui16_a_pll = PWM_COUNTER_MAX / 2 ;//   840 in tsdz8   // 4*210 from tsdz2
volatile uint16_t ui16_b_pll = PWM_COUNTER_MAX / 2 ;//   840 in tsdz8   // 4*210 from tsdz2
volatile uint16_t ui16_c_pll = PWM_COUNTER_MAX / 2 ;//   840 in tsdz8   // 4*210 from tsdz2



// motor variables
uint8_t ui8_hall_360_ref_valid = 0; // fill with a hall pattern to check sequence is correct
uint8_t ui8_motor_commutation_type = BLOCK_COMMUTATION;
//uint8_t ui8_motor_phase_absolute_angle = 0;
volatile uint16_t ui16_hall_counter_total = 0xffff; // number of tim3 ticks between 2 rotations// inTSDZ2 it was a u16
static uint16_t ui16_hall_counter_total_previous = 0;  // could be used to check if erps is stable
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
static uint8_t ui8_foc_angle_accumulated = 0;
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


// Hall offset for current Hall state; This offset is added in the interpolation process (so based also on the erps)
// the value is in ticks (1 ticks = 4 usec); we need  55usec/4 : 55 = 39 + 16 (39 = 3/4 of 55usec = delay between measuring and PWM change); 16=delay hall sensor
// based on the regression tests, there should probably be a correction of about 2 depending it is a rising or a falling edge of hall pattern
// still this should have only a small impact
uint8_t ui8_hall_counter_offset = 14; 
uint16_t ui16_angle_for_id_q8_8;   // position without taking care of lead angle; updated at the end of ISR 0 



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


uint16_t hall_pattern_error_counter = 0; // to debug only
// used to calculate hall angles based of linear regression of all ticks intervals
// are filled in irq0 and transmitted in ebike_app.c using segger_rtt_print 
#if ( GENERATE_DATA_FOR_REGRESSION_ANGLES == (1) )
uint16_t ticks_intervals[8]; // ticks intervals between 2 pattern changes;
uint8_t ticks_intervals_status; // 0 =  new data can be written; 1 data being written; 2 all data written, must be transmitted
uint16_t previous_hall_pattern_change_ticks;  // save the ticks of last pattern change
#endif

volatile uint16_t ticks_hall_pattern_irq_last = 0;
volatile uint8_t current_hall_pattern_irq = 0;
uint16_t ui16_last_timer_ticks ;
uint32_t ui32_tick_overflow_count ;
uint32_t ui32_last_timer_ticks ; // last timer with overflow.
volatile uint32_t ui32_prev_hall_timestamp = 0;

// Variables globales (ou statiques ISR) to manage dynamically hall positions differed (by one ISR) update 
bool hall_event_pending = false;
uint8_t pending_hall_pattern;
uint16_t pending_hall_ticks;
uint32_t pending_overflow_count;
uint32_t pending_prev_hall_ts32 = 0;

// ---------------- Buffer circulaire pour update dynamique LUT de position des halls----------------
typedef struct {
    uint32_t delta;     // intervalle entre deux transitions
    uint8_t hall_idx;   // index du pattern Hall (0..5)
} hall_sample_t;

#define BUFFER_SIZE (60) // 10*6 hall transitions
hall_sample_t delta_buffer[BUFFER_SIZE];
uint8_t buffer_write_pos = 0;
volatile uint8_t buffer_count = 0;
volatile bool buffer_ready = false;

// for current calculation
uint32_t ui32_adc_battery_current_15b = 0; // value from adc

// to calculate moving average on Idc 
uint32_t ui32_adc_battery_current_15b_moving_average = 0;
int battery_current_moving_avg_index = 0;
int battery_current_moving_avg_sum = 0;
int battery_current_moving_avg_buffer[64] = {0};

extern uq8_8_t ui16_angle_for_id_prev_q8_8; // is defined later with other code for DYNAMIC_LEAD_ANGLE == 1 (PID + optimiser)

// to manage torque sensor using the logic of mspider in https://github.com/TSDZ2-ESP32/TSDZ2-Smart-EBike
// 1 = PAS state value changed
// 0x80  = PAS state invalid -> reset
volatile uint8_t ui8_pas_new_transition = 0;

static inline __attribute__((always_inline)) uint32_t update_moving_average(uint32_t new_value){
    battery_current_moving_avg_sum -= battery_current_moving_avg_buffer[battery_current_moving_avg_index];
    battery_current_moving_avg_buffer[battery_current_moving_avg_index] = new_value;
    battery_current_moving_avg_sum += new_value;
    battery_current_moving_avg_index = (battery_current_moving_avg_index + 1) & 0x3F; 
    // Retourne la moyenne actuelle
    return (battery_current_moving_avg_sum + 32) >> 6; // divide by 64; add 32 for better rounding
}

static inline __attribute__((always_inline)) uint32_t filtering_function(uint32_t ui32_temp_15b , uint32_t ui32_filtered_15b , uint32_t alpha){
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


// Convertir hall_position (Q16.16 signed ) en uint16 Q8.8 cyclique 0..65535
static inline __attribute__((always_inline)) uint16_t hall_q16_to_q8_8(q16_16_t i32_hall_pos) {
    uint32_t ua = (uint32_t)i32_hall_pos;
    return (uint16_t)((ua & 0xFFFFFFu) >> 8);
}

// Convertir lead_angle Q8.8 (signed -180..+180) en uint16 Q8.8 cyclique 0..65535
static inline __attribute__((always_inline)) uint16_t lead_angle_to_q8_8(int16_t i16_lead_angle) {
    return (uint16_t)i16_lead_angle; // conversion signed → uint16, wrap automatique
}

// mise à jour dynamique de la LUT contenant les positions de hall
#define DELTAS_PER_HALL        (BUFFER_SIZE / 6)  // 10
#define MAX_VARIATION_PERCENT  15 // max % of variation of time interval 
#define MAX_TICKS_HALL_INTERVAL (1000000U/40/4)     // when interval is to big (speed is to low), do not update
        // min speed = 40 erps => delay is 1000000usec/40 => ticks = 1000000usec/40/4 = 
#define ANGLE_Q16_FULL (1U << 16) // 360° en Q16
void Update_LUT_periodic(void) {
    if (!buffer_ready)
        return;  // aucun nouveau lot complet
    uint32_t somme_deltas[6] = {0};
    uint32_t max_deltas[6] = {0};
    uint32_t min_deltas[6];
    for (int i = 0; i < 6; i++)
        min_deltas[i] = 0xFFFFFFFF;

    // Regroupement par Hall
    for (int i = 0; i < BUFFER_SIZE; i++) {
        uint8_t idx = delta_buffer[i].hall_idx;
        uint32_t delta = delta_buffer[i].delta;

        somme_deltas[idx] += delta;
        if (delta > max_deltas[idx]) max_deltas[idx] = delta;
        if (delta < min_deltas[idx]) min_deltas[idx] = delta;
    }

    bool stable = true;
    uint32_t somme_total = 0;
    uint32_t moyenne_deltas[6];
    for (int i = 0; i < 6; i++) {
        moyenne_deltas[i] = somme_deltas[i] / DELTAS_PER_HALL;
        somme_total += moyenne_deltas[i];

        uint32_t variation = (max_deltas[i] - min_deltas[i]) * 100 / moyenne_deltas[i];
        if (variation > MAX_VARIATION_PERCENT)
            stable = false;
    }
    // Vérifier la vitesse
    if (((somme_total / 6) > MAX_TICKS_HALL_INTERVAL) || (somme_total == 0))
        stable = false;

    if (stable) {
        uint32_t angle_accum = 0;
        for (int i = 0; i < 6; i++) {
            uint32_t new_angle = (uint32_t)(((uint64_t)angle_accum * ANGLE_Q16_FULL) / somme_total);
            // Lissage exponentiel (alpha = 1/8) ; we use i+1 because LUT table has 8 items
            u16_hall_angle_table_Q8_8[i+1] = ((uint32_t) u16_hall_angle_table_Q8_8[i+1] * 7 + new_angle) >> 3;
            angle_accum += moyenne_deltas[i];
        }
    }

    // --- Fin de traitement : on libère le buffer ---
    buffer_ready = false;   // ✅ on indique à l’ISR qu’elle peut recommencer à écrire
    buffer_count = 0;       // on réinitialise le compteur
}

// *********** wrap funtions to manage different Q format and roll over

// wrap an unsigned Q8.8 angle (0..256*256-1) into uq8_8_t
static inline uq8_8_t wrap_uq8_8(uint32_t x) {
    return (uq8_8_t)(x & 0xFFFFu);
}

// compute signed shortest difference (a - b) in Q8.8
// both a and b are uq8_8_t (0..65535), returns q8_8_t in range [-128..+128) *256
static inline q8_8_t angle_diff_q8_8(uq8_8_t a, uq8_8_t b) {
    // compute (a - b) modulo 2^16 then shift to signed 16 bits
    int32_t diff = (int32_t)((uint16_t)(a - b)); // modulo 2^16
    if (diff & 0x8000) diff -= 0x10000; // sign extend
    return (q8_8_t)diff; // still Q8.8 representation
}

// add a signed q8_8_t delta to unsigned uq8_8_t angle
static inline uq8_8_t add_signed_to_uq8_8(uq8_8_t base, q8_8_t delta) {
    return (uq8_8_t)((uint16_t)(base + (uint16_t)delta));
}

// helper to convert elapsed ticks (uint16_t) into Q8.8 elapsed (just multiply by 1)
static inline q8_8_t ticks_to_q8_8(uint32_t ticks) {
    // velocity units are Q8.8 per timer tick, so delta position = velocity * ticks >> 8
    // we return ticks as integer; multiplications are done outside
    return (q8_8_t)(ticks); // used as int32 in multiplies
}

// ************************************** begin of IRQ *************************
// *************** irq 0 of ccu8
__RAM_FUNC void CCU80_0_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting UP (= 1/4 of 19mhz cycles with 1680 ticks at 64mHz and centered aligned)
// here we only calculate the new compare values used for the 3 slices (0,1,2) that generates the 3 PWM
#if (USE_IRQ_FOR_HALL == (1))
    uint32_t critical_section_value = XMC_EnterCriticalSection();
    // get the current ticks
    uint16_t current_ISR_timer_ticks = (uint16_t) (XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW) );
    // get the last changed pattern ticks (from posif irq)
    uint16_t last_hall_pattern_change_ticks = ticks_hall_pattern_irq_last;
    // get the current hall pattern as saved duting the posif irq
    current_hall_pattern = current_hall_pattern_irq;
    XMC_ExitCriticalSection(critical_section_value);
#else // irq0 when using a XMC_CCU4_SLICE_CAPTURE
    // get the current ticks
    uint16_t current_ISR_timer_ticks = (uint16_t) HALL_SPEED_TIMER_HW->TIMER;
    // get the capture register = last changed pattern = current pattern
    uint16_t last_hall_pattern_change_ticks = (uint16_t) XMC_CCU4_SLICE_GetCaptureRegisterValue(HALL_SPEED_TIMER_HW , 1);
    // get the current hall pattern
    current_hall_pattern = XMC_POSIF_HSC_GetLastSampledPattern(HALL_POSIF_HW) ;
#endif
    // elapsed time between now and last pattern change
    // used for interpolation and to detect too low speed
    // elapsed_ticks : durée depuis le dernier front Hall (utile pour interpolation, détection d'arrêt et basculement)
    uint16_t elapsed_ticks =  current_ISR_timer_ticks - last_hall_pattern_change_ticks ; // ticks between now and last pattern change
    //q16_16_t i32_elapsed_ticks_Q16 = ((q16_16_t) elapsed_ticks) << Q16_16_SHIFT;
    
    ui8_hall_sensors_state = current_hall_pattern; // duplicate just for easier maintenance of ebike_app.c for 860c (sent to display)

    // check overflow of timer
    uint16_t ui16_timer_ticks = current_ISR_timer_ticks ; 
    if ( ui16_timer_ticks < ui16_last_timer_ticks) ui32_tick_overflow_count ++; 
    ui16_last_timer_ticks = ui16_timer_ticks;


    // to debug time in irq
    //uint16_t start_ticks = current_ISR_timer_ticks; // save to calculate enlased time inside the irq // just for debug could be removed
    #define DEBUG_IRQ0_INTERVALS (0) // 1 = calculate min and max intervals between 2 irq0
    #if (DEBUG_IRQ0_INTERVALS == (1))
    interval_ticks = current_ISR_timer_ticks - prev_ticks;
    if (first_ticks == 0){
        if ( (interval_ticks <=13) || (interval_ticks >= 13)) {
            error_ticks_counter++;
            //error_ticks_value = current_ISR_timer_ticks;
            //error_ticks_prev = prev_ticks;
            if (interval_ticks_min > interval_ticks) interval_ticks_min = interval_ticks;
            if (interval_ticks_max < interval_ticks) interval_ticks_max = interval_ticks;
             
        }
    } else {
        first_ticks = 0; 
    }
    prev_ticks = current_ISR_timer_ticks ;
    #endif

    #if (DYNAMIC_LEAD_ANGLE == (1)) // 1 dynamic based on Id and a PID + optimiser 
    // added by mstrens to calculate Id with position used for PWM beeing currently applied (so when timer reached 0 match)
    ui16_angle_for_id_prev_q8_8 = ui16_angle_for_id_q8_8;  // save angle to use it in ISR 1 (when current iu, iv, iw are measured for actual pwm)
    #endif

    // Traitement du front Hall en attente (même si buffer_ready est true)
    // this process must be done before testing if there is a hall transition (otherwise pending is true and process is not differed))
    if (hall_event_pending) {
        hall_event_pending = false; // consommé

        uint32_t base = pending_overflow_count << 16;
        uint32_t hall_ts32 = base | pending_hall_ticks;

        // Correction si l'événement a eu lieu avant l’overflow
        if (pending_hall_ticks > ui16_timer_ticks) {
            hall_ts32 -= 0x10000;
        }

        // Calcul du delta par rapport au front précédent
        uint32_t delta = hall_ts32 - pending_prev_hall_ts32;
        pending_prev_hall_ts32 = hall_ts32;

        // --- Écriture dans le buffer LUT si disponible ---
        if (!buffer_ready) {
            delta_buffer[buffer_count].delta = delta;
            delta_buffer[buffer_count].hall_idx = pending_hall_pattern - 1;

            buffer_count++;
            if (buffer_count >= BUFFER_SIZE) {
                buffer_ready = true; // lot complet à traiter par la boucle lente
            }
        }
    }
    

    // Détection d'un nouveau front Hall
    if (current_hall_pattern != previous_hall_pattern) {
        // Sauvegarde du contexte minimal to handle data in next ISR
        hall_event_pending = true;
        pending_hall_pattern = current_hall_pattern;
        pending_hall_ticks = last_hall_pattern_change_ticks;
        pending_overflow_count = ui32_tick_overflow_count;

        if (current_hall_pattern != expected_pattern_table[previous_hall_pattern]){ 
            // new pattern is not the expected one
            ui8_motor_commutation_type = BLOCK_COMMUTATION; // 0x00
            ui8_hall_360_ref_valid = 0;  // reset the indicator saying no error for a 360° electric rotation 
            i16_hall_velocity_raw_q8_8 =  0;  // reset the speed for interpolation in block commutation
            //hall_pattern_error_counter++; // for debuging
            //        SEGGER_RTT_printf(0, "error %u\r\n", hall_pattern_error_counter);
            //            ui32_ref_angle = 0 ; // reset the position at pattern 1
        
        } else { // valid transition
            if (current_hall_pattern ==  0x01) {  // rotor at 210° (??? not sure this is right)
                if (ui8_hall_360_ref_valid) { // check that we have a full rotation without pattern sequence error
                    ui16_hall_counter_total = last_hall_pattern_change_ticks - previous_360_ref_ticks; // save the total number of tick for one electric rotation
                    // --- calcul de la vitesse instantanée en Q8.8 ---
                    // 1 tour électrique = 360° = 65536 en Q8.8
                    // vitesse = 65536 / ui16_hall_counter_total  (en Q8.8 par tick)
                    if (ui8_motor_commutation_type == BLOCK_COMMUTATION) {
                        // Update raw velocity
                        if (ui16_hall_counter_total > 10) { // avoid dision by 0 and error in uint if counter would ve to low
                            i16_hall_velocity_raw_q8_8 = (uint32_t)(65536UL << 8) / ui16_hall_counter_total;  // Q8.8 per tick
                        }
                        //  Switch from block to PLL commutation (vitesse suffisante)
                        if ( (ui16_hall_counter_total < (1000000U /4 / (300 / 60 *4) ))) { // to calculate to have 300 rpm (mecanical) ==> 12500
                            // erps = rpm / 60 * 4 poles;
                            // with 4usec/tick => 1000000us / 4 us/tick => 250000 ticks/s => 250000/20 = 12500 ticks 

                            // Réinitialisation du PLL
                            i32_pll_integrator_q8_8 = 0;
                            i16_hall_position_q8_8 = u16_hall_angle_table_Q8_8[current_hall_pattern];     // initialisation à la position actuelle
                            i16_hall_velocity_q8_8 = i16_hall_velocity_raw_q8_8 ; // use raw velocity based on time interval over 360°
                            ui8_motor_commutation_type = PLL_COMMUTATION; // 0x80 ; it says that we can interpolate because speed is known
                        } 
                    }
                    ui8_hall_360_ref_valid = 0x01; 
                    previous_360_ref_ticks = last_hall_pattern_change_ticks ; 
                } // end  ui8_hall_360_ref_valid is true   
            } else if (current_hall_pattern ==  0x03) {  // rotor at 150°)
                // update ui8_g_foc_angle once every ERPS (used for g_foc_angle calculation) ;
                // I do not know why this is done when hall pattern = 0X03 and not with 0X01 to avoid a test
                // could be removed when foc angle is calculated in a new way
                ui8_foc_flag = 1;     
            } // end current hall pattern == 1 or 3; we are still in the case of a hall change
            // +++++ new code added for PLL +++++++++++++
            // masured angle Hall en Q8.8 unsigned
            uq8_8_t measured_hall_q8_8 = u16_hall_angle_table_Q8_8[current_hall_pattern];
            
            // convert estimated position to uq8_8_t for diff (i16_hall_position_q8_8 is signed q8_8_t)
            // but angle_diff expects unsigned representations
            uq8_8_t est_uq8 = (uq8_8_t)i16_hall_position_q8_8; // reinterpret bits

            // compute signed phase error (Q8.8)
            q8_8_t phase_err_q8_8 = angle_diff_q8_8(measured_hall_q8_8, est_uq8);
            i16_hall_phase_error_q8_8 = phase_err_q8_8;

            // proportional term (Q8.8): (phase_err * Kp_q8_8) >> 8
            int32_t tmp = (int32_t)phase_err_q8_8 * (int32_t)Kp_q8_8; // fits in 32-bit
            q8_8_t delta_p_q8_8 = (q8_8_t)(tmp >> Q8_8_SHIFT);

            // integrator (Ki in Q8.8)
            if (Ki_q8_8 != 0) {
                int32_t tmp_i = (int32_t)phase_err_q8_8 * (int32_t)Ki_q8_8;
                int32_t delta_i_q8_8 = (tmp_i >> Q8_8_SHIFT);
                i32_pll_integrator_q8_8 += delta_i_q8_8;
                if (i32_pll_integrator_q8_8 > PLL_INT_MAX_Q8_8) i32_pll_integrator_q8_8 = PLL_INT_MAX_Q8_8;
                if (i32_pll_integrator_q8_8 < PLL_INT_MIN_Q8_8) i32_pll_integrator_q8_8 = PLL_INT_MIN_Q8_8;
            }

            // total correction Q8.8
            int32_t correction_q8_8 = (int32_t)delta_p_q8_8 + i32_pll_integrator_q8_8; // still Q8.8

            // apply correction to velocity and position (Q8.8)
            // velocity is in Q8.8 per tick
            int32_t vtmp = (int32_t)i16_hall_velocity_q8_8 + correction_q8_8;
            if (vtmp > 0x7FFF) vtmp = 0x7FFF;
            if (vtmp < -0x8000) vtmp = -0x8000;
            i16_hall_velocity_q8_8 = (q8_8_t)vtmp;

            int32_t ptmp = (int32_t)i16_hall_position_q8_8 + correction_q8_8;
            i16_hall_position_q8_8 = (q8_8_t)((int16_t)(ptmp & 0xFFFF));

            // save last measured position
            i16_last_hall_position_q8_8 = (q8_8_t)measured_hall_q8_8;

            last_hall_pattern_change_ticks = current_ISR_timer_ticks;
            //End of new code added for pll ++++++++++
        } // end of code for a valid hall change 
        // save current hall pattern if change is valid or not
        previous_hall_pattern = current_hall_pattern; // saved to detect future change and check for valid transition
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
    }    // end of a pattern change occured 
    else { // no hall patern change occured
        // Verify if rotor stopped (< 10 ERPS)
        // 150 = rpm; erps= (150*4poles/60sec); /6 because 6 hall for 1 electric tour
        #define RPM_FOR_STOP 150
        #define HALL_COUNTER_THRESHOLD_FOR_SINE_TO_BLOCK 200
        if (elapsed_ticks > (HALL_COUNTER_FREQ/(RPM_FOR_STOP * 4 / 60)/6)) {  
            ui8_motor_commutation_type = BLOCK_COMMUTATION; // 0
            ui8_g_foc_angle = 0;
            ui8_hall_360_ref_valid = 0;
            i16_hall_velocity_raw_q8_8 = 0;  // reset the speed for interpolation in block commutation
            //ui32_angle_per_tick_X16shift = 0; // 0 means unvalid value
            ui16_hall_counter_total = 0xffff;
        } else {
            // Check for PLL → Hall when we are in PLL commutation mode and speed is quite low
            if (ui8_motor_commutation_type == PLL_COMMUTATION) {
                if (elapsed_ticks > (HALL_COUNTER_FREQ/(HALL_COUNTER_THRESHOLD_FOR_SINE_TO_BLOCK  * 4 / 60)/6)) {
                    // Repasser en Hall uniquement, moteur toujours tournant
                    ui8_motor_commutation_type = BLOCK_COMMUTATION; 
                    // first calculate raw velocity (not calculated in BLOCL_COMMUTATION to avoid a division)
                    if (ui16_hall_counter_total > 10) { // avoid dision by 0 and error in uint if counter would ve to low
                        i16_hall_velocity_raw_q8_8 = (uint32_t)(65536UL << 8) / ui16_hall_counter_total;  // Q8.8 per tick
                    }
                    i16_hall_velocity_q8_8 = i16_hall_velocity_raw_q8_8 ;
                    i32_pll_integrator_q8_8 = 0;
                    // on passe sur la postion calculée par les hall et l'interpolation
                    int32_t delta = ((int32_t)i16_hall_velocity_raw_q8_8 * (int32_t)elapsed_ticks) >> Q8_8_SHIFT;
                    // Saturation à ±60°
                    if (delta > HALL_INTERP_MAX_DELTA_Q8_8)  delta = HALL_INTERP_MAX_DELTA_Q8_8;        
                    i16_hall_position_q8_8 = (q8_8_t) ( (int32_t) u16_hall_angle_table_Q8_8[current_hall_pattern] + delta);    
                } // end switch to block commutation 
            }// end switch from PLL to block commutation
        } // end checks on speed
    } // end no change pattern occured

    // update rotor position  
    int32_t delta_pos = 0; // by default 0° (and not 30° to avoid step after first electric rotation)
    if (ui8_motor_commutation_type != BLOCK_COMMUTATION) {  // as long as hall patern are OK and motor is running
        // -----------------------------------------------------------
        // 4. Interpolation linéaire entre fronts Hall
        // hall_position = dernière position + vitesse * elapsed_ticks
        // -----------------------------------------------------------
        // i16_hall_position_q8_8 = last_hall_pos + velocity * elapsed_ticks
        // compute delta = (velocity_q8_8 * elapsed_ticks) >> 8
        delta_pos = ( (int32_t)i16_hall_velocity_q8_8 * (int32_t)elapsed_ticks ) >> Q8_8_SHIFT;
        i16_hall_position_q8_8 = (q8_8_t)((int16_t)(((int32_t)i16_last_hall_position_q8_8 + delta_pos) & 0xFFFF));
    } else {
        // when motor is blocked (speed < 10erps),
        if( i16_hall_velocity_raw_q8_8 > 0 ) { // when raw speed (only hall) is known
            // Calcul de l'avance angulaire = vitesse * temps (Q8.8)
            delta_pos = ((int32_t)i16_hall_velocity_raw_q8_8 * (int32_t)elapsed_ticks) >> Q8_8_SHIFT;
            // Saturation à ±60°
            if (delta_pos > HALL_INTERP_MAX_DELTA_Q8_8)  delta_pos = HALL_INTERP_MAX_DELTA_Q8_8;
        }
        // else delta_pos = 0 so it is based only on Hall position. We could perhaps add 30°
        i16_hall_position_q8_8 = (q8_8_t)((int16_t)(((int32_t)u16_hall_angle_table_Q8_8[current_hall_pattern] + delta_pos) & 0xFFFF));
    }

    // Wrap automatique géré par le cast en uint16_t
    uint16_t u16_hall_q8_8 = (uq8_8_t)i16_hall_position_q8_8;
    // convert Hall reference to Q8.8 unsigned
    uint8_t u8_hall_reference_angle = DEFAULT_HALL_REFERENCE_ANGLE; // 66 = environ 90°/360*256 ; this is about 90° to forward the magnetic field
    uint16_t u16_hall_reference_angle_q8_8 = (uq8_8_t) u8_hall_reference_angle << 8; 
    // Add hall_reference to hall position (wrap)
    ui16_angle_for_id_q8_8 = (uq8_8_t) ((u16_hall_q8_8 + u16_hall_reference_angle_q8_8) & 0xFFFF);
    
    int16_t i16_lead_angle_Q8_8;
    #if (DYNAMIC_LEAD_ANGLE == (1))
        i16_lead_angle_Q8_8 = i16_lead_angle_pid_Id_Q8_8; // i16_lead_angle_pid_Id_Q8_8 is already calculated by a PID in a slow loop
    #elif (DYNAMIC_LEAD_ANGLE == (2))
        i16_lead_angle_Q8_8 = i16_lead_final_esc_q_8_8 ;  
    #else
    // Conversion du lead_angle Q8.8 signé (-180..+180°) en uint16 Q8.8 cyclique
    // Wrap automatique : valeurs négatives deviennent grandes valeurs uint16 cycliques
    //  Assurez-vous que lead_angle reste dans ±32767 pour éviter un wrap inattendu
    // !!!!!!!!!  temporary solution waiting for dynamic lead angle
        i16_lead_angle_Q8_8 = (int16_t) ui8_g_foc_angle << 8; //!!!!!!!!!!!!!!!!!
    #endif
    uint16_t u16_lead_q8_8 = (uint16_t)i16_lead_angle_Q8_8;
    
    // Add lead advance 
    // Addition cyclique sûre
    // Addition cyclique hall + lead_angle en Q8.8
    // Le résultat est un angle cyclique 0..65535 (wrap automatique)
    // Convient pour calculer l’index LUT et la fraction Q8.8 pour interpolation
    uq8_8_t u16_final_angle_q8_8 = (uq8_8_t)((ui16_angle_for_id_q8_8 + u16_lead_q8_8 ) & 0xFFFF);


    // -----------------------------
    // 6. Index LUT et fraction
    // -----------------------------
    // u8_lut_index = partie entière de Q8.8, sert d’index dans la LUT (0..255)
    // u8_lut_frac = partie fractionnaire Q8.8 pour interpolation linéaire entre N et N+1
    uint8_t u8_lut_index = u16_final_angle_q8_8 >> 8;           // 0..255
    uint8_t u8_lut_frac  = u16_final_angle_q8_8 & 0xFF;           // fraction Q8.8 pour interpolation fine

    // Calcul des index LUT pour les 3 phases A/B/C
    // Les décalages +171/-120° et +85/+120° sont en unités LUT 0..255
    // &0xFF garantit le rollover correct sur 256 éléments de la LUT
    uint8_t u8_lut_index_A = (u8_lut_index + 171) & 0xFF; // -120° = 256*2/3 ≈ 171
    uint8_t u8_lut_index_B = u8_lut_index ;
    uint8_t u8_lut_index_C = (u8_lut_index + 85) & 0xFF; // + 120°

    // -----------------------------
    // 7. Lecture LUT avec rollover N+1 and interpolation
    // -----------------------------
    int16_t yA0 = i16_LUT_SINUS[u8_lut_index_A];
    int16_t yA1 = i16_LUT_SINUS[(u8_lut_index_A + 1) & 0xFF];
    int16_t svm_A = yA0 + (( (int32_t)(yA1 - yA0) * u8_lut_frac) >> 8);
    
    int16_t yB0 = i16_LUT_SINUS[u8_lut_index_B];
    int16_t yB1 = i16_LUT_SINUS[(u8_lut_index_B + 1) & 0xFF];
    int16_t svm_B = yB0 + (((int32_t)(yB1 - yB0) * u8_lut_frac) >> 8);
    
    int16_t yC0 = i16_LUT_SINUS[u8_lut_index_C];
    int16_t yC1 = i16_LUT_SINUS[(u8_lut_index_C + 1) & 0xFF];
    int16_t svm_C = yC0 + (((int32_t)(yC1 - yC0) * u8_lut_frac) >> 8);

    ui16_a_pll = (uint16_t) (MIDDLE_SVM_TABLE + (( svm_A * (int16_t) ui8_g_duty_cycle)>>8)); // >>8 because duty_cycle 100% is 256
    ui16_b_pll = (uint16_t) (MIDDLE_SVM_TABLE + (( svm_B * (int16_t) ui8_g_duty_cycle)>>8)); // >>8 because duty_cycle 100% is 256
    ui16_c_pll = (uint16_t) (MIDDLE_SVM_TABLE + (( svm_C * (int16_t) ui8_g_duty_cycle)>>8)); // >>8 because duty_cycle 100% is 256
    // end of added for PLL

    #define DEBUG_IRQO_TIME (0) // 1 = calculate the time spent in irq0
    #if (DEBUG_IRQO_TIME == (1))
    uint16_t temp  = XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW) ;
    temp = temp - current_ISR_timer_ticks;
    if (irq0_min > temp) irq0_min = temp; // store the in elapsed time in the irq
    if (irq0_max < temp) irq0_max = temp; // store the max elapsed time in the irq
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

    // this could be moved to ISR1 if ISR0 is to long
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
    ui32_adc_battery_current_15b_moving_average = update_moving_average(ui32_adc_battery_current_15b);

    // mstrens : moved from irq1 to irq0 to use average current over 1 rotation for regulation
    if (ui32_adc_battery_current_15b_moving_average > (255 << 5)) { // clamp for safety
        ui32_adc_battery_current_15b_moving_average = 255 << 5;
    }    
    ui8_adc_battery_current_filtered = ui32_adc_battery_current_15b_moving_average  >> 5;
    // to see on prove scope oscillo
    //I_t = ui32_adc_battery_current_15b >> 3; 

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
                if (ui8_foc_flag > 29)
                    ui8_foc_flag = 29;
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
    //if (temp1d > debug_time_ccu8_irq1d) debug_time_ccu8_irq1d = temp1d; // store the max elapsed time in the irq
    
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
    //if (temp1e > debug_time_ccu8_irq1e) debug_time_ccu8_irq1e = temp1e; // store the max elapsed time in the irq

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
    if (irq1_min > temp1) irq1_min = temp1; // store the min elapsed time in the irq
    if (irq1_max < temp1) irq1_max = temp1; // store the min elapsed time in the irq
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


#if (DYNAMIC_LEAD_ANGLE == (1) ) //1 dynamic based on Id and a PID + optimiser 
// to calculate Id
uq8_8_t ui16_angle_for_id_prev_q8_8; // position; saved at begin of ISR 0 to match with current iu,Iv,iw measured at begin of ISR 1
uint16_t ADC_Bias_Iu = 1 << 11; // ADC is 12 bits, 0 = mid point 
uint16_t ADC_Bias_Iv = 1 << 11; // ADC is 12 bits, 0 = mid point 
uint16_t ADC_Bias_Iw = 1 << 11; // ADC is 12 bits, 0 = mid point 
volatile int32_t i32_id_pid_acc = 0 ;    // accumulate the Id value to be able to calculate the avg
volatile int32_t i32_id_pid_cnt = 0 ;    // count the Id value in acc to be able to calculate the avg

int32_t i32_lead_angle_q31 = 0 ; // lead angle in Q31
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

__RAM_FUNC inline void calculate_id_part1(){  // to be called in begin of ISR 1 when rotor position has been updated and current are measured
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
        // ui16_angle_for_id_prev_q8_8 (Q8.8 0..360°) → Q31 (-π..+π), wrap OK pour CORDIC
        MATH->CORDZ = ((int16_t) ui16_angle_for_id_prev_q8_8) << 16; // we convert angle in 0/255 with 8 bit as digital to Q31 signed 
    
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
        // save data to calculate AVG at 100hz (PID) : cnt max = 19000 /100= 190; 190*10000 fit in i32 (so OK)
        i32_id_pid_acc += i32_id; // accumulate
        i32_id_pid_cnt++;         // count
        // update of foc angle occurs in 100 hz and not in ISR
    }

    // +++++++++++++++++ from here the code to apply a pid+optimiser for lead angle using id +++++++++++++++++++++
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

// slew rate pour i32_lead_angle_final_q16
#define MAX_FINAL_STEP_Q16  ( ((5) * Q16_ONE) / 3600 )  // 0.05° par step (~10ms)


// ============================================================================
// VARIABLES GLOBALES
// ============================================================================

// intégrateur PID (int64 pour éviter overflow)
// unité : mA·s approximatif
static int64_t i64_pid_integrator_q16 = 0;

// composantes lead angle (Q16 signé, -180°..+180° environ)
static int32_t i32_lead_angle_pid_q16   = 0;
static int32_t i32_lead_angle_optim_q16 = 0;
static int32_t i32_lead_angle_final_q16 = 0;  // utilisé par la génération PWM

// optimiser
static int i8_optim_dir = 1;                // direction hill-climbing
static int32_t i32_last_id_avg_mA = 0;          // in mA
static int32_t i32_optim_buffer_mA[OPTIM_BUF_LEN];
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


//int32_t apply_PID_on_lead_angle(int32_t Id_filt,int32_t i32_lead_angle_q31);
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
        i32_lead_angle_q31 = 0; 
        foc_pid_I_term = 0;
        i64_pid_integrator_q16 = 0;
        i32_lead_angle_pid_q16 = 0;
        i32_lead_angle_optim_q16 = 0;
        i32_lead_angle_final_q16 = 0;
    }
    else {
        // apply PI on id
        //i32_lead_angle_q31 = apply_PID_on_lead_angle(i32_id_filtr , i32_lead_angle_q31);
        //i32_lead_angle_q31 = apply_PID_on_lead_angle();
        apply_PID_on_lead_angle();
    }
}    


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
    int64_t tentative_full = (int64_t)P_q16 + (((int64_t)KI_Q16 * i64_pid_integrator_q16) >> 16);
    if (!((tentative_full > PID_MAX_Q16 && error > 0) ||
          (tentative_full < -PID_MAX_Q16 && error < 0))) {
        // safe to integrate
        i64_pid_integrator_q16 += ((int64_t)error * (int64_t)DT_Q16) >> 16;
    }

    if (i64_pid_integrator_q16 > INTEGRATOR_MAX)  i64_pid_integrator_q16 = INTEGRATOR_MAX;
    if (i64_pid_integrator_q16 < -INTEGRATOR_MAX) i64_pid_integrator_q16 = -INTEGRATOR_MAX;

    // I term (Q16)
    int64_t I_tmp = (int64_t)KI_Q16 * i64_pid_integrator_q16;
    int32_t I_q16 = (int32_t)(I_tmp >> 16);
    
    // sum and saturation
    int64_t out_tmp = (int64_t)P_q16 + (int64_t)I_q16;
    if (out_tmp > PID_MAX_Q16)      i32_lead_angle_pid_q16 = PID_MAX_Q16;
    else if (out_tmp < -PID_MAX_Q16) i32_lead_angle_pid_q16 = -PID_MAX_Q16;
    else                             i32_lead_angle_pid_q16 = (int32_t)out_tmp;
    
    // --- buffer optimiser---
    i32_optim_buffer_mA[optim_index] = abs32(Id_avg);
    optim_index = (optim_index + 1) % OPTIM_BUF_LEN;
    if (optim_count < OPTIM_BUF_LEN) optim_count++;

    // --- Lead angle final ---
    int32_t tmp = i32_lead_angle_pid_q16 + i32_lead_angle_optim_q16;
    tmp = clamp_q16(tmp, LEAD_MIN_Q16, LEAD_MAX_Q16);

    // ---------------------- SLEW RATE ----------------------
    int32_t delta = tmp - i32_lead_angle_final_q16;
    if (delta > MAX_FINAL_STEP_Q16) delta = MAX_FINAL_STEP_Q16;
    else if (delta < -MAX_FINAL_STEP_Q16) delta = -MAX_FINAL_STEP_Q16;
    i32_lead_angle_final_q16 += delta;

    // Ré-échantillonner 65536->256 en conservant la correspondance angulaire (MSB)
    i16_lead_angle_pid_Id_Q8_8 = i32_lead_angle_final_q16 >> 8;
    
    return ; 
}

// ---------------------- OPTIMISER UPDATE (5 Hz) ----------------------
static int32_t i32_id_filtered_mA = 0;    // filtre low-pass pour i32_id_avg_mA
static int32_t i32_sigma_filtered_mA = 0;   // sigma filtré
static int32_t i32_lead_step_filtered_q16     = 0;      // step filtré pour i32_lead_angle_optim_q16
#define LPF_ALPHA  4  // 1..255, plus grand = plus lent, valeur typique ~4
void update_foc_optimiser(void) {
    if (optim_count == 0) return;

    // Calcul avg
    int64_t sum = 0;
    for (int i = 0; i < optim_count; i++) sum += i32_optim_buffer_mA[i];
    int32_t avg = (int32_t)(sum / optim_count); // mA

    // filtrage low-pass (exponentiel) : i32_id_filtered_mA = α*prev + (1-α)*avg
    // approximation entier : i32_id_filtered_mA = (prev*(255-α) + avg*α)/255
    i32_id_filtered_mA = ( (i32_id_filtered_mA*(255-LPF_ALPHA) + avg*LPF_ALPHA) ) / 255;

    // variance (mA^2)
    int64_t var_sum = 0;
    for (int i = 0; i < optim_count; i++) {
        int32_t diff = i32_optim_buffer_mA[i] - avg;
        var_sum += ((int64_t)diff * diff) ;  // >>0 car déjà 64 bits
    }
    int32_t variance = (int32_t)(var_sum / optim_count); // in mA^2

    // --- filtrage low-pass de sigma (écart type) ---
    int32_t sigma = (int32_t)sqrt((double)variance);
    i32_sigma_filtered_mA = (i32_sigma_filtered_mA*(255-LPF_ALPHA) + sigma*LPF_ALPHA)/255;

    // --- seuils adaptatifs en fonction de sigma filtré ---
    int32_t VAR_LOW  = (int32_t)(4 * i32_sigma_filtered_mA * i32_sigma_filtered_mA);   // 2*sigma
    int32_t VAR_HIGH = (int32_t)(36 * i32_sigma_filtered_mA * i32_sigma_filtered_mA);  // 6*sigma

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
    if (i32_id_filtered_mA > i32_last_id_avg_mA) i8_optim_dir = -i8_optim_dir;

    // --- filtrage du step appliqué ---
    i32_lead_step_filtered_q16 = (i32_lead_step_filtered_q16*(255-LPF_ALPHA) + step*LPF_ALPHA)/255;

    // update optimiser
    int64_t new_opt = (int64_t)i32_lead_angle_optim_q16 + (int64_t)i8_optim_dir * i32_lead_step_filtered_q16;
    if (new_opt > LEAD_MAX_Q16) new_opt = LEAD_MAX_Q16;
    if (new_opt < LEAD_MIN_Q16) new_opt = LEAD_MIN_Q16;
    i32_lead_angle_optim_q16 = (int32_t)new_opt;

    i32_last_id_avg_mA = i32_id_filtered_mA;
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
i32_lead_angle_pid_q16              ±5°                   ±910                                  Sortie PID seule
i32_lead_angle_optim_q16            -15…+30°              -2730…+5461                           Valeur hill-climbing
i32_lead_angle_final_q16            -15…+30°              -2730…+5461                           PID + optimiseur, borné
lead_angle_LUT_256          0…255                 i32_lead_angle_final_q16 >> 8                 Pour LUT 256 entrées
i32_id_pid_acc               0…?                  -                                     Accumulateur ADC Id
i32_id_pid_cnt               1…?                  -                                     Nombre d’échantillons ADC
i32_id_avg_mA                       0…20 A               0…20000                               Courant en mA
error                        -20…+20 A            -20000…+20000                         Objectif Id=0
P_q16                        -2.77…+2.77°         ±910                                  Terme proportionnel PID
i64_pid_integrator_q16               ±5°                   ±99277                               Terme intégral PID
I_q16                        ±5°                   ±910                                  Terme intégral PID appliqué
out_tmp                       ±5°                   ±910                                  Somme P+I, saturée ±PID_MAX_Q16
i32_optim_buffer_mA[]                0…20 A               0…20000                               Buffer pour optimiser Id_abs
i32_id_filtered_mA                   0…20 A               0…20000                               Low-pass sur i32_id_avg_mA
variance                       0…?                  0…?                                   Variance brute Id
sigma                          0…?                  0…?                                   Ecart type Id
i32_sigma_filtered_mA                 0…?                  0…?                                   Filtrage low-pass sigma
step                           0…0.2°               0…36                                  Pas PID adaptatif
i32_lead_step_filtered_q16                  0…0.2°               0…36                                  Pas filtré appliqué
i8_optim_dir                      -1 / +1             -1 / +1                               Direction hill-climbing
i32_last_id_avg_mA                    0…20 A               0…20000                               Dernier Id filtré




Signal / Terme            Plage réelle (° / A)       Q16 value    Commentaire
-------------------------  -------------------------  ---------   -------------------------------------------------
i32_id_avg_mA                     0…20 A                     0…20000     Courant moyen en mA
error (PID)                -20…+20 A                  -20000…+20000  -i32_id_avg_mA
P_q16                      -2.77…+2.77°               ±910        Gain proportionnel, saturé ±PID_MAX_Q16
i64_pid_integrator_q16             ±5°                         ±99277      Borné par INTEGRATOR_MAX
I_q16                      ±5°                         ±910        Terme intégral appliqué
out_tmp                     ±5°                        ±910        P+I, saturé ±PID_MAX_Q16
i32_lead_angle_pid_q16             ±5°                         ±910        Sortie PID uniquement
i32_lead_angle_optim_q16           -15…+30°                    -2730…+5461  Optimiseur hill-climbing
i32_lead_angle_final_q16           -15…+30°                    -2730…+5461  PID + optimiseur, borné LEAD_MIN…LEAD_MAX
LEAD_STEP_Q16              0.2°                        36          Pas de l’optimiseur (0.2° = 2/3600 tour)
i32_id_filtered_mA                0…20 A                      0…20000     Low-pass sur i32_id_avg_mA
i32_sigma_filtered_mA             0…?                         0…?         Low-pass sur écart type de Id
i32_lead_step_filtered_q16              0…0.2°                       0…36       Low-pass sur step appliqué à i32_lead_angle_optim_q16
lead_angle_LUT_256         0…255                        0…255      Conversion Q16 → 8 bits pour LUT


+-----------------------------------------+----------------+-----------------+-----------------+
| Variable / Terme                         | Plage réelle   | Q16 Value       | Commentaire     |
+-----------------------------------------+----------------+-----------------+-----------------+
| Courant Id                               | 0 … 20 A       | 0 … 20000 mA    | Mesure en mA    |
| Erreur PID                               | -20 … +20 A    | -20000 … +20000 | Objectif Id=0   |
| i32_lead_angle_pid_q16                           | -5° … +5°      | -910 … +910     | Terme proportionnel PID (saturé) |
| i64_pid_integrator_q16                            | ±5°            | ±99277          | Limite intégrateur Q16 |
| I_q16                                    | ±5°            | ±910            | Terme intégral PID appliqué |
| out_tmp                                  | ±5°            | ±910            | Somme P+I, saturée ±PID_MAX_Q16 |
| i32_lead_angle_optim_q16                          | -15° … +30°    | -2730 … +5461   | Valeur hill-climbing |
| i32_lead_angle_final_q16                           | -15° … +30°    | -2730 … +5461   | PID + optimiseur, borné |
| lead_angle_LUT_256                         | 0 … 360°       | 0 … 255         | Pour LUT 256 entrées (MSB Q16) |
| LEAD_STEP_Q16                              | 0.2°           | 36              | Pas optimiseur |
| i32_id_filtered_mA                                | 0 … 20 A       | 0 … 20000       | Low-pass sur i32_id_avg_mA |
| variance                                   | 0 … ?          | 0 … ?           | Variance Id (mA^2) |
| sigma                                      | 0 … ?          | 0 … ?           | Ecart-type Id (mA) |
| i32_sigma_filtered_mA                             | 0 … ?          | 0 … ?           | Filtrage low-pass sigma |
| step                                       | 0 … 0.2°       | 0 … 36          | Pas PID adaptatif |
| i32_lead_step_filtered_q16                              | 0 … 0.2°       | 0 … 36          | Pas filtré appliqué |
| i8_optim_dir                                  | -1 / +1       | -1 / +1         | Direction hill-climbing |
| i32_last_id_avg_mA                                | 0 … 20 A       | 0 … 20000       | Dernier Id filtré |
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

i32_lead_angle_pid_q16 : [-PID_MAX_Q16, +PID_MAX_Q16] ≈ [-910, +910]

--------------------------------------------------------------------------
Optimiseur
--------------------------------------------------------------------------
i32_lead_angle_optim_q16 : [LEAD_MIN_Q16, LEAD_MAX_Q16]
                  ≈ [-2731, +5461]   // -15° à +30° en Q16

STEP adaptatif (LEAD_STEP_Q16) :
    - Variance faible     : 2/3600 ≈ 364 (0.2°)
    - Variance moyenne    : 0.1° (LEAD_STEP_Q16 /2)
    - Variance élevée     : 0.05° (LEAD_STEP_Q16 /4)
Direction : +1 / -1 selon i32_id_filtered_mA

--------------------------------------------------------------------------
Lead angle final (avant PWM LUT)
--------------------------------------------------------------------------
i32_lead_angle_final_q16 = i32_lead_angle_pid_q16 + i32_lead_angle_optim_q16
borne : [LEAD_MIN_Q16, LEAD_MAX_Q16] ≈ [-2731, +5461]

Slew rate : MAX_F_


*/
#endif // #if (DYNAMIC_LEAD_ANGLE == (1))

/* for documentation purpose
Convert angle Q16.16 signed avec 1 (= 65536) correspondant à 360/256= 1,4° vers
un angle en Q8.8 unsigned avec 1 (=256) correspondant à 360/256 et range équivalent à 0...360°
uint16_t convert_angle(int32_t a) {
    uint32_t ua = (uint32_t)a;
    return (uint16_t)((ua & 0xFFFFFFu) >> 8);
}

Convert angle Q8.8 signed avec 1 (= 256) correspondant à 360/256= 1,4° et donc range équivalent à -180...+180 vers
un angle en Q8.8 unsigned avec 1 (=256) correspondant à 360/256 et range équivalent à 0...360°
a est défini en int16 et b en uint16
uint16_t b = (uint16_t)a;
*/


// ======================  estimate lead angle with hall_total_ticks with an ESC based on Idc
#if (DYNAMIC_LEAD_ANGLE == (2))

/******************************************************************************
 * Lead Angle Optimization - ESC Control (Cortex-M0, Fixed-Point, Low CPU)
 * ---------------------------------------------------------------------------
 * Objectif :
 *   Adapter dynamiquement le lead angle pour minimiser le courant bus Idc,
 *   tout en garantissant stabilité, sécurité, et faible charge CPU.
 *
 *   - Basé sur le suivi du courant moyen (moving average)
 *   - Pas de flottants (entiers et format Q8.8)
 *   - Fonction appelée par la boucle principale (période variable)
 *   - Compatible avec ISR PWM à 19 kHz et boucle de courant lente (25 ms)
 *   - Deux slew rates :
 *        • rapide sur le lead "base" (lié au RPM)
 *        • lent sur la correction ESC
 *   - Optimisation CPU : calcul du RPM et base seulement si hall change
 *   Cette fonction peut être appelée par la boucle existante toutes les 25 ms
 ******************************************************************************/

 #include <stdint.h>
 #include <stdbool.h>
 
 /* --------------------------------------------------------------------------
    Constantes de configuration (définies en degrés)
    -------------------------------------------------------------------------- */
 #define CORRECTION_MIN_DEG        (-5)
 #define CORRECTION_MAX_DEG        (+5)
 #define LEAD_FINAL_MIN_DEG        (-5)
 #define LEAD_FINAL_MAX_DEG        (+30)
 #define LEAD_BASE_MAX_DEG         30      // max lead base angle
 #define CORRECTION_STEP_DEG       1       // pas d’ajustement correction
 #define MAX_CORRECTION_RATE_DPS   10      // °/s pour slew correction ESC
 #define MAX_BASE_RATE_DPS         100     // °/s pour slew lead base
 
 #define RPM_LEAD_FULL_SCALE       3000    // vitesse où le lead atteint max
 #define RPM_RESET_THRESHOLD       200
 #define RPM_RESET_HYSTERESIS      50
 
 #define DEVIATION_ALLOWED_MA      2000
 #define STABILITY_REQUIRED_COUNT  4
 
 #define TICK_PERIOD_US            4       // 1 tick = 4 µs
 #define POLE_PAIRS                4
 #define CURRENT_SCALE             5
 #define MAX_CURRENT_MA            100000
 
 /* --------------------------------------------------------------------------
    Conversion compile-time ° -> Q8.8
    -------------------------------------------------------------------------- */
 #define Q8_8_DEG(x)               ((int16_t)((x) * 65536 / 360))
 
 // Toutes les constantes utilisées directement en Q8.8
 #define CORRECTION_MIN_Q8_8         Q8_8_DEG(CORRECTION_MIN_DEG)
 #define CORRECTION_MAX_Q8_8         Q8_8_DEG(CORRECTION_MAX_DEG)
 #define LEAD_FINAL_MIN_Q8_8         Q8_8_DEG(LEAD_FINAL_MIN_DEG)
 #define LEAD_FINAL_MAX_Q8_8         Q8_8_DEG(LEAD_FINAL_MAX_DEG)
 #define LEAD_BASE_MAX_Q8_8          Q8_8_DEG(LEAD_BASE_MAX_DEG)
 #define CORRECTION_STEP_Q8_8        Q8_8_DEG(CORRECTION_STEP_DEG)
 #define MAX_CORRECTION_RATE_Q8_8    Q8_8_DEG(MAX_CORRECTION_RATE_DPS)
 #define MAX_BASE_RATE_Q8_8          Q8_8_DEG(MAX_BASE_RATE_DPS)
 
 /* --------------------------------------------------------------------------
    Calcul du numérateur pour approximation fixe (compiler compute)
    -------------------------------------------------------------------------- */
 #define INV_RPM_LEAD_SCALE_SHIFT  17
 #define INV_RPM_LEAD_SCALE_NUM    ((uint32_t)(((1UL << INV_RPM_LEAD_SCALE_SHIFT) + (RPM_LEAD_FULL_SCALE/2)) / RPM_LEAD_FULL_SCALE))
 
 /* --------------------------------------------------------------------------
    Variables statiques
    -------------------------------------------------------------------------- */
 static int16_t i16_lead_base_q8_8 = 0;
 static int16_t i16_lead_base_target_q8_8 = 0;
 static int16_t i16_lead_correction_q8_8 = 0;
 static int16_t i16_lead_final_q8_8 = 0;
 
 static int32_t i32_best_current_mA = MAX_CURRENT_MA;
 static uint8_t u8_stable_count = 0;
 static bool low_speed_reset_done = false;
 
 /* --------------------------------------------------------------------------
    Conversion ticks -> RPM mécanique
    -------------------------------------------------------------------------- */
 static uint32_t ticks_to_rpm_mech(uint16_t hall_ticks)
 {
     if (hall_ticks == 0 || hall_ticks >= 0x7FFF) return 0;
     uint32_t num = 60000000UL; // 60s/min × 1e6 µs/s
     uint32_t denom = (uint32_t)hall_ticks << 4; // × TICK_PERIOD_US (4) × POLE_PAIRS (4)
     return num / denom;
 }
 
 /* --------------------------------------------------------------------------
    Fonction principale
    -------------------------------------------------------------------------- */
 void update_lead_angle_esc(uint16_t ui16_hall_counter_total,
                       uint32_t ui32_adc_battery_current_15b_moving_average,
                       uint32_t Idc_target_mA,
                       uint32_t ui32_delta_ticks) // 1 tick = 4 µs
 {
     /* ---- 1. Conversion du courant ---- */
     uint32_t u32_current_mA = ui32_adc_battery_current_15b_moving_average * CURRENT_SCALE;
     if (u32_current_mA > MAX_CURRENT_MA) u32_current_mA = MAX_CURRENT_MA;
 
     /* ---- 2. Calcul du lead base uniquement si le Hall change ---- */
     static uint16_t u16_prev_hall_counter_total = 0;
     static uint32_t u32_rpm = 0;
 
     if (ui16_hall_counter_total != u16_prev_hall_counter_total) {
         u16_prev_hall_counter_total = ui16_hall_counter_total;
 
         // RPM mécanique à partir des ticks Hall
         u32_rpm = ticks_to_rpm_mech(ui16_hall_counter_total);
 
         // Lead base proportionnel à la vitesse avec clamp
         if (u32_rpm < RPM_LEAD_FULL_SCALE) {
             i16_lead_base_target_q8_8 =
                 (int16_t)((u32_rpm * LEAD_BASE_MAX_Q8_8 * INV_RPM_LEAD_SCALE_NUM)
                            >> INV_RPM_LEAD_SCALE_SHIFT);
         } else {
             i16_lead_base_target_q8_8 = LEAD_BASE_MAX_Q8_8; // clamp
         }
 
         // Clamp final pour sécurité
         if (i16_lead_base_target_q8_8 > LEAD_FINAL_MAX_Q8_8)
             i16_lead_base_target_q8_8 = LEAD_FINAL_MAX_Q8_8;
         if (i16_lead_base_target_q8_8 < LEAD_FINAL_MIN_Q8_8)
             i16_lead_base_target_q8_8 = LEAD_FINAL_MIN_Q8_8;
     }
 
     /* ---- 3. Gestion du reset si vitesse trop basse ---- */
     if (!low_speed_reset_done && u32_rpm < RPM_RESET_THRESHOLD) {
         i16_lead_correction_q8_8 = 0;
         i32_best_current_mA = MAX_CURRENT_MA;
         u8_stable_count = 0;
         low_speed_reset_done = true;
     }
     if (low_speed_reset_done && u32_rpm > (RPM_RESET_THRESHOLD + RPM_RESET_HYSTERESIS)) {
         low_speed_reset_done = false;
     }
 
     /* ---- 4. Slew rate sur le lead base ---- */
     static int16_t i16_prev_base_q8_8 = 0;
     int16_t i16_base_delta_q8_8 = i16_lead_base_target_q8_8 - i16_prev_base_q8_8;
 
     /*
      * Conversion ticks → équivalent millisecondes :
      *   1 tick = 4 µs = 0.004 ms
      *   1 s = 250000 ticks
      * Donc (>>18) ≈ ÷262144 ≈ ÷250000 pour un temps en secondes
      * => approximation suffisante et sans division coûteuse.
      */
     int16_t i16_max_base_delta_q8_8 =
         (int16_t)((int32_t)MAX_BASE_RATE_Q8_8 * ui32_delta_ticks >> 18);
 
     if (i16_base_delta_q8_8 > i16_max_base_delta_q8_8) i16_base_delta_q8_8 = i16_max_base_delta_q8_8;
     else if (i16_base_delta_q8_8 < -i16_max_base_delta_q8_8) i16_base_delta_q8_8 = -i16_max_base_delta_q8_8;
 
     i16_lead_base_q8_8 = i16_prev_base_q8_8 + i16_base_delta_q8_8;
     i16_prev_base_q8_8 = i16_lead_base_q8_8;
 
     /* ---- 5. Vérification stabilité du courant ---- */
     bool allow_adapt = false;
     int32_t i32_deviation = (int32_t)u32_current_mA - (int32_t)Idc_target_mA;
     if (i32_deviation < 0) i32_deviation = -i32_deviation;
 
     if (i32_deviation <= DEVIATION_ALLOWED_MA) {
         if (u8_stable_count < 0xFF) u8_stable_count++;
     } else {
         u8_stable_count = 0;
     }
 
     if (u8_stable_count >= STABILITY_REQUIRED_COUNT)
         allow_adapt = true;
 
     /* ---- 6. Adaptation de la correction ---- */
     if (allow_adapt && !low_speed_reset_done) {
         if (u32_current_mA < i32_best_current_mA) {
             i32_best_current_mA = u32_current_mA;  // nouveau minimum atteint
         } else {
             static bool direction_positive = true;
             if (direction_positive)
                 i16_lead_correction_q8_8 += CORRECTION_STEP_Q8_8;
             else
                 i16_lead_correction_q8_8 -= CORRECTION_STEP_Q8_8;
             direction_positive = !direction_positive;
         }
     }
 
     /* ---- 7. Slew rate sur la correction ESC ---- */
     static int16_t i16_prev_correction_q8_8 = 0;
     int16_t i16_delta_q8_8 = i16_lead_correction_q8_8 - i16_prev_correction_q8_8;
 
     /*
      * Même conversion : 1 tick = 4 µs
      *   >>18 ≈ division par 250000, cohérent avec l’échelle °/s
      */
     int16_t i16_max_corr_delta_q8_8 =
         (int16_t)((int32_t)MAX_CORRECTION_RATE_Q8_8 * ui32_delta_ticks >> 18);
 
     if (i16_delta_q8_8 > i16_max_corr_delta_q8_8) i16_delta_q8_8 = i16_max_corr_delta_q8_8;
     else if (i16_delta_q8_8 < -i16_max_corr_delta_q8_8) i16_delta_q8_8 = -i16_max_corr_delta_q8_8;
 
     i16_lead_correction_q8_8 = i16_prev_correction_q8_8 + i16_delta_q8_8;
     i16_prev_correction_q8_8 = i16_lead_correction_q8_8;
 
     /* ---- 8. Saturations ---- */
     if (i16_lead_correction_q8_8 > CORRECTION_MAX_Q8_8)
         i16_lead_correction_q8_8 = CORRECTION_MAX_Q8_8;
     if (i16_lead_correction_q8_8 < CORRECTION_MIN_Q8_8)
         i16_lead_correction_q8_8 = CORRECTION_MIN_Q8_8;
 
     /* ---- 9. Calcul du lead final ---- */
     i16_lead_final_q8_8 = i16_lead_base_q8_8 + i16_lead_correction_q8_8;
 
     if (i16_lead_final_q8_8 < LEAD_FINAL_MIN_Q8_8)
         i16_lead_final_q8_8 = LEAD_FINAL_MIN_Q8_8;
     if (i16_lead_final_q8_8 > LEAD_FINAL_MAX_Q8_8)
         i16_lead_final_q8_8 = LEAD_FINAL_MAX_Q8_8;
 
     // Version non signée 0..360° en Q8.8 pour l’ESC
     i16_lead_final_esc_q_8_8 = (uint16_t)i16_lead_final_q8_8;
 }
 
#endif // end DYNAMIC_LEAD_ANGLE == (2) ; ESC

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


// **************  to test slow rotation without using the hall sensor and so discover pattern sequence
// just to test rotation at a low speed and low power to verify the the hall sequence is OK
#define SPEED_COUNTER_MAX 19000 /360  // one electrical rotation per sec ; so 1 mecanical rotation takes 4 sec ; so 15 rpm
#define DUTY_CYCLE_TEST 30// 256 = 100% ; 40 gives a current = 1A from ADC on pin 2.8 with a 12V battery
#define ANGLE_INIT 0
// end of those test parameters


// ---------------- DEFINE for PLL ---------------- 
#define NB_SECTORS      6
#define MOTOR_POLE_PAIRS          4UL      // nombre de paires de pôles moteur

#define PWM_HZ          19000u
#define PLL_HZ          1000u
#define HALL_TIMEOUT_MS (50U)  // millis sec

/* timeout exprimé en nombre de ticks PWM (arrondi vers le haut) */
#define HALL_TIMEOUT_TICKS   ( (uint32_t)(((uint32_t)HALL_TIMEOUT_MS * (uint32_t)PWM_HZ + 999UL) / 1000UL) )


#define HALLS_PER_ELEC_REV 6
#define HALLS_PER_MECH_REV (HALLS_PER_ELEC_REV * MOTOR_POLE_PAIRS)

#define KP_Q4_12   (20)
#define KI_Q2_14   (2)

#define DEG_TO_Q8_8(d) ((int32_t)((d) * 65536 / 360))
#define HALL_OFFSET_DEG (30)
#define HALL_OFFSET_Q8_8 ( (uint16_t)DEG_TO_Q8_8(HALL_OFFSET_DEG) )

#define SNAP_THRESHOLD_Q8_8   DEG_TO_Q8_8(40)
#define BLEND_THRESHOLD_DEG   5
#define SPEED_FRAC_BITS 8
#define BLEND_THRESHOLD_Q8_16 ( (DEG_TO_Q8_8(BLEND_THRESHOLD_DEG)) << SPEED_FRAC_BITS )
#define BLEND_SHIFT 2
#define MAX_ANGLE_BETWEEN_HALL_Q8_8 DEG_TO_Q8_8(70)
#define INTEGRATOR_MAX_Q8_8  (1500)
#define INTEGRATOR_MIN_Q8_8  (-1500)
#define MAX_SPEED_Q8_16  (400000)
#define MIN_SPEED_Q8_16  (0)
#define PWM_TICK_US     (1000000u / PWM_HZ)

#define RPM_LOW_SPEED   (100)   // below this speed, at each hall front, rotor position = hall centered (+ no interpolation)
                                // just above, at each hall front, rotor position = hall (+ interpolation based on previous sector speed
#define RPM_START_PLL_SPEED   (300)   // above this speed, at each front position, rotor position is partly corrected (bend) when hall is in advance more than a threshold
                                    // interpolation is done with speed pll correction 
#define RPM_HIGH_SPEED   (1500)  // above high, rotor position is not corrected at each front
                                // interpolation is done with speed pll correction
#define RPM_MIN_SPEED_TO_CALIBRATE_HALL (500) // calibrate hall sensor only when this speed has been reached

/* convert RPM -> dt_us threshold between successive Hall transitions (microseconds) */
#define RPM_TO_DT_US(rpm)   ( (uint32_t)(60000000UL / ((uint32_t)(rpm) * (uint32_t)HALLS_PER_MECH_REV)) )

#define DT_US_LOW_SPEED RPM_TO_DT_US(RPM_LOW_SPEED)
#define DT_US_START_PLL_SPEED  RPM_TO_DT_US(RPM_START_PLL_SPEED)
#define DT_US_HIGH_SPEED  RPM_TO_DT_US(RPM_HIGH_SPEED)
#define DT_US_TO_CALIBRATE  RPM_TO_DT_US(RPM_MIN_SPEED_TO_CALIBRATE_HALL)

// ++++++++++++ DEFINE for clarck and park (optimising lead angle based on Id) // some are used only if cordic is used
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

// Hall positions in Q8.8
// Position rotorique in Q8.8 et vitesse en Q16.16
typedef int32_t q16_16_t; // (signed) (16 bits for decimal, 16bits for unit, 1 unit = 360/256 = 1.4°)
/* Q8.8 typedefs */
typedef int16_t q8_8_t;     // valeur signée Q8.8 (−128..+127.996) si on veut signed
typedef uint16_t uq8_8_t;   // valeur non signée Q8.8 (0..255.996) pour index / LUT

// for hall position (pattern , sector, angle 
uq8_8_t ui16_curr_base_angle_q8_8 = 0;  // position of hall at the begin of the current sector
uint8_t ui8_curr_sector = 0; //
uint8_t ui8_prev_sector = 0; 
uint8_t ui8_curr_hall_pattern = 0;
volatile uint8_t ui8_hall_sensors_state = 0; // name used by ebike_app.c to identify ui8_curr_hall_pattern; added here for compatibility
uint16_t hall_pattern_error_counter = 0; // to debug only

uint8_t hall_reference_angle = 0 ; // !! Is not in Q8_8 but only in uint8 ;This value is initialised in ebike_app.c with DEFAULT_HALL_REFERENCE_ANGLE and m_config.global_offset_angle 
// Hall offset for current Hall state; This offset is added in the interpolation process (so based also on the erps)
// the value is in ticks = usec ; we need  about 60 usec : 
//     55usec = delay between measuring at begin of ISR0 and applying PWM change at end of PWM cycle;
//     There is also some delay in hall sensor but it is probably included in hall calibration process
// based on the regression tests, there should probably be a correction of about 2 depending it is a rising or a falling edge of hall pattern
// still this should have only a small impact
//uint8_t ui8_hall_counter_offset = 60;  !! currently not used in PLL interpol !!!!!!!!!!!!!

// to calculate Id and optimise lead angle
uint16_t ui16_angle_for_id_prev_q8_8; // position; saved at begin of ISR 0 to match with current iu,Iv,iw measured at begin of ISR 1
volatile uint16_t ADC_Bias_Iu = 1 << 11; // ADC is 12 bits, 0 = mid point 
volatile uint16_t ADC_Bias_Iv = 1 << 11; // ADC is 12 bits, 0 = mid point 
volatile uint16_t ADC_Bias_Iw = 1 << 11; // ADC is 12 bits, 0 = mid point 

// used in systick to optimise lead angle based on average Id, Iq 
int32_t i32_id_sum = 0;     // accumulate Id to use an average in systick based on 64 values
int32_t i32_iq_sum = 0;     // idem for Iq
uint8_t ui8_id_iq_counter = ID_IQ_COUNTER; // 64 ; used to filter id & iq ; pwm at 19kHz and systick at 200Hz => 19000/200 = 95 measurements

// to debug or used with cordic
int16_t I_u; // to check current in each phase
int16_t I_v;
int16_t I_w;
int16_t I_t;



// for current calculation
uint32_t ui32_adc_battery_current_15b = 0; // value from adc
volatile uint8_t ui8_adc_battery_current_filtered = 0; // current in adc10 bits units (= moving average on 64 PWM cycles)

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

// to debug time spent in irq0 and irq1
volatile uint16_t debug_time_ccu8_irq0 = 0;
//volatile uint16_t debug_time_ccu8_irq1 = 0;
//volatile uint16_t debug_time_ccu8_irq1b = 0;
//volatile uint16_t debug_time_ccu8_irq1c = 0;
//volatile uint16_t debug_time_ccu8_irq1d = 0;
//volatile uint16_t debug_time_ccu8_irq1e = 0;

uint16_t irq0_min = 0xFFFF;
uint16_t irq0_max = 0;
uint16_t irq1_min = 0xFFFF;
uint16_t irq1_max = 0;

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


// to debug isr 
volatile uint32_t debug_isr1_timer_start = 0;
volatile uint32_t debug_isr1_timer_end = 0;
volatile uint32_t debug_isr0_timer_start = 0;
volatile uint32_t debug_isr0_timer_end = 0;

//to check that adc conversion has been done when reading
volatile uint32_t ui32_adc_conversion_gr0 = 0;  
volatile uint32_t ui32_adc_conversion_gr1 = 0;


//+++++++++++++++  for cadence (and torque when spider logic is used)
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


        
// this function has to be called in ISR0 or ISR1 (at 19kHz) to collect the data that are processed in a systick irq at 1kHz
static inline __attribute__((always_inline))  void collect_wheel_cadence_data(){
    static uint8_t ui8_prev_cadence_state = 0;   // 2 bits combinés prev A/B
    static uint8_t ui8_prev_wheel_state = 0;
        
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



inline uint32_t update_moving_average(uint32_t new_value){
    battery_current_moving_avg_sum -= battery_current_moving_avg_buffer[battery_current_moving_avg_index];
    battery_current_moving_avg_buffer[battery_current_moving_avg_index] = new_value;
    battery_current_moving_avg_sum += new_value;
    battery_current_moving_avg_index = (battery_current_moving_avg_index + 1) & 0x3F; 
    // Retourne la moyenne actuelle
    return (battery_current_moving_avg_sum + 32) >> 6; // divide by 64; add 32 for better rounding
}
/*
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
*/


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


// =========  filtrage sans reliquat du au calcul en entier =========
//int32_t diff = omega_mech - i32_omega_est X256;
//int32_t delta = diff >> OMEGA_ALPHA_SHIFT;
//if (delta == 0 && diff != 0)  delta = (diff > 0) ? 1 : -1;
//i32_omega_est X256 += delta;


void capture_3_phase_current_offset(){  // called by main
    // when motor is blocked since some time, we update first the ADC bias for Iu, iv, iW
    // when motor is not running (based on ui8_motor_enabled) we reset foc and foc PID
    // when motor is running we use a PI based on ID (calculated and filtered in ISR) to update FOC angle
    // in a second step we can calculate a value for foc angle based on rpm and current and apply pid as a correction.
    #define ADC_BIAS_SHIFT (9)
    #define SHIFT_BIAS_CURRENT_LPF 7

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

// ++++++++++++   for park transfom when sinus table is used +++++++++++++++++
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



// ************************************** begin of IRQ *************************
// *************** irq 0 of ccu8   
//   it takes between 5 and 18 usec (so less than 26 usec)
__RAM_FUNC void CCU80_0_IRQHandler(void)
{
    static uint16_t ui16_prev_hall_ticks = 0;
    static uint8_t ui8_prev_hall_pattern = 0;
    static bool valid_prev_hall_ticks = false;
    static bool first_run_in_PWM_ISR = true;

    // to set flux position
    uint16_t ui16_angle_no_ref_no_lead_q8_8;
    uint16_t ui16_angle_no_lead_q8_8;  // angle based on Hall or hybrid with ref angle but no lead angle
    //uint16_t ui16_hall_angle_no_ref_no_lead_q8_8;
    uint16_t ui16_SVM_table_index_q8_8;
    int16_t svm_A = 0;
    int16_t svm_B = 0;
    int16_t svm_C = 0;

    uint8_t ui8_curr_hall_pattern_local = ui8_curr_hall_pattern;   // local copy just for faster processing
    
    // read irq data before reading current time stamp (to be sure that time now follow the ISR timestamp)
    hall_sample_t hall_isr_sample_local;
    bool hall_event_pending_local = hall_event_pending;
    hall_isr_sample_local.raw = hall_irq_sample.raw;// Lecture atomique (32 bits)
        
    // get now timestamp
    uint16_t ui16_curr_ISR0_ticks = (uint16_t) (XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW) );
    
    debug_isr0_timer_start = XMC_CCU8_SLICE_GetTimerValue(PWM_IRQ_TIMER_HW);

    //uint16_t curr_ticks = (uint16_t)(XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW));
    //uint8_t curr_pattern = ui8_curr_hall_pattern;
    //hall_sample_t hs_local = hall_irq_sample;     // lecture atomique
    //bool hall_pending = hall_event_pending;

    if (first_run_in_PWM_ISR) {
        first_run_in_PWM_ISR = false;
        ui8_curr_hall_pattern_local = read_hall_pattern();
        ui8_curr_sector = hall_to_sector[ui8_curr_hall_pattern_local];               // get current sector
        ui16_curr_base_angle_q8_8 =  ui16_base_sector_q8_8[ui8_curr_sector] ;  // get current base angle
        ui8_prev_sector = (ui8_curr_sector>0) ? ui8_curr_sector-1 : 5;        // get previous sector
        pll_init(); // on initialise le PLL sur le secteur courant
    }

    if (hall_event_pending_local) { //set on true in hall ISR when a new hall pattern occured
        hall_event_pending = false; // reset flag localement
        // split hall ISR data (pattern & timestamp)
        uint16_t ui16_curr_hall_ticks = hall_isr_sample_local.ticks; // Extraction from ISR
        ui8_curr_hall_pattern_local = hall_isr_sample_local.pattern;  

        ui8_curr_sector = hall_to_sector[ui8_curr_hall_pattern_local];               // get current sector
        ui16_curr_base_angle_q8_8 =  ui16_base_sector_q8_8[ui8_curr_sector] ;  // get current base angle
        // get previous sector
        ui8_prev_sector = (ui8_curr_sector>0) ? ui8_curr_sector-1 : 5;
        //ui16_prev_base_angle_q8_8 = ui16_base_sector_q8_8[ui8_prev_sector];  // get base of previous sector
        
        // Vérification de séquence
        bool seq_ok = (ui8_curr_hall_pattern_local == expected_pattern_table[ui8_prev_hall_pattern]);

        // Calcul dt seulement si timestamp précédent valide
        uint16_t ui16_us_between_2_hall_fronts = 0;
        if (valid_prev_hall_ticks && seq_ok) {
            ui16_us_between_2_hall_fronts = ui8_curr_hall_pattern_local - ui16_prev_hall_ticks;
            if (ui16_us_between_2_hall_fronts == 0) ui16_us_between_2_hall_fronts = 1;
        }
        
        // call even when sequence is wrong or when no valid_ui16_prev_hall_ticks
        pll_on_hall_event(ui16_us_between_2_hall_fronts, (uint16_t) ui16_curr_base_angle_q8_8, seq_ok);       // gère phase, speed, timeout

        if (!seq_ok) {
            if (hall_calib_state == HALL_CALIBRATING) hall_calib_state = HALL_TO_CALIBRATE;
        } else if ((hall_calib_state == HALL_TO_CALIBRATE) && (ui16_us_between_2_hall_fronts < DT_US_TO_CALIBRATE)) { // vitesse > 500 rpm
            hall_calib_state = HALL_CALIBRATING;
            hall_cal_total_count = 6*200;
            for(uint8_t i=0;i<6;i++) { hall_cal_sum[i]=0; hall_cal_count[i]=0; }
        } else if ((hall_calib_state == HALL_CALIBRATING) && valid_prev_hall_ticks) {
            hall_cal_sum[ui8_prev_sector] += ui16_us_between_2_hall_fronts;
            hall_cal_count[ui8_prev_sector]++;
            hall_cal_total_count--;
            if (hall_cal_total_count == 0) hall_calib_state = HALL_MEASURED;
        }

        ui16_prev_hall_ticks = ui16_curr_hall_ticks;
        valid_prev_hall_ticks = true;
        ui8_prev_hall_pattern = ui8_curr_hall_pattern_local;  // used to check the sequence
    } // end of new hall front registerd in hall ISR

    // ---- incrémentation phase par step même si erreur séquence ----
    pll_on_pwm_tick();

    // ---- publication angle FOC/SVM ----
    ui16_angle_no_ref_no_lead_q8_8 = pll_get_angle_q8_8();
    ui16_angle_no_lead_q8_8 = ui16_angle_no_ref_no_lead_q8_8 + ((uint16_t)hall_reference_angle<<8);
    ui16_SVM_table_index_q8_8 = ui16_angle_no_lead_q8_8 + ui16_lead_total_q8_8;

    ui8_curr_hall_pattern = ui8_curr_hall_pattern_local;
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


/*  !!!!!!!!!! perhaps better to do it like infineon
// still to disable, perhaps better to set all pins used by PWM to LOW level instead of tristate (same level as when PWM is disabled)
// to check what happens in debug mode when cpu is halted (what happens with PWM pins???)

void pmsm_foc_disable_inverter(void){
    // added by MStrens to discard INVERTER_EN_PIN for TSDZ8
    #ifdef INVERTER_EN_PIN
    XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, DISABLE_LEVEL); // Disable gate driver. 
    #endif
    XMC_GPIO_SetMode(PHASE_U_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    XMC_GPIO_SetMode(PHASE_U_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    XMC_GPIO_SetMode(PHASE_V_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    XMC_GPIO_SetMode(PHASE_V_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    XMC_GPIO_SetMode(PHASE_W_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    XMC_GPIO_SetMode(PHASE_W_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
    Motor.Inverter_status = 0;
  
  }
  void pmsm_foc_enable_inverter(void){
    // added by MStrens to discard INVERTER_EN_PIN for TSDZ8
    #ifdef INVERTER_EN_PIN
    XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, ENABLE_LEVEL); // Enable gate driver.
    #endif
    XMC_GPIO_SetMode(PHASE_U_HS_PIN, PHASE_U_HS_ALT_SELECT);
    XMC_GPIO_SetMode(PHASE_U_LS_PIN, PHASE_U_LS_ALT_SELECT);
    XMC_GPIO_SetMode(PHASE_V_HS_PIN, PHASE_V_HS_ALT_SELECT);
    XMC_GPIO_SetMode(PHASE_V_LS_PIN, PHASE_V_LS_ALT_SELECT);
    XMC_GPIO_SetMode(PHASE_W_HS_PIN, PHASE_W_HS_ALT_SELECT);
    XMC_GPIO_SetMode(PHASE_W_LS_PIN, PHASE_W_LS_ALT_SELECT);
    Motor.Inverter_status = 1;
  
  }
*/  

void motor_enable_pwm(void) { //set posif with current position & restart the timers
    get_curr_hall_pattern(); // refresh hall pattern in ui8_curr_hall_pattern
    
    // one solution to activate is to generate an event that starts all timers in a synchronized way
    // Enable Global Start Control CCU80  in a synchronized way
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

void get_curr_hall_pattern(){  // use to initialise at power on and in motor_enable()
    ui8_curr_hall_pattern = read_hall_pattern();
}


// ----------- here code used to manage pll ----------------

/* ---------------- Volatile / ISR shared ---------------- */
volatile uint16_t g_pll_phase_q8_8 = 0; // is best position

/* ---------------- PLL state ---------------- */
typedef struct {
    uint32_t phase_acc_q8_16;
    uint32_t phase_acc_max_q8_16;
    volatile int32_t  speed_q8_16;
    int32_t  integrator_q8_8;

    uint16_t last_hall_phase_q8_8;
    uint16_t prev_hall_phase_q8_8;

    uint32_t pwm_counter;          // compteur global PWM
    uint32_t last_hall_pwm_count;  // compteur capturé au dernier front hall

    uint8_t  hall_transition_count;
} pll_state_t;

pll_state_t pll;

/* ---------------- Helpers ---------------- */
//static inline int32_t i32_abs(int32_t v) { return (v < 0) ? -v : v; }

inline int32_t phase_diff_q8_8(uint16_t target, uint16_t current)
{
    int32_t d = (int32_t)target - (int32_t)current;
    if (d >  32767) d -= 65536;
    if (d < -32768) d += 65536;
    return d;
}

/* ---------------- Init ---------------- */
void pll_init(void)
{
    // se baser sur le secteur courant et la base angle déjà préparés
    uint16_t init_phase_q8_8 = ui16_curr_base_angle_q8_8;
    pll.phase_acc_q8_16 = ((uint32_t)init_phase_q8_8) << SPEED_FRAC_BITS;
    pll.phase_acc_max_q8_16 = pll.phase_acc_q8_16 + ((uint32_t)MAX_ANGLE_BETWEEN_HALL_Q8_8 << SPEED_FRAC_BITS);
    pll.speed_q8_16 = 0;
    pll.integrator_q8_8 = 0;
    pll.last_hall_phase_q8_8 = init_phase_q8_8;
    pll.prev_hall_phase_q8_8 = init_phase_q8_8;  // this is not really correct but it does not matter; will be filled at next front
    pll.hall_transition_count = 0;
    g_pll_phase_q8_8 = init_phase_q8_8;
    pll.pwm_counter = 0;
    pll.last_hall_pwm_count = 0;
}


/* ---------------- PWM tick integration (always called every PWM) ---------------- */
// called at each PWM cycle
// check for timeout (more than 50 msec since previous front -  check done in PWM ticks)
// interpolate position (accumulated for better precision in speed step)
// clamp to a max value (usualy nex hall position + some margin)
// avoid moving position backward
// Store position in Q8.8 in g_pll_phase_q8_8
inline void pll_on_pwm_tick(void)
{
    pll.pwm_counter++;
    if ((pll.pwm_counter - pll.last_hall_pwm_count) > HALL_TIMEOUT_TICKS) {

        uint16_t hall_center_q8_8 = pll.last_hall_phase_q8_8 + HALL_OFFSET_Q8_8;
        pll.phase_acc_q8_16 = ((uint32_t)hall_center_q8_8) << SPEED_FRAC_BITS;
        pll.phase_acc_max_q8_16 = pll.phase_acc_q8_16 + ((uint32_t)MAX_ANGLE_BETWEEN_HALL_Q8_8 << SPEED_FRAC_BITS);
        pll.speed_q8_16 = 0;
        pll.integrator_q8_8 = 0;
        pll.hall_transition_count = 0;
        // IMPORTANT: remettre la référence pour éviter rebouclage immédiat
        pll.last_hall_pwm_count = pll.pwm_counter;
    }
    
    pll.phase_acc_q8_16 = (uint32_t)((int32_t)pll.phase_acc_q8_16 + pll.speed_q8_16);

    if (pll.phase_acc_q8_16 > pll.phase_acc_max_q8_16) pll.phase_acc_q8_16 = pll.phase_acc_max_q8_16;
    uint32_t phase_acc_min = ((uint32_t)pll.last_hall_phase_q8_8) << SPEED_FRAC_BITS;
    if (pll.phase_acc_q8_16 < phase_acc_min) pll.phase_acc_q8_16 = (uint32_t)phase_acc_min;

    g_pll_phase_q8_8 = pll.phase_acc_q8_16 >> SPEED_FRAC_BITS;
}


/* ---------------- Hall event from ISR PWM (dt_us already computed) ---------------- */
// called when a hall front occurs, update phase_acc, speed to be used by ISR0; (keep also integrator for pll)
// apply different rules:
// when sequence is wrong, discard (keep phase and speed)
// When low number of transitions (at start) , position = hall centerd and speed = 0
// when phase error is too big (and positive), SNAP = use hall position and speed from previous sector
// when speed is low, use hall position centered + speed = 0
// when speed is lower than start pwm, use hall position + speed from previous sector
// when speed is lower than high and error is bigger than threshold (positive), use pll  position + part of error and corrected speed
// Other (speed high or (mid+ no big positive error)), use ony pll position and corrected speed
inline void pll_on_hall_event(uint16_t dt_us, uint16_t hall_phase_q8_8, bool seq_ok)
{
    pll.prev_hall_phase_q8_8 = pll.last_hall_phase_q8_8;
    pll.last_hall_phase_q8_8 = hall_phase_q8_8;

    // --- update pwm counter to avoid timeout
    pll.last_hall_pwm_count = pll.pwm_counter;

    if (!seq_ok) return; // keep phase & speed, next PWM increments by speed

    // when speed is unknow, phase is aligned on hall + 30° and speed is set on 0 (no interpolation)
    if (pll.hall_transition_count < 2) {
        uint16_t hall_center_q8_8 = hall_phase_q8_8 + HALL_OFFSET_Q8_8;
        pll.phase_acc_q8_16 = ((uint32_t)hall_center_q8_8) << SPEED_FRAC_BITS;
        pll.phase_acc_max_q8_16 = pll.phase_acc_q8_16 + ( (uint32_t)MAX_ANGLE_BETWEEN_HALL_Q8_8 << SPEED_FRAC_BITS );
        pll.speed_q8_16 = 0;
        pll.integrator_q8_8 = 0;
        pll.hall_transition_count++;
        return;
    }
    // calculate measured speed based on previous sector
    if (dt_us == 0) dt_us = 1;
    int32_t delta_phase_q8_8 = phase_diff_q8_8(pll.last_hall_phase_q8_8, pll.prev_hall_phase_q8_8);
    int32_t speed_meas_q8_16 = (delta_phase_q8_8 * (int32_t)PWM_TICK_US * (1<<SPEED_FRAC_BITS)) / dt_us;

    // calculate phase error
    int32_t phase_est_q8_8 = (int32_t)(pll.phase_acc_q8_16 >> SPEED_FRAC_BITS);
    int32_t e_q8_8 = phase_diff_q8_8(pll.last_hall_phase_q8_8, (uint16_t)phase_est_q8_8);

    // when error is big, phase is aligned on hall and speed is based only on previous sector
    if (e_q8_8 > SNAP_THRESHOLD_Q8_8) {
        uint16_t hall_center_q8_8 = hall_phase_q8_8 + HALL_OFFSET_Q8_8;
        pll.phase_acc_q8_16 = ((uint32_t)hall_center_q8_8) << SPEED_FRAC_BITS;
        pll.phase_acc_max_q8_16 = pll.phase_acc_q8_16 + ( (uint32_t)MAX_ANGLE_BETWEEN_HALL_Q8_8 << SPEED_FRAC_BITS );
        pll.speed_q8_16 = speed_meas_q8_16;
        if (pll.speed_q8_16 > MAX_SPEED_Q8_16) pll.speed_q8_16 = MAX_SPEED_Q8_16;
        else if (pll.speed_q8_16 < MIN_SPEED_Q8_16) pll.speed_q8_16 = MIN_SPEED_Q8_16;
        pll.integrator_q8_8 = 0;
    } else {
        // apply pll on speed
        int32_t p_term_q8_8 = (e_q8_8 * KP_Q4_12) >> 4;
        int32_t i_delta_q8_8 = (e_q8_8 * KI_Q2_14) >> 6;
        pll.integrator_q8_8 += i_delta_q8_8;
        if (pll.integrator_q8_8 > INTEGRATOR_MAX_Q8_8) pll.integrator_q8_8 = INTEGRATOR_MAX_Q8_8;
        else if (pll.integrator_q8_8 < INTEGRATOR_MIN_Q8_8) pll.integrator_q8_8 = INTEGRATOR_MIN_Q8_8;

        int32_t speed_with_pll_correction_q8_16 = speed_meas_q8_16 + ((p_term_q8_8 + pll.integrator_q8_8)<<SPEED_FRAC_BITS);
        if (speed_with_pll_correction_q8_16 > MAX_SPEED_Q8_16) speed_with_pll_correction_q8_16 = MAX_SPEED_Q8_16;
        else if (speed_with_pll_correction_q8_16 < MIN_SPEED_Q8_16) speed_with_pll_correction_q8_16 = MIN_SPEED_Q8_16;

        pll.speed_q8_16 = speed_with_pll_correction_q8_16;

        uint16_t hall_center_q8_8 = hall_phase_q8_8 + HALL_OFFSET_Q8_8;
        // at low speed, 
        if (dt_us > DT_US_LOW_SPEED) { // very low speed  = use only hall position (centered)
            pll.phase_acc_q8_16 = ((uint32_t)hall_center_q8_8) << SPEED_FRAC_BITS;
            pll.phase_acc_max_q8_16 = pll.phase_acc_q8_16 + ( (uint32_t)MAX_ANGLE_BETWEEN_HALL_Q8_8 << SPEED_FRAC_BITS );
            pll.speed_q8_16 = 0;
            pll.integrator_q8_8 = 0;
        } else if (dt_us > DT_US_START_PLL_SPEED) { // speed is less than required for PLL: use hall position with interpolation based on last sector speed
            pll.phase_acc_q8_16 = ((uint32_t)hall_center_q8_8) << SPEED_FRAC_BITS;
            pll.phase_acc_max_q8_16 = pll.phase_acc_q8_16 + ( (uint32_t)MAX_ANGLE_BETWEEN_HALL_Q8_8 << SPEED_FRAC_BITS );
            pll.speed_q8_16 = speed_meas_q8_16; 
            pll.integrator_q8_8 = 0;
        } else if (dt_us > DT_US_HIGH_SPEED) { // speed is less than high speed : blend phase if error is above a threshold, interpolate speed with correction
            int32_t e_q8_16 = (int32_t)e_q8_8 << SPEED_FRAC_BITS;
            if (e_q8_16 > BLEND_THRESHOLD_Q8_16) {  // only when positive (when hall is in advance)
                pll.phase_acc_q8_16 += (uint32_t)(e_q8_16 >> BLEND_SHIFT);
                pll.phase_acc_max_q8_16 = pll.phase_acc_q8_16 + ( (uint32_t)MAX_ANGLE_BETWEEN_HALL_Q8_8 << SPEED_FRAC_BITS );
            }
        } 
        // else = high speed or small error: no blend and interpolate with speed including pll correction
    }

    if (pll.hall_transition_count < 255) pll.hall_transition_count++;
    
}

/* ---------------- Accessors ---------------- */
inline uint16_t pll_get_angle_q8_8(void) { return g_pll_phase_q8_8; }

// erps = pll.speed_q8_16 * 6 * PWM_HZ / (1<<24); Num exceed 1<<31, so we split 24 in 2 part 6 + 18
inline uint16_t pll_get_erps(void){
    #define PLL_SPEED_TO_ERPS_SHIFT18 (((uint32_t)(6U*PWM_HZ)) >> 6) 
    return (uint16_t)((pll.speed_q8_16 * PLL_SPEED_TO_ERPS_SHIFT18) >> 18);
}

// velocity is in Q8.8 X1024
uint32_t pll_get_velocity(void){
    return (uint32_t) ((((uint32_t) pll.speed_q8_16) * 4891) >> 16) ; // *4891 / 65536 = 0,076 to get velocity units
}

// reset time out ; fill last count with current value of pwm cunter
void reset_pll_timeout(){
    pll.last_hall_pwm_count = pll.pwm_counter;   
}
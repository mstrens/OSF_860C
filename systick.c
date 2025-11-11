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



#define PWM_HZ           19000UL
volatile uint32_t ui32_ms_counter = 0;

// new wheel and cadence variables
// =============== VARIABLES PARTAGÉES =============== 
volatile uint32_t ui32_pwm_ticks = 0;          // compteur soft 19kHz
volatile uint32_t ui32_cadence_last_ticks[6] = {0};   // timestamps pédalage (codes 0..5)
volatile uint32_t ui32_wheel_last_pwm_ticks = 0; // dernier front roue (ui32_pwm_ticks)

uint8_t lead_angle_multiplicator = 64;

//uint16_t ui16_debug_fw_cnt= 0;
//int8_t i8_debug_idx_ref = -2;
//uint32_t ui32_debug_delta_ticks = 0;

// this function is called in systick ISR (at 1kHz) 
// it calculates wheel and cadence ticks using the data collected at 19 kHz; so ticks are at PWM frequency
// conversion to rpm is done is ebike.app
void SysTick_Handler(void) {
    
    // --- Wheel --- 
    static uint32_t ui32_prev_wheel_pwm_tick = 0;
    static uint32_t ui32_last_wheel_ms = 0;
    // --- cadence --- 
    static int8_t i8_prev_cadence_index = -1;      // -1 = pas encore de référence
    static uint32_t ui32_prev_cadence_tick = 0;
    static uint32_t ui32_last_cadence_ms = 0;
    static uint32_t ui32_prev_cadence_tick_max = 0;          // pour détecter un vrai nouveau front

//  ========= only for documentation if we have to use pwm ticks
//static inline uint32_t read_ui32_pwm_ticks_atomic(void) {
//    uint32_t a,b;
//    do { a = ui32_pwm_ticks; b = ui32_pwm_ticks; } while (a != b);
//    return a;
//}
    ui32_ms_counter++;  // used to detect timeout
    // --------- 1) cadence --------- 
    // Cherche l’index (0..4) ayant le timestamp le plus grand
    uint8_t ui8_cadence_idx_max = 0;
    uint32_t ui32_cadence_tick_max = ui32_cadence_last_ticks[0];
    for (uint8_t i = 1; i <= 4; ++i) {
        uint32_t t = ui32_cadence_last_ticks[i];
        if(t > ui32_cadence_tick_max) { ui32_cadence_tick_max = t; ui8_cadence_idx_max = i; }
    }

    // Check if a new cadence event occured
    if (ui32_cadence_tick_max != ui32_prev_cadence_tick_max) {
        ui32_prev_cadence_tick_max = ui32_cadence_tick_max;
        if (ui8_cadence_idx_max == 4) { // --- reverse cadence rotation ---
            ui16_cadence_sensor_ticks = 0; // reset value used in ebike_app.c
            i8_prev_cadence_index = -1;
             ui8_pas_new_transition = 0x80; // used in mspider logic for torque sensor // to do
        } else { // --- forward cadence (codes 0..3) ---
            //ui16_debug_fw_cnt++;
            if (i8_prev_cadence_index < 0) {   // Premier front après arrêt → initialise seulement
                i8_prev_cadence_index = (int8_t)ui8_cadence_idx_max;
                //i8_debug_idx_ref = i8_prev_cadence_index;
                ui32_prev_cadence_tick = ui32_cadence_tick_max;
                ui8_pas_counter = 0; // mstrens :  reset the counter for full rotation used to detect a full rotation for torque (spider)
            } else { // On a déjà une référence
                uint32_t ui32_curr_cadence_tick = ui32_cadence_last_ticks[i8_prev_cadence_index]; 
                // if tick for same index is different, then calculate elapsed ticks
                if (ui32_curr_cadence_tick != ui32_prev_cadence_tick) {
                    uint32_t ui32_cadence_delta_ticks = ui32_curr_cadence_tick  - ui32_prev_cadence_tick;
                    //ui32_debug_delta_ticks = ui32_cadence_delta_ticks ; 
                    ui16_cadence_sensor_ticks = (uint16_t) ui32_cadence_delta_ticks;
                    ui32_prev_cadence_tick =  ui32_curr_cadence_tick;
                    
                    ui8_pas_new_transition = 1; // mspider logic for torque sensor;mark for one of the 20 transitions per rotation
                    ui8_pas_counter++; // mstrens : increment the counter when the transition is valid           
                } else {
                    // when max timestamp changed (but not yet the timestamp of reference transition)
                    //  set the cadence to 7 RPM for immediate start if it was 0
                    if (ui16_cadence_sensor_ticks == 0) ui16_cadence_sensor_ticks = CADENCE_TICKS_STARTUP; // 7619
                }
            }
        }
        ui32_last_cadence_ms = ui32_ms_counter;
    }

    // cadence TIMEOUTS --------- 
    if ((ui32_ms_counter - ui32_last_cadence_ms) > (ui16_cadence_ticks_count_min_speed_adj)) { // adj =4270 at 4km/h ... 341 at 40 km/h
        ui16_cadence_sensor_ticks = 0; // reset cadence
        i8_prev_cadence_index = -1;
        ui32_prev_cadence_tick = 0;
        ui8_pas_new_transition = 0x80; // for mspider logic for torque sensor
        ui8_pas_counter = 0; // mstrens :  reset the counter for full rotation
    }
     
    // --------- 2) Wheel --------- 
    uint32_t ui32_wheel_pwm_tick = ui32_wheel_last_pwm_ticks;
    if (ui32_wheel_pwm_tick != ui32_prev_wheel_pwm_tick) {
        uint32_t ui32_wheel_delta_ticks;
        if (ui32_prev_wheel_pwm_tick == 0) {
            ui32_wheel_delta_ticks = 0; // first ticks after a stop
        } else {
            ui32_wheel_delta_ticks = (ui32_wheel_pwm_tick - ui32_prev_wheel_pwm_tick);
        }
        ui32_prev_wheel_pwm_tick = ui32_wheel_pwm_tick;
        ui32_last_wheel_ms = ui32_ms_counter;
        if (ui32_wheel_delta_ticks > 0) {
            // set the value used in ebike_app.c to wheel speed
            ui16_wheel_speed_sensor_ticks = ui32_wheel_delta_ticks ; // ticks are based on PWM frequency
        }
    }

    // wheel TIMEOUTS --------- 
    if ((ui32_ms_counter - ui32_last_wheel_ms) > (WHEEL_SPEED_SENSOR_TICKS_COUNTER_MIN/19)) {
        ui16_wheel_speed_sensor_ticks = 0; // reset wheel speed
        ui32_prev_wheel_pwm_tick = 0;
    }

    //      3)  get raw adc torque sensor (in 10 bits) and filter
    uint16_t ui16_adc_torque_raw   = (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , VADC_TORQUE_RESULT_REG ) & 0xFFF) >> 2; // torque gr0 ch7 result 7 in bg p2.2
    //filter it (3 X previous + 1 X new)
    uint16_t ui16_adc_torque_new_filtered = ( ui16_adc_torque_raw + (ui16_adc_torque_filtered<<1) + ui16_adc_torque_filtered) >> 2;
    if (ui16_adc_torque_new_filtered == ui16_adc_torque_filtered){ // code to ensure it reaches the limits
        if ( ui16_adc_torque_new_filtered < ui16_adc_torque_raw) 
            ui16_adc_torque_new_filtered++; 
        else if (ui16_adc_torque_new_filtered > ui16_adc_torque_raw) 
            ui16_adc_torque_new_filtered--;
    }
    ui16_adc_torque_filtered = ui16_adc_torque_new_filtered;
    
    //      4) get the voltage
     //ui16_adc_voltage  = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 4 ) & 0x0FFF) >> 2; // battery gr1 ch6 result 4
    // changed to take care of infineon VADC init (result in reg 6)
    ui16_adc_voltage  = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , VADC_VDC_RESULT_REG ) & 0x0FFF) >> 2; // battery gr1 ch6 result 6

    //     5) get lead angle 
    static uint32_t ui32_foc_angle_accum = 0; // use more bits for better accuracy in IIR
    // update foc_angle and adc_motor_phase_current
        // foc_angle is added to the position given by hall sensor + interpolation )
        if (ui8_g_duty_cycle > 0) {
            // calculate phase current.
            if (ui8_g_duty_cycle > 2) {
                ui16_adc_motor_phase_current = (uint16_t)((uint16_t)(((uint16_t)ui8_adc_battery_current_filtered) << 8)) / ui8_g_duty_cycle;
            } else {
                ui16_adc_motor_phase_current = (uint16_t)ui8_adc_battery_current_filtered;
            }
//            if (ui8_foc_flag) { // is set on 1 when rotor is at 150° so once per electric rotation
				//uint16_t ui16_adc_foc_angle_current = ((uint16_t)(ui8_adc_battery_current_filtered ) + (ui16_adc_motor_phase_current )) >> 1;
                // mstrens : added 128 for better rounding
                //ui8_foc_flag = ((ui16_adc_foc_angle_current * ui8_foc_angle_multiplicator) + 128) >> 8 ; // multiplier = 39 for 48V tsdz2, 
                // foc based on current
                //uint16_t ui8_foc = ((uint16_t) ui8_adc_battery_current_filtered * (uint16_t) ui8_foc_angle_multiplicator)  ; // multiplier = 39 for 48V tsdz2, 
                // foc based on velocity
                // ratio RPM to velocity is 4,474; so for 1000 rpm => velocity = 4474
                // with multiplicator = 64  ,  foc = 4474 * 64 / 256 => foc = about 1120 = about 4 ° in Q8_8
            uint32_t ui32_foc = ((ui32_hall_velocity_q8_8X1024 * (uint32_t) lead_angle_multiplicator )>>8) ; 
            // * 64 >> 8 = 64/256 
            
            // max = 23 *100 / 16 * 40 = 22
            if (ui32_foc > (25 * 256)) // limit in Q8_8
                ui32_foc = (25 * 256);
            // filtre iir convergent
            ui32_foc_angle_accum = ui32_foc_angle_accum - (ui32_foc_angle_accum >> 4) + (ui32_foc);
            ui16_g_foc_angle_q8_8 = (uint16_t)(ui32_foc_angle_accum >> 4);
            
                
                //ui8_foc_flag = 0;
                // added by mstrens
//                ui8_g_foc_angle = ui8_foc_flag ;
//            }
        } else { // duty cycle = 0
            ui16_adc_motor_phase_current = 0;
            ui32_foc_angle_accum = 0; // reset accumulator (used for accuracy)
            // removed by mstrens
            //if (ui8_foc_flag) {
            //    ui8_foc_angle_accumulated = ui8_foc_angle_accumulated - (ui8_foc_angle_accumulated >> 4);
            //    ui8_g_foc_angle = ui8_foc_angle_accumulated >> 4;
            //    ui8_foc_flag = 0;
            //}
            // added by mstrens
            ui16_g_foc_angle_q8_8 = 0; 
            
        }



} // end systick_handler


// ---------------------------------------------------
// Constantes globales et formats
// ---------------------------------------------------
#define Q30_SCALE           (1UL << 30)
#define DEG_TO_Q8_8(x)      ((uint16_t)((x) * (65536.0f / 360.0f) + 0.5f))
#define HALL_RATIO          4.474f

#define LEAD_STEP_MIN_DEGREE  0.02       // ≈ 0.022°
#define LEAD_STEP_MAX_DEGREE  0.35       // ≈ 0.35°
#define MAX_LEAD_CORR_DEGREE 10        // max for correction (in +and min)

#define LOW_SPEED_RPM        200
#define SPEED_FILTER_A_Q15   30000  // coeff IIR vitesse (α≈0.9)
#define SPEED_FILTER_B_Q15   (32768 - SPEED_FILTER_A_Q15)

#define IDABS_DEFAULT        100       // seuil absolu min en ADC units
#define K_REL_Q15            1638      // 0.05 * 32768 (5%)
#define HYST_FACTOR_Q15      29491     // 0.9 en Q15


#define MAX_LEAD_CORR_Q8_8  ((uint16_t)((MAX_LEAD_CORR_DEGREE << 16)/360))  // apply on corection
#define LEAD_STEP_MIN_Q8_8  ((uint16_t)(LEAD_STEP_MIN_DEGREE * (65536.0f / 360.0f) + 0.5f)) // apply on total   
#define LEAD_STEP_MAX_Q8_8  ((uint16_t)(LEAD_STEP_MAX_DEGREE * (65536.0f / 360.0f) + 0.5f)) // apply on total

// ---------------------------------------------------
// Tables de base (utilisateur)
// ---------------------------------------------------
const uint16_t speed_tab[] = {0, 500, 1000, 2000, 3000, 4700};
#define SPEED_TAB_SIZE (sizeof(speed_tab) / sizeof(speed_tab[0]))

const float lead_base_deg[] = {0.0f, 2.0f, 5.0f, 10.0f, 14.0f, 18.0f};

// ---------------------------------------------------
// Tables internes générées au premier passage
// ---------------------------------------------------
static uint16_t hall_tab[SPEED_TAB_SIZE];
static uint16_t lead_base_q8_8[SPEED_TAB_SIZE];
static uint32_t inv_delta_hall_q30[SPEED_TAB_SIZE - 1];
static uint32_t hall_low_speed_threshold = 0;
static uint8_t tables_initialized = 0;

// ---------------------------------------------------
// Variables dynamiques
// ---------------------------------------------------
static uint16_t tick_5ms = 0;
static int32_t  hall_filt = 0;
static int32_t  lead_corr_q8_8 = 0;
static uint16_t lead_base_q8_8_val = 0;
static uint16_t lead_total_q8_8 = 0;

// Deadband adaptatif
static int32_t last_deadband = 0;

// ---------------------------------------------------
// Fonctions utilitaires
// ---------------------------------------------------

static void init_lead_tables(void)
{
    for (uint8_t i = 0; i < SPEED_TAB_SIZE; i++) {
        hall_tab[i] = (uint16_t)(speed_tab[i] * HALL_RATIO + 0.5f);
        lead_base_q8_8[i] = DEG_TO_Q8_8(lead_base_deg[i]);
    }

    for (uint8_t i = 0; i < SPEED_TAB_SIZE - 1; i++) {
        uint32_t delta = (uint32_t)(hall_tab[i + 1] - hall_tab[i]);
        if (delta == 0) delta = 1;
        inv_delta_hall_q30[i] = Q30_SCALE / delta;
    }
    hall_low_speed_threshold = (uint32_t)(LOW_SPEED_RPM * HALL_RATIO + 0.5f);

    tables_initialized = 1;
}

static uint16_t interpolate_lead_base_from_hall_velocity(uint16_t hall_vel)
{
    if (hall_vel <= hall_tab[0])
        return lead_base_q8_8[0];
    if (hall_vel >= hall_tab[SPEED_TAB_SIZE - 1])
        return lead_base_q8_8[SPEED_TAB_SIZE - 1];

    uint8_t idx = 0;
    while (hall_vel > hall_tab[idx + 1])
        idx++;

    uint32_t delta_hall_vel = hall_vel - hall_tab[idx];
    uint32_t t_q30 = delta_hall_vel * inv_delta_hall_q30[idx];
    uint32_t delta_angle = (uint32_t)(lead_base_q8_8[idx + 1] - lead_base_q8_8[idx]);
    uint32_t interp = (uint32_t)lead_base_q8_8[idx] + ((t_q30 * delta_angle) >> 30);

    return (uint16_t)interp;
}

static inline int32_t filter_hall_iir(int32_t prev, int32_t new_val)
{
    return ( (prev * SPEED_FILTER_A_Q15) + (new_val * SPEED_FILTER_B_Q15) ) >> 15;
}

static inline int32_t clamp32(int32_t val, int32_t min, int32_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

// ---------------------------------------------------
// Boucle systick 1kHz
// ---------------------------------------------------
void update_lead_angle(void)
{
    if (!tables_initialized)
        init_lead_tables();

    tick_5ms++;
    if (tick_5ms < 5)
        return; // update à 200 Hz
    tick_5ms = 0;

    extern int32_t Hall_velocity;  // mesuré
    extern int32_t Id_filt;
    extern int32_t Iq_filt;
    extern void set_lead_angle(uint16_t angle_q8_8);

    // Filtrage Hall (évite le jitter)
    hall_filt = filter_hall_iir(hall_filt, (int32_t) ui32_hall_velocity_q8_8X1024);
    int32_t hall_used = hall_filt;

    // ----------------------
    // Lead base interpolation based on hall velocity
    // ----------------------
    lead_base_q8_8_val = interpolate_lead_base_from_hall_velocity((uint16_t)hall_used);

    // lead correction based on Id (taking care of Iq and speed)
    // ----------------------
    // Deadband adaptatif
    // ----------------------
    int32_t abs_Id = (Id_filt < 0) ? -Id_filt : Id_filt;
    int32_t abs_Iq = (Iq_filt < 0) ? -Iq_filt : Iq_filt;
    if (abs_Iq < 1) abs_Iq = 1; // protection

    int32_t Trel = (abs_Iq * K_REL_Q15) >> 15;
    int32_t T = (Trel > IDABS_DEFAULT) ? Trel : IDABS_DEFAULT;

    int32_t T_low = (last_deadband == 0) ? T : ((last_deadband * HYST_FACTOR_Q15) >> 15);

    if (T > last_deadband)
        last_deadband = T;
    else if (abs_Id < T_low)
        last_deadband = T;

    int32_t Id_effective = (abs_Id < last_deadband) ? 0 : Id_filt;

    // Reset du correctif si vitesse trop basse
    if (hall_used < hall_low_speed_threshold) {
        lead_corr_q8_8 = 0;
    } else if (Id_effective != 0) {
        // Step adaptatif proportionnel à |Id/Iq|
        int32_t step_q15 = ( (abs(Id_effective) << 15) / abs_Iq ); // Q15 ratio
        int32_t step = (step_q15 * LEAD_STEP_MAX_Q8_8) >> 15;
        step = clamp32(step, LEAD_STEP_MIN_Q8_8, LEAD_STEP_MAX_Q8_8); // limit step per iteration
        if (Id_effective > 0)
            lead_corr_q8_8 -= step;
        else
            lead_corr_q8_8 += step;
        // clamp correction 
        lead_corr_q8_8 = clamp32(lead_corr_q8_8, -MAX_LEAD_CORR_Q8_8, MAX_LEAD_CORR_Q8_8);
    }
 
    // Calcul total
    lead_total_q8_8 = (uint16_t)lead_base_q8_8_val + (uint16_t)lead_corr_q8_8;

    set_lead_angle(lead_total_q8_8);
}

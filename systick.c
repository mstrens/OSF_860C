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

} // end systick_handler


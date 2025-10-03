// code to put in motor.c in ISR to calculate position using a PLL as proposed by chatgpt
// increase precisions of angles and of sinus interpolation
// there are still some changes to do: LUT table for PWM must become in int16 and contains negative values
//    formula to calculate PWM value must be changed to take care of this

// lead angle can be kept like now in a first step (still format must be changed in int16 Q8_8 so with value that could be negative 
// in a second step, lead angle can be dynamically calculate to minimise Idc ; there is already some code prepared by chatgpt for this
//

//#define DISPLAY_PLL
#ifdef DISPLAY_PLL
/* ================   code actuel simplifié en retirant les #if qui ne sont plus utilisés =============
// get the current ticks
    uint16_t current_speed_timer_ticks = (uint16_t) HALL_SPEED_TIMER_HW->TIMER;
    
    // get the capture register = last changed pattern = current pattern
    uint16_t last_hall_pattern_change_ticks = (uint16_t) XMC_CCU4_SLICE_GetCaptureRegisterValue(HALL_SPEED_TIMER_HW , 1);
    
    // get the current hall pattern
    current_hall_pattern = XMC_POSIF_HSC_GetLastSampledPattern(HALL_POSIF_HW) ;
    ui8_hall_sensors_state = current_hall_pattern; // duplicate just for easier maintenance of ebike_app.c for 860c (sent to display)

    // elapsed time between now and last pattern change (used for interpolation)
    uint16_t enlapsed_time =  current_speed_timer_ticks - last_hall_pattern_change_ticks ; // ticks between now and last pattern change
    
    prev_ticks = current_speed_timer_ticks ;
    
    // when pattern change
    if ( current_hall_pattern != previous_hall_pattern) {
        if (current_hall_pattern != expected_pattern_table[previous_hall_pattern]){ // new pattern is not the expected one
            ui8_motor_commutation_type = BLOCK_COMMUTATION; // 0x00
            ui8_hall_360_ref_valid = 0;  // reset the indicator saying no error for a 360° electric rotation 
            ui32_angle_per_tick_X16shift = 0; // 0 means unvalid value
            
        } else {   // valid transition
            if (current_hall_pattern ==  0x01) {  // rotor at 210°
                if (ui8_hall_360_ref_valid) { // check that we have a full rotation without pattern sequence error
                    ui16_hall_counter_total = last_hall_pattern_change_ticks - previous_360_ref_ticks; // save the total number of tick for one electric rotation               
                    ui32_angle_per_tick_X16shift = ((uint32_t) ( 1 << 24)) / ui16_hall_counter_total; // new value for interpolation and updating table with reference angle
                    ui8_motor_commutation_type = SINEWAVE_INTERPOLATION_60_DEGREES; // 0x80 ; it says that we can interpolate because speed is known
                }
                ui8_hall_360_ref_valid = 0x01;
                previous_360_ref_ticks = last_hall_pattern_change_ticks ;    
            } else if (current_hall_pattern ==  0x03) {  // rotor at 150°)
                // update ui8_g_foc_angle once every ERPS (used for g_foc_angle calculation) ;
                ui8_foc_flag = 1;     
            }    
        }
        previous_hall_pattern = current_hall_pattern; // saved to detect future change and check for valid transition
        ui8_motor_phase_absolute_angle = ui8_hall_ref_angles[current_hall_pattern]; // use  hall_ref_angles[]
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
    // - calculate interpolation angle and sine wave table index when speed is known
    ui8_interpolation_angle = 0; // interpolation angle
    uint32_t compensated_enlapsed_time = 0; 
    if (ui8_motor_commutation_type != BLOCK_COMMUTATION) {  // as long as hall patern are OK and motor is running
        // hall counter offset take care of the delay between entering this ISR and applying the new PWM and also the delay of the hall sensors      
        compensated_enlapsed_time = enlapsed_time + ui8_hall_counter_offset;
        // convert time tick to angle (256 = 360°)
        // add 1<<15 for better rounding
        ui8_interpolation_angle = ((((uint32_t) compensated_enlapsed_time) *  ui32_angle_per_tick_X16shift) + (1<<15) )>> 16 ; 
    }
    ui8_angle_for_id = ui8_interpolation_angle + ui8_motor_phase_absolute_angle + hall_reference_angle + FINE_TUNE_ANGLE_OFFSET ;
    
    uint8_t ui8_svm_table_index = ui8_angle_for_id + ui8_g_foc_angle; // add lead angle (that is updated by a PID at 100hz)

*/



    //============ code proposé par chatgpt =============================================
// -----------------------------------------------------------
// Déclarations globales
// -----------------------------------------------------------
#include <stdint.h>

// motor states
#define BLOCK_COMMUTATION 			            0
#define SINEWAVE_INTERPOLATION_60_DEGREES 	    0x80

#define HALL_TRANSITIONS          6      // 3 capteurs Hall -> 6 transitions / tour électrique
#define Q16_16_SHIFT              16     // position Q16.16
#define TICK_US                   4      // 1 tick timer = 4 µs
//#define Kp_PLL_Q16                16     // gain PLL en Q16 (à ajuster selon moteur)
#define MAX_HALL_PATTERNS         8      // nombre de patterns Hall possible

// Position rotorique et vitesse en Q16.16
typedef int32_t q16_16_t; // (signed)

q16_16_t i32_hall_position = 0;           // position rotorique absolue (Q16.16)
q16_16_t i32_last_hall_position = 0;      // position rotorique au dernier front Hall
q16_16_t i32_hall_velocity = 0;           // vitesse rotorique (Q16.16 / tick)
q16_16_t i32_hall_phase_error = 0;        // erreur de phase pour PLL

uint16_t last_hall_pattern_change_ticks = 0;  // timestamp dernier front Hall
uint8_t ui8_hall_sensors_state = 0;          // état capteurs Hall
uint8_t previous_hall_pattern = 0;


// Lead angle Q8.8 (signed, -180°..+180°)
int16_t i16_lead_angle_Q8_8 = 0;


// Kp en Q8.8 (signed, pour permettre Kp négatif si besoin)
int16_t Kp_Q8_8 = 5; // Kp = 0.02 → Kp_Q8_8 = round(0.02 * 256) = 5

// Optionnel : Ki pour intégration (Q8.8)
int16_t Ki_Q8_8 = 0;

// Intégrateur (Q16.16)
q16_16_t i32_pll_integrator = 0;

// Anti-windup limits (Q16.16)
const q16_16_t PLL_INT_MAX = (q16_16_t)(1 << 20);
const q16_16_t PLL_INT_MIN = -(q16_16_t)(1 << 20);

// Angles mesurés pour les patterns Hall valides (Q8.8 = angle° * 256); sequence is 1,3,2,6,4, 5
const uint16_t u16_hall_angle_table_Q8_8[8] = {
    0,      // pattern 0 invalide
    24<<8,  // 1 -> 24 * 360 / 256 degré
    107<<8, // 2 -> 107 * 360 / 256 degré
    66<<8,  // 3 -> 66 * 360 / 256 degré
    195<<8, // 4 -> 195 * 360 / 256 degré
    235<<8, // 5 -> 235 * 360 / 256 degré
    152<<8, // 6 -> 152 * 360 / 256 degré
    0       // 7 invalidee
};

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


extern int16_t i16_LUT_SINUS[256] ; 

// Convertir hall_position (Q16.16 signed ) en uint16 Q8.8 cyclique 0..65535
static inline uint16_t hall_q16_to_q8_8(q16_16_t i32_hall_pos) {
    uint32_t ua = (uint32_t)i32_hall_pos;
    return (uint16_t)((ua & 0xFFFFFFu) >> 8);
}

// Convertir lead_angle Q8.8 (signed -180..+180) en uint16 Q8.8 cyclique 0..65535
static inline uint16_t lead_angle_to_q8_8(int16_t i16_lead_angle) {
    return (uint16_t)i16_lead_angle; // conversion signed → uint16, wrap automatique
}

// -----------------------------------------------------------
// ISR / boucle principale 19 kHz
// -----------------------------------------------------------
void HallLoopISR(void)
{
    // -----------------------------------------------------------
    // 1. Lire timer et POSIF
    // -----------------------------------------------------------
    uint16_t current_speed_timer_ticks = (uint16_t) HALL_SPEED_TIMER_HW->TIMER;
    uint16_t last_capture_ticks = (uint16_t) XMC_CCU4_SLICE_GetCaptureRegisterValue(HALL_SPEED_TIMER_HW , 1);
    uint8_t current_hall_pattern = XMC_POSIF_HSC_GetLastSampledPattern(HALL_POSIF_HW);

    // Mettre à jour l'état Hall pour affichage ou debug
    ui8_hall_sensors_state = current_hall_pattern;

    // -----------------------------------------------------------
    // 2. Calcul du temps écoulé depuis le dernier front Hall
    // -----------------------------------------------------------
    uint16_t elapsed_ticks = current_speed_timer_ticks - last_hall_pattern_change_ticks;
    q16_16_t i32_elapsed_ticks_Q16 = ((q16_16_t)elapsed_ticks) << Q16_16_SHIFT;

    // -----------------------------------------------------------
    // 3. Détection de changement de pattern Hall
    // -----------------------------------------------------------
    if (current_hall_pattern != previous_hall_pattern)
    {
        // verifier séquence valide
        if (current_hall_pattern != expected_pattern_table[previous_hall_pattern]){
            ui8_motor_commutation_type = BLOCK_COMMUTATION;
            ui8_hall_360_ref_valid = 0;
        } else {
            if (current_hall_pattern ==  0x01) {  // rotor at 210°
                if (ui8_hall_360_ref_valid) { // check that we have a full rotation without pattern sequence error
                    //ui16_hall_counter_total is used in other part of the code (e.g. for rpm in ebike_app.c) 
                    ui16_hall_counter_total = last_hall_pattern_change_ticks - previous_360_ref_ticks; // save the total number of tick for one electric rotation               
                    ui8_motor_commutation_type = SINEWAVE_INTERPOLATION_60_DEGREES; // 0x80 ; it says that we can interpolate because speed is known
                }
                ui8_hall_360_ref_valid = 0x01;
                previous_360_ref_ticks = last_hall_pattern_change_ticks ;    
            } else if (current_hall_pattern ==  0x03) {  // rotor at 150°)
                // flag to update ui8_g_foc_angle once every ERPS (used for g_foc_angle calculation) ;
                ui8_foc_flag = 1;     
            }
            // angle Hall en Q16.16 : Conversion Q8.8 -> Q16.16 : shift de 8 bits
            q16_16_t i32_measured_hall_position = ((q16_16_t)u16_hall_angle_table_Q8_8[current_hall_pattern]) << 8;
            
            // Calcul de l'erreur de phase pour PLL
            // Calcul de l'erreur de phase pour PLL (Q16.16)
            i32_hall_phase_error = i32_measured_hall_position - i32_hall_position;
            // Proportionnel : delta = (hall_phase_error * Kp_Q8_8) >> 8
            int64_t i64_tmp = (int64_t)i32_hall_phase_error * (int32_t)Kp_Q8_8; // produit en 64 bits
            q16_16_t i32_delta_p = (q16_16_t)(i64_tmp >> 8);            // ramène en Q16.16

            // Optionnel : intégrateur Ki
            if (Ki_Q8_8 != 0) {
                int64_t i64_tmp_i = (int64_t)i32_hall_phase_error * (int32_t)Ki_Q8_8;
                q16_16_t i32_delta_i = (q16_16_t)(i64_tmp_i >> 8);
                i32_pll_integrator += i32_delta_i;
                // anti-windup
                if (i32_pll_integrator > PLL_INT_MAX) i32_pll_integrator = PLL_INT_MAX;
                if (i32_pll_integrator < PLL_INT_MIN) i32_pll_integrator = PLL_INT_MIN;
            } else {
                // si pas d'intégration, éviter lecture inutile
                // i32_pll_integrator reste inchangé
            }

            // Calcul de correction totale (Q16.16)
            q16_16_t i32_correction = i32_delta_p + i32_pll_integrator;

            // Appliquer correction à vitesse et position
            i32_hall_velocity += i32_correction;   // hall_velocity est Q16.16 / tick
            i32_hall_position += i32_correction;   // hall_position Q16.16
            
            i32_last_hall_position = i32_measured_hall_position;
            last_hall_pattern_change_ticks = current_speed_timer_ticks;
        }
        previous_hall_pattern = current_hall_pattern;
    } 
    else { // no hall patern change
        // Verify if rotor stopped (< 10 ERPS) (250000 ticks/sec, 6 states/erps)
        if (elapsed_ticks > (250000 / 10 / 6 )) { //  4166
            ui8_motor_commutation_type = BLOCK_COMMUTATION; // 0
            ui8_g_foc_angle = 0;
            ui8_hall_360_ref_valid = 0;
            ui16_hall_counter_total = 0xffff;
        }
    }    
    if (ui8_motor_commutation_type != BLOCK_COMMUTATION) {  // as long as hall patern are OK and motor is running
        // -----------------------------------------------------------
        // 4. Interpolation linéaire entre fronts Hall
        // hall_position = dernière position + vitesse * elapsed_ticks
        // -----------------------------------------------------------
        i32_hall_position = i32_last_hall_position + ((i32_hall_velocity * i32_elapsed_ticks_Q16) >> Q16_16_SHIFT);
    } else {
        // when motor is blocked (speed < 10erps), no interpolation
        i32_hall_position = ((q16_16_t)u16_hall_angle_table_Q8_8[current_hall_pattern]) << 8;
    }

    // // --- (b) Conversion Q16.16 -> Q8.8 signé et addition lead_angle ---
    // Faire la conversion en signed Q8.8, addition signée, ensuite cast en uint16 (wrap)
    
    // Conversion de hall_position de Q16.16 (signed) vers Q8.8 cyclique (0..65535)
    // Les 8 bits supérieurs de la partie fractionnaire Q16.16 deviennent la partie fractionnaire Q8.8
    // Wrap automatique géré par le cast en uint16_t
    uint16_t u16_hall_q8_8 = hall_q16_to_q8_8(i32_hall_position);

    // Conversion du lead_angle Q8.8 signé (-180..+180°) en uint16 Q8.8 cyclique
    // Wrap automatique : valeurs négatives deviennent grandes valeurs uint16 cycliques
    //  Assurez-vous que lead_angle reste dans ±32767 pour éviter un wrap inattendu
    uint16_t u16_lead_q8_8 = lead_angle_to_q8_8(i16_lead_angle_Q8_8);

    uint8_t u8_hall_reference_angle = DEFAULT_HALL_REFERENCE_ANGLE; // = 66*360/256° ; this is about 90° to forward the magnetic field
    // convert to Q24_8 unsigned
    uint32_t u32__hall_reference_angle_q24_8 = (uint32_t) u8_hall_reference_angle << 8; 
    // Addition cyclique sûre
    // Addition cyclique hall + lead_angle en Q8.8
    // Le résultat est un angle cyclique 0..65535 (wrap automatique)
    // Convient pour calculer l’index LUT et la fraction Q8.8 pour interpolation
    uint32_t u32_sum = (uint32_t)u16_hall_q8_8 + (uint32_t)u16_lead_q8_8 + u32__hall_reference_angle_q24_8;
    uint16_t u16_final_angle_q8_8 = (uint16_t)u32_sum;
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
    // 7. Lecture LUT avec rollover N+1
    // -----------------------------
    int16_t yA0 = i16_LUT_SINUS[u8_lut_index_A];
    int16_t yA1 = i16_LUT_SINUS[(u8_lut_index_A + 1) & 0xFF];
    int16_t svm_A = yA0 + (((yA1 - yA0) * u8_lut_frac) >> 8);
    
    int16_t yB0 = i16_LUT_SINUS[u8_lut_index_B];
    int16_t yB1 = i16_LUT_SINUS[(u8_lut_index_B + 1) & 0xFF];
    int16_t svm_B = yB0 + (((yB1 - yB0) * u8_lut_frac) >> 8);
    
    int16_t yC0 = i16_LUT_SINUS[u8_lut_index_C];
    int16_t yC1 = i16_LUT_SINUS[(u8_lut_index_C + 1) & 0xFF];
    int16_t svm_C = yC0 + (((yC1 - yC0) * u8_lut_frac) >> 8);

    ui16_a = (uint16_t) (MIDDLE_SVM_TABLE + ( svm_A * (int16_t) ui8_g_duty_cycle)>>8); // >>8 because duty_cycle 100% is 256
    ui16_b = (uint16_t) (MIDDLE_SVM_TABLE + ( svm_B * (int16_t) ui8_g_duty_cycle)>>8); // >>8 because duty_cycle 100% is 256
    ui16_c = (uint16_t) (MIDDLE_SVM_TABLE + ( svm_C * (int16_t) ui8_g_duty_cycle)>>8); // >>8 because duty_cycle 100% is 256

    // svm_value, svm_B, svm_C → PWM / SVM
}

/*
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


#endif // end DISPLAY_PLL

//#define DISPLAY_UPDATE_LEAD_ANGLE
#ifdef DISPLAY_UPDATE_LEAD_ANGLE
//chatgpt proposal to calculate dynamically lead angle in order to minimise IDC (avec angle Q8.8 avec 1.0 = 360/256°)
#ifndef ESC_H
#define ESC_H

#include <stdint.h>
#include <stdbool.h>

/*
  ESC fixed-point final for Cortex-M0
  - ANGLES: Q8.8 in LUT units (1 step = 360/256 degrees)
    -> type q88_t (int16_t)
  - CURRENTS: Q15 (int16_t), normalized by caller: q15 = (I / MAX_CURRENT_AMP) * 32768
  - RPM: int16_t (raw RPM)
  - ESC_step must be called at ESC_UPDATE_FREQ_HZ (typ. 1000 Hz)
*/

// ------------------------
// CONFIGURATION (tweak)
// ------------------------

// Normalisation courant (pour information/callers)
#define MAX_CURRENT_AMP            100.0f   // example (used by caller to scale to Q15)

// Timing
#define ESC_UPDATE_FREQ_HZ         1000     // call ESC_step at 1 kHz
#define ESC_DITHER_FREQ_HZ         50       // dither toggle frequency (Hz)
#define RPM_UPDATE_PERIOD_MS       50       // lead_base LUT update period (ms)

// Freeze times
#define LEAD_BASE_SMOOTH_ALPHA_F   0.10f    // smoothing alpha for lead_base

#define LEAD_BASE_DELTA_THRESH_DEG 1.0f     // if LUT jump > this => freeze short
#define LEAD_BASE_FREEZE_MS        200
#define FREEZE_TIME_MS             300      // freeze ESC when transient detected
#define HIST_SIZE_POWER            3
#define HIST_SIZE                  (1<<HIST_SIZE_POWER)  // = 8        // length of history buffers (small, e.g. 8)

// Lead / limits (DEGREES given for human readability; converted to Q8.8 internally)
#define LEAD_MIN_DEG               0.0f
#define LEAD_MAX_DEG              40.0f
#define ESC_OFFSET_LIMIT_DEG      10.0f

// Dither amplitude in degrees (will be converted)
// Step detection thresholds (fractions / absolute)
#define IREF_STEP_THRESH_FRAC      0.10f    // 10% of full scale current (Q15 fraction)
#define IDC_STEP_THRESH_FRAC       0.08f    // 8% of full scale current
#define RPM_STEP_THRESH            250      // rpm step threshold (rpm)
// ESC adaptation
#define ESC_GAIN_F                 0.001f   // small gain
#define ESC_LPF_ALPHA_F            0.05f    // LPF alpha for correlation

#define DITHER_AMP_DEG             1.0f


// ------------------------
// Fixed-point types & macros
// ------------------------
typedef int16_t q15_t;    // Q15 for currents (signed int16_t)
typedef int32_t q31_t;
typedef int16_t q88_t;    // Q8.8 (LUT units): 1 LSB = 1/256 of a LUT step; range = -180°/180°

// Convert float->fixed (used at compile/init only)
#define FLOAT_TO_Q15(x)   ((q15_t)((x) * 32768.0f))
#define FLOAT_TO_Q88_LUT_DEG(x) ((q88_t)((x) * (65536.0f / 360.0f))) 

#define RPM_RAW_MAX     6000       // rpm maximum mesuré
#define Q15_MAX         32768      // 2^15
#define RPM_LEAD_MAX    4800       // rpm that provide the max lead declared in a #define; used to ajust lead base with rpm 

/* Explanation: deg * (65536/360) gives Q8.8 in LUT units (1 LUT step = 360/256 deg) */

// ------------------------
// Externals (to implement in your project)
// ------------------------
// PWM LUT (256 entries) -> values for PWM comparator (e.g. 0..PWM_MAX)
extern const int16_t pwm_lut[256];

// ------------------------
// Filtrage Idc / Corrélation
// ------------------------

// Filtre "rapide" Idc (détection step, transients) ~20 Hz
#define ALPHA_IDC_FAST_Q15   FLOAT_TO_Q15(0.12f)  //  0.12 × 32768 ≈ 3932

// Filtre "lent" Idc utilisé pour J (minimisation) ~5 Hz
#define ALPHA_IDC_SLOW_Q15   FLOAT_TO_Q15(0.03f)   // 0.03 × 32768 ≈ 983

// Filtre très lent pour la corrélation J·dither ~1 Hz
#define ALPHA_CORR_Q15       FLOAT_TO_Q15(0.0063f)  // 0.0063 × 32768 ≈ 206

// Filtre lead_base smoothing ~5 Hz (déjà présent, tu avais 0.10f ; plus raisonnable : 0.05f)
#define ALPHA_LEADBASE_Q15   FLOAT_TO_Q15(0.05f)  // 0.05 × 32768 ≈ 1638

// ------------------------
// Public API
// ------------------------
void ESC_init(void);

// ESC step, called at ESC_UPDATE_FREQ_HZ
// Iref_q15, Idc_q15 must be normalized by caller: q15 = (I / MAX_CURRENT_AMP) * 32768
// rpm_raw = measured rpm (int16_t)
void ESC_step_q15(q15_t Iref_q15, q15_t Idc_q15, int16_t rpm_raw);

// Apply currently computed commanded_lead to PWM, using rotor index (0..255).
// rotor_idx is your internal rotor angle index (0..255) that you use to select LUT entry.
// This function will interpolate LUT between entries to produce a duty and call pwm_set_duty().
void ESC_apply_to_pwm(uint8_t rotor_idx);

#endif // ESC_H


#include "esc.h"
#include <string.h> // memset
#include <stdlib.h> // abs

// ------------------------
// Local LUT for rpm->lead_base
// Stored in Q8.8 LUT units using macro FLOAT_TO_Q88_LUT_DEG
// Adjust these degree values to match your motor; they will be converted to LUT units
// ------------------------
typedef struct {
    int16_t rpm;
    q88_t lead_q88;
} LeadLUT_t;

// ------------------------
// Internal state (fixed-point)
// ------------------------

static q88_t lead_base_q88;        // smoothed base lead (Q8.8 LUT units)
static q88_t lead_base_raw_q88;    // raw LUT output before smoothing
static q88_t esc_offset_q88;       // ESC learned offset (Q8.8 LUT units)
static q88_t commanded_lead_q88;   // commanded (base + offset + dither) Q8.8

static q88_t lead_angle_actuel;    // Q8.8 LUT units (1 step = 360/256 deg) (range -180..180)


static q31_t lpf_corr_q15;         // LPF on correlation (Q15) (accumulate Q15 in 32 bits)

// dither toggling
static int dither_sign;
static int sample_cnt_toggle;
static int samples_per_toggle;

// freeze/lock
static int freeze_timer_ms;
static bool esc_locked;

// histories for Iref, Idc, rpm
static q15_t iref_hist[HIST_SIZE];
static q15_t idc_hist[HIST_SIZE];
static int16_t rpm_hist[HIST_SIZE];
static int hist_idx_iref;
static int hist_idx_idc;
static int hist_idx_rpm;

// filtered signals (Q15)
static q15_t idc_filt_q15; // filtered Idc (Q15)
static int32_t rpm_filt_accum = 0; // accumulateur 32 bits pour filter_rpm_q15

// Fixed thresholds/gains in fixed point (initialized in ESC_init)
static q15_t iref_step_thresh_q15;
static q15_t idc_step_thresh_q15;
static int16_t rpm_step_thresh_int;

// LPF / gain in Q15 / Q88
static q15_t alpha_idc_q15;
static q15_t alpha_rpm_q15;
static q15_t esc_lpf_alpha_q15;
static q15_t esc_gain_q15;
static q15_t leadbase_smooth_alpha_q15;
static q88_t dither_amp_q88;
static q88_t esc_offset_limit_q88;
static q88_t lead_min_q88;
static q88_t lead_max_q88;

// ------------------------
// Utilities
// ------------------------

static inline void hist_push_q15(q15_t buf[], int *idx, q15_t val) {
    buf[*idx] = val;
    *idx = (*idx + 1) & (HIST_SIZE-1);
}
static inline q15_t hist_avg_q15(q15_t buf[]) {
    q31_t s = 0;
    for (int i=0;i<HIST_SIZE;i++) s += buf[i];
    return (q15_t)(s >> HIST_SIZE_POWER);
}

static inline void hist_push_i16(int16_t buf[], int *idx, int16_t val) {
    buf[*idx] = val;
    *idx = (*idx + 1) & (HIST_SIZE - 1);
}
static inline int16_t hist_avg_i16(int16_t buf[]) {
    int32_t s = 0;
    for (int i=0;i<HIST_SIZE;i++) s += buf[i];
    return (int16_t)(s >> HIST_SIZE_POWER);
}

// clamp q88
static inline q88_t clamp_q88(q88_t x, q88_t lo, q88_t hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// ------------------------
// Filtrage RPM en Q15 avec accumulateur 32 bits
// rpm_raw: rpm mesuré (0..6000)
// alpha_q15: gain du filtre IIR (0..32768)
// rpm_filt_accum: pointeur sur accumulateur 32 bits (état du filtre)
// Retour: rpm filtré en Q15
static inline q15_t filter_rpm_q15(int16_t rpm_raw, q15_t alpha_q15, int32_t *rpm_filt_accum)
{
    // Échelle sans division : rpm_raw -> 0..32768 Q15
    // RPM_SCALE = Q15_MAX / RPM_RAW_MAX
    const uint32_t RPM_SCALE = Q15_MAX / RPM_RAW_MAX; // 32768 / 6000 ≈ 5

    uint32_t rpm_in_q32 = (uint32_t)rpm_raw * RPM_SCALE; // 0..32768, sûr pour uint32

    // IIR Q15: rpm_filt = alpha * rpm + (1-alpha) * rpm_filt_prev
    *rpm_filt_accum = ( (int32_t)alpha_q15 * (int32_t)rpm_in_q32
                      + (32768 - alpha_q15) * (*rpm_filt_accum) ) >> 15;

    // Retour en Q15
    if (*rpm_filt_accum > 32767) return 32767;
    if (*rpm_filt_accum < 0) return 0;
    return (q15_t)(*rpm_filt_accum);
}

// ------------------------
// Calcul lead angle proportionnel au rpm
// ------------------------
/**
 * @brief   Convertit la vitesse moteur en lead angle.
 *
 * @param   rpm_in_q15   Vitesse mécanique en Q15 (rpm << 15 / 32768 ≈ rpm * RPM_SCALE).
 *                       L'entrée est bornée à 6000 rpm max.
 * @return  lead_angle_q88  Angle d'avance en Q8.8 (int16_t, -128..+127 degrés ≈ -32768..+32767 en Q8.8).
 *
 * L'angle est proportionnel au régime moteur (progression linéaire).
 * L'échelle est fixée par #define LEAD_ANGLE_MAX_Q88 (valeur atteinte à rpm = 4800).
 * Aucun division : l'interpolation utilise uniquement des décalages.
 */
static inline q88_t lead_angle_from_rpm_q15(q15_t rpm_q15)
{
    uint32_t rpm = ((uint32_t)rpm_q15 * RPM_LEAD_MAX) >> 15;
    if(rpm >= RPM_LEAD_MAX) return LEAD_MAX_Q88;

    const uint32_t scale_q15 = ((uint32_t)LEAD_MAX_Q88 << 15) / RPM_LEAD_MAX;
    return (q88_t)((rpm * scale_q15) >> 15);
}

// ------------------------
// Public API
// ------------------------
void ESC_init(void)
{
    // initial state
    lead_base_q88 = 0;
    lead_base_raw_q88 =0;
    lead_angle_actuel = 0 ; 
    esc_offset_q88 = 0;
    commanded_lead_q88 = 0;
    lpf_corr_q15 = 0;
    idc_filt_q15 = 0;
    rpm_filt_accum = 0;
    dither_sign = +1;
    sample_cnt_toggle = 0;
    // compute samples per toggle: ESC_UPDATE_FREQ_HZ/(2*dither_freq)
    samples_per_toggle = (ESC_UPDATE_FREQ_HZ) / (2 * ESC_DITHER_FREQ_HZ);
    if (samples_per_toggle < 1) samples_per_toggle = 1;
    freeze_timer_ms = 0;
    esc_locked = false;
    

    // init hist buffers to zero
    memset(iref_hist, 0, sizeof(iref_hist));
    memset(idc_hist, 0, sizeof(idc_hist));
    memset(rpm_hist, 0, sizeof(rpm_hist));
    hist_idx_iref = hist_idx_idc = hist_idx_rpm = 0;

    // thresholds / gains converted to fixed point
    iref_step_thresh_q15 = FLOAT_TO_Q15(IREF_STEP_THRESH_FRAC); // fraction of Q15 = 1.0
    idc_step_thresh_q15  = FLOAT_TO_Q15(IDC_STEP_THRESH_FRAC);
    rpm_step_thresh_int  = RPM_STEP_THRESH;

     // LPF alpha & gains in Q15
     alpha_rpm_q15 = FLOAT_TO_Q15(0.05f); // rpm filter alpha (~20 ms tau)
     alpha_idc_q15           = ALPHA_IDC_FAST_Q15;   // Idc rapide pour détection step
    esc_lpf_alpha_q15       = ALPHA_CORR_Q15;       // corrélation J·dither très lente
    esc_gain_q15            = FLOAT_TO_Q15(ESC_GAIN_F);
    leadbase_smooth_alpha_q15 = ALPHA_LEADBASE_Q15; // lissage lead_base
    

    dither_amp_q88 = FLOAT_TO_Q88_LUT_DEG(DITHER_AMP_DEG);
    esc_offset_limit_q88 = FLOAT_TO_Q88_LUT_DEG(ESC_OFFSET_LIMIT_DEG);
    lead_min_q88 = FLOAT_TO_Q88_LUT_DEG(LEAD_MIN_DEG);
    lead_max_q88 = FLOAT_TO_Q88_LUT_DEG(LEAD_MAX_DEG);
}

// ESC_step: call at ESC_UPDATE_FREQ_HZ (e.g. 1kHz)
// Iref_q15, Idc_q15 normalized to MAX_CURRENT_AMP by caller
// rpm_raw is integer rpm (0..6000)
void ESC_step_q15(q15_t Iref_q15, q15_t Idc_q15, int16_t rpm_raw)
{
    // 1) Basic IIR filters
    // idc_filt_q15 = alpha * Idc + (1-alpha) * idc_filt
    idc_filt_q15 = ( ( (q31_t)alpha_idc_q15 * Idc_q15 ) + ( (q31_t)(32768 - alpha_idc_q15) * idc_filt_q15 ) ) >> 15;

    q15_t rpm_filt_q15 = filter_rpm_q15(rpm_raw, alpha_rpm_q15, &rpm_filt_accum);
    
    // 2) update history buffers (circular)
    hist_push_q15(iref_hist, &hist_idx_iref, Iref_q15);
    hist_push_q15(idc_hist,  &hist_idx_idc,  Idc_q15);
    hist_push_i16(rpm_hist,  &hist_idx_rpm,  rpm_raw);

    q15_t iref_avg_q15 = hist_avg_q15(iref_hist);
    q15_t idc_avg_q15  = hist_avg_q15(idc_hist);
    int16_t rpm_avg     = hist_avg_i16(rpm_hist);

    // 3) detect rapid changes comparing instantaneous value to historical average
    q15_t iref_delta = Iref_q15 - iref_avg_q15; if (iref_delta < 0) iref_delta = -iref_delta;
    q15_t idc_delta  = Idc_q15  - idc_avg_q15;  if (idc_delta  < 0) idc_delta  = -idc_delta;
    int16_t rpm_delta = rpm_raw - rpm_avg; if (rpm_delta < 0) rpm_delta = -rpm_delta;

    // thresholds are fractions of full-scale Q15 for currents
    if (iref_delta > iref_step_thresh_q15 || idc_delta > idc_step_thresh_q15 || rpm_delta > rpm_step_thresh_int) {
        esc_locked = true;
        freeze_timer_ms = FREEZE_TIME_MS;
    }

    // 4) Update lead_base periodically from LUT (every RPM_UPDATE_PERIOD_MS)
    static int accum_ms = 0;
    accum_ms++;
    if (accum_ms >= RPM_UPDATE_PERIOD_MS) {
        accum_ms = 0;
        q88_t new_lead_raw = lead_angle_from_rpm_q15(rpm_filt_q15);
        q88_t delta = new_lead_raw - lead_base_raw_q88;
        if (delta < 0) delta = -delta;
        if (delta > FLOAT_TO_Q88_LUT_DEG(LEAD_BASE_DELTA_THRESH_DEG)) {
            esc_locked = true;
            freeze_timer_ms = LEAD_BASE_FREEZE_MS;
        }

        lead_base_raw_q88 = new_lead_raw;
        // smooth: lead_base = alpha * raw + (1-alpha) * lead_base
        // alpha in Q15
        q31_t tmp = ( (q31_t)leadbase_smooth_alpha_q15 * (q31_t)lead_base_raw_q88
                    +  (q31_t)(32768 - leadbase_smooth_alpha_q15) * (q31_t)lead_base_q88 ) >> 15;
        lead_base_q88 = (q88_t)tmp;
    }

    // 5) freeze timer
    if (freeze_timer_ms > 0) {
        freeze_timer_ms--;
        if (freeze_timer_ms <= 0) esc_locked = false;
    }

    if (esc_locked) {
       lead_angle_actuel = lead_base_q88;   // sécurité, pas de dither ni offset
       return;
    } 
    
    // 6) Dither toggling
    sample_cnt_toggle++;
    if (sample_cnt_toggle >= samples_per_toggle) {
        sample_cnt_toggle = 0;
        dither_sign = -dither_sign;
    }
    q88_t dither_q88 = (dither_sign > 0) ? dither_amp_q88 : -dither_amp_q88;

    // 7) commanded lead = lead_base + esc_offset + dither
    int32_t tmp_cmd = (int32_t)lead_base_q88 + (int32_t)esc_offset_q88 + (int32_t)dither_q88;

    // clamp in int32 domain using lead_min/lead_max promoted to int32
    int32_t lead_min_i32 = (int32_t)lead_min_q88;
    int32_t lead_max_i32 = (int32_t)lead_max_q88;
    if (tmp_cmd < lead_min_i32) tmp_cmd = lead_min_i32;
    if (tmp_cmd > lead_max_i32) tmp_cmd = lead_max_i32;

    commanded_lead_q88 = (q88_t)tmp_cmd;


    // 8) compute cost J = idc_filt_q15
    q15_t J_q15 = idc_filt_q15;

    // 9) correlation with dither sign
    q31_t corr_q15 = (q31_t)J_q15 * (q31_t)dither_sign;

    // LPF corr: lpf_corr = alpha * corr + (1-alpha) * lpf_corr
    lpf_corr_q15 = ( (q31_t)esc_lpf_alpha_q15 * corr_q15 + (q31_t)(32768 - esc_lpf_alpha_q15) * lpf_corr_q15 ) >> 15;

    // 10) update esc_offset: esc_offset -= gain * lpf_corr  (scale to Q8.8)
    // gain (Q15) * lpf_corr (Q15) >>15 => Q15. Convert Q15 -> Q8.8 by >>7
    q31_t delta_offset_q15 = ( (q31_t)esc_gain_q15 * lpf_corr_q15 ) >> 15;
    if (delta_offset_q15 > (int32_t) (INT16_MAX << 7)) delta_offset_q15 = (INT16_MAX << 7);
    if (delta_offset_q15 < (int32_t) (INT16_MIN << 7)) delta_offset_q15 = (INT16_MIN << 7);
    q88_t delta_offset_q88 = (q88_t)(delta_offset_q15 >> 7);
    esc_offset_q88 -= delta_offset_q88;

    // clamp esc_offset to ±limit
    if (esc_offset_q88 < -esc_offset_limit_q88) esc_offset_q88 = -esc_offset_limit_q88;
    if (esc_offset_q88 >  esc_offset_limit_q88) esc_offset_q88 =  esc_offset_limit_q88;
    lead_angle_actuel = commanded_lead_q88; // base + offset + dither
}

/*
Remarques et instructions d’intégration

Normalisation courant (appelant) : Iref_q15 and Idc_q15 must be provided by caller scaled so that 1.0 = MAX_CURRENT_AMP. Example (in integer, avoid float in caller if desired):

// I_amp: measured/current reference in amps (integer or fixed)
// Convert to Q15 without float: q15 = (I_amp * 32768) / MAX_CURRENT_AMP_INT
*/

/*
Utilisation : 


*/

// ------------------------
// LUT interpolation => produce PWM value (no division, no modulo heavy)
#include <stdint.h>

// LUT fournie par ton projet (peut contenir des valeurs négatives)
extern const int16_t pwm_lut[256];

// Wrap Q8.8 (0..65535 = 0..360°)
static inline uint16_t wrap_q88(int32_t angle_q88)
{
    return (uint16_t)(angle_q88 & 0xFFFF);
}

// rotor_angle_q88 et lead_angle_q88 en Q8.8 (int16_t) donc chacun dans range -180°...180°
// Retourne le duty interpolé (int16_t car LUT est en int16_t)
int16_t pwm_from_angle_q88(int16_t rotor_angle_q88, int16_t lead_angle_q88)
{
    // 1) Somme des angles en Q8.8
    int32_t tmp_q88 = (int32_t)rotor_angle_q88 + (int32_t)lead_angle_q88;

    // 2) Réduction modulo 360° (wrap dans 0..65535)
    uint16_t angle_wrapped = wrap_q88(tmp_q88);

    // 3) Extraire index LUT et fraction
    uint8_t idx  = angle_wrapped >> 8;    // entier LUT step
    uint8_t frac = angle_wrapped & 0xFF;  // fraction 0..255

    // 4) Interpolation linéaire (LUT est en int16)
    int16_t v1 = pwm_lut[idx];
    int16_t v2 = pwm_lut[(idx + 1) & 0xFF];

    // interpolation : v1 + (v2 - v1) * frac / 256
    int16_t duty = v1 + ((int32_t)(v2 - v1) * frac >> 8);

    return duty;
}

// pour faire des additions de 2 Q88 int16, il faut
- passer les 2 opérandes en int32,
- faire la somme en int32
- sur la somme faire un & 0xFFFF (pour avoir un modulo) 
- faire un cast en uint16 si on veut un range 0...360 comme pour lire une lut
 

/*
🔹 Fréquences et filtres
Signal / Action	    Fréquence / périodicité	    Filtre / méthode	      Q-format	                Commentaire
Idc (mesuré PWM)	    19 kHz	       Moyenne mobile 64 échantillons	    Q15	                    Filtrage haute fréquence pour lisser le courant
Idc filtré rapide	1 kHz (ESC_step)	IIR α = 0.12 → ~20 Hz	            Q15                     Détection steps / transients
Idc filtré lent (J)	1 kHz	            IIR α = 0.03 → ~5 Hz                Q15                     Utilisé pour calcul du coût J = Idc pour minimisation
Lead_base           1 kHz (mise à jour tous les 50 ms)	IIR α = 0.05 → ~5 Hz   Q8.8	                Lissage du lead angle selon RPM
Corrélation J·dither 1 kHz              IIR α = 0.0063 → ~1 Hz              Q15                     Très lent pour suivre tendance moyenne et ne pas réagir aux transients
RPM filtré           1 kHz              IIR α = 0.05 → ~20 Hz               Q15                     Filtrage de la vitesse pour calcul lead_base
🔹 Schéma temporel (simplifié)
PWM 19kHz ──┐
            ├─> Idc → moyenne mobile 64 échantillons → Idc_raw
1kHz ESC_step┘
            │
            ├─> Idc_fast (IIR α=0.12) ──┐
            │                          │
            ├─> Idc_slow (IIR α=0.03) ─┼─> Coût J = Idc_slow
            │                          │
            ├─> RPM_filt (IIR α=0.05) ─┘
            │
            ├─> Lead_base update (tous 50ms) → lissage α=0.05
            │
            ├─> ESC offset update via corrélation J·dither α=0.0063
            │
            └─> Commanded lead = lead_base + offset + dither

🔹 Notes importantes

Idc_fast sert uniquement à détecter transients ou steps pour sécuriser l’ESC (freeze).

Idc_slow sert à la minimisation du courant Idc via la boucle adaptative (J).

Corrélation est très lente pour ne pas réagir aux fluctuations instantanées du courant.

La mise à jour de lead_base est périodique tous les 50 ms, mais le lissage α=0.05 permet un suivi progressif.

Les Q-formats sont bien choisis pour éviter overflow et préserver la précision sans flottant.



*/


#endif // end DISPLAY_UPDATE_LEAD_ANGLE
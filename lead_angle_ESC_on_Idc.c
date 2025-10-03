
#include "lead_angle_ESC_on_Idc.h"
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

// Fixed thresholds/gains in fixed point (initialized in ESC_init)
static q15_t iref_step_thresh_q15;
static q15_t idc_step_thresh_q15;

// LPF / gain in Q15 / Q88
static q15_t alpha_idc_q15;
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
    if(rpm >= RPM_LEAD_MAX) return lead_max_q88;

    const uint32_t scale_q15 = ((uint32_t)lead_max_q88 << 15) / RPM_LEAD_MAX;
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
    
     // LPF alpha & gains in Q15
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
    
    int16_t rpm_delta = rpm_raw - rpm_avg;
    if (rpm_delta < 0) rpm_delta = -rpm_delta;
    // calcule variation relative en % (évite division lente)
    // rpm_delta_pct = (100 * delta) / (rpm_avg + 1)
    // +1 évite /0 à l’arrêt
    int32_t rpm_delta_pct = ( (int32_t)rpm_delta * 100 ) / (rpm_avg + 1);
    // blocage ESC si variation trop forte
    if (iref_delta > iref_step_thresh_q15 ||
        idc_delta  > idc_step_thresh_q15  ||
        rpm_delta_pct > RPM_STEP_THRESH_PCT) {
        esc_locked = true;
        freeze_timer_ms = FREEZE_TIME_MS;
    }

    // 4) Update lead_base from LUT (using rpm_avg)
    static int accum_ms = 0;
    accum_ms++;
    if (accum_ms >= RPM_UPDATE_PERIOD_MS) {
        accum_ms = 0;
        q88_t new_lead_raw = lead_angle_from_rpm_q15((q15_t)rpm_avg);
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

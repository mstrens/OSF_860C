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
#define RPM_STEP_THRESH_PCT        5   // 5% est plus réaliste pour ton moteur

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

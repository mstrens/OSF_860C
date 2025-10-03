
// esc_lead.c
// Implémentation ESC pour lead angle BLDC avec base LUT + offset
// Version float (lisible), fréquence PWM = 19 kHz, boucle ESC = 1 kHz
// this version does not contains all changes done in integer version

//#define DISPLAY_FLOAT_VERSION
#ifdef DISPLAY_FLOAT_VERSION
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// ----------------------------
// Configuration ESC / Lead
// ----------------------------
#define ANGLE_MIN        0.0f
#define ANGLE_MAX       40.0f
#define DITHER_AMP       1.0f       // amplitude du dither [deg]
#define ESC_GAIN         0.001f     // gain intégrateur ESC
#define LPF_ALPHA        0.05f      // filtre corrélation ESC (0..1)
#define UPDATE_DT_MS     1.0f       // boucle ESC = 1 kHz

// Transitoires
#define IDC_DERIV_THRESH   50.0f    // A/s
#define IREF_DERIV_THRESH  50.0f
#define RPM_DERIV_THRESH  500.0f    // tr/min/s
#define FREEZE_TRANS_MS   300       // durée de freeze ESC après transitoire

// Lead base (LUT en fonction du rpm)
typedef struct {
    float rpm;
    float lead_deg;
} LeadLUT;

LeadLUT lut[] = {
    {   0.0f,  5.0f },
    { 500.0f, 10.0f },
    {1000.0f, 15.0f },
    {2000.0f, 20.0f },
    {4000.0f, 30.0f },
    {6000.0f, 35.0f }
};
#define LUT_SIZE (sizeof(lut)/sizeof(lut[0]))

#define LEAD_BASE_UPDATE_MS     50      // update lead_base toutes les 50 ms
#define LEAD_BASE_SMOOTH_ALPHA  0.1f    // lissage lead_base
#define LEAD_BASE_DELTA_THRESH  1.0f    // seuil de changement fort [deg]
#define LEAD_BASE_FREEZE_MS     200     // freeze ESC après grosse variation

// ----------------------------
// Etats internes ESC
// ----------------------------
static float Idc_filt = 0.0f;
static float rpm_filt = 0.0f;

static float lead_base = 10.0f;
static float lead_base_raw = 10.0f;

static float esc_offset = 0.0f;
static float lpf_corr = 0.0f;

static int freeze_timer_ms = 0;
static bool esc_locked = false;

// dither
static int dither_sign = +1;
static int sample_cnt = 0;
const int samples_per_toggle = 10; // dither toggle toutes les 10 ms = 50 Hz

// ----------------------------
// Outils filtres
// ----------------------------
float filter_iir(float prev, float input, float alpha) {
    return alpha * input + (1.0f - alpha) * prev;
}

// Lookup LUT rpm->lead
float lut_lookup_lead(float rpm) {
    if (rpm <= lut[0].rpm) return lut[0].lead_deg;
    if (rpm >= lut[LUT_SIZE-1].rpm) return lut[LUT_SIZE-1].lead_deg;

    for (int i=0; i<LUT_SIZE-1; i++) {
        if (rpm >= lut[i].rpm && rpm < lut[i+1].rpm) {
            float t = (rpm - lut[i].rpm) / (lut[i+1].rpm - lut[i].rpm);
            return lut[i].lead_deg + t * (lut[i+1].lead_deg - lut[i].lead_deg);
        }
    }
    return lut[0].lead_deg;
}

float clampf(float x, float minv, float maxv) {
    if (x < minv) return minv;
    if (x > maxv) return maxv;
    return x;
}

// ----------------------------
// Simulation PWM LUT (256 steps)
// Ici juste un exemple placeholder
// ----------------------------
void set_commutation_lead(float lead_deg) {
    int idx = (int)(256.0f * (lead_deg / 360.0f)) % 256;
    // dans ton vrai code : utiliser idx pour décaler la LUT PWM
    printf("Lead = %.2f deg (idx=%d)\n", lead_deg, idx);
}

// ----------------------------
// Boucle ESC appelée à 1 kHz
// ----------------------------
void ESC_step(float Iref, float Idc_raw, float rpm_raw) {
    // 1) Filtrage signaux
    Idc_filt = filter_iir(Idc_filt, Idc_raw, 0.01f); // tau ~100 ms
    rpm_filt = filter_iir(rpm_filt, rpm_raw, 0.05f); // tau ~20 ms

    // 2) Mise à jour périodique lead_base
    static int accum_ms = 0;
    accum_ms += (int)UPDATE_DT_MS;    // +1ms
    if (accum_ms >= LEAD_BASE_UPDATE_MS) {   // every 200ms
        accum_ms = 0;
        float new_lead_raw = lut_lookup_lead(rpm_filt);

        if (fabsf(new_lead_raw - lead_base_raw) > LEAD_BASE_DELTA_THRESH) {
            esc_locked = true;
            freeze_timer_ms = LEAD_BASE_FREEZE_MS;
        }

        lead_base_raw = new_lead_raw;
        lead_base = LEAD_BASE_SMOOTH_ALPHA * lead_base_raw
                  + (1.0f - LEAD_BASE_SMOOTH_ALPHA) * lead_base; // lead base filtré
    }

    // 3) Gestion freeze ESC
    if (freeze_timer_ms > 0) {
        freeze_timer_ms -= (int)UPDATE_DT_MS;
        if (freeze_timer_ms <= 0) esc_locked = false;
    }

    if (esc_locked) {
        set_commutation_lead(lead_base); // sans dither ni offset
        return;
    }

    // 4) Dither (±A)
    if (++sample_cnt >= samples_per_toggle) { // change de sens tous le 10ms
        sample_cnt = 0;
        dither_sign = -dither_sign;
    }
    float dither = dither_sign * DITHER_AMP;

    // 5) Calcul lead commandé
    float commanded_lead = lead_base + esc_offset + dither;
    commanded_lead = clampf(commanded_lead, ANGLE_MIN, ANGLE_MAX);
    set_commutation_lead(commanded_lead);

    // 6) Coût = Idc filtré
    float J = Idc_filt;

    // 7) Corrélation dither*J filtrée
    float corr = J * (float)dither_sign;
    lpf_corr = LPF_ALPHA * corr + (1.0f - LPF_ALPHA) * lpf_corr;

    // 8) Mise à jour ESC offset
    esc_offset -= ESC_GAIN * lpf_corr;
    esc_offset = clampf(esc_offset, -10.0f, +10.0f);
}

// ----------------------------
// Exemple test main
// ----------------------------
int main(void) {
    for (int t=0; t<2000; t++) { // 2s @1kHz
        float Iref = 10.0f;
        float Idc  = 9.5f + 0.1f * sinf(0.01f*t); // simulateur bidon
        float rpm  = 1500.0f + 50.0f * sinf(0.005f*t);
        ESC_step(Iref, Idc, rpm);
    }
    return 0;
}



#endif
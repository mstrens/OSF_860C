#ifdef _USE_THIS

uint16_t ui16_adc_pedal_torque_offset;       // Offset effectif utilisé
uint16_t ui16_adc_pedal_torque_adc_knee;    // ADC au point knee
//uint16_t ui16_adc_pedal_torque_adc_80kg;     // ADC théorique à 80 kg

uint32_t ui32_pedal_torque_k1_q16;           // pente segment 1, Q16
uint32_t ui32_pedal_torque_k2_q16;           // pente segment 2, Q16
uint8_t  ui8_pedal_torque_knee_value;        // sortie au knee (0...160)

uint8_t  ui8_pedal_torque_knee_adc_percent = 10;
uint8_t  ui8_pedal_torque_knee_weight      = 50;
uint8_t  ui8_pedal_torque_max_weight       = 80;

uint16_t ui16_adc_pedal_torque_offset_set;
uint16_t ui16_adc_pedal_torque_range;

Boolean pedal_torque_parameter_error = false;

void pedal_torque_update_parameters(void)
{
    uint32_t ui32_temp;
    uint32_t ui32_adc_knee;
    uint32_t ui32_adc_max;

    uint32_t ui32_delta_adc_2;
    uint32_t ui32_delta_weight_2;

    uint32_t ui32_adc80_delta;

    // determine offset
    ui16_adc_pedal_torque_offset = ui16_adc_pedal_torque_offset_set;

    // calculate ADC max
    ui32_adc_max = (uint32_t) ui16_adc_pedal_torque_offset + (uint32_t) ui16_adc_pedal_torque_range;

    pedal_torque_parameter_error = false;
    // first check some parameters
    if ( ( ui16_adc_pedal_torque_offset < 100) ||
            ( ui16_adc_pedal_torque_offset > 300) ||
            ( ui16_adc_pedal_torque_range < 50) ||
            ( ui8_pedal_torque_knee_weight < 20) || 
            ( ui8_pedal_torque_knee_weight > 70) ||
            ( ui8_pedal_torque_max_weight > 100) || 
            ( ui8_pedal_torque_knee_weight >= ui8_pedal_torque_max_weight) || 
            ( ( ui8_pedal_torque_max_weight - ui8_pedal_torque_knee_weight ) < 10)||
            (ui8_pedal_torque_knee_adc_percent == 0) ||
            (ui8_pedal_torque_knee_adc_percent > 34) ||
            ((ui32_adc_max < 100) || (ui32_adc_max > 600))  
        ) {
        pedal_torque_parameter_error = true;
        return;
    }
  
    /*
     * ------------------------------------------------------------
     * ADC at the knee
     *
     * knee_adc_percent is the percentage of the ADC range
     * corresponding to the saturated part.
     * Therefore:  100 - knee_adc_percent is the ADC range up to the knee.
     * ------------------------------------------------------------
     */
    ui32_adc_knee =
        (uint32_t) ui16_adc_pedal_torque_range
        * (100U - ui8_pedal_torque_knee_adc_percent);
    ui32_adc_knee /= 100U;
    ui32_adc_knee += ui16_adc_pedal_torque_offset;
    ui16_adc_pedal_torque_adc_knee = (uint16_t) ui32_adc_knee;
    
    // normalized Value corresponding to the knee because 80kg -> 160
    // 0 kg  -> 0
    // Wk    -> Wk * 2
    ui8_pedal_torque_knee_value = (uint8_t) (2U * ui8_pedal_torque_knee_weight);
    
    // First segment coefficient (from ADC offset -> 0 up to ADC knee   -> knee_value) in  Q16 coefficient:
    // K1 = knee_value * 65536 / delta_ADC
    ui32_temp = (uint32_t) ui16_adc_pedal_torque_adc_knee - (uint32_t) ui16_adc_pedal_torque_offset;
    if (ui32_temp != 0)    {
        ui32_pedal_torque_k1_q16 = ((uint64_t) ui8_pedal_torque_knee_value << 16) / ui32_temp;
    }    else     {
        ui32_pedal_torque_k1_q16 = 0;
    }

    // Second segment (from Wk -> ADC_knee up to Wm -> ADC_max);  * We extrapolate/interpolate to 80 kg.
    // We don't actually calculate ADC80 with integer rounding.
    // Instead, we calculate its equivalent slope directly.
    ui32_delta_adc_2 = ui32_adc_max - (uint32_t) ui16_adc_pedal_torque_adc_knee;
    ui32_delta_weight_2 = (uint32_t) ui8_pedal_torque_max_weight - (uint32_t) ui8_pedal_torque_knee_weight;

    // ADC difference corresponding to 80 kg:
    // ADC80 - ADCknee =  ( delta_ADC * (80 - Wk) )    /   (  Wm - Wk )
    // This is kept as a rational value so that we don't lose precision by rounding ADC80 first.
    if ((ui32_delta_adc_2 != 0)
        && (ui32_delta_weight_2 != 0)
        && (ui8_pedal_torque_knee_weight < 80))     {
        ui32_adc80_delta = (uint32_t) (80U - ui8_pedal_torque_knee_weight);
        // K2 =  (160 - knee_value) * 65536 / (  ADC80 - ADCknee )
        // with  (ADC80 - ADCknee) =  delta_ADC * (80-Wk)/(Wm-Wk)
        // Therefore: K2 = (160-knee) * 65536 * (Wm-Wk) / ( delta_ADC * (80-Wk) )

        ui32_pedal_torque_k2_q16 = ((uint64_t) (160U - ui8_pedal_torque_knee_value) * 65536ULL  * ui32_delta_weight_2)
                                    /   ((uint64_t) ui32_delta_adc_2 * ui32_adc80_delta);
    }     else     {
        ui32_pedal_torque_k2_q16 = 0;
    }
                
    // check that slope of first segment is higher than for second segment 
    if (((ui32_adc_knee - ui16_adc_pedal_torque_offset) * (ui8_pedal_torque_max_weight - ui8_pedal_torque_knee_weight))
        <=  ((ui32_adc_max - ui32_adc_knee) * ui8_pedal_torque_knee_weight))
    {
        pedal_torque_parameter_error = true;
    }
}

uint8_t pedal_torque_process_adc(uint16_t ui16_adc_torque_filtered)
{
    uint32_t ui32_delta;
    uint32_t ui32_result;

    if (pedal_torque_parameter_error) {
        return 0;
    }
    // Below 0 kg
    if (ui16_adc_torque_filtered <= ui16_adc_pedal_torque_offset)   { 
        return 0;
    }

    // First segment    
    if (ui16_adc_torque_filtered <= ui16_adc_pedal_torque_adc_knee)     {
        ui32_delta = (uint32_t) ui16_adc_torque_filtered - (uint32_t) ui16_adc_pedal_torque_offset;
        ui32_result = ((uint32_t) ui32_delta * ui32_pedal_torque_k1_q16 + 32768U) >> 16;
        if (ui32_result > 160) {  return 160; }
        return (uint8_t) ui32_result;
    }

    // Second segment
    ui32_delta = (uint32_t) ui16_adc_torque_filtered - (uint32_t) ui16_adc_pedal_torque_adc_knee;
    ui32_result = ui8_pedal_torque_knee_value + (((uint32_t) ui32_delta * ui32_pedal_torque_k2_q16 + 32768U) >> 16);
    if (ui32_result >= 160)    { return 160; }
    return (uint8_t) ui32_result;
}

#endif
/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include "common.h"
#include "cybsp.h"
#include "cy_utils.h"
#include "cy_retarget_io.h"

// Function to map a value from one range to another based on given input and output ranges.
// Uses nearest integer rounding for precision.
// Note: Input min has to be smaller than input max.
// Parameters:
// - in: Value to be mapped.
// - in_min: Minimum value of the input range. 
// - in_max: Maximum value of the input range.
// - out_min: Minimum value of the output range.
// - out_max: Maximum value of the output range.
// Returns the mapped value within the specified output range.
uint16_t map_ui16(uint16_t in, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    // If input is out of bounds, clamp it to the nearest boundary value
    if (in < in_min) {return out_min;}
    if (in >= in_max) {return out_max;}

    // Calculate the input and output ranges
    uint16_t in_range = in_max - in_min;

    uint16_t out;
    if (out_max < out_min) {
        out = out_min - (uint16_t)(uint32_t)(((uint32_t)((uint32_t)(uint16_t)(in - in_min) * (uint32_t)(uint16_t)(out_min - out_max)) + (uint32_t)(uint16_t)(in_range/2U)) / in_range);
    } else {
        out = out_min + (uint16_t)(uint32_t)(((uint32_t)((uint32_t)(uint16_t)(in - in_min) * (uint32_t)(uint16_t)(out_max - out_min)) + (uint32_t)(uint16_t)(in_range/2U)) / in_range);
    }
    return out;
}

// Function to map 8bit a values from one range to another based on given input and output ranges.
// Uses floor integer rounding for maximum performance.
// Note: Input min has to be smaller than input max.
// Parameters:
// - in: Value to be mapped.
// - in_min: Minimum value of the input range.
// - in_max: Maximum value of the input range.
// - out_min: Minimum value of the output range.
// - out_max: Maximum value of the output range.
// Returns the mapped value within the specified output range.
uint8_t map_ui8(uint8_t in, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max) {
    // If input is out of bounds, clamp it to the nearest boundary value
    if (in <= in_min) {return out_min;}
    if (in >= in_max) {return out_max;}

    if (out_max < out_min) {
        return out_min - (uint8_t)(uint16_t)((uint16_t)((uint8_t)(in - in_min) * (uint8_t)(out_min - out_max)) / (uint8_t)(in_max - in_min)); // cppcheck-suppress misra-c2012-10.8 ; direct cast to a wider essential to ensure mul in,a usage
    } else {
        return out_min + (uint8_t)(uint16_t)((uint16_t)((uint8_t)(in - in_min) * (uint8_t)(out_max - out_min)) / (uint8_t)(in_max - in_min)); // cppcheck-suppress misra-c2012-10.8 ; direct cast to a wider essential to ensure mul in,a usage
    }
}


uint8_t ui8_min(uint8_t value_a, uint8_t value_b) {
    if (value_a < value_b) {
        return value_a;
    } else {
        return value_b;
    }
}

uint8_t ui8_max(uint8_t value_a, uint8_t value_b) {
    if (value_a > value_b)
        return value_a;
    else
        return value_b;
}

uint16_t filter(uint16_t ui16_new_value, uint16_t ui16_old_value, uint8_t ui8_alpha) {
    if (ui8_alpha < 11) {
        uint32_t ui32_temp_new = (uint32_t) ui16_new_value * (uint32_t)(10U - ui8_alpha);
		uint32_t ui32_temp_old = (uint32_t) ui16_old_value * (uint32_t) ui8_alpha;
        uint16_t ui16_filtered_value = (uint16_t)((ui32_temp_new + ui32_temp_old + 5U) / 10U);

        if (ui16_filtered_value == ui16_old_value) {
            if (ui16_filtered_value < ui16_new_value)
				ui16_filtered_value++;
			else if (ui16_filtered_value > ui16_new_value)
				ui16_filtered_value--;
        }

        return ui16_filtered_value;
    } else {
        return 0;
    }
}

// from here: https://github.com/FxDev/PetitModbus/blob/master/PetitModbus.c
/*
 * Function Name        : CRC16
 * @param[in]           : ui8_data  - Data to Calculate CRC
 * @param[in/out]       : ui16_crc   - Anlik CRC degeri
 * @How to use          : First initial data has to be 0xFFFF.
 */
void crc16(uint8_t ui8_data, uint16_t *ui16_crc) {
    unsigned int i;

    *ui16_crc = *ui16_crc ^ (uint16_t) ui8_data;

    for (i = 8; i > 0; i--) {
        if (*ui16_crc & 0x0001) {
            *ui16_crc = (*ui16_crc >> 1) ^ 0xA001;
        } else {
            *ui16_crc >>= 1;
        }
    }
}

// mstrens :  this function was moved from another file in TSDZ2 and adapted for new mcu
void lights_set_state(uint8_t ui8_state) {
    if (ui8_state) {
        XMC_GPIO_SetOutputHigh(OUT_LIGHT_PORT,OUT_LIGHT_PIN) ; // GPIO_WriteHigh(LIGHTS__PORT, LIGHTS__PIN);
    } else {
        XMC_GPIO_SetOutputLow(OUT_LIGHT_PORT,OUT_LIGHT_PIN) ;  // GPIO_WriteLow(LIGHTS__PORT, LIGHTS__PIN);
    }
}

uint32_t last_action_systicks[10]= {0};

// retun true when enlapsed time expired
bool take_action(uint32_t index, uint32_t interval){
    if (index < 10) {
        if ((system_ticks - last_action_systicks[index]) > interval){
            last_action_systicks[index] = system_ticks;
            return true;
        }
    }
    return false;
}


void wait_ms(uint32_t time){
    uint32_t start = system_ticks;
    uint32_t counter = 0;
    while ( (system_ticks - start) < time){
        counter++;
    }
    counter = 0;
}
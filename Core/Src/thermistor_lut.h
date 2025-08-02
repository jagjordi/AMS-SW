/*
 * thermistor_lut.h
 *
 *  Created on: Aug 2, 2025
 *      Author: Jordi
 */

#ifndef THERMISTOR_LUT_H
#define THERMISTOR_LUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configuration:
// - NTC: 10 kΩ @ 25 °C, Beta = 3500 K
// - Divider: 10 kΩ pull-up to 3 V, NTC to GND
// - ADC scale: 0 … 30000 counts ≙ 0 … 3.0 V
// Temperature range: −10 … +90 °C (inclusive)

#define THERM_LUT_MIN_C   (-10)
#define THERM_LUT_MAX_C   (90)
#define THERM_LUT_SIZE    (THERM_LUT_MAX_C - THERM_LUT_MIN_C + 1)
#define THERM_ADC_MAX     (30000u)

// ADC codes for each integer °C from −10 to +90 (monotonic decreasing).
extern const uint16_t therm_lut_adc[THERM_LUT_SIZE];

/** Convert ADC code (0..30000) to temperature in °C as float.
 *  Clamps outside [−10,90] to the nearest endpoint.
 */
float thermistor_adc_to_c_float(uint16_t adc);

#ifdef __cplusplus
}
#endif

#endif // THERMISTOR_LUT_H

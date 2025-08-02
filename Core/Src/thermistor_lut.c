/*
 * thermistor_lut.c
 *
 *  Created on: Aug 2, 2025
 *      Author: Jordi
 */

#include "thermistor_lut.h"

// Generated from Beta model: R(T) = R0 * exp(B*(1/T - 1/T0))
// with R0=10k, B=3500K, T0=298.15K, 10k pull-up, Vref=Vcc=3.0V.
// ADC(T) = round(30000 * R / (R + 10000))

const uint16_t therm_lut_adc[THERM_LUT_SIZE] = {
    /*  -10 C */ 24796, /*   -9 C */ 24576, /*   -8 C */ 24351, /*   -7 C */ 24120,
    /*   -6 C */ 23883, /*   -5 C */ 23642, /*   -4 C */ 23396, /*   -3 C */ 23145,
    /*   -2 C */ 22889, /*   -1 C */ 22628, /*    0 C */ 22363, /*    1 C */ 22094,
    /*    2 C */ 21821, /*    3 C */ 21544, /*    4 C */ 21264, /*    5 C */ 20980,
    /*    6 C */ 20693, /*    7 C */ 20403, /*    8 C */ 20111, /*    9 C */ 19816,
    /*   10 C */ 19519, /*   11 C */ 19221, /*   12 C */ 18921, /*   13 C */ 18619,
    /*   14 C */ 18317, /*   15 C */ 18014, /*   16 C */ 17710, /*   17 C */ 17407,
    /*   18 C */ 17103, /*   19 C */ 16799, /*   20 C */ 16497, /*   21 C */ 16195,
    /*   22 C */ 15894, /*   23 C */ 15594, /*   24 C */ 15296, /*   25 C */ 15000,
    /*   26 C */ 14706, /*   27 C */ 14414, /*   28 C */ 14124, /*   29 C */ 13837,
    /*   30 C */ 13552, /*   31 C */ 13271, /*   32 C */ 12992, /*   33 C */ 12717,
    /*   34 C */ 12445, /*   35 C */ 12177, /*   36 C */ 11912, /*   37 C */ 11651,
    /*   38 C */ 11394, /*   39 C */ 11140, /*   40 C */ 10890, /*   41 C */ 10645,
    /*   42 C */ 10403, /*   43 C */ 10166, /*   44 C */  9933, /*   45 C */  9704,
    /*   46 C */  9479, /*   47 C */  9258, /*   48 C */  9041, /*   49 C */  8829,
    /*   50 C */  8621, /*   51 C */  8417, /*   52 C */  8218, /*   53 C */  8022,
    /*   54 C */  7831, /*   55 C */  7644, /*   56 C */  7461, /*   57 C */  7282,
    /*   58 C */  7107, /*   59 C */  6935, /*   60 C */  6768, /*   61 C */  6605,
    /*   62 C */  6445, /*   63 C */  6289, /*   64 C */  6137, /*   65 C */  5989,
    /*   66 C */  5844, /*   67 C */  5702, /*   68 C */  5564, /*   69 C */  5430,
    /*   70 C */  5299, /*   71 C */  5170, /*   72 C */  5046, /*   73 C */  4924,
    /*   74 C */  4805, /*   75 C */  4689, /*   76 C */  4577, /*   77 C */  4467,
    /*   78 C */  4360, /*   79 C */  4255, /*   80 C */  4153, /*   81 C */  4054,
    /*   82 C */  3958, /*   83 C */  3864, /*   84 C */  3772, /*   85 C */  3683,
    /*   86 C */  3596, /*   87 C */  3511, /*   88 C */  3428, /*   89 C */  3348,
    /*   90 C */  3270,
};

static int find_segment(uint16_t adc)
{
    // Returns index i such that adc is between [lut[i], lut[i+1]] (lut decreasing)
    int lo = 0;
    int hi = THERM_LUT_SIZE - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (therm_lut_adc[mid] > adc) {
            lo = mid;    // move toward higher temperature (lower ADC)
        } else {
            hi = mid;    // move toward lower temperature (higher ADC)
        }
    }
    return lo;
}

float thermistor_adc_to_c_float(uint16_t adc)
{
    // Clamp outside table range
    if (adc >= therm_lut_adc[0])                return (float)THERM_LUT_MIN_C;
    if (adc <= therm_lut_adc[THERM_LUT_SIZE-1]) return (float)THERM_LUT_MAX_C;

    int i = find_segment(adc);
    uint16_t a_hi = therm_lut_adc[i];       // at T = T_lo
    uint16_t a_lo = therm_lut_adc[i + 1];   // at T = T_lo + 1
    float dA = (float)a_hi - (float)a_lo;   // > 0
    float frac = ((float)a_hi - (float)adc) / dA; // 0..1 within the 1 °C span
    float t_lo = (float)(THERM_LUT_MIN_C + i);
    return t_lo + frac; // linear interpolation
}


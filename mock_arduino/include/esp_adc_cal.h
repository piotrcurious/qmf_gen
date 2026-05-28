#pragma once

#include <stdint.h>
#include "driver/adc.h"

typedef struct {
    adc_unit_t adc_num;
    adc_atten_t atten;
    adc_bits_width_t bit_width;
    uint32_t vref;
} esp_adc_cal_characteristics_t;

esp_err_t esp_adc_cal_check_efuse(int value_type);
esp_adc_cal_characteristics_t* esp_adc_cal_characterize(adc_unit_t adc_num, adc_atten_t atten, adc_bits_width_t bit_width, uint32_t default_vref, esp_adc_cal_characteristics_t *chars);
uint32_t esp_adc_cal_raw_to_voltage(uint32_t adc_reading, const esp_adc_cal_characteristics_t *chars);

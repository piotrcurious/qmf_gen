#pragma once

#include <stdint.h>
#include <stddef.h>
#include "driver/adc.h"

// Types are shared with driver/adc.h in this mock for simplicity
typedef adc1_channel_t adc_channel_t;

typedef enum {
    ADC_BITWIDTH_9 = 9,
    ADC_BITWIDTH_10 = 10,
    ADC_BITWIDTH_11 = 11,
    ADC_BITWIDTH_12 = 12,
} adc_bitwidth_t;

typedef void* adc_continuous_handle_t;

typedef struct {
    uint32_t max_store_buf_size;
    uint32_t conv_frame_size;
} adc_continuous_handle_cfg_t;

typedef enum {
    ADC_CONV_SINGLE_UNIT_1,
    ADC_CONV_SINGLE_UNIT_2,
    ADC_CONV_BOTH_UNIT,
    ADC_CONV_ALTER_UNIT,
} adc_continuous_conv_mode_t;

typedef enum {
    ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    ADC_DIGI_OUTPUT_FORMAT_TYPE2,
} adc_continuous_output_format_t;

typedef struct {
    adc_atten_t atten;
    adc_channel_t channel;
    adc_unit_t unit;
    adc_bitwidth_t bit_width;
} adc_digi_pattern_config_t;

typedef struct {
    uint32_t pattern_num;
    adc_digi_pattern_config_t *adc_pattern;
    uint32_t sample_freq_hz;
    adc_continuous_conv_mode_t conv_mode;
    adc_continuous_output_format_t format;
} adc_continuous_config_t;

typedef struct {
    struct {
        uint32_t data : 12;
        uint32_t channel : 4;
        uint32_t unit : 1;
        uint32_t reserved : 15;
    } type1;
} adc_digi_output_data_t;

#define ESP_ERROR_CHECK(x) (void)(x)

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t adc_continuous_new_handle(const adc_continuous_handle_cfg_t *hdl_config, adc_continuous_handle_t *ret_handle);
esp_err_t adc_continuous_config(adc_continuous_handle_t handle, const adc_continuous_config_t *config);
esp_err_t adc_continuous_start(adc_continuous_handle_t handle);
esp_err_t adc_continuous_read(adc_continuous_handle_t handle, uint8_t *buf, uint32_t length_max, uint32_t *out_length, uint32_t timeout_ms);
esp_err_t adc_continuous_stop(adc_continuous_handle_t handle);
esp_err_t adc_continuous_deinit(adc_continuous_handle_t handle);

#ifdef __cplusplus
}
#endif

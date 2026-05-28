#pragma once

#include <stdint.h>
#include <stddef.h>

typedef enum {
    I2S_NUM_0 = 0,
    I2S_NUM_1 = 1,
    I2S_NUM_MAX,
} i2s_port_t;

typedef enum {
    I2S_MODE_MASTER = 1,
    I2S_MODE_SLAVE = 2,
    I2S_MODE_TX = 4,
    I2S_MODE_RX = 8,
    I2S_MODE_DAC_BUILT_IN = 16,
    I2S_MODE_ADC_BUILT_IN = 32,
    I2S_MODE_PDM = 64,
} i2s_mode_t;

typedef enum {
    I2S_BITS_PER_SAMPLE_8BIT = 8,
    I2S_BITS_PER_SAMPLE_16BIT = 16,
    I2S_BITS_PER_SAMPLE_24BIT = 24,
    I2S_BITS_PER_SAMPLE_32BIT = 32,
} i2s_bits_per_sample_t;

typedef enum {
    I2S_CHANNEL_FMT_RIGHT_LEFT = 0,
    I2S_CHANNEL_FMT_ALL_RIGHT,
    I2S_CHANNEL_FMT_ALL_LEFT,
    I2S_CHANNEL_FMT_ONLY_RIGHT,
    I2S_CHANNEL_FMT_ONLY_LEFT,
} i2s_channel_fmt_t;

typedef enum {
    I2S_COMM_FORMAT_STAND_I2S = 0x01,
    I2S_COMM_FORMAT_STAND_MSB = 0x02,
    I2S_COMM_FORMAT_STAND_PCM_SHORT = 0x04,
    I2S_COMM_FORMAT_STAND_PCM_LONG = 0x08,
} i2s_comm_format_t;

typedef struct {
    i2s_mode_t mode;
    uint32_t sample_rate;
    i2s_bits_per_sample_t bits_per_sample;
    i2s_channel_fmt_t channel_format;
    i2s_comm_format_t communication_format;
    int intr_alloc_flags;
    int dma_buf_count;
    int dma_buf_len;
    bool use_apll;
    bool tx_desc_auto_clear;
    int fixed_mclk;
} i2s_config_t;

typedef enum {
    I2S_DAC_CHANNEL_DISABLE = 0,
    I2S_DAC_CHANNEL_RIGHT_EN = 1,
    I2S_DAC_CHANNEL_LEFT_EN = 2,
    I2S_DAC_CHANNEL_BOTH_EN = 3,
    I2S_DAC_CHANNEL_MAX,
} i2s_dac_mode_t;

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1

esp_err_t i2s_driver_install(i2s_port_t i2s_num, const i2s_config_t *i2s_config, int queue_size, void* i2s_queue);
esp_err_t i2s_set_pin(i2s_port_t i2s_num, const void *pin_config);
esp_err_t i2s_set_dac_mode(i2s_dac_mode_t dac_mode);
esp_err_t i2s_read(i2s_port_t i2s_num, void *dest, size_t size, size_t *bytes_read, uint32_t ticks_to_wait);
esp_err_t i2s_write(i2s_port_t i2s_num, const void *src, size_t size, size_t *bytes_written, uint32_t ticks_to_wait);
esp_err_t i2s_set_adc_mode(int adc_unit, int adc_channel);
esp_err_t i2s_adc_enable(i2s_port_t i2s_num);
esp_err_t i2s_adc_disable(i2s_port_t i2s_num);

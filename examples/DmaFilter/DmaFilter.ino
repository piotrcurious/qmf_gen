/*
  DMA-based QMF Filter Example for ESP32 (Arduino Core 3.0+ / ESP-IDF 5.x)

  This sketch uses the ADC Continuous Driver (official ESP-IDF API)
  to achieve high-speed background sampling, and I2S-based DAC for output.

  This matches the recommended framework for ESP32 Arduino 3.3.1+.
*/

#include <Arduino.h>
#include <driver/i2s.h>
#include "../../qmf_24.h"

extern "C" {
  #include "esp_adc/adc_continuous.h"
}

// Configuration
#define SAMPLE_RATE         44100
#define ADC_CHANNELS_COUNT  1
#define ADC_READ_BUF_BYTES  1024
#define ADC_FRAME_SIZE      256

static adc_channel_t adc_channels[ADC_CHANNELS_COUNT] = {ADC_CHANNEL_0}; // GPIO 36
static adc_continuous_handle_t adc_handle = NULL;

QMF2 qmf;

// I2S configuration for DAC output
void setup_dac() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
    i2s_set_pin(I2S_NUM_0, NULL); // Use internal DAC
}

void setup_adc_continuous() {
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = ADC_READ_BUF_BYTES,
        .conv_frame_size = ADC_FRAME_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    adc_continuous_config_t dig_cfg = {
        .pattern_num = ADC_CHANNELS_COUNT,
        .adc_pattern = NULL,
        .sample_freq_hz = SAMPLE_RATE,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    static adc_digi_pattern_config_t adc_pattern[ADC_CHANNELS_COUNT];
    for (int i = 0; i < ADC_CHANNELS_COUNT; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = adc_channels[i];
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = ADC_BITWIDTH_12;
    }
    dig_cfg.adc_pattern = adc_pattern;

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

void setup() {
    Serial.begin(115200);
    Serial.println("QMF ADC Continuous Filter Example Started");

    setup_dac();
    setup_adc_continuous();
}

void loop() {
    uint8_t read_raw[ADC_FRAME_SIZE];
    uint32_t out_len = 0;

    // Non-blocking read from ADC
    esp_err_t ret = adc_continuous_read(adc_handle, read_raw, ADC_FRAME_SIZE, &out_len, 10);

    if (ret == ESP_OK && out_len > 0) {
        size_t count = out_len / sizeof(adc_digi_output_data_t);
        adc_digi_output_data_t *p = (adc_digi_output_data_t*)read_raw;

        for (size_t i = 0; i < count; i++) {
            uint32_t val = p[i].type1.data & 0xFFF;
            float in_sample = (val - 2048) / 2048.0f;

            float low, high;
            if (qmf.process(in_sample, low, high)) {
                // Convert back for internal DAC
                uint16_t out_low = (uint16_t)((low + 1.0f) * 127.5f) << 8;
                uint16_t out_high = (uint16_t)((high + 1.0f) * 127.5f) << 8;

                uint16_t write_buff[2] = {out_low, out_high};
                size_t bytes_written;
                i2s_write(I2S_NUM_0, (const char *)write_buff, sizeof(write_buff), &bytes_written, 0);
            }
        }
    }
}

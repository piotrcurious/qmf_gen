/*
  DMA-based QMF Filter Example for ESP32

  This sketch uses ADC continuous DMA sampling and I2S-based DAC output
  for efficient real-time audio filtering.
*/

#include <Arduino.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include "../../qmf_24.h"

// Configuration
#define SAMPLE_RATE 44100
#define I2S_NUM      I2S_NUM_0
#define DMA_BUF_LEN  64
#define DMA_BUF_CNT  8

QMF2 qmf;

// I2S configuration
void setup_i2s() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = DMA_BUF_CNT,
        .dma_buf_len = DMA_BUF_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL_0);
    i2s_adc_enable(I2S_NUM);
    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
    i2s_set_pin(I2S_NUM, NULL); // Use internal DAC
}

void setup() {
    Serial.begin(115200);
    Serial.println("QMF DMA Filter Example Started");
    setup_i2s();
}

void loop() {
    uint16_t i2s_read_buff[DMA_BUF_LEN];
    size_t bytes_read;

    // Read from ADC via DMA
    esp_err_t result = i2s_read(I2S_NUM, (char *)i2s_read_buff, DMA_BUF_LEN * sizeof(uint16_t), &bytes_read, portMAX_DELAY);

    if (result == ESP_OK && bytes_read > 0) {
        size_t samples_read = bytes_read / sizeof(uint16_t);
        uint16_t i2s_write_buff[DMA_BUF_LEN];
        size_t write_idx = 0;

        for (size_t i = 0; i < samples_read; i++) {
            // ESP32 ADC in DMA mode gives 12-bit samples in a 16-bit word
            // The format can depend on the ESP-IDF version, typically it's [V_unit:4, val:12]
            float in_sample = ((i2s_read_buff[i] & 0x0FFF) - 2048) / 2048.0f;

            float low, high;
            if (qmf.process(in_sample, low, high)) {
                // Here we process the low and high bands.
                // For demonstration, we just mix them back (reconstruction).
                // In a real crossover, you'd send low to one DAC channel and high to another.

                // Convert back to 8-bit for internal DAC (0-255)
                // Internal DAC expects the high byte of the 16-bit I2S word for 8-bit mode usually
                uint16_t out_low = (uint16_t)((low + 1.0f) * 127.5f) << 8;
                uint16_t out_high = (uint16_t)((high + 1.0f) * 127.5f) << 8;

                // Pack into write buffer (stereo)
                i2s_write_buff[write_idx++] = out_low;
                i2s_write_buff[write_idx++] = out_high;
            }
        }

        if (write_idx > 0) {
            size_t bytes_written;
            i2s_write(I2S_NUM, (const char *)i2s_write_buff, write_idx * sizeof(uint16_t), &bytes_written, portMAX_DELAY);
        }
    }
}

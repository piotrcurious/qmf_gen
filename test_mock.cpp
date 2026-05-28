#include "Arduino.h"
#include "driver/i2s.h"
#include <cassert>

int main() {
    Serial.begin(115200);
    Serial.println("Testing Mock Arduino Environment...");

    uint32_t t1 = millis();
    delay(10);
    uint32_t t2 = millis();
    assert(t2 >= t1 + 10);

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    assert(err == ESP_OK);

    Serial.println("Mock Environment Test Passed!");
    return 0;
}

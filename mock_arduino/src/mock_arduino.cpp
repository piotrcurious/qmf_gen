#include "Arduino.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_adc/adc_continuous.h"
#include <chrono>
#include <thread>
#include <cstdarg>

MockSerial Serial;

void pinMode(uint8_t pin, uint8_t mode) {}
int analogRead(uint8_t pin) { return 0; }
void analogWrite(uint8_t pin, int value) {}

void delay(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void delayMicroseconds(uint32_t us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

uint32_t millis() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

uint32_t micros() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
}

esp_err_t i2s_driver_install(i2s_port_t i2s_num, const i2s_config_t *i2s_config, int queue_size, void* i2s_queue) { return ESP_OK; }
esp_err_t i2s_set_pin(i2s_port_t i2s_num, const void *pin_config) { return ESP_OK; }
esp_err_t i2s_set_dac_mode(i2s_dac_mode_t dac_mode) { return ESP_OK; }
esp_err_t i2s_read(i2s_port_t i2s_num, void *dest, size_t size, size_t *bytes_read, uint32_t ticks_to_wait) {
    *bytes_read = size;
    return ESP_OK;
}
esp_err_t i2s_write(i2s_port_t i2s_num, const void *src, size_t size, size_t *bytes_written, uint32_t ticks_to_wait) {
    *bytes_written = size;
    return ESP_OK;
}
esp_err_t i2s_set_adc_mode(int adc_unit, int adc_channel) { return ESP_OK; }
esp_err_t i2s_adc_enable(i2s_port_t i2s_num) { return ESP_OK; }
esp_err_t i2s_adc_disable(i2s_port_t i2s_num) { return ESP_OK; }

esp_err_t adc1_config_width(adc_bits_width_t width_bit) { return ESP_OK; }
esp_err_t adc1_config_channel_atten(adc1_channel_t channel, adc_atten_t atten) { return ESP_OK; }
int adc1_get_raw(adc1_channel_t channel) { return 0; }

esp_err_t esp_adc_cal_check_efuse(int value_type) { return ESP_OK; }
esp_adc_cal_characteristics_t* esp_adc_cal_characterize(adc_unit_t adc_num, adc_atten_t atten, adc_bits_width_t bit_width, uint32_t default_vref, esp_adc_cal_characteristics_t *chars) {
    chars->adc_num = adc_num;
    chars->atten = atten;
    chars->bit_width = bit_width;
    chars->vref = default_vref;
    return chars;
}
uint32_t esp_adc_cal_raw_to_voltage(uint32_t adc_reading, const esp_adc_cal_characteristics_t *chars) {
    return (adc_reading * chars->vref) / 4095;
}

static adc_continuous_result_t mock_adc_results[16];
static adc_continuous_result_t* mock_adc_ptr = mock_adc_results;

bool analogContinuous(const uint8_t pins[], size_t pins_count, uint32_t conversions_per_pin, uint32_t sampling_freq_hz, void (*userFunc)(void)) {
    for(size_t i=0; i<pins_count && i<16; i++) {
        mock_adc_results[i].pin = pins[i];
        mock_adc_results[i].avg_read_raw = 2048; // mid-scale
    }
    return true;
}
bool analogContinuousRead(adc_continuous_result_t ** buffer, uint32_t timeout_ms) {
    *buffer = mock_adc_results;
    return true;
}
bool analogContinuousStart() { return true; }
bool analogContinuousStop() { return true; }
bool analogContinuousDeinit() { return true; }
void analogContinuousSetAtten(uint8_t attenuation) {}
void analogContinuousSetWidth(uint8_t bits) {}

esp_err_t adc_continuous_new_handle(const adc_continuous_handle_cfg_t *hdl_config, adc_continuous_handle_t *ret_handle) {
    *ret_handle = (void*)0x1234;
    return ESP_OK;
}
esp_err_t adc_continuous_config(adc_continuous_handle_t handle, const adc_continuous_config_t *config) {
    return ESP_OK;
}
esp_err_t adc_continuous_start(adc_continuous_handle_t handle) {
    return ESP_OK;
}
esp_err_t adc_continuous_read(adc_continuous_handle_t handle, uint8_t *buf, uint32_t length_max, uint32_t *out_length, uint32_t timeout_ms) {
    *out_length = sizeof(adc_digi_output_data_t) * 4; // Mock 4 samples
    if (*out_length > length_max) *out_length = length_max;

    adc_digi_output_data_t *p = (adc_digi_output_data_t*)buf;
    for(size_t i=0; i < (*out_length / sizeof(adc_digi_output_data_t)); i++) {
        p[i].type1.data = 2048;
        p[i].type1.channel = 0;
    }
    return ESP_OK;
}
esp_err_t adc_continuous_stop(adc_continuous_handle_t handle) {
    return ESP_OK;
}
esp_err_t adc_continuous_deinit(adc_continuous_handle_t handle) {
    return ESP_OK;
}

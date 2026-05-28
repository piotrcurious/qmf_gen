#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdarg>

#define IRAM_ATTR
#define ESP32

typedef uint8_t byte;
typedef bool boolean;

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x01
#define OUTPUT 0x02
#define PULLUP 0x04
#define INPUT_PULLUP 0x05
#define PULLDOWN 0x08
#define INPUT_PULLDOWN 0x09

void pinMode(uint8_t pin, uint8_t mode);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int value);
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
uint32_t millis();
uint32_t micros();

class MockSerial {
public:
    void begin(unsigned long baud) {}
    template<typename T> void print(T val) { std::cout << val; }
    template<typename T> void println(T val) { std::cout << val << std::endl; }
    void printf(const char* format, ...) {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }
};

extern MockSerial Serial;

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559

// ADC Continuous Mode (Arduino Core 3.0+)
typedef struct {
    uint8_t pin;
    uint8_t channel;
    int avg_read_raw;
    int avg_read_mvolts;
} adc_continuous_result_t;

bool analogContinuous(const uint8_t pins[], size_t pins_count, uint32_t conversions_per_pin, uint32_t sampling_freq_hz, void (*userFunc)(void));
bool analogContinuousRead(adc_continuous_result_t ** buffer, uint32_t timeout_ms);
bool analogContinuousStart();
bool analogContinuousStop();
bool analogContinuousDeinit();
void analogContinuousSetAtten(uint8_t attenuation);
void analogContinuousSetWidth(uint8_t bits);

#include "Arduino_extra.h"

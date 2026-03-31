// Auto-generated QMF analysis filter for ESP32 / Arduino

// Design summary:
// fs = 44100.000000 Hz
// crossover = 1000.000000 Hz
// taps = 61
// transition width = 1840.000000 Hz
// target slope = 12.000000 dB/oct
// achieved slope = -25.395515 dB/oct
// stopband attenuation = 65.440 dB
// passband ripple = 0.061 dB
// group delay = 30.0 samples
// stopband weight = 1e+08
//
// Notes:
// - Analysis-only 2-channel QMF.
// - Inputs are float samples.
// - Outputs are decimated by 2; call process() for every input sample.
// - The implementation exploits linear-phase symmetry to reduce multiplications.
// - For exact PR reconstruction, keep the classic fs/4 QMF geometry.
//
// Example Usage:
//   #include "qmf_esp32_generated.h"
//   QMF2 qmf;
//
//   void loop() {
//       float in_sample = analogRead(A0) / 4095.0f; // Mock input
//       float low, high;
//
//       // process() returns true every 2nd sample (decimation)
//       if (qmf.process(in_sample, low, high)) {
//           // low and high are now ready to be used
//           Serial.printf("L: %f, H: %f\n", low, high);
//       }
//   }

#pragma once

#include <Arduino.h>
#include <math.h>

#if defined(ESP32) || defined(ESP8266)
  #define QMF_FASTRUN IRAM_ATTR
#else
  #define QMF_FASTRUN
#endif

class QMF2 {
public:
    static constexpr int TAPS = 61;
    static constexpr int HALF = TAPS / 2;
    static constexpr int MID = TAPS / 2;
    static constexpr float CUTOFF_HZ = 1000.000000000f;
    static constexpr float SAMPLE_RATE_HZ = 44100.000000000f;
    static constexpr int DELAY_SAMPLES = 30;

    QMF2() {
        reset();
    }

    void reset() {
        for (int i = 0; i < TAPS; ++i) {
            delay_[i] = 0.0f;
        }
        wr_ = 0;
        phase_ = false;
    }

    // Feed one sample. Returns true when low/high outputs are ready.
    bool QMF_FASTRUN process(float x, float &low, float &high) {
        delay_[wr_] = x;
        wr_ = (wr_ + 1 >= TAPS) ? 0 : (wr_ + 1); // Branchless increment is faster than modulo

        phase_ = !phase_;
        if (!phase_) {
            return false;
        }

        const float* h0 = lowpassCoeffs();
        const float* h1 = highpassCoeffs();
        float y0 = 0.0f;
        float y1 = 0.0f;

        // Oldest and newest samples in the current window.
        int left = (wr_ == 0) ? (TAPS - 1) : (wr_ - 1);  // newest = x[n]
        int right = wr_;                                 // oldest = x[n - TAPS + 1]

        for (int k = 0; k < HALF; ++k) {
            const float s = delay_[left] + delay_[right];

            y0 += h0[k] * s;
            y1 += h1[k] * s;

            left = (left == 0) ? (TAPS - 1) : (left - 1);
            right = (right + 1 == TAPS) ? 0 : (right + 1);
        }

        // Center tap.
        const float center = delay_[left];
        y0 += h0[MID] * center;
        y1 += h1[MID] * center;

        low = y0;
        high = y1;
        return true;
    }

    // Keeps the static arrays initialized inside the function scope to avoid ODR violations
    // in Arduino environments when included in multiple translation units.
    static const float* lowpassCoeffs() {
        static const float H0[TAPS] = {
            0.0004669713089f, 0.0005209998926f, 0.0007970406441f, 0.001155883772f, 0.001609491999f, 0.002169431886f, 0.002846307587f, 0.003649217775f,
        0.004585610703f, 0.005660152528f, 0.006874829531f, 0.008228505962f, 0.009716834873f, 0.01133143622f, 0.01306001749f, 0.01488644537f,
        0.01679146662f, 0.01875231788f, 0.02074244618f, 0.02273368835f, 0.02469633333f, 0.02659795061f, 0.02840713039f, 0.03009262867f,
        0.03162333742f, 0.03297217935f, 0.0341129452f, 0.03502431139f, 0.03568822145f, 0.03609210625f, 0.03622752056f, 0.03609210625f,
        0.03568822145f, 0.03502431139f, 0.0341129452f, 0.03297217935f, 0.03162333742f, 0.03009262867f, 0.02840713039f, 0.02659795061f,
        0.02469633333f, 0.02273368835f, 0.02074244618f, 0.01875231788f, 0.01679146662f, 0.01488644537f, 0.01306001749f, 0.01133143622f,
        0.009716834873f, 0.008228505962f, 0.006874829531f, 0.005660152528f, 0.004585610703f, 0.003649217775f, 0.002846307587f, 0.002169431886f,
        0.001609491999f, 0.001155883772f, 0.0007970406441f, 0.0005209998926f, 0.0004669713089f
        };
        return H0;
    }

    static const float* highpassCoeffs() {
        static const float H1[TAPS] = {
            -0.0004669713089f, -0.0005209998926f, -0.0007970406441f, -0.001155883772f, -0.001609491999f, -0.002169431886f, -0.002846307587f, -0.003649217775f,
        -0.004585610703f, -0.005660152528f, -0.006874829531f, -0.008228505962f, -0.009716834873f, -0.01133143622f, -0.01306001749f, -0.01488644537f,
        -0.01679146662f, -0.01875231788f, -0.02074244618f, -0.02273368835f, -0.02469633333f, -0.02659795061f, -0.02840713039f, -0.03009262867f,
        -0.03162333742f, -0.03297217935f, -0.0341129452f, -0.03502431139f, -0.03568822145f, -0.03609210625f, 0.9637724757f, -0.03609210625f,
        -0.03568822145f, -0.03502431139f, -0.0341129452f, -0.03297217935f, -0.03162333742f, -0.03009262867f, -0.02840713039f, -0.02659795061f,
        -0.02469633333f, -0.02273368835f, -0.02074244618f, -0.01875231788f, -0.01679146662f, -0.01488644537f, -0.01306001749f, -0.01133143622f,
        -0.009716834873f, -0.008228505962f, -0.006874829531f, -0.005660152528f, -0.004585610703f, -0.003649217775f, -0.002846307587f, -0.002169431886f,
        -0.001609491999f, -0.001155883772f, -0.0007970406441f, -0.0005209998926f, -0.0004669713089f
        };
        return H1;
    }

private:
    float delay_[TAPS];
    int wr_;
    bool phase_;
};

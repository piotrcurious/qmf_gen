// Auto-generated QMF analysis filter for ESP32 / Arduino

// Design summary:
// fs = 44100.000000 Hz
// crossover = 10000.000000 Hz
// taps = 33
// transition width = 4377.352265 Hz
// target slope = 24.000000 dB/oct
// achieved slope = -70.992952 dB/oct
// stopband attenuation = 60.120 dB
// passband ripple = 0.035 dB
// group delay = 16.0 samples
// stopband weight = 2.05353
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
    static constexpr int TAPS = 33;
    static constexpr int HALF = TAPS / 2;
    static constexpr int MID = TAPS / 2;
    static constexpr float CUTOFF_HZ = 10000.000000000f;
    static constexpr float SAMPLE_RATE_HZ = 44100.000000000f;
    static constexpr int DELAY_SAMPLES = 16;

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

        const float* h = lowpassCoeffs();
        float y0 = 0.0f;
        float y1 = 0.0f;

        // Oldest and newest samples in the current window.
        int left = (wr_ == 0) ? (TAPS - 1) : (wr_ - 1);  // newest = x[n]
        int right = wr_;                                 // oldest = x[n - TAPS + 1]

        for (int k = 0; k < HALF; ++k) {
            const float s = delay_[left] + delay_[right];
            const float c = h[k];

            // h1[k] = h0[k] * (-1)^k. Since TAPS is odd, the highpass is also symmetric.
            const float mod = (k & 1) ? -c : c;

            y0 += c * s;
            y1 += mod * s;

            left = (left == 0) ? (TAPS - 1) : (left - 1);
            right = (right + 1 == TAPS) ? 0 : (right + 1);
        }

        // Center tap.
        const float center = delay_[left];
        const float cMid = h[MID];
        const float modMid = (MID & 1) ? -cMid : cMid;

        y0 += cMid * center;
        y1 += modMid * center;

        low = y0;
        high = y1;
        return true;
    }

    // Keeps the static array initialized inside the function scope to avoid ODR violations
    // in Arduino environments when included in multiple translation units.
    static const float* lowpassCoeffs() {
        static const float H0[TAPS] = {
            -0.0003446312621f, 0.001760531217f, 0.002495361958f, -0.002319549676f, -0.006402628496f, 0.001728238305f, 0.01294507645f, 0.002471606247f,
        -0.02175185271f, -0.01336646918f, 0.03169497475f, 0.03631735966f, -0.04097418487f, -0.08752179891f, 0.04759237543f, 0.3111745715f,
        0.4490020573f, 0.3111745715f, 0.04759237543f, -0.08752179891f, -0.04097418487f, 0.03631735966f, 0.03169497475f, -0.01336646918f,
        -0.02175185271f, 0.002471606247f, 0.01294507645f, 0.001728238305f, -0.006402628496f, -0.002319549676f, 0.002495361958f, 0.001760531217f,
        -0.0003446312621f
        };
        return H0;
    }

private:
    float delay_[TAPS];
    int wr_;
    bool phase_;
};

// Auto-generated QMF analysis filter for ESP32 / Arduino

// Design summary:
// fs = 44100.000000 Hz
// crossover = 1000.000000 Hz
// taps = 115
// transition width = 1840.000000 Hz
// target slope = 48.000000 dB/oct
// achieved slope = -47.809775 dB/oct
// stopband attenuation = 129.401 dB
// passband ripple = 0.116 dB
// group delay = 57.0 samples
// stopband weight = 3.51119e+06
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
    static constexpr int TAPS = 115;
    static constexpr int HALF = TAPS / 2;
    static constexpr int MID = TAPS / 2;
    static constexpr float CUTOFF_HZ = 1000.000000000f;
    static constexpr float SAMPLE_RATE_HZ = 44100.000000000f;
    static constexpr int DELAY_SAMPLES = 57;

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
            4.654389727e-07f, 9.878809806e-07f, 2.008654747e-06f, 3.695225587e-06f, 6.344752364e-06f, 1.034561683e-05f, 1.61940734e-05f, 2.451342334e-05f,
        3.60721242e-05f, 5.180281005e-05f, 7.282085426e-05f, 0.0001004414225f, 0.0001361928153f, 0.0001818305027f, 0.000239342553f, 0.0003109542595f,
        0.0003991242265f, 0.0005065347068f, 0.0006360755651f, 0.0007908179541f, 0.0009739820962f, 0.001188893919f, 0.001438934705f, 0.001727481489f,
        0.002057839883f, 0.002433168702f, 0.002856400795f, 0.003330157837f, 0.003856661497f, 0.004437646829f, 0.005074274261f, 0.005767046008f,
        0.006515729707f, 0.007319288794f, 0.008175826631f, 0.009082541801f, 0.01003569644f, 0.01103061065f, 0.0120616639f, 0.01312232763f,
        0.01420520619f, 0.01530211233f, 0.01640415005f, 0.01750182174f, 0.01858515851f, 0.01964385435f, 0.02066741884f, 0.02164535038f,
        0.02256729081f, 0.02342320979f, 0.02420357242f, 0.0248995088f, 0.02550296485f, 0.0260068588f, 0.02640520781f, 0.02669323236f,
        0.02686745673f, 0.0269257687f, 0.02686745673f, 0.02669323236f, 0.02640520781f, 0.0260068588f, 0.02550296485f, 0.0248995088f,
        0.02420357242f, 0.02342320979f, 0.02256729081f, 0.02164535038f, 0.02066741884f, 0.01964385435f, 0.01858515851f, 0.01750182174f,
        0.01640415005f, 0.01530211233f, 0.01420520619f, 0.01312232763f, 0.0120616639f, 0.01103061065f, 0.01003569644f, 0.009082541801f,
        0.008175826631f, 0.007319288794f, 0.006515729707f, 0.005767046008f, 0.005074274261f, 0.004437646829f, 0.003856661497f, 0.003330157837f,
        0.002856400795f, 0.002433168702f, 0.002057839883f, 0.001727481489f, 0.001438934705f, 0.001188893919f, 0.0009739820962f, 0.0007908179541f,
        0.0006360755651f, 0.0005065347068f, 0.0003991242265f, 0.0003109542595f, 0.000239342553f, 0.0001818305027f, 0.0001361928153f, 0.0001004414225f,
        7.282085426e-05f, 5.180281005e-05f, 3.60721242e-05f, 2.451342334e-05f, 1.61940734e-05f, 1.034561683e-05f, 6.344752364e-06f, 3.695225587e-06f,
        2.008654747e-06f, 9.878809806e-07f, 4.654389727e-07f
        };
        return H0;
    }

    static const float* highpassCoeffs() {
        static const float H1[TAPS] = {
            -4.654389727e-07f, -9.878809806e-07f, -2.008654747e-06f, -3.695225587e-06f, -6.344752364e-06f, -1.034561683e-05f, -1.61940734e-05f, -2.451342334e-05f,
        -3.60721242e-05f, -5.180281005e-05f, -7.282085426e-05f, -0.0001004414225f, -0.0001361928153f, -0.0001818305027f, -0.000239342553f, -0.0003109542595f,
        -0.0003991242265f, -0.0005065347068f, -0.0006360755651f, -0.0007908179541f, -0.0009739820962f, -0.001188893919f, -0.001438934705f, -0.001727481489f,
        -0.002057839883f, -0.002433168702f, -0.002856400795f, -0.003330157837f, -0.003856661497f, -0.004437646829f, -0.005074274261f, -0.005767046008f,
        -0.006515729707f, -0.007319288794f, -0.008175826631f, -0.009082541801f, -0.01003569644f, -0.01103061065f, -0.0120616639f, -0.01312232763f,
        -0.01420520619f, -0.01530211233f, -0.01640415005f, -0.01750182174f, -0.01858515851f, -0.01964385435f, -0.02066741884f, -0.02164535038f,
        -0.02256729081f, -0.02342320979f, -0.02420357242f, -0.0248995088f, -0.02550296485f, -0.0260068588f, -0.02640520781f, -0.02669323236f,
        -0.02686745673f, 0.9730742574f, -0.02686745673f, -0.02669323236f, -0.02640520781f, -0.0260068588f, -0.02550296485f, -0.0248995088f,
        -0.02420357242f, -0.02342320979f, -0.02256729081f, -0.02164535038f, -0.02066741884f, -0.01964385435f, -0.01858515851f, -0.01750182174f,
        -0.01640415005f, -0.01530211233f, -0.01420520619f, -0.01312232763f, -0.0120616639f, -0.01103061065f, -0.01003569644f, -0.009082541801f,
        -0.008175826631f, -0.007319288794f, -0.006515729707f, -0.005767046008f, -0.005074274261f, -0.004437646829f, -0.003856661497f, -0.003330157837f,
        -0.002856400795f, -0.002433168702f, -0.002057839883f, -0.001727481489f, -0.001438934705f, -0.001188893919f, -0.0009739820962f, -0.0007908179541f,
        -0.0006360755651f, -0.0005065347068f, -0.0003991242265f, -0.0003109542595f, -0.000239342553f, -0.0001818305027f, -0.0001361928153f, -0.0001004414225f,
        -7.282085426e-05f, -5.180281005e-05f, -3.60721242e-05f, -2.451342334e-05f, -1.61940734e-05f, -1.034561683e-05f, -6.344752364e-06f, -3.695225587e-06f,
        -2.008654747e-06f, -9.878809806e-07f, -4.654389727e-07f
        };
        return H1;
    }

private:
    float delay_[TAPS];
    int wr_;
    bool phase_;
};

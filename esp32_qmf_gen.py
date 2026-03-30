#!/usr/bin/env python3 """qmf_arduino_esp32_generator.py

Design a 2-channel QMF (quadrature mirror filter) prototype for an ESP32 / Arduino project and emit ready-to-paste C++ code.

Practical note: For FIR QMFs, "slope" is not an independent knob. The real design knobs are transition width, stopband attenuation, tap count, and coefficient precision. This script accepts a target local slope (dB/oct) as a soft goal, but it actually searches over odd tap counts and designs a halfband prototype that meets stopband/ripple constraints while trying to match the requested slope.

Dependencies: pip install numpy scipy

Typical use: python qmf_arduino_esp32_generator.py 
--fs 48000 
--crossover-hz 12000 
--target-slope-db-per-oct 48 
--stopband-db 80 
--max-taps 401 
--output qmf_esp32_generated.cpp

The emitted code is analysis-only (low/high subband outputs). A synthesis stage can be added later if needed. """

from future import annotations

import argparse import dataclasses import json import math import textwrap from pathlib import Path from typing import Optional, Tuple

import numpy as np from scipy import signal

@dataclasses.dataclass class QMFDesignResult: fs: float crossover_hz: float transition_hz: float target_slope_db_per_oct: float achieved_slope_db_per_oct: float stopband_db: float passband_ripple_db: float taps: int delay_samples: float h0: np.ndarray h1: np.ndarray notes: str = ""

def db(x: np.ndarray, eps: float = 1e-20) -> np.ndarray: return 20.0 * np.log10(np.maximum(np.abs(x), eps))

def _freq_response(h: np.ndarray, fs: float, nfft: int = 32768) -> Tuple[np.ndarray, np.ndarray]: w, H = signal.freqz(h, worN=nfft, fs=fs) return w, H

def _halfband_bands(fs: float, crossover_hz: float, transition_hz: float) -> Tuple[list[float], list[float]]: nyq = fs / 2.0 if not (0.0 < crossover_hz < nyq): raise ValueError("crossover_hz must lie strictly between 0 and fs/2") if transition_hz <= 0: raise ValueError("transition_hz must be > 0") pb = max(1e-9, crossover_hz - transition_hz / 2.0) sb = min(nyq - 1e-9, crossover_hz + transition_hz / 2.0) if not (0.0 < pb < sb < nyq): raise ValueError("transition_hz too large for the chosen crossover_hz / fs") return [0.0, pb, sb, nyq], [1.0, 0.0]

def design_halfband_prototype( fs: float, crossover_hz: float, taps: int, transition_hz: float, stopband_weight: float, ) -> np.ndarray: if taps % 2 == 0: raise ValueError("Use an odd number of taps for linear-phase QMF prototypes") bands, desired = _halfband_bands(fs, crossover_hz, transition_hz) # Parks-McClellan / equiripple halfband-like prototype. h = signal.remez( taps, bands, desired, weight=[1.0, stopband_weight], fs=fs, maxiter=200, ) # Normalize DC gain to 1.0 for the low-pass channel. h /= np.sum(h) return h

def qmf_highpass_from_lowpass(h0: np.ndarray) -> np.ndarray: n = np.arange(len(h0)) return h0 * ((-1.0) ** n)

def estimate_local_slope_db_per_oct(h: np.ndarray, fs: float, crossover_hz: float) -> float: """Estimate slope near the crossover by fitting dB vs log2(f) around crossover.

This is a heuristic metric, not a formal filter spec.
"""
f, H = _freq_response(h, fs)
mag_db = db(H)

# Focus on a band around the crossover, but keep it away from DC and Nyquist.
lo = max(1.0, crossover_hz / 2.0)
hi = min(fs / 2.0 - 1.0, crossover_hz * 2.0)
mask = (f >= lo) & (f <= hi)
if mask.sum() < 10:
    return float("nan")

x = np.log2(f[mask] / crossover_hz)
y = mag_db[mask]

# Weight points nearer the crossover more heavily.
w = 1.0 / np.maximum(np.abs(x), 0.05)
# Linear least squares fit y = a*x + b
A = np.column_stack([x, np.ones_like(x)])
Aw = A * w[:, None]
yw = y * w
coef, *_ = np.linalg.lstsq(Aw, yw, rcond=None)
slope = float(coef[0])
return slope

def analyze_prototype(h0: np.ndarray, fs: float, crossover_hz: float) -> Tuple[float, float, float]: f, H = _freq_response(h0, fs) mag_db = db(H)

nyq = fs / 2.0
pb_end = min(crossover_hz * 0.9, nyq)
sb_start = max(crossover_hz * 1.1, 1.0)

passband = f <= pb_end
stopband = f >= sb_start

if passband.any():
    pb_ripple = float(np.max(mag_db[passband]) - np.min(mag_db[passband]))
else:
    pb_ripple = float("nan")

if stopband.any():
    stopband_db = float(-np.max(mag_db[stopband]))
else:
    stopband_db = float("nan")

slope = estimate_local_slope_db_per_oct(h0, fs, crossover_hz)
return slope, stopband_db, pb_ripple

def search_best_design( fs: float, crossover_hz: float, target_slope_db_per_oct: float, stopband_db: float, max_taps: int, min_taps: int = 17, ) -> QMFDesignResult: if min_taps < 5: min_taps = 5 if min_taps % 2 == 0: min_taps += 1 if max_taps % 2 == 0: max_taps -= 1 if max_taps < min_taps: raise ValueError("max_taps must be >= min_taps")

# Search over tap counts and transition widths.
# Start with a moderate transition and let the search widen/narrow as needed.
nyq = fs / 2.0
base_transition = max(fs * 0.005, fs / 2048.0)
candidate_transitions = np.unique(
    np.clip(
        np.geomspace(base_transition / 4.0, nyq / 2.5, num=18),
        fs / 65536.0,
        nyq * 0.49,
    )
)

best: Optional[QMFDesignResult] = None
best_score = float("inf")

for taps in range(min_taps, max_taps + 1, 2):
    for transition_hz in candidate_transitions:
        # Roughly relate attenuation target to stopband weight.
        stopband_weight = max(1.0, 10.0 ** (stopband_db / 20.0) / 5.0)
        try:
            h0 = design_halfband_prototype(
                fs=fs,
                crossover_hz=crossover_hz,
                taps=taps,
                transition_hz=float(transition_hz),
                stopband_weight=stopband_weight,
            )
        except Exception:
            continue

        slope, achieved_stopband_db, pb_ripple_db = analyze_prototype(h0, fs, crossover_hz)
        if not np.isfinite(slope) or not np.isfinite(achieved_stopband_db):
            continue

        h1 = qmf_highpass_from_lowpass(h0)
        delay_samples = (taps - 1) / 2.0

        # Scoring: satisfy attenuation first, then slope, then keep taps low.
        slope_err = abs(abs(slope) - abs(target_slope_db_per_oct))
        attn_err = max(0.0, stopband_db - achieved_stopband_db)
        ripple_penalty = max(0.0, pb_ripple_db - 0.5)
        score = (
            8.0 * attn_err
            + 2.0 * slope_err
            + 0.2 * ripple_penalty
            + 0.002 * taps
        )

        feasible = achieved_stopband_db >= stopband_db * 0.90
        if feasible and score < best_score:
            best_score = score
            best = QMFDesignResult(
                fs=fs,
                crossover_hz=crossover_hz,
                transition_hz=float(transition_hz),
                target_slope_db_per_oct=target_slope_db_per_oct,
                achieved_slope_db_per_oct=slope,
                stopband_db=achieved_stopband_db,
                passband_ripple_db=pb_ripple_db,
                taps=taps,
                delay_samples=delay_samples,
                h0=h0,
                h1=h1,
                notes=(
                    "Designed as a halfband-like equiripple analysis QMF prototype. "
                    "Slope is an estimated local metric near the crossover."
                ),
            )

if best is None:
    raise RuntimeError(
        "No design found. Try increasing max_taps or relaxing stopband_db / target_slope."
    )
return best

def _format_float_array(x: np.ndarray, per_line: int = 6, precision: int = 9) -> str: vals = [f"{float(v):.{precision}g}f" for v in x] lines = [] for i in range(0, len(vals), per_line): lines.append(", ".join(vals[i : i + per_line])) return ",\n    ".join(lines)

def generate_arduino_cpp(result: QMFDesignResult, class_name: str = "QMF2") -> str: taps = result.taps h0 = result.h0.astype(np.float32) h1 = result.h1.astype(np.float32)

h0_txt = _format_float_array(h0)
h1_txt = _format_float_array(h1)

code = f'''// Auto-generated QMF analysis filter for ESP32 / Arduino

// // Design summary: //   fs                     = {result.fs:.6f} Hz //   crossover              = {result.crossover_hz:.6f} Hz //   taps                   = {taps} //   transition width       = {result.transition_hz:.6f} Hz //   target slope           = {result.target_slope_db_per_oct:.6f} dB/oct //   achieved slope         = {result.achieved_slope_db_per_oct:.6f} dB/oct //   stopband attenuation   = {result.stopband_db:.3f} dB //   passband ripple        = {result.passband_ripple_db:.3f} dB //   group delay            = {result.delay_samples:.1f} samples // // Notes: //   - Analysis-only 2-channel QMF. //   - Inputs are float samples. //   - Outputs are decimated by 2; call process() for every input sample. //   - For a full codec, add a synthesis stage using the same prototype family.

#include <Arduino.h> #include <math.h>

class {class_name} {{ public: static constexpr int TAPS = {taps}; static constexpr float CUTOFF_HZ = {result.crossover_hz:.9f}f; static constexpr float SAMPLE_RATE_HZ = {result.fs:.9f}f; static constexpr int DELAY_SAMPLES = {int(round(result.delay_samples))};

{class_name}() {{ reset(); }}

void reset() {{
    for (int i = 0; i < TAPS; ++i) {{
        delay_[i] = 0.0f;
    }}
    wr_ = 0;
    phase_ = false;
}}

// Feed one input sample. Returns true when a new low/high output pair is ready.
bool process(float x, float &low, float &high) {{
    delay_[wr_] = x;
    wr_ = (wr_ + 1) % TAPS;
    phase_ = !phase_;
    if (!phase_) {{
        return false; // output every second input sample
    }}

    float y0 = 0.0f;
    float y1 = 0.0f;
    int idx = wr_;
    for (int k = 0; k < TAPS; ++k) {{
        idx = (idx == 0) ? (TAPS - 1) : (idx - 1);
        const float s = delay_[idx];
        y0 += H0[k] * s;
        y1 += H1[k] * s;
    }}
    low = y0;
    high = y1;
    return true;
}}

static const float* lowpassCoeffs() {{ return H0; }}
static const float* highpassCoeffs() {{ return H1; }}

private: float delay_[TAPS]; int wr_; bool phase_;

static constexpr float H0[TAPS] = {{
    {h0_txt}
}};

static constexpr float H1[TAPS] = {{
    {h1_txt}
}};

}}; ''' return textwrap.dedent(code)

def main() -> None: ap = argparse.ArgumentParser(description="Design and export an ESP32 Arduino QMF.") ap.add_argument("--fs", type=float, required=True, help="Sample rate in Hz") ap.add_argument("--crossover-hz", type=float, required=True, help="QMF crossover / halfband center frequency") ap.add_argument("--target-slope-db-per-oct", type=float, default=48.0, help="Target local slope in dB/oct (soft goal)") ap.add_argument("--stopband-db", type=float, default=80.0, help="Minimum stopband attenuation in dB") ap.add_argument("--min-taps", type=int, default=17, help="Minimum odd tap count to search") ap.add_argument("--max-taps", type=int, default=401, help="Maximum odd tap count to search") ap.add_argument("--output", type=Path, required=True, help="Output .cpp file") ap.add_argument("--json-metadata", type=Path, default=None, help="Optional metadata JSON output") ap.add_argument("--class-name", type=str, default="QMF2", help="C++ class name in emitted code") args = ap.parse_args()

result = search_best_design(
    fs=args.fs,
    crossover_hz=args.crossover_hz,
    target_slope_db_per_oct=args.target_slope_db_per_oct,
    stopband_db=args.stopband_db,
    max_taps=args.max_taps,
    min_taps=args.min_taps,
)

cpp = generate_arduino_cpp(result, class_name=args.class_name)
args.output.write_text(cpp, encoding="utf-8")

if args.json_metadata is not None:
    meta = dataclasses.asdict(result)
    meta["h0"] = result.h0.tolist()
    meta["h1"] = result.h1.tolist()
    args.json_metadata.write_text(json.dumps(meta, indent=2), encoding="utf-8")

print(f"Wrote: {args.output}")
print(f"taps={result.taps}, transition={result.transition_hz:.3f} Hz, stopband={result.stopband_db:.2f} dB, slope={result.achieved_slope_db_per_oct:.2f} dB/oct")

if name == "main": main()

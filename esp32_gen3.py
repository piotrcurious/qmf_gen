#!/usr/bin/env python3
"""qmf_arduino_esp32_generator.py

Design a 2-channel analysis QMF / cosine-modulated filter-bank prototype
for an ESP32 / Arduino project and emit ready-to-paste C++ code.

Dependencies:
    pip install numpy scipy

Example:
    python qmf_arduino_esp32_generator.py \
        --fs 48000 \
        --crossover-hz 12000 \
        --target-slope-db-per-oct 48 \
        --stopband-db 80 \
        --max-taps 401 \
        --output qmf_esp32_generated.h
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
from pathlib import Path
from typing import Iterable, Optional, Tuple

import numpy as np
from scipy import signal


@dataclasses.dataclass
class QMFDesignResult:
    fs: float
    crossover_hz: float
    transition_hz: float
    target_slope_db_per_oct: float
    achieved_slope_db_per_oct: float
    stopband_db: float
    passband_ripple_db: float
    taps: int
    delay_samples: float
    h0: np.ndarray
    h1: np.ndarray
    weight: float
    notes: str = ""


def db20(x: np.ndarray, eps: float = 1e-20) -> np.ndarray:
    return 20.0 * np.log10(np.maximum(np.abs(x), eps))


def is_odd(n: int) -> bool:
    return (n & 1) == 1


def normalize_odd(n: int) -> int:
    return n if is_odd(n) else n + 1


def _freq_response(h: np.ndarray, fs: float, nfft: int = 32768) -> Tuple[np.ndarray, np.ndarray]:
    f, H = signal.freqz(h, worN=nfft, fs=fs)
    return f, H


def _band_edges(fs: float, crossover_hz: float, transition_hz: float) -> Tuple[float, float, float, float]:
    nyq = fs / 2.0
    if not (0.0 < crossover_hz < nyq):
        raise ValueError("crossover_hz must be between 0 and fs/2")
    if transition_hz <= 0.0:
        raise ValueError("transition_hz must be positive")

    pb = crossover_hz - transition_hz / 2.0
    sb = crossover_hz + transition_hz / 2.0
    if pb <= 0.0 or sb >= nyq:
        raise ValueError("transition_hz is too large for the selected crossover_hz")
    return 0.0, pb, sb, nyq


def _design_with_weight(
    fs: float, crossover_hz: float, taps: int, transition_hz: float, stopband_weight: float
) -> np.ndarray:
    _, pb, sb, nyq = _band_edges(fs, crossover_hz, transition_hz)
    h = signal.remez(
        taps,
        [0.0, pb, sb, nyq],
        [1.0, 0.0],
        weight=[1.0, stopband_weight],
        fs=fs,
        maxiter=200,
    )
    # Normalize DC gain to unity.
    h = h / np.sum(h)
    return h


def qmf_highpass_from_lowpass(h0: np.ndarray) -> np.ndarray:
    n = np.arange(h0.size, dtype=np.int64)
    return h0 * ((-1.0) ** n)


def estimate_local_slope_db_per_oct(h: np.ndarray, fs: float, crossover_hz: float) -> float:
    """Heuristic slope estimate around the crossover frequency."""
    f, H = _freq_response(h, fs)
    mag_db = db20(H)
    lo = max(1.0, crossover_hz / 2.0)
    hi = min(fs / 2.0 - 1.0, crossover_hz * 2.0)
    mask = (f >= lo) & (f <= hi)
    
    if int(mask.sum()) < 24:
        return float("nan")
        
    x = np.log2(f[mask] / crossover_hz)
    y = mag_db[mask]
    
    # Weight points nearest the crossover more heavily.
    w = 1.0 / np.maximum(np.abs(x), 0.05)
    A = np.column_stack([x, np.ones_like(x)])
    Aw = A * w[:, None]
    yw = y * w
    coef, *_ = np.linalg.lstsq(Aw, yw, rcond=None)
    return float(coef[0])


def analyze_prototype(h0: np.ndarray, fs: float, crossover_hz: float) -> Tuple[float, float, float]:
    f, H = _freq_response(h0, fs)
    mag_db = db20(H)
    nyq = fs / 2.0

    pb_end = min(crossover_hz * 0.90, nyq)
    sb_start = max(crossover_hz * 1.10, 1.0)
    
    pb_mask = f <= pb_end
    sb_mask = f >= sb_start
    
    pb_ripple_db = float(np.max(mag_db[pb_mask]) - np.min(mag_db[pb_mask])) if np.any(pb_mask) else float("nan")
    stopband_db = float(-np.max(mag_db[sb_mask])) if np.any(sb_mask) else float("nan")
    slope_db_per_oct = estimate_local_slope_db_per_oct(h0, fs, crossover_hz)
    
    return slope_db_per_oct, stopband_db, pb_ripple_db


def _candidate_transitions(fs: float, crossover_hz: float, count: int = 18) -> np.ndarray:
    nyq = fs / 2.0
    max_transition = 2.0 * min(crossover_hz, nyq - crossover_hz)
    max_transition *= 0.92
    min_transition = max(fs / 65536.0, fs / 8192.0)
    max_transition = max(max_transition, min_transition * 1.5)
    return np.unique(np.geomspace(min_transition, max_transition, num=count))


def _design_for_target(
    fs: float, crossover_hz: float, taps: int, transition_hz: float, target_stopband_db: float, iterations: int = 8
) -> Optional[Tuple[np.ndarray, float]]:
    """Binary-search a stopband weight that reaches the requested attenuation."""
    lo_w = 1.0
    hi_w = 1e8
    best_h: Optional[np.ndarray] = None
    best_w: Optional[float] = None
    best_stopband = -float("inf")
    
    # A few warm-start points help when the response is non-monotone.
    probe_weights = [1.0, 10.0, 100.0, 1e3, 1e4]
    for w in probe_weights:
        try:
            h = _design_with_weight(fs, crossover_hz, taps, transition_hz, w)
        except Exception:
            continue
            
        slope, stopband_db, _ = analyze_prototype(h, fs, crossover_hz)
        if np.isfinite(stopband_db) and stopband_db > best_stopband:
            best_h = h
            best_w = w
            best_stopband = stopband_db
            
    if best_h is None:
        return None
        
    # Bracket the target if possible.
    try:
        h_lo = _design_with_weight(fs, crossover_hz, taps, transition_hz, lo_w)
        _, sb_lo, _ = analyze_prototype(h_lo, fs, crossover_hz)
    except Exception:
        sb_lo = -float("inf")
        
    try:
        h_hi = _design_with_weight(fs, crossover_hz, taps, transition_hz, hi_w)
        _, sb_hi, _ = analyze_prototype(h_hi, fs, crossover_hz)
    except Exception:
        sb_hi = -float("inf")
        
    # If the target is bracketed, use binary search; otherwise keep probing powers of ten.
    if np.isfinite(sb_lo) and np.isfinite(sb_hi) and sb_lo <= target_stopband_db <= sb_hi:
        left, right = lo_w, hi_w
        for _ in range(iterations):
            mid = math.sqrt(left * right)
            try:
                h = _design_with_weight(fs, crossover_hz, taps, transition_hz, mid)
            except Exception:
                left = mid
                continue
                
            _, sb, _ = analyze_prototype(h, fs, crossover_hz)
            if np.isfinite(sb) and sb >= target_stopband_db:
                best_h, best_w, best_stopband = h, mid, sb
                right = mid
            else:
                left = mid
    else:
        for w in np.geomspace(1.0, 1e8, num=12):
            try:
                h = _design_with_weight(fs, crossover_hz, taps, transition_hz, float(w))
            except Exception:
                continue
            _, sb, _ = analyze_prototype(h, fs, crossover_hz)
            if np.isfinite(sb) and sb > best_stopband:
                best_h, best_w, best_stopband = h, float(w), sb
                
    if best_h is not None and best_w is not None:
        return best_h, float(best_w)
    return None


def search_best_design(
    fs: float, crossover_hz: float, target_slope_db_per_oct: float, stopband_db: float, max_taps: int, min_taps: int = 17
) -> QMFDesignResult:
    min_taps = normalize_odd(min_taps)
    max_taps = normalize_odd(max_taps)
    if max_taps < min_taps:
        raise ValueError("max_taps must be >= min_taps")

    best: Optional[QMFDesignResult] = None
    best_score = float("inf")
    
    print(f"Searching for optimal filter (up to {max_taps} taps)...")
    
    for taps in range(min_taps, max_taps + 1, 2):
        for transition_hz in _candidate_transitions(fs, crossover_hz):
            pair = _design_for_target(fs, crossover_hz, taps, float(transition_hz), stopband_db)
            if pair is None:
                continue
                
            h0, weight = pair
            slope, achieved_stopband_db, pb_ripple_db = analyze_prototype(h0, fs, crossover_hz)
            
            if not np.isfinite(slope) or not np.isfinite(achieved_stopband_db):
                continue
                
            h1 = qmf_highpass_from_lowpass(h0)
            delay_samples = (taps - 1) / 2.0
            slope_err = abs(abs(slope) - abs(target_slope_db_per_oct))
            stopband_err = max(0.0, stopband_db - achieved_stopband_db)
            ripple_penalty = max(0.0, pb_ripple_db - 0.5)
            
            # Prefer satisfying attenuation first, then matching slope, then fewer taps.
            score = (12.0 * stopband_err + 2.5 * slope_err + 0.25 * ripple_penalty + 0.0025 * taps)
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
                    weight=weight,
                    notes=(
                        "Designed as a linear-phase analysis prototype. "
                        "For exact alias-canceling QMF reconstruction, use the standard fs/4 geometry."
                    ),
                )

    if best is None:
        raise RuntimeError(
            "No design found. Try increasing --max-taps or relaxing --stopband-db / --target-slope-db-per-oct."
        )
    return best


def _format_float_array(values: Iterable[float], per_line: int = 8, precision: int = 10) -> str:
    items = []
    for v in values:
        items.append(f"{float(v):.{precision}g}f")
    lines = []
    for i in range(0, len(items), per_line):
        lines.append(", ".join(items[i : i + per_line]))
    return ",\n        ".join(lines)


def generate_arduino_cpp(result: QMFDesignResult, class_name: str = "QMF2") -> str:
    taps = result.taps
    h0_txt = _format_float_array(result.h0.astype(np.float32))

    return f'''// Auto-generated QMF analysis filter for ESP32 / Arduino 

// Design summary:
// fs = {result.fs:.6f} Hz
// crossover = {result.crossover_hz:.6f} Hz
// taps = {taps}
// transition width = {result.transition_hz:.6f} Hz
// target slope = {result.target_slope_db_per_oct:.6f} dB/oct
// achieved slope = {result.achieved_slope_db_per_oct:.6f} dB/oct
// stopband attenuation = {result.stopband_db:.3f} dB
// passband ripple = {result.passband_ripple_db:.3f} dB
// group delay = {result.delay_samples:.1f} samples
// stopband weight = {result.weight:.6g}
//
// Notes:
// - Analysis-only 2-channel QMF.
// - Inputs are float samples.
// - Outputs are decimated by 2; call process() for every input sample.
// - The implementation exploits linear-phase symmetry to reduce multiplications.
// - For exact PR reconstruction, keep the classic fs/4 QMF geometry.

#pragma once

#include <Arduino.h>
#include <math.h>

class {class_name} {{
public:
    static constexpr int TAPS = {taps};
    static constexpr int HALF = TAPS / 2;
    static constexpr int MID = TAPS / 2;
    static constexpr float CUTOFF_HZ = {result.crossover_hz:.9f}f;
    static constexpr float SAMPLE_RATE_HZ = {result.fs:.9f}f;
    static constexpr int DELAY_SAMPLES = {int(round(result.delay_samples))};

    {class_name}() {{
        reset();
    }}

    void reset() {{
        for (int i = 0; i < TAPS; ++i) {{
            delay_[i] = 0.0f;
        }}
        wr_ = 0;
        phase_ = false;
    }}

    // Feed one sample. Returns true when low/high outputs are ready.
    bool process(float x, float &low, float &high) {{
        delay_[wr_] = x;
        wr_ = (wr_ + 1 >= TAPS) ? 0 : (wr_ + 1); // Branchless increment is faster than modulo on ESP32
        
        phase_ = !phase_;
        if (!phase_) {{
            return false;
        }}

        const float* h = lowpassCoeffs();
        float y0 = 0.0f;
        float y1 = 0.0f;

        // Oldest and newest samples in the current window.
        int left = (wr_ == 0) ? (TAPS - 1) : (wr_ - 1);  // newest = x[n]
        int right = wr_;                                 // oldest = x[n - TAPS + 1]

        for (int k = 0; k < HALF; ++k) {{
            const float s = delay_[left] + delay_[right];
            const float c = h[k];
            const float mod = (k & 1) ? -c : c;
            y0 += c * s;
            y1 += mod * s;

            left = (left == 0) ? (TAPS - 1) : (left - 1);
            right = (right + 1 == TAPS) ? 0 : (right + 1);
        }}

        // Center tap.
        const float center = delay_[left];
        const float cMid = h[MID];
        const float modMid = (MID & 1) ? -cMid : cMid;
        y0 += cMid * center;
        y1 += modMid * center;

        low = y0;
        high = y1;
        return true;
    }}

    // Keeps the static array initialized inside the function scope to avoid ODR violations
    // in Arduino environments when included in multiple translation units.
    static const float* lowpassCoeffs() {{
        static const float H0[TAPS] = {{
            {h0_txt}
        }};
        return H0;
    }}

private:
    float delay_[TAPS];
    int wr_;
    bool phase_;
}};
'''


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Design and export an ESP32 Arduino QMF.")
    ap.add_argument("--fs", type=float, required=True, help="Sample rate in Hz")
    ap.add_argument(
        "--crossover-hz", type=float, default=None, help="QMF crossover / prototype corner in Hz. Default: fs/4",
    )
    ap.add_argument(
        "--target-slope-db-per-oct", type=float, default=48.0, help="Soft slope goal in dB/oct",
    )
    ap.add_argument(
        "--stopband-db", type=float, default=80.0, help="Minimum acceptable stopband attenuation in dB",
    )
    ap.add_argument("--min-taps", type=int, default=17, help="Minimum odd tap count to search")
    ap.add_argument("--max-taps", type=int, default=401, help="Maximum odd tap count to search")
    ap.add_argument("--output", type=Path, required=True, help="Output .h file (Header-only usage recommended)")
    ap.add_argument("--json-metadata", type=Path, default=None, help="Optional metadata JSON output")
    ap.add_argument("--class-name", type=str, default="QMF2", help="C++ class name in emitted code")
    return ap.parse_args()


def main() -> None:
    args = _parse_args()
    crossover_hz = float(args.crossover_hz) if args.crossover_hz is not None else float(args.fs) / 4.0

    if crossover_hz <= 0.0 or crossover_hz >= args.fs / 2.0:
        sys.exit("Error: --crossover-hz must lie between 0 and fs/2")

    result = search_best_design(
        fs=float(args.fs),
        crossover_hz=crossover_hz,
        target_slope_db_per_oct=float(args.target_slope_db_per_oct),
        stopband_db=float(args.stopband_db),
        max_taps=int(args.max_taps),
        min_taps=int(args.min_taps),
    )
    
    cpp = generate_arduino_cpp(result, class_name=args.class_name)
    args.output.write_text(cpp, encoding="utf-8")
    
    if args.json_metadata is not None:
        meta = dataclasses.asdict(result)
        meta["h0"] = result.h0.tolist()
        meta["h1"] = result.h1.tolist()
        args.json_metadata.write_text(json.dumps(meta, indent=2), encoding="utf-8")
        
    print(f"Wrote: {args.output}")
    print(
        f"taps={result.taps}, transition={result.transition_hz:.3f} Hz, "
        f"stopband={result.stopband_db:.2f} dB, slope={result.achieved_slope_db_per_oct:.2f} dB/oct"
    )


if __name__ == "__main__":
    main()

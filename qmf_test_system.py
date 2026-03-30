import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import json
from mock_qmf import MockQMF2

class QMFAnalyzer:
    def __init__(self, fs):
        self.fs = fs

    def generate_sine_sweep(self, duration, f_start, f_end):
        t = np.linspace(0, duration, int(self.fs * duration), endpoint=False)
        return t, signal.chirp(t, f0=f_start, f1=f_end, t1=duration, method='logarithmic')

    def analyze_filter(self, mock_qmf, input_signal):
        outputs_low = []
        outputs_high = []
        for x in input_signal:
            res = mock_qmf.process(x)
            if res:
                low, high = res
                outputs_low.append(low)
                outputs_high.append(high)
        return np.array(outputs_low), np.array(outputs_high)

    def plot_theoretical_response(self, h0, taps, target_slope, filename):
        w, h = signal.freqz(h0, worN=8192, fs=self.fs)

        # Highpass is h0[n] * (-1)^n
        h1 = h0 * ((-1.0) ** np.arange(len(h0)))
        w1, hh1 = signal.freqz(h1, worN=8192, fs=self.fs)

        plt.figure(figsize=(10, 6))
        plt.plot(w, 20 * np.log10(np.abs(h)), label='Low-pass (H0)')
        plt.plot(w1, 20 * np.log10(np.abs(hh1)), label='High-pass (H1)')

        plt.title(f"Theoretical Frequency Response ({target_slope} dB/oct, {taps} taps)")
        plt.xlabel('Frequency [Hz]')
        plt.ylabel('Magnitude [dB]')
        plt.grid(True, which="both", ls="-")
        plt.ylim([-100, 5])
        plt.legend()
        plt.savefig(filename)
        plt.close()

    def plot_spectrogram(self, signal_data, fs, title, filename):
        f, t, Sxx = signal.spectrogram(signal_data, fs, nperseg=256)
        plt.figure(figsize=(10, 6))
        plt.pcolormesh(t, f, 10 * np.log10(Sxx + 1e-12), shading='gouraud')
        plt.title(title)
        plt.ylabel('Frequency [Hz]')
        plt.xlabel('Time [sec]')
        plt.colorbar(label='Intensity [dB]')
        plt.savefig(filename)
        plt.close()

def run_demonstration(fs, crossover_hz, target_slope, stopband_db, min_taps, max_taps, name):
    import subprocess
    import sys

    json_file = f"{name}.json"
    header_file = f"{name}.h"

    # Generate the filter
    cmd = [
        sys.executable, "esp32_qmf_gen4.py",
        "--fs", str(fs),
        "--crossover-hz", str(crossover_hz),
        "--target-slope-db-per-oct", str(target_slope),
        "--stopband-db", str(stopband_db),
        "--min-taps", str(min_taps),
        "--max-taps", str(max_taps),
        "--output", header_file,
        "--json-metadata", json_file
    ]
    print(f"Generating filter for {target_slope} dB/oct ({max_taps} max taps)...")
    subprocess.run(cmd, check=True)

    # Load metadata
    with open(json_file, 'r') as f:
        meta = json.load(f)

    h0 = np.array(meta['h0'])
    taps = meta['taps']

    analyzer = QMFAnalyzer(fs)

    # Plot theoretical response
    analyzer.plot_theoretical_response(h0, taps, target_slope, f"{name}_response.png")

    # Generate test signal: Sine sweep
    sweep_duration = 2.0
    t_sweep, sweep = analyzer.generate_sine_sweep(sweep_duration, 20, fs/2)

    # Analyze with Mock system
    mock_qmf = MockQMF2(taps, h0)
    low, high = analyzer.analyze_filter(mock_qmf, sweep)

    # Plotting spectrograms
    analyzer.plot_spectrogram(sweep, fs, f"Input Sweep Spectrogram", f"{name}_spec_in.png")
    analyzer.plot_spectrogram(low, fs/2, f"Low Channel Spectrogram ({target_slope} dB/oct)", f"{name}_spec_low.png")
    analyzer.plot_spectrogram(high, fs/2, f"High Channel Spectrogram ({target_slope} dB/oct)", f"{name}_spec_high.png")

    print(f"Finished demonstration for {name}\n")

if __name__ == "__main__":
    FS = 44100
    CROSSOVER = 10000

    # 1. 12 dB/oct
    run_demonstration(FS, CROSSOVER, 12, 60, 17, 31, "qmf_12")

    # 2. 24 dB/oct
    run_demonstration(FS, CROSSOVER, 24, 60, 33, 63, "qmf_24")

    # 3. 48 dB/oct with restricted tap search
    run_demonstration(FS, CROSSOVER, 48, 60, 17, 31, "qmf_48_limited")

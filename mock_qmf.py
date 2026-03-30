import numpy as np

class MockQMF2:
    def __init__(self, taps, h0):
        self.taps = taps
        self.h0 = np.array(h0)
        self.mid = taps // 2
        self.half = taps // 2
        self.reset()

    def reset(self):
        self.delay = np.zeros(self.taps, dtype=np.float32)
        self.wr = 0
        self.phase = False

    def process(self, x):
        self.delay[self.wr] = x
        self.wr = (self.wr + 1) if (self.wr + 1 < self.taps) else 0

        self.phase = not self.phase
        if not self.phase:
            return None # Equivalent to returning false in C++

        h = self.h0
        y0 = 0.0
        y1 = 0.0

        # Oldest and newest samples in the current window.
        left = (self.taps - 1) if (self.wr == 0) else (self.wr - 1)
        right = self.wr

        for k in range(self.half):
            s = self.delay[left] + self.delay[right]
            c = h[k]

            mod = -c if (k & 1) else c

            y0 += c * s
            y1 += mod * s

            left = (self.taps - 1) if (left == 0) else (left - 1)
            right = 0 if (right + 1 == self.taps) else (right + 1)

        # Center tap.
        center = self.delay[left]
        cMid = h[self.mid]
        modMid = -cMid if (self.mid & 1) else cMid

        y0 += cMid * center
        y1 += modMid * center

        return y0, y1

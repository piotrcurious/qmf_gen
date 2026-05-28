/*
  Basic QMF Filter Example

  This sketch demonstrates using the generated QMF2 filter class with standard
  analogRead and analogWrite functions. Note that for high-quality audio,
  DMA-based I/O is recommended.
*/

#include "../../qmf_24.h"

QMF2 qmf;

void setup() {
  Serial.begin(115200);
  Serial.println("QMF Basic Filter Example Started");

  // Initialize pins (using standard Arduino pins for demonstration)
  pinMode(A0, INPUT);
}

void loop() {
  // Read an analog sample (0-4095 for ESP32)
  int raw_in = analogRead(A0);

  // Convert to float in range [-1.0, 1.0]
  float in_sample = (raw_in - 2048) / 2048.0f;

  float low, high;

  // process() returns true every 2nd sample due to decimation
  if (qmf.process(in_sample, low, high)) {
    // In a real application, you would do something with low and high bands here.
    // For this example, we just print them occasionally.
    static int count = 0;
    if (++count >= 1000) {
      Serial.printf("L: %f, H: %f\n", low, high);
      count = 0;
    }

    // To output to a DAC, you would convert back to integer:
    // int out_val = (int)((low + 1.0f) * 127.5f);
    // dacWrite(25, out_val);
  }
}

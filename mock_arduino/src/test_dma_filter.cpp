#include "Arduino.h"
#include "../../examples/DmaFilter/DmaFilter.ino"

int main() {
    setup();
    for (int i = 0; i < 100; i++) {
        loop();
    }
    Serial.println("DmaFilter test completed.");
    return 0;
}

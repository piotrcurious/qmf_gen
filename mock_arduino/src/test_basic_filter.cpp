#include "Arduino.h"
#include "../../examples/BasicFilter/BasicFilter.ino"

int main() {
    setup();
    for (int i = 0; i < 2000; i++) {
        loop();
    }
    Serial.println("BasicFilter test completed.");
    return 0;
}

#include <Servo.h>

Servo servo[3];

void setup() {  // Attach motors and begin serial communication
    servo[0].attach(9);
    servo[1].attach(10);
    servo[2].attach(11);

    Serial.begin(115200);
}

void loop() {
    // Set motor number
    int index = 3;
    do {
    index = Serial.read() - 'a';
    } while (index != 0 && index != 1 && index != 2);

    // Get number
    delay(1);
    int digit1 = Serial.read() - '0';
    delay(1);
    int digit2 = Serial.read() - '0';
    delay(1);
    int digit3 = Serial.read() - '0';

    // Set motor value
    servo[index].write(digit1*100+digit2*10+digit3);
}

#include <Servo.h>
#define STOP_PULSE_LENGTH 1000 // Neutral/Stop pulse length in µs
#define MIN_PULSE_LENGTH 1150 // Minimum pulse length in µs
#define MAX_FORWARD_PULSE_LENGTH 1480 // Maximum pulse for forward in µs
#define MAX_REVERSE_PULSE_LENGTH 2000 // Maximum pulse for reverse in µs

Servo motA, motB, motAR, motBR; // Servos for all motors
char data;

void setup() {
    Serial.begin(9600);

    motA.attach(2, MIN_PULSE_LENGTH, MAX_REVERSE_PULSE_LENGTH);
    motB.attach(3, MIN_PULSE_LENGTH, MAX_REVERSE_PULSE_LENGTH);
    motAR.attach(9, MIN_PULSE_LENGTH, MAX_REVERSE_PULSE_LENGTH);
    motBR.attach(10, MIN_PULSE_LENGTH, MAX_REVERSE_PULSE_LENGTH);

    displayInstructions();

    setThrottle(1000);
}

void loop() {
    if (Serial.available()) {
        data = Serial.read();

        switch (data) {
            case '1': // Neutral/Stop
                Serial.println("Stopping motors (neutral position)");
                setThrottle(1000);
                break;

            case '2': // Forward minimum throttle
                Serial.println("Moving forward at minimum throttle");
                setThrottle(MIN_PULSE_LENGTH);
                break;

            case '3': // Forward maximum throttle
                Serial.println("Moving forward at maximum throttle");
                setThrottle(MAX_FORWARD_PULSE_LENGTH);
                break;

            case '4': // Reverse minimum throttle
                Serial.println("Moving reverse at minimum throttle");
                setThrottle(STOP_PULSE_LENGTH + 10); // Slightly above neutral for reverse
                break;

            case '5': // Reverse maximum throttle
                Serial.println("Moving reverse at maximum throttle");
                setThrottle(MAX_REVERSE_PULSE_LENGTH);
                break;

            case '6': // Gradual throttle increase
                Serial.println("Gradually increasing throttle from minimum to maximum");
                gradualThrottleIncrease();
                break;

            default:
                Serial.println("Invalid command. Please send 1-6.");
                displayInstructions();
        }
    }
}

void setThrottle(int pulseLength) {
    motA.writeMicroseconds(pulseLength);
    motB.writeMicroseconds(pulseLength);
    motAR.writeMicroseconds(pulseLength);
    motBR.writeMicroseconds(pulseLength);
}

void gradualThrottleIncrease() {
    for (int i = MIN_PULSE_LENGTH; i <= MAX_REVERSE_PULSE_LENGTH; i += 10) {
        Serial.print("Current throttle pulse: ");
        Serial.println(i);

        setThrottle(i);
        delay(1250);
    }

    Serial.println("Reached maximum throttle.");
    setThrottle(STOP_PULSE_LENGTH);
}

void displayInstructions() {
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t1 : Stop motors (neutral)");
    Serial.println("\t2 : Move forward (min throttle)");
    Serial.println("\t3 : Move forward (max throttle)");
    Serial.println("\t4 : Move reverse (min throttle)");
    Serial.println("\t5 : Move reverse (max throttle)");
    Serial.println("\t6 : Gradual throttle increase (min to max)");
}

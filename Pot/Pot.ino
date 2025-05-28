#include <Servo.h>

// Define the number of servos and potentiometers
#define NUM_SERVOS 6

// Create servo objects
Servo servos[NUM_SERVOS];

// Define pin numbers for servos and potentiometers
int servoPins[NUM_SERVOS] = {3, 5, 6, 9, 10, 11};
int potPins[NUM_SERVOS] = {A0, A1, A2, A3, A4, A5};

void setup() {
    // Attach each servo to its respective pin
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].attach(servoPins[i]);
    }
}

void loop() {
    for (int i = 0; i < NUM_SERVOS; i++) {
        int potValue = analogRead(potPins[i]); // Read potentiometer value (0-1023)
        int servoAngle = map(potValue, 0, 1023, 0, 180); // Map to servo angle
        servos[i].write(servoAngle); // Move servo to the mapped position
    }
    delay(10); // Small delay to prevent excessive updates
}

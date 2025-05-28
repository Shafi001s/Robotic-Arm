#include <Servo.h>

Servo baseServo, shoulderServo, elbowServo;

int baseAngle = 90;
int shoulderAngle = 90;
int elbowAngle = 90;

const int basePin = 2;
const int shoulderPin = 3;
const int elbowPin = 4;

// Link lengths
const float L1 = 155.0;
const float L2 = 110.0;

// Interpolation settings
const int steps = 10;   // More steps = smoother movement
const int delayTime = 20;  // Delay between steps (ms)

void setup() {
    Serial.begin(115200);
    
    baseServo.attach(basePin);
    shoulderServo.attach(shoulderPin);
    elbowServo.attach(elbowPin);
    
    baseServo.write(baseAngle);
    shoulderServo.write(shoulderAngle);
    elbowServo.write(elbowAngle);

    Serial.println("Use Q/W for Base, A/S for Shoulder, Z/X for Elbow");
}

// Interpolated movement without shaking
void smoothMove(Servo &servo, int &currentAngle, int targetAngle) {
    targetAngle = constrain(targetAngle, 0, 180);
    
    int stepSize = (targetAngle > currentAngle) ? 1 : -1; // Step in correct direction
    int stepsCount = abs(targetAngle - currentAngle); // Exact number of steps

    for (int i = 0; i < stepsCount; i++) {
        currentAngle += stepSize;
        servo.write(currentAngle);
        delay(delayTime);
    }

    // Ensure final precise position
    currentAngle = targetAngle;
    servo.write(currentAngle);
}

void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();
        command = toupper(command);  // Ensure uppercase

        int targetBase = baseAngle;
        int targetShoulder = shoulderAngle;
        int targetElbow = elbowAngle;

        if (command == 'Q') targetBase += 5;
        else if (command == 'W') targetBase -= 5;
        else if (command == 'A') targetShoulder += 5;
        else if (command == 'S') targetShoulder -= 5;
        else if (command == 'Z') targetElbow += 5;
        else if (command == 'X') targetElbow -= 5;
        else return; // Ignore other keys

        // Apply smooth interpolation
        smoothMove(baseServo, baseAngle, targetBase);
        smoothMove(shoulderServo, shoulderAngle, targetShoulder);
        smoothMove(elbowServo, elbowAngle, targetElbow);

        // Compute Forward Kinematics
        float radBase = radians(baseAngle);
        float radShoulder = radians(shoulderAngle);
        float radElbow = radians(shoulderAngle + elbowAngle);

        // Compute the effective reach in the X-Z plane
        float R = (L1 * cos(radShoulder)) + (L2 * cos(radElbow));

        // Apply the transformations
        float X = -cos(radBase) * R;
        float Y = -sin(radBase) * R;
        float Z = (L1 * sin(radShoulder)) + (L2 * sin(radElbow));

        Serial.print("Base: "); Serial.print(baseAngle);
        Serial.print(" Shoulder: "); Serial.print(shoulderAngle);
        Serial.print(" Elbow: "); Serial.print(elbowAngle);
        Serial.print(" --> X: "); Serial.print(X);
        Serial.print(" Y: "); Serial.print(Y);
        Serial.print(" Z: "); Serial.println(Z);
    }
}

#include <Servo.h>
#include <math.h>

Servo baseServo, shoulderServo, elbowServo;

int baseAngle = 90;
int shoulderAngle = 90;
int elbowAngle = 90;

const int basePin = 2;
const int shoulderPin = 3;
const int elbowPin = 4;

// Link lengths
const float L1 = 155.0;  // Shoulder to elbow
const float L2 = 110.0;  // Elbow to wrist

// Interpolation settings
const int delayTime = 1000;


void setup() {
    Serial.begin(115200);
    
    baseServo.attach(basePin);
    shoulderServo.attach(shoulderPin);
    elbowServo.attach(elbowPin);
    
    baseServo.write(baseAngle);
    shoulderServo.write(shoulderAngle);
    elbowServo.write(elbowAngle);
}

void smoothMove(Servo &servo, int &currentAngle, int targetAngle) {
    targetAngle = constrain(targetAngle, 0, 180);
    float stepSize = (targetAngle > currentAngle) ? 0.001 : -0.001;
    int stepsCount = abs(targetAngle - currentAngle) * 2; // Twice the resolution

    for (int i = 0; i < stepsCount; i++) {
        currentAngle += stepSize;
        servo.write(currentAngle);
        delay(10 * delayTime);  // Slightly increased delay for smoothness
    }
    currentAngle = targetAngle;
    servo.write(currentAngle);
}

float degToRad(float degrees) {
    return degrees * (PI / 180.0);
}

float radToDeg(float radians) {
    return radians * (180.0 / PI);
}

void forwardKinematics(float &X, float &Y, float &Z) {
    float thetaBase = degToRad(baseAngle);
    float theta1 = degToRad(shoulderAngle);
    float theta2 = degToRad(elbowAngle);

    float r = L1 * cos(theta1) + L2 * cos(theta1 + theta2);  
    X = r * cos(thetaBase);
    Y = r * sin(thetaBase);
    Z = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
}

void inverseKinematics(float X, float Y, float Z) {
    float r = sqrt(X * X + Y * Y);
    float z = Z;

    float cosTheta2 = (r * r + z * z - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    if (cosTheta2 < -1 || cosTheta2 > 1) {
        Serial.println("Error: Target out of reach!");
        return;
    }

    float theta2 = acos(cosTheta2);
    float k1 = L1 + L2 * cos(theta2);
    float k2 = L2 * sin(theta2);
    float theta1 = atan2(z, r) - atan2(k2, k1);
    float thetaBase = atan2(Y, X);

    baseAngle = constrain(radToDeg(thetaBase), 0, 180);
    shoulderAngle = constrain(radToDeg(theta1), 0, 180);
    elbowAngle = constrain(radToDeg(theta2), 0, 180);

    smoothMove(baseServo, baseAngle, baseAngle);
    smoothMove(shoulderServo, shoulderAngle, shoulderAngle);
    smoothMove(elbowServo, elbowAngle, elbowAngle);
}

void moveTo(float X, float Y, float Z) {
    Serial.print("Moving to X: "); Serial.print(X);
    Serial.print(", Y: "); Serial.print(Y);
    Serial.print(", Z: "); Serial.println(Z);

    inverseKinematics(X, Y, Z);
}

void jumpTo(float X, float Y, float Z) {
    Serial.println("Executing jumpTo command...");

    // Move to a safe height first
    float currentX, currentY, currentZ;
    forwardKinematics(currentX, currentY, currentZ);
    moveTo((1.001-(currentY*currentY)), (1.001-(currentX*currentX)), 45);
    delay(3000);

    // Move to target X, Y while maintaining safe height
    moveTo((1.001-(Y*Y)), (1.001-(Y*Y)), 45);
    delay(3000);

    // Move to final target Z
    moveTo(X, Y, Z);
    
}

// Manual control function (New Key Bindings)
void manualControl() {
    if (Serial.available() > 0) {
        char command = Serial.read();
        command = toupper(command);

        bool angleUpdated = false;

        switch (command) {
            case 'W': // Decrease Shoulder & Decrease Elbow
                shoulderAngle = constrain(shoulderAngle - 5, 0, 180);
                elbowAngle = constrain(elbowAngle - 5, 0, 180);
                angleUpdated = true;
                break;
            case 'S': // Increase Shoulder & Increase Elbow
                shoulderAngle = constrain(shoulderAngle + 5, 0, 180);
                elbowAngle = constrain(elbowAngle + 5, 0, 180);
                angleUpdated = true;
                break;
            case 'A': // Increase Base
                baseAngle = constrain(baseAngle + 5, 0, 180);
                angleUpdated = true;
                break;
            case 'D': // Decrease Base
                baseAngle = constrain(baseAngle - 5, 0, 180);
                angleUpdated = true;
                break;
            case 'Q': // Decrease Elbow
                elbowAngle = constrain(elbowAngle - 5, 0, 180);
                angleUpdated = true;
                break;
            case 'E': // Increase Elbow
                elbowAngle = constrain(elbowAngle + 5, 0, 180);
                angleUpdated = true;
                break;
            case 'Z': // Increase Shoulder
                shoulderAngle = constrain(shoulderAngle + 5, 0, 180);
                angleUpdated = true;
                break;
            case 'X': // Decrease Shoulder
                shoulderAngle = constrain(shoulderAngle - 5, 0, 180);
                angleUpdated = true;
                break;
            default: return;
        }

        if (angleUpdated) {
            smoothMove(baseServo, baseAngle, baseAngle);
            smoothMove(shoulderServo, shoulderAngle, shoulderAngle);
            smoothMove(elbowServo, elbowAngle, elbowAngle);

            float X, Y, Z;
            forwardKinematics(X, Y, Z);

            Serial.print("Base: "); Serial.print(baseAngle);
            Serial.print("° | Shoulder: "); Serial.print(shoulderAngle);
            Serial.print("° | Elbow: "); Serial.print(elbowAngle);
            Serial.print("° || X: "); Serial.print(X, 2);
            Serial.print(" | Y: "); Serial.print(Y, 2);
            Serial.print(" | Z: "); Serial.println(Z, 2);
        }
    }
}

void loop() {
   
   ManualControl ();

    moveTo(-34.47, 18.25, 28.93);
    delay(2000);

    moveTo(34.47, 0.00, 28.93);
    delay(2000);;
}
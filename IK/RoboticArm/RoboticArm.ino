#include <Servo.h>
#include <math.h>

Servo baseServo, shoulderServo, elbowServo, gripServo;

// Set Home Positions
int baseAngle = 90;
int shoulderAngle = 130;
int elbowAngle = 170;
int gripAngle = 90;

//Set up Pins
const int basePin = 2;
const int shoulderPin = 3;
const int elbowPin = 4;
const int gripPin = 5;

// Link lengths
const float L1 = 155.0;  // Shoulder to elbow
const float L2 = 110.0;  // Elbow to wrist

// Interpolation settings
const int delayTime = 1000;

//Do not Edit Below Unless Necessary
void setup() {
    Serial.begin(115200);
    
    baseServo.attach(basePin);
    shoulderServo.attach(shoulderPin);
    elbowServo.attach(elbowPin);
    gripServo.attach(gripPin);
    
    baseServo.write(baseAngle);
    shoulderServo.write(shoulderAngle);
    elbowServo.write(elbowAngle);
    gripServo.write(gripAngle);
}

//Linear Interpolation
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

//Conversion of degrees to radians
float degToRad(float degrees) {
    return degrees * (PI / 180.0);
}

//Conversion of radias to degrees
float radToDeg(float radians) {
    return radians * (180.0 / PI);
}


//Forward Kinematics Function
void forwardKinematics(float &X, float &Y, float &Z) {
    float thetaBase = degToRad(baseAngle);
    float theta1 = degToRad(shoulderAngle);
    float theta2 = degToRad(elbowAngle);

    float r = L1 * cos(theta1) + L2 * cos(theta1 + theta2);  
    X = r * cos(thetaBase);
    Y = r * sin(thetaBase);
    Z = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
}


//Inverse Kinematics Function
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


//Move To Movement Control Function 
void moveTo(float X, float Y, float Z) {
    Serial.print("Moving to X: "); Serial.print(X);
    Serial.print(", Y: "); Serial.print(Y);
    Serial.print(", Z: "); Serial.println(Z);

    inverseKinematics(X, Y, Z);
}

//Jump To Movement Control Function
void jumpTo(float X, float Y, float Z) {
    Serial.println("Executing jumpTo command...");

    // Move to a safe height first
    float currentX, currentY, currentZ;
    forwardKinematics(currentX, currentY, currentZ);
    moveTo(0.0, -36.54, 28.63);
    delay(3000);
    // Move to final target Z
    moveTo(X, Y, Z);
    
}

// Manual control function 
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
//Do not Edit Above Unless Necessary

//Control Functions are Executed from this section
void loop() {
    

    manualControl();// After Excecuting Command, Use Serial Monitor and reffer to User Instruction for Controls

    
    moveTo(5.99, -5.02, 44.32);
    delay(2000);
    
    gripServo.write(90);
    delay(2000);
    gripServo.write(0);
    delay(2000);

    moveTo(0.00, -33.90, 28.63);
    delay(2000);

    moveTo(-7.81, 0.00, 44.32);
    delay(3000); 

    gripServo.write(90);
    delay(2000);
    gripServo.write(0);
    delay(2000);   

}

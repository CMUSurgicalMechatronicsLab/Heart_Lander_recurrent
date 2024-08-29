#include <Servo.h>

Servo myServo;
const int potPin = A0; // Potentiometer connected to A0
const int servoPin = 9; // Servo signal pin connected to D9

int targetPosition = 90; // Target position in degrees
int currentPosition = 0; // Current position
int error = 0;
int previousError = 0;
int integral = 0;
int derivative = 0;

// PID coefficients
float Kp = 2.0;
float Ki = 0.1;
float Kd = 1.0;

void setup() {
    Serial.begin(115200); // Start serial communication
    myServo.attach(servoPin); // Attach servo to pin
    myServo.write(90); // Set initial position to 90 degrees
}

void loop() {
    // Read the current position from the potentiometer
    int sensorValue = analogRead(potPin);
    currentPosition = map(sensorValue, 0, 1023, 0, 180); // Map to servo angle

    // PID control
    error = targetPosition - currentPosition;
    integral += error;
    derivative = error - previousError;

    int output = Kp * error + Ki * integral + Kd * derivative;
    output = constrain(output, 0, 180); // Ensure output is within servo range

    myServo.write(output); // Move the servo to the new position
    previousError = error;

    // Check for incoming serial data
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim(); // Remove any trailing whitespace

        if (command.startsWith("SET_POSITION")) {
            int newPosition = command.substring(13).toInt();
            targetPosition = constrain(newPosition, 0, 180); // Ensure it's within the valid range
            Serial.println("OK");
        } else if (command.startsWith("GET_POSITION")) {
            Serial.println(currentPosition);
        } else if (command.startsWith("ENABLE")) {
            // Placeholder for any enable logic
            Serial.println("ENABLED");
        } else if (command.startsWith("DISABLE")) {
            // Placeholder for any disable logic
            Serial.println("DISABLED");
        } else {
            Serial.println("UNKNOWN COMMAND");
        }
    }

    delay(20); // Small delay to stabilize the loop
}

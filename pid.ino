#include <Servo.h>

// Define pins for the six servos
const int servoPins[6] = {3, 5, 6, 9, 10, 11};

// Create servo objects
Servo servos[6];

// PID constants
const float Kp = 2.0;
const float Ki = 0.1;
const float Kd = 1.0;

// Variables for PID control
float setpointX = 0;
float setpointY = 0;
float errorX = 0;
float errorY = 0;
float lastErrorX = 0;
float lastErrorY = 0;
float integralX = 0;
float integralY = 0;

// Time variables
unsigned long lastTime = 0;
const long interval = 20; // Control loop interval in milliseconds

void setup() {
  Serial.begin(9600);
  
  // Attach servos to pins
  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(90); // Set initial position to neutral
  }
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastTime >= interval) {
    // Read ball position from Raspberry Pi (assuming it sends X and Y as comma-separated values)
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      int commaIndex = input.indexOf(',');
      if (commaIndex != -1) {
        float ballX = input.substring(0, commaIndex).toFloat();
        float ballY = input.substring(commaIndex + 1).toFloat();
        
        // Calculate errors
        errorX = setpointX - ballX;
        errorY = setpointY - ballY;
        
        // Calculate integral and derivative terms
        integralX += errorX * (interval / 1000.0);
        integralY += errorY * (interval / 1000.0);
        float derivativeX = (errorX - lastErrorX) / (interval / 1000.0);
        float derivativeY = (errorY - lastErrorY) / (interval / 1000.0);
        
        // Calculate PID output
        float outputX = Kp * errorX + Ki * integralX + Kd * derivativeX;
        float outputY = Kp * errorY + Ki * integralY + Kd * derivativeY;
        
        // Map PID output to servo angles
        int servoAngles[6];
        mapOutputToServoAngles(outputX, outputY, servoAngles);
        
        // Update servo positions
        for (int i = 0; i < 6; i++) {
          servos[i].write(servoAngles[i]);
        }
        
        // Update last error for next iteration
        lastErrorX = errorX;
        lastErrorY = errorY;
      }
    }
    
    lastTime = currentTime;
  }
}

void mapOutputToServoAngles(float outputX, float outputY, int servoAngles[6]) {
  // This is a simplified mapping function and may need to be adjusted
  // based on your specific Stewart platform geometry
  for (int i = 0; i < 6; i++) {
    float angle = 90; // Neutral position
    
    // Adjust angle based on outputX and outputY
    if (i < 3) {
      angle += outputX * 10; // Adjust multiplier as needed
    } else {
      angle -= outputX * 10;
    }
    
    if (i % 2 == 0) {
      angle += outputY * 10;
    } else {
      angle -= outputY * 10;
    }
    
    // Constrain angle to valid servo range
    servoAngles[i] = constrain(int(angle), 0, 180);
  }
}

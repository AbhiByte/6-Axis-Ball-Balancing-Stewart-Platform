#include <Wire.h>

// Constants for I2C and servo channels
const int SDA_Pin = 20;
const int SCL_Pin = 21;
const uint8_t X_SERVO_CHANNEL = 1;  // Channel for X-axis servo
const uint8_t Y_SERVO_CHANNEL = 2;  // Channel for Y-axis servo

// PID Constants
const float KP = 0.8;  // Proportional gain
const float KI = 0.2;  // Integral gain
const float KD = 0.1;  // Derivative gain

// PID Variables for each axis
float x_integral = 0;
float y_integral = 0;
float x_prev_error = 0;
float y_prev_error = 0;
unsigned long last_time = 0;

// Target positions (center of frame)
const int TARGET_X = 128;  // Assuming 256x256 frame
const int TARGET_Y = 128;

// Variables to store received coordinates
volatile int message_id = 0;
volatile int x_coord = 0;
volatile int y_coord = 0;
bool dataReceived = false;

void setup() {
  // Initialize I2C communication
  Wire.begin(0x8);
  Wire.onReceive(receiveEvent);
  
  // Turn off built-in pull-up resistors
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
  
  Serial.begin(9600);
}

void loop() {
  if (dataReceived) {
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0;  // Convert to seconds
    
    // Calculate errors (distance from center)
    int x_error = x_coord - TARGET_X;
    int y_error = y_coord - TARGET_Y;
    
    // PID calculations for X axis
    x_integral += x_error * dt;
    float x_derivative = (x_error - x_prev_error) / dt;
    float x_output = (KP * x_error) + (KI * x_integral) + (KD * x_derivative);
    
    // PID calculations for Y axis
    y_integral += y_error * dt;
    float y_derivative = (y_error - y_prev_error) / dt;
    float y_output = (KP * y_error) + (KI * y_integral) + (KD * y_derivative);
    
    // Simulate actual motor movement (add some artificial limitations/behavior)
    float x_actual = simulateMotorResponse(x_output);
    float y_actual = simulateMotorResponse(y_output);
    
    // Convert PID outputs to servo positions
    int x_servo_position = map(constrain(x_actual, -255, 255), -255, 255, 1000, 2000);
    int y_servo_position = map(constrain(y_actual, -255, 255), -255, 255, 1000, 2000);
    
    // Move servos
    setServoPosition(X_SERVO_CHANNEL, x_servo_position);
    setServoPosition(Y_SERVO_CHANNEL, y_servo_position);
    
    // Debug output
    Serial.print("X Error: "); Serial.print(x_error);
    Serial.print(" X Output: "); Serial.print(x_output);
    Serial.print(" X Actual: "); Serial.print(x_actual);
    Serial.print(" X Servo: "); Serial.println(x_servo_position);
    
    // Store values for next iteration
    x_prev_error = x_error;
    y_prev_error = y_error;
    last_time = current_time;
    
    dataReceived = false;
  }
  
  delay(50);
}

void receiveEvent(int howMany) {
  if (howMany >= 3) {
    message_id = Wire.read();
    x_coord = Wire.read();
    y_coord = Wire.read();
    dataReceived = true;
  }
}

void setServoPosition(uint8_t channel, uint16_t position) {
  uint16_t target = position * 4;  // Convert to quarter-microseconds
  
  Serial.write(0x84);  // Command byte for 'Set Target'
  Serial.write(channel);
  Serial.write(target & 0x7F);
  Serial.write((target >> 7) & 0x7F);
}

// Simulate realistic motor behavior
float simulateMotorResponse(float commanded_output) {
    // Add deadband
    if (abs(commanded_output) < 20) {
        return 0;
    }
    
    // Add saturation
    float saturated = constrain(commanded_output, -255, 255);
    
    // Add some "mechanical lag"
    return saturated * 0.8;  // Only achieve 80% of commanded movement
}
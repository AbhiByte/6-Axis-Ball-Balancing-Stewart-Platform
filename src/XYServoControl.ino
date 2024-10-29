#include <Wire.h>

// Constants for I2C and servo channels
const int SDA_Pin = 20;
const int SCL_Pin = 21;
const uint8_t X_SERVO_CHANNEL = 1;  // Channel for X-axis servo
const uint8_t Y_SERVO_CHANNEL = 2;  // Channel for Y-axis servo

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
    // Map X coordinate (0-255) to servo range (1000-2000 microseconds)
    int x_servo_position = map(x_coord, 0, 255, 1000, 2000);
    int y_servo_position = map(y_coord, 0, 255, 1000, 2000);
    
    // Move both servos
    setServoPosition(X_SERVO_CHANNEL, x_servo_position);
    setServoPosition(Y_SERVO_CHANNEL, y_servo_position);
    
    // Debug output
    Serial.print("Message ID: "); Serial.print(message_id);
    Serial.print(" X: "); Serial.print(x_coord);
    Serial.print(" Y: "); Serial.print(y_coord);
    Serial.print(" X Servo: "); Serial.print(x_servo_position);
    Serial.print(" Y Servo: "); Serial.println(y_servo_position);
    
    dataReceived = false;
  }
  
  delay(50);  // Small delay to prevent overwhelming the system
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
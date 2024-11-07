#include <Wire.h>
#include "PololuMaestro.h"
#include "math.h"

#define maestroSerial SERIAL_PORT_HARDWARE_OPEN

// I2C Constants
const int ARDUINO_I2C_ADDRESS = 0x8;
const int MESSAGE_ID = 0x01;

// Objects
MicroMaestro maestro(maestroSerial);

// Platform geometry constants
float l0 = 73.025;
float lf = 67.775;
float d1 = 36.8893;
float d2 = 38.1;
float m = 12.7;
float p1 = 31.75;
float p2 = 129;

// Servo constants
float abs_0 = 1000;    // us position of absolute 0 degrees
float abs_90 = 2000;   // us position of absolute 90 degrees
float toDeg = 180/PI;  // radians to degrees conversion

// Array indices
int x = 0, y = 1, z = 2;
String ID[6] = {"a1", "a2", "b1", "b2", "c1", "c2"};

// Servo ranges and offsets
float range[6][2] = {
    {-45, 45}, {45, -45},  // a1, a2
    {-45, 45}, {45, -45},  // b1, b2
    {-45, 45}, {45, -45}   // c1, c2
};

float offset[6] = {0, 0, 0, 0, 0, 0};  // Servo offsets

// Platform vectors and points
float nab[3] = {sqrt(3) * 0.5, -0.5, 0};
float nac[3] = {sqrt(3) * 0.5, 0.5, 0};
float nbc[3] = {0, 1, 0};

// Other necessary platform geometry variables
float t, u;  // Add initialization in setup()
float a10[3], a20[3], b10[3], b20[3], c10[3], c20[3];  // Platform points
float ab[3], ac[3], bc[3];  // Platform vectors
float theta[6];  // Servo angles

// Ball position variables
volatile int ball[2] = {0, 0};  // X and Y coordinates from Raspberry Pi
bool dataReceived = false;

// Target position (center of platform)
const float origin[2] = {128, 128};  // Center coordinates (based on 255x255 image)

// PID constants and variables
float kp = 6e-4;      // proportional constant
float kd = 0.56;      // derivative constant
float error[2];       // Current error
float error_prev[2];  // Previous error
float deriv[2];       // Derivative term
float out[2];         // Output values (nx and ny)
unsigned long time_prev;

// Add with other constants
float nz = 1;                        // nz is predefined
float hz_norm = 118.19374266158451;  // normal hz value (at this height all servos are at 0 degrees)
float r_max = 0.25;                  // max radius of the nx and ny graph when nz = 1

// Add these constants at the top with other constants
const uint16_t MIN_PULSE = 1000;   // Minimum pulse width (us)
const uint16_t MAX_PULSE = 2000;   // Maximum pulse width (us)
const uint16_t NEUTRAL_PULSE = 1500;  // Neutral position (us)

// Add these constants at the top
// Servo channels
const uint8_t SERVO_1 = 0;
const uint8_t SERVO_2 = 1;
const uint8_t SERVO_3 = 2;
const uint8_t SERVO_4 = 3;
const uint8_t SERVO_5 = 4;
const uint8_t SERVO_6 = 5;

// Platform servo neutral positions (microseconds)
const int NEUTRAL_1 = 1500;
const int NEUTRAL_2 = 1500;
const int NEUTRAL_3 = 1500;
const int NEUTRAL_4 = 1500;
const int NEUTRAL_5 = 1500;
const int NEUTRAL_6 = 1500;

// Update servo control functions
void setServoPosition(uint8_t channel, uint16_t position) {
    position = constrain(position, 1000, 2000);  // Safety constraint
    uint16_t target = position * 4;
    Serial.write(0x84);
    Serial.write(channel);
    Serial.write(target & 0x7F);
    Serial.write((target >> 7) & 0x7F);
}

void moveServo(int i, float pos, int spd, int acc) {
    // Add position constraints and error checking
    pos = constrain(pos + offset[i], -40, 40);  // Constrain to safe angles
    
    // Map angle to pulse width (1000-2000us range)
    uint16_t pulse = map(pos, range[i][0], range[i][1], MIN_PULSE, MAX_PULSE);
    pulse = constrain(pulse, MIN_PULSE, MAX_PULSE);
    
    // Debug output
    Serial.print("Servo ");
    Serial.print(ID[i]);
    Serial.print(": Angle=");
    Serial.print(pos);
    Serial.print(" Pulse=");
    Serial.println(pulse);
    
    // Set servo position using direct control
    setServoPosition(i, pulse);
}

void moveServos(int spd, int acc) {
    for (int i = 0; i < 6; i++) {
        // Get angle from inverse kinematics
        float pos = theta[i];
        
        // Add safety checks
        if (isnan(pos) || isinf(pos)) {
            Serial.print("Error: Invalid angle for servo ");
            Serial.println(ID[i]);
            stop();
            return;
        }
        
        // Move each servo
        moveServo(i, pos, spd, acc);
    }
    
    // Debug output platform state
    Serial.println("Platform State:");
    Serial.print("Ball pos: (");
    Serial.print(ball[0]);
    Serial.print(",");
    Serial.print(ball[1]);
    Serial.println(")");
}

void setAllServosNeutral() {
    setServoPosition(SERVO_1, NEUTRAL_1);
    setServoPosition(SERVO_2, NEUTRAL_2);
    setServoPosition(SERVO_3, NEUTRAL_3);
    setServoPosition(SERVO_4, NEUTRAL_4);
    setServoPosition(SERVO_5, NEUTRAL_5);
    setServoPosition(SERVO_6, NEUTRAL_6);
}

void stop() {
    setAllServosNeutral();
    while(1) {}
}

void setup() {
    Wire.begin(ARDUINO_I2C_ADDRESS);
    Wire.onReceive(receiveEvent);
    
    Serial.begin(115200);
    maestroSerial.begin(9600);
    
    initializePlatformGeometry();
    
    // Initialize platform to neutral position
    InverseKinematics(0, 0, hz_norm, 0, 0, 0);
    moveServos(20, 20);
    
    time_prev = millis();
}

void loop() {
    if (dataReceived) {
        updatePlatform();
        dataReceived = false;
    }
}

void receiveEvent(int howMany) {
    if (howMany >= 3) {  // Message ID + X + Y coordinates
        byte messageId = Wire.read();
        if (messageId == MESSAGE_ID) {
            ball[0] = Wire.read();  // X coordinate
            ball[1] = Wire.read();  // Y coordinate
            dataReceived = true;
        }
    }
}

void updatePlatform() {
    // Calculate error (distance from center)
    error[0] = origin[0] - ball[0];
    error[1] = ball[1] - origin[1];
    
    // Calculate time delta
    unsigned long current_time = millis();
    float dt = (current_time - time_prev) / 1000.0;  // Convert to seconds
    time_prev = current_time;
    
    // Calculate derivative terms
    deriv[0] = (error[0] - error_prev[0]) / dt;
    deriv[1] = (error[1] - error_prev[1]) / dt;
    
    // Handle NaN/Inf in derivative
    if (isnan(deriv[0]) || isinf(deriv[0])) deriv[0] = 0;
    if (isnan(deriv[1]) || isinf(deriv[1])) deriv[1] = 0;
    
    // Store current error for next iteration
    error_prev[0] = error[0];
    error_prev[1] = error[1];
    
    // Calculate PD outputs
    out[0] = (error[0] * kp) + (deriv[0] * kd);
    out[1] = (error[1] * kp) + (deriv[1] * kd);
    
    // Apply platform constraints
    float r_out = sqrt(pow(out[0], 2) + pow(out[1], 2));
    if (r_out > r_max) {
        out[0] = out[0] * (r_max / r_out);
        out[1] = out[1] * (r_max / r_out);
    }
    
    // Update platform position
    InverseKinematics(0, 0, hz_norm, out[0], out[1], 0);
    moveServos(0, 0);
    
    // Add bounds checking for ball coordinates
    if (ball[0] < 0 || ball[0] > 255 || ball[1] < 0 || ball[1] > 255) {
        Serial.println("Error: Invalid ball coordinates");
        return;
    }
}

// ... (keep all the existing helper functions from BallBalance.ino)
// Including: moveServo(), moveServos(), stop(), InverseKinematics(), mag(), dot() 

void InverseKinematics(float hx, float hy, float hz, float nx, float ny, float ax) {  // calculates theta values given hx, hy, hz, nx, ny, and ax (nz = 1 always)
  //define vectors and points
  float a[3] = { ax, 0, 0 }, a1f[3], a2f[3];
  float b[3], b1f[3], b2f[3];
  float c[3], c1f[3], c2f[3];

  float n[3] = { nx, ny, nz };  // defines normal vector
  n[x] = nx / mag(n);
  n[y] = ny / mag(n);
  n[z] = nz / mag(n);  // converts vector 'n' to a unit vector

  float h[3] = { hx, hy, hz };  // defined point h (center point of platform)

  // **STAGE 1 CALCULATIONS**
  //-------------------------
  // in regards to e, g, and k indexes 0, 1, and 2 are a, b, and c respectively
  float e[3], g[3], k[3];

  // af components
  e[0] = a[x] - h[x];                                                                                                       // 'ea'
  a[z] = ((n[y] * sqrt(pow(lf, 2) * (1 - pow(n[x], 2)) - pow(e[0], 2)) - n[z] * n[x] * e[0]) / (1 - pow(n[x], 2))) + h[z];  // 'az'
  g[0] = a[z] - h[z];                                                                                                       // calculates 'ga'
  a[y] = h[y] - sqrt(pow(lf, 2) - pow(g[0], 2) - pow(e[0], 2));                                                             // 'ay'
  k[0] = a[y] - h[y];                                                                                                       // 'ka'

  float w = sqrt(3) * (n[x] * g[0] - n[z] * e[0]);  // intermediate variable

  // bf components
  b[y] = h[y] + ((sqrt(pow(w, 2) - 3 * pow(lf, 2) * (1 - pow(n[y], 2)) + pow(2 * k[0], 2)) - w) / 2);  // 'by'
  k[1] = b[y] - h[y];                                                                                  // 'kb'
  b[x] = ((e[0] * k[1] - n[z] * t) / k[0]) + h[x];                                                     // 'bx'
  e[1] = b[x] - h[x];                                                                                  // 'eb'
  b[z] = ((n[x] * t + g[0] * k[1]) / k[0]) + h[z];                                                     //'bz'
  g[1] = b[z] - h[z];                                                                                  // 'gb'

  // cf components
  c[y] = h[y] + ((w + sqrt(pow(w, 2) - 3 * pow(lf, 2) * (1 - pow(n[y], 2)) + pow(2 * k[0], 2))) / 2);  // 'cy'
  k[2] = c[y] - h[y];                                                                                  // 'kc'
  c[x] = ((e[0] * k[2] + n[z] * t) / k[0]) + h[x];                                                     // 'cx'
  e[2] = c[x] - h[x];                                                                                  // 'ec'
  c[z] = ((g[0] * k[2] - n[x] * t) / k[0]) + h[z];                                                     // 'cz'
  g[2] = c[z] - h[z];                                                                                  // 'gc'

  // STAGE 2 CALCULATIONS
  //---------------------

  // a1
  a1f[x] = a[x] + (m / lf) * (n[z] * k[0] - n[y] * g[0]);  // a1fx
  if (e[0] == 0) {                                         //if e[0] is 0 then there will be a divide by zero error
    a1f[y] = a[y];                                         // a1fy
    a1f[z] = a[z];                                         // a1fz
  } else {
    a1f[y] = a[y] + ((a1f[x] - a[x]) * k[0] - n[z] * lf * m) / e[0];  // a1fy
    a1f[z] = a[z] + (n[y] * lf * m + (a1f[x] - a[x]) * g[0]) / e[0];  // a1fz
  }
  float a1[3] = { a1f[x] - a10[x], a1f[y] - a10[y], a1f[z] - a10[z] };  // vector 'a1'

  // a2
  a2f[x] = 2 * a[x] - a1f[x];                                           // a2fx
  a2f[y] = 2 * a[y] - a1f[y];                                           // a2fy
  a2f[z] = 2 * a[z] - a1f[z];                                           // a2fz
  float a2[3] = { a2f[x] - a20[x], a2f[y] - a20[y], a2f[z] - a20[z] };  // vector 'a2'

  // b1
  b1f[x] = b[x] + (m / lf) * (n[z] * k[1] - n[y] * g[1]);               // b1fx
  b1f[y] = b[y] + ((b1f[x] - b[x]) * k[1] - n[z] * lf * m) / e[1];      // b1fy
  b1f[z] = b[z] + (n[y] * lf * m + (b1f[x] - b[x]) * g[1]) / e[1];      // b1fz
  float b1[3] = { b1f[x] - b10[x], b1f[y] - b10[y], b1f[z] - b10[z] };  // vector 'b1'

  // b2
  b2f[x] = 2 * b[x] - b1f[x];                                           // b2fx
  b2f[y] = 2 * b[y] - b1f[y];                                           // b2fy
  b2f[z] = 2 * b[z] - b1f[z];                                           // b2fz
  float b2[3] = { b2f[x] - b20[x], b2f[y] - b20[y], b2f[z] - b20[z] };  // vector 'b2'

  // c1
  c1f[x] = c[x] + (m / lf) * (n[z] * k[2] - n[y] * g[2]);               // c1fx
  c1f[y] = c[y] + ((c1f[x] - c[x]) * k[2] - n[z] * lf * m) / e[2];      // c1fy
  c1f[z] = c[z] + (n[y] * lf * m + (c1f[x] - c[x]) * g[2]) / e[2];      // c1fz
  float c1[3] = { c1f[x] - c10[x], c1f[y] - c10[y], c1f[z] - c10[z] };  // vector 'c1'

  // c2
  c2f[x] = 2 * c[x] - c1f[x];                                           // c2fx
  c2f[y] = 2 * c[y] - c1f[y];                                           // c2fy
  c2f[z] = 2 * c[z] - c1f[z];                                           // c2fz
  float c2[3] = { c2f[x] - c20[x], c2f[y] - c20[y], c2f[z] - c20[z] };  // vector 'c2'

  //**STAGE 3 CALCULATIONS**
  //------------------------

  // theta_a1
  float a1s[3] = { nac[x] * dot(a1, nac), nac[y] * dot(a1, nac), nac[z] * dot(a1, nac) };                                // vector 'a1s'
  float mag_a1s = mag(a1s);                                                                                              // magnitude of vector 'a1s'
  float a1_proj[3] = { a1[x] - a1s[x], a1[y] - a1s[y], a1[z] - a1s[z] };                                                 // projection of vector 'a1' onto the ac plane
  float mag_a1_proj = mag(a1_proj);                                                                                      // magnitude of vector 'a1' projected on the ac plane
  float mag_p2a1 = sqrt(pow(p2, 2) - pow(mag_a1s, 2));                                                                   // magnitude of link p2 projected on the ac plane
  theta[0] = acos(-dot(a1_proj, ac) / (2 * d2 * mag_a1_proj));                                                           // theta a1
  theta[0] = (theta[0] - acos((pow(mag_a1_proj, 2) + pow(p1, 2) - pow(mag_p2a1, 2)) / (2 * mag_a1_proj * p1))) * toDeg;  // theta a1 continued calculation

  // theta_a2
  float a2s[3] = { nab[x] * dot(a2, nab), nab[y] * dot(a2, nab), nab[z] * dot(a2, nab) };                                // vector 'a2s'
  float mag_a2s = mag(a2s);                                                                                              // magnitude of vector 'a2s'
  float a2_proj[3] = { a2[x] - a2s[x], a2[y] - a2s[y], a2[z] - a2s[z] };                                                 // projection of vector 'a2' onto the ab plane
  float mag_a2_proj = mag(a2_proj);                                                                                      // magnitude of vector 'a2' projected on the ab plane
  float mag_p2a2 = sqrt(pow(p2, 2) - pow(mag_a2s, 2));                                                                   // magnitude of link p2 projected on the ab plane
  theta[1] = acos(-dot(a2_proj, ab) / (2 * d2 * mag_a2_proj));                                                           // theta a2
  theta[1] = (theta[1] - acos((pow(mag_a2_proj, 2) + pow(p1, 2) - pow(mag_p2a2, 2)) / (2 * mag_a2_proj * p1))) * toDeg;  // theta a2 continued calculation

  // theta_b1
  float b1s[3] = { nab[x] * dot(b1, nab), nab[y] * dot(b1, nab), nab[z] * dot(b1, nab) };                                // vector 'b1s'
  float mag_b1s = mag(b1s);                                                                                              // magnitude of vector 'b1s'
  float b1_proj[3] = { b1[x] - b1s[x], b1[y] - b1s[y], b1[z] - b1s[z] };                                                 // projection of vector 'b1' onto the ab plane
  float mag_b1_proj = mag(b1_proj);                                                                                      // magnitude of vector 'b1' projected on the ab plane
  float mag_p2b1 = sqrt(pow(p2, 2) - pow(mag_b1s, 2));                                                                   // magnitude of link p2 projected on the ab plane
  theta[2] = acos(dot(b1_proj, ab) / (2 * d2 * mag_b1_proj));                                                            // theta b1
  theta[2] = (theta[2] - acos((pow(mag_b1_proj, 2) + pow(p1, 2) - pow(mag_p2b1, 2)) / (2 * mag_b1_proj * p1))) * toDeg;  // theta b1 continued calculation

  // theta_b2
  float b2s[3] = { nbc[x] * dot(b2, nbc), nbc[y] * dot(b2, nbc), nbc[z] * dot(b2, nbc) };                                // vector 'b2s'
  float mag_b2s = mag(b2s);                                                                                              // magnitude of vector 'b2s'
  float b2_proj[3] = { b2[x] - b2s[x], b2[y] - b2s[y], b2[z] - b2s[z] };                                                 // projection of vector 'b2' onto the bc plane
  float mag_b2_proj = mag(b2_proj);                                                                                      // magnitude of vector 'b2' projected on the bc plane
  float mag_p2b2 = sqrt(pow(p2, 2) - pow(mag_b2s, 2));                                                                   // magnitude of link p2 projected on the bc plane
  theta[3] = acos(-dot(b2_proj, bc) / (2 * d2 * mag_b2_proj));                                                           // theta b2
  theta[3] = (theta[3] - acos((pow(mag_b2_proj, 2) + pow(p1, 2) - pow(mag_p2b2, 2)) / (2 * mag_b2_proj * p1))) * toDeg;  // theta b2 continued calculation

  // theta_c1
  float c1s[3] = { nbc[x] * dot(c1, nbc), nbc[y] * dot(c1, nbc), nbc[z] * dot(c1, nbc) };                                // vector 'c1s'
  float mag_c1s = mag(c1s);                                                                                              // magnitude of vector 'c1s'
  float c1_proj[3] = { c1[x] - c1s[x], c1[y] - c1s[y], c1[z] - c1s[z] };                                                 // projection of vector 'c1' onto the bc plane
  float mag_c1_proj = mag(c1_proj);                                                                                      // magnitude of vector 'c1' projected on the bc plane
  float mag_p2c1 = sqrt(pow(p2, 2) - pow(mag_c1s, 2));                                                                   // magnitude of link p2 projected on the bc plane
  theta[4] = acos(dot(c1_proj, bc) / (2 * d2 * mag_c1_proj));                                                            // theta c1
  theta[4] = (theta[4] - acos((pow(mag_c1_proj, 2) + pow(p1, 2) - pow(mag_p2c1, 2)) / (2 * mag_c1_proj * p1))) * toDeg;  // theta c1 continued calculation

  //theta_c2
  float c2s[3] = { nac[x] * dot(c2, nac), nac[y] * dot(c2, nac), nac[z] * dot(c2, nac) };                                // vector 'c2s'
  float mag_c2s = mag(c2s);                                                                                              // magnitude of vector 'c2s'
  float c2_proj[3] = { c2[x] - c2s[x], c2[y] - c2s[y], c2[z] - c2s[z] };                                                 // projection of vector 'c2' onto the ac plane
  float mag_c2_proj = mag(c2_proj);                                                                                      // magnitude of vector 'c2' projected on the ac plane
  float mag_p2c2 = sqrt(pow(p2, 2) - pow(mag_c2s, 2));                                                                   // magnitude of link p2 projected on the ac plane
  theta[5] = acos(dot(c2_proj, ac) / (2 * d2 * mag_c2_proj));                                                            // theta c2
  theta[5] = (theta[5] - acos((pow(mag_c2_proj, 2) + pow(p1, 2) - pow(mag_p2c2, 2)) / (2 * mag_c2_proj * p1))) * toDeg;  // theta c2 continued calculation

  for (int i = 0; i < 6; i++) {  //checks for errors to see if theta values are between -40 and 40 degrees and if they are real numbers
    //Serial.print(String("theta "+ID[i]+": ")); // prints servo ID
    //Serial.println(theta[i], 6); // prints theta value

    if (abs(theta[i]) > 40) {
      Serial.println("ERROR: CURRENT VALUES EXCEED ANGLE RANGE");
      stop();
    }
    if (isnan(theta[i])) {
      Serial.println("ERROR: CURRENT VALUES CANNOT PHYSICALLY BE EXECUTED");
      stop();
    }
  }
}

float mag(float array[]) {  //finds the magnitude of an array of size 3
  float mag = 0;
  for (int i = 0; i < 3; i++) {
    mag = mag + pow(array[i], 2);  //adds component i of array squared
  }
  mag = sqrt(mag);
  return mag;
}

float dot(float array1[], float array2[]) {  //calculates the dot product of two arrays
  return array1[0] * array2[0] + array1[1] * array2[1] + array1[2] * array2[2];
}

void initializePlatformGeometry() {
    // Calculate intermediate variables
    t = (pow(lf, 2) * sqrt(3)) / 2;
    u = sqrt(pow(l0, 2) + pow(d1, 2)) * sin((2 * PI / 3) - atan(l0 / d1));
    
    // Initialize platform points
    a10[0] = (d2 - u * sqrt(3)) / 2;
    a10[1] = (-u - d2 * sqrt(3)) / 2;
    a10[2] = 0;
    
    a20[0] = -a10[0];
    a20[1] = a10[1];
    a20[2] = 0;
    
    b10[0] = (u * sqrt(3) + d2) / 2;
    b10[1] = (d2 * sqrt(3) - u) / 2;
    b10[2] = 0;
    
    b20[0] = d2;
    b20[1] = u;
    b20[2] = 0;
    
    c10[0] = -b20[0];
    c10[1] = b20[1];
    c10[2] = 0;
    
    c20[0] = -b10[0];
    c20[1] = b10[1];
    c20[2] = 0;
    
    // Initialize platform vectors
    ab[0] = a20[0] - b10[0];
    ab[1] = a20[1] - b10[1];
    ab[2] = a20[2] - b10[2];
    
    ac[0] = a10[0] - c20[0];
    ac[1] = a10[1] - c20[1];
    ac[2] = a10[2] - c20[2];
    
    bc[0] = b20[0] - c10[0];
    bc[1] = b20[1] - c10[1];
    bc[2] = b20[2] - c10[2];
}
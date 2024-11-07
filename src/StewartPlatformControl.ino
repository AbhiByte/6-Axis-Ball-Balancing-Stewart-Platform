#include <Wire.h>
#include <math.h>

// I2C Constants
const int ARDUINO_I2C_ADDRESS = 0x8;
const int MESSAGE_ID = 0x01;

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

// Platform geometry constants (from BallBalance)
float l0 = 73.025;
float lf = 67.775;
float d1 = 36.8893;
float d2 = 38.1;
float m = 12.7;
float p1 = 31.75;
float p2 = 129;

// Servo ranges (in degrees)
float range[6][2] = {
    {-45, 45}, {45, -45},  // a1, a2
    {-45, 45}, {45, -45},  // b1, b2
    {-45, 45}, {45, -45}   // c1, c2
};

// Keep the PID structure from 6MotorPID
struct PIDConstants {
    float kp;
    float ki;
    float kd;
} xPID = {6e-4, 0, 0.56},  // Using values from BallBalance
  yPID = {6e-4, 0, 0.56};

struct PIDVariables {
    float integral;
    float errorPrev;
    long prevT;
} xVars = {0, 0, 0},
  yVars = {0, 0, 0};

// Ball position variables (from 6MotorPID)
volatile int ball[2] = {0, 0};  // X and Y coordinates
bool dataReceived = false;

// Target position (center of platform)
const int TARGET_X = 128;
const int TARGET_Y = 128;

// Platform constants from BallBalance
float nz = 1;                        // nz is predefined
float hz_norm = 118.19374266158451;  // normal hz value
float r_max = 0.25;                  // max radius of the nx and ny graph when nz = 1

// Add other necessary platform geometry variables and vectors from BallBalance
// ... (platform points and vectors)

// Platform vectors and points
float nab[3] = {sqrt(3) * 0.5, -0.5, 0};
float nac[3] = {sqrt(3) * 0.5, 0.5, 0};
float nbc[3] = {0, 1, 0};

// Platform points and vectors
float t, u;  // Intermediate variables
float a10[3], a20[3], b10[3], b20[3], c10[3], c20[3];  // Platform points
float ab[3], ac[3], bc[3];  // Platform vectors
float theta[6];  // Servo angles

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

float mag(float array[]) {
    float mag = 0;
    for (int i = 0; i < 3; i++) {
        mag = mag + pow(array[i], 2);
    }
    return sqrt(mag);
}

float dot(float array1[], float array2[]) {
    return array1[0] * array2[0] + array1[1] * array2[1] + array1[2] * array2[2];
}

void InverseKinematics(float hx, float hy, float hz, float nx, float ny, float ax) {
    // Define vectors and points
    float a[3] = {ax, 0, 0}, a1f[3], a2f[3];
    float b[3], b1f[3], b2f[3];
    float c[3], c1f[3], c2f[3];
    
    // Define normal vector and normalize it
    float n[3] = {nx, ny, nz};
    float n_mag = mag(n);
    n[0] = nx / n_mag;
    n[1] = ny / n_mag;
    n[2] = nz / n_mag;
    
    float h[3] = {hx, hy, hz};  // Platform center point
    
    // Stage 1: Calculate platform points
    float e[3], g[3], k[3];
    
    // af components
    e[0] = a[0] - h[0];
    a[2] = ((n[1] * sqrt(pow(lf, 2) * (1 - pow(n[0], 2)) - pow(e[0], 2)) - 
             n[2] * n[0] * e[0]) / (1 - pow(n[0], 2))) + h[2];
    g[0] = a[2] - h[2];
    a[1] = h[1] - sqrt(pow(lf, 2) - pow(g[0], 2) - pow(e[0], 2));
    k[0] = a[1] - h[1];
    
    float w = sqrt(3) * (n[0] * g[0] - n[2] * e[0]);
    
    // bf components
    b[0] = h[0] + (2 * e[0] + w) / 4;
    b[2] = h[2] + (2 * g[0] + n[1] * t / n[0]) / 4;
    g[1] = b[2] - h[2];
    e[1] = b[0] - h[0];
    b[1] = h[1] - sqrt(pow(lf, 2) - pow(g[1], 2) - pow(e[1], 2));
    k[1] = b[1] - h[1];
    
    // cf components
    c[0] = h[0] + (2 * e[0] - w) / 4;
    c[2] = h[2] + (2 * g[0] - n[1] * t / n[0]) / 4;
    g[2] = c[2] - h[2];
    e[2] = c[0] - h[0];
    c[1] = h[1] - sqrt(pow(lf, 2) - pow(g[2], 2) - pow(e[2], 2));
    k[2] = c[1] - h[1];
    
    // Stage 2: Calculate platform points
    // a1
    a1f[0] = a[0] + (m / lf) * (n[2] * k[0] - n[1] * g[0]);
    a1f[1] = a[1] + ((a1f[0] - a[0]) * k[0] - n[2] * lf * m) / e[0];
    a1f[2] = a[2] + (n[1] * lf * m + (a1f[0] - a[0]) * g[0]) / e[0];
    float a1[3] = {a1f[0] - a10[0], a1f[1] - a10[1], a1f[2] - a10[2]};
    
    // a2
    a2f[0] = 2 * a[0] - a1f[0];
    a2f[1] = 2 * a[1] - a1f[1];
    a2f[2] = 2 * a[2] - a1f[2];
    float a2[3] = {a2f[0] - a20[0], a2f[1] - a20[1], a2f[2] - a20[2]};
    
    // b1
    b1f[0] = b[0] + (m / lf) * (n[2] * k[1] - n[1] * g[1]);
    b1f[1] = b[1] + ((b1f[0] - b[0]) * k[1] - n[2] * lf * m) / e[1];
    b1f[2] = b[2] + (n[1] * lf * m + (b1f[0] - b[0]) * g[1]) / e[1];
    float b1[3] = {b1f[0] - b10[0], b1f[1] - b10[1], b1f[2] - b10[2]};
    
    // b2
    b2f[0] = 2 * b[0] - b1f[0];
    b2f[1] = 2 * b[1] - b1f[1];
    b2f[2] = 2 * b[2] - b1f[2];
    float b2[3] = {b2f[0] - b20[0], b2f[1] - b20[1], b2f[2] - b20[2]};
    
    // c1
    c1f[0] = c[0] + (m / lf) * (n[2] * k[2] - n[1] * g[2]);
    c1f[1] = c[1] + ((c1f[0] - c[0]) * k[2] - n[2] * lf * m) / e[2];
    c1f[2] = c[2] + (n[1] * lf * m + (c1f[0] - c[0]) * g[2]) / e[2];
    float c1[3] = {c1f[0] - c10[0], c1f[1] - c10[1], c1f[2] - c10[2]};
    
    // c2
    c2f[0] = 2 * c[0] - c1f[0];
    c2f[1] = 2 * c[1] - c1f[1];
    c2f[2] = 2 * c[2] - c1f[2];
    float c2[3] = {c2f[0] - c20[0], c2f[1] - c20[1], c2f[2] - c20[2]};
    
    // Stage 3: Calculate angles
    float toDeg = 180.0 / PI;  // Convert radians to degrees
    
    // theta_a1
    float a1s[3] = {nac[0] * dot(a1, nac), nac[1] * dot(a1, nac), nac[2] * dot(a1, nac)};
    float mag_a1s = mag(a1s);
    float a1_proj[3] = {a1[0] - a1s[0], a1[1] - a1s[1], a1[2] - a1s[2]};
    float mag_a1_proj = mag(a1_proj);
    float mag_p2a1 = sqrt(pow(p2, 2) - pow(mag_a1s, 2));
    theta[0] = acos(-dot(a1_proj, ac) / (2 * d2 * mag_a1_proj));
    theta[0] = (theta[0] - acos((pow(mag_a1_proj, 2) + pow(p1, 2) - pow(mag_p2a1, 2)) / 
               (2 * mag_a1_proj * p1))) * toDeg;
    
    // theta_a2
    float a2s[3] = {nab[0] * dot(a2, nab), nab[1] * dot(a2, nab), nab[2] * dot(a2, nab)};
    float mag_a2s = mag(a2s);
    float a2_proj[3] = {a2[0] - a2s[0], a2[1] - a2s[1], a2[2] - a2s[2]};
    float mag_a2_proj = mag(a2_proj);
    float mag_p2a2 = sqrt(pow(p2, 2) - pow(mag_a2s, 2));
    theta[1] = acos(-dot(a2_proj, ab) / (2 * d2 * mag_a2_proj));
    theta[1] = (theta[1] - acos((pow(mag_a2_proj, 2) + pow(p1, 2) - pow(mag_p2a2, 2)) / 
               (2 * mag_a2_proj * p1))) * toDeg;

    // theta_b1
    float b1s[3] = {nab[0] * dot(b1, nab), nab[1] * dot(b1, nab), nab[2] * dot(b1, nab)};
    float mag_b1s = mag(b1s);
    float b1_proj[3] = {b1[0] - b1s[0], b1[1] - b1s[1], b1[2] - b1s[2]};
    float mag_b1_proj = mag(b1_proj);
    float mag_p2b1 = sqrt(pow(p2, 2) - pow(mag_b1s, 2));
    theta[2] = acos(dot(b1_proj, ab) / (2 * d2 * mag_b1_proj));
    theta[2] = (theta[2] - acos((pow(mag_b1_proj, 2) + pow(p1, 2) - pow(mag_p2b1, 2)) / 
               (2 * mag_b1_proj * p1))) * toDeg;

    // theta_b2
    float b2s[3] = {nbc[0] * dot(b2, nbc), nbc[1] * dot(b2, nbc), nbc[2] * dot(b2, nbc)};
    float mag_b2s = mag(b2s);
    float b2_proj[3] = {b2[0] - b2s[0], b2[1] - b2s[1], b2[2] - b2s[2]};
    float mag_b2_proj = mag(b2_proj);
    float mag_p2b2 = sqrt(pow(p2, 2) - pow(mag_b2s, 2));
    theta[3] = acos(-dot(b2_proj, bc) / (2 * d2 * mag_b2_proj));
    theta[3] = (theta[3] - acos((pow(mag_b2_proj, 2) + pow(p1, 2) - pow(mag_p2b2, 2)) / 
               (2 * mag_b2_proj * p1))) * toDeg;

    // theta_c1
    float c1s[3] = {nbc[0] * dot(c1, nbc), nbc[1] * dot(c1, nbc), nbc[2] * dot(c1, nbc)};
    float mag_c1s = mag(c1s);
    float c1_proj[3] = {c1[0] - c1s[0], c1[1] - c1s[1], c1[2] - c1s[2]};
    float mag_c1_proj = mag(c1_proj);
    float mag_p2c1 = sqrt(pow(p2, 2) - pow(mag_c1s, 2));
    theta[4] = acos(dot(c1_proj, bc) / (2 * d2 * mag_c1_proj));
    theta[4] = (theta[4] - acos((pow(mag_c1_proj, 2) + pow(p1, 2) - pow(mag_p2c1, 2)) / 
               (2 * mag_c1_proj * p1))) * toDeg;

    // theta_c2
    float c2s[3] = {nac[0] * dot(c2, nac), nac[1] * dot(c2, nac), nac[2] * dot(c2, nac)};
    float mag_c2s = mag(c2s);
    float c2_proj[3] = {c2[0] - c2s[0], c2[1] - c2s[1], c2[2] - c2s[2]};
    float mag_c2_proj = mag(c2_proj);
    float mag_p2c2 = sqrt(pow(p2, 2) - pow(mag_c2s, 2));
    theta[5] = acos(dot(c2_proj, ac) / (2 * d2 * mag_c2_proj));
    theta[5] = (theta[5] - acos((pow(mag_c2_proj, 2) + pow(p1, 2) - pow(mag_p2c2, 2)) / 
               (2 * mag_c2_proj * p1))) * toDeg;
    
    // Check angle limits
    for (int i = 0; i < 6; i++) {
        if (abs(theta[i]) > 40 || isnan(theta[i])) {
            Serial.println("Error: Invalid servo angle calculated");
            stop();
        }
    }
}

void setup() {
    Wire.begin(ARDUINO_I2C_ADDRESS);
    Wire.onReceive(receiveEvent);
    Serial.begin(115200);
    
    initializePlatformGeometry();  // Initialize platform geometry
    
    // Initialize platform to neutral position
    InverseKinematics(0, 0, hz_norm, 0, 0, 0);
    moveServos(20, 20);
}

void loop() {
    if (dataReceived) {
        // Calculate PID outputs for X and Y axes
        float x_output = calculatePID(ball[0], TARGET_X, xVars, xPID);
        float y_output = calculatePID(ball[1], TARGET_Y, yVars, yPID);
        
        // Constrain outputs to platform limits
        float r_out = sqrt(pow(x_output, 2) + pow(y_output, 2));
        if (r_out > r_max) {
            x_output = x_output * (r_max / r_out);
            y_output = y_output * (r_max / r_out);
        }
        
        // Update platform position using inverse kinematics
        InverseKinematics(0, 0, hz_norm, x_output, y_output, 0);
        moveServos(0, 0);
        
        dataReceived = false;
    }
    delay(20);
}

// Keep the proven motor control from 6MotorPID
void setServoPosition(uint8_t channel, uint16_t position) {
    position = constrain(position, 1000, 2000);  // Safety constraint
    uint16_t target = position * 4;
    Serial.write(0x84);
    Serial.write(channel);
    Serial.write(target & 0x7F);
    Serial.write((target >> 7) & 0x7F);
}

void moveServos(int spd, int acc) {
    for (int i = 0; i < 6; i++) {
        // Get angle from inverse kinematics
        float pos = theta[i];
        
        // Add safety checks
        if (isnan(pos) || isinf(pos) || abs(pos) > 40) {
            Serial.print("Error: Invalid angle for servo ");
            Serial.println(i);
            stop();
            return;
        }
        
        // Map angle to pulse width (1000-2000us range)
        uint16_t pulse = map(pos, range[i][0], range[i][1], 1000, 2000);
        pulse = constrain(pulse, 1000, 2000);
        
        // Move servo
        setServoPosition(i, pulse);
    }
}

// Keep the PID calculation from 6MotorPID
float calculatePID(int current, int target, PIDVariables &vars, PIDConstants &constants) {
    long currT = micros();
    float deltaT = ((float)(currT - vars.prevT)) / 1.0e6;
    vars.prevT = currT;
    
    int error = target - current;
    vars.integral = constrain(vars.integral + error * deltaT, -100, 100);
    float derivative = (error - vars.errorPrev) / deltaT;
    vars.errorPrev = error;
    
    return constants.kp * error + 
           constants.ki * vars.integral + 
           constants.kd * derivative;
}

// Keep the I2C communication from 6MotorPID
void receiveEvent(int howMany) {
    if (howMany >= 3) {
        byte messageId = Wire.read();
        if (messageId == MESSAGE_ID) {
            ball[0] = Wire.read();  // X coordinate
            ball[1] = Wire.read();  // Y coordinate
            dataReceived = true;
        }
    }
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
//灌和連接時，要斷電源

#include <Arduino.h>

// ===============================
// Configuration for BASE Motor
// ===============================
int Status = 0; // 0 normal 1 reset_ms_base 2 reset_ms_fort
int StatusCount = 0;

const int BASE_pinA = 18;           // Encoder Pin A
const int BASE_pinB = 19;           // Encoder Pin B
const int BASE_m_A = 10;            // Motor Input A
const int BASE_m_B = 11;            // Motor Input B
const int BASE_ms = 13;            // Micro Swith
const float BASE_k = 360.0 / (64 * 168.0); // Degrees per encoder step

volatile int BASE_encoderPos = 0;
volatile int BASE_lastEncoded = 0;
int BASE_targetAngle = 0;
float BASE_filteredAngle = 0.0;
float BASE_currentAngle = 0.0;

// PID Parameters for BASE
float BASE_Kp = 35, BASE_Ki = 1.5, BASE_Kd = 0.8;
float BASE_Kp_min = 35, BASE_Kp_max = 40;
float BASE_Ki_min = 1.5, BASE_Ki_max = 1.5;
float BASE_Kd_min = 0.8, BASE_Kd_max = 0.8;

// LMS Parameters for BASE
float BASE_mu = 0.001;
float BASE_momentum = 0.95;
float BASE_damping_factor = 0.8;
float BASE_theta[] = {BASE_Kp, BASE_Ki, BASE_Kd};
float BASE_theta_velocity[] = {0, 0, 0};

// Control Variables for BASE
float BASE_error, BASE_lastError = 0;
float BASE_I_term = 0;
float BASE_lastD_term = 0;
float BASE_maxOutput = 75;
float BASE_I_term_limit = 10.0;
float BASE_lastOutput = 0.0;

// Kalman Filter Variables for BASE
float BASE_Q = 0.1;
float BASE_R = 5.0;
float BASE_P = 1.0;
float BASE_K_gain = 0.0;
const int BASE_MAX_ANGLE = 65;
const int BASE_MIN_ANGLE = -65;

// ===============================
// Configuration for FORT Motor
// ===============================
const int FORT_pinA = 2;            // Encoder Pin A
const int FORT_pinB = 3;            // Encoder Pin B
const int FORT_m_A = 5;             // Motor Input A
const int FORT_m_B = 6;             // Motor Input B
const int FORT_ms = 7;            // Micro Swith
const float FORT_k = 360.0 / (64 * 810.0); // Degrees per encoder step

volatile int FORT_encoderPos = 0;
volatile int FORT_lastEncoded = 0;
float FORT_targetAngle = 0;
float FORT_filteredAngle = 0.0;
float FORT_currentAngle = 0.0;

// PID Parameters for FORT
float FORT_Kp = 40, FORT_Ki = 1.2, FORT_Kd = 0.2;
float FORT_Kp_min = 40, FORT_Kp_max = 45;
float FORT_Ki_min = 1.2, FORT_Ki_max = 1.2;
float FORT_Kd_min = 0.2, FORT_Kd_max = 0.2;

// LMS Parameters for FORT
float FORT_mu = 0.001;
float FORT_momentum = 0.9;
float FORT_damping_factor = 0.8;
float FORT_theta[] = {FORT_Kp, FORT_Ki, FORT_Kd};
float FORT_theta_velocity[] = {0, 0, 0};

// Control Variables for FORT
float FORT_error, FORT_lastError = 0;
float FORT_I_term = 0;
float FORT_lastD_term = 0;
float FORT_maxOutput = 40;
float FORT_minOutput = -45;
float FORT_I_term_limit = 8.0;
float FORT_lastOutput = 0.0;

// Kalman Filter Variables for FORT
float FORT_Q = 0.1;
float FORT_R = 5.0;
float FORT_P = 1.0;
float FORT_K_gain = 0.0;
const int FORT_MAX_ANGLE = 15;
const int FORT_MIN_ANGLE = -30;

const int LaunchPin = 54; //發射砲彈腳位
int LaunchKeepTime = 0;

unsigned long now = 0;//用於計算兩次讀取的間隔 單位 us
unsigned long last = 0;
double duration =0;
// ===============================
// Setup Function
// ===============================
void setup() {
    // Initialize BASE Motor Pins
    pinMode(BASE_pinA, INPUT_PULLUP);
    pinMode(BASE_pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BASE_pinA), BASE_updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BASE_pinB), BASE_updateEncoder, CHANGE);
    pinMode(BASE_m_A, OUTPUT);
    pinMode(BASE_m_B, OUTPUT);
    pinMode(BASE_ms, INPUT_PULLUP);

    // Initialize FORT Motor Pins
    pinMode(FORT_pinA, INPUT_PULLUP);
    pinMode(FORT_pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FORT_pinA), FORT_updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FORT_pinB), FORT_updateEncoder, CHANGE);
    pinMode(FORT_m_A, OUTPUT);
    pinMode(FORT_m_B, OUTPUT);
    pinMode(FORT_ms, INPUT_PULLUP);
    pinMode(LaunchPin,OUTPUT);

    // Initialize Serial Communication
    Serial.begin(57600);
    Serial.setTimeout(25);
}



// ===============================
// Reset Function for BASE
// ===============================
void BASE_resetVariables() {
    BASE_encoderPos = 0;
    BASE_lastEncoded = 0;
    BASE_targetAngle = 0;
    BASE_filteredAngle = 0.0;
    BASE_lastError = 0;
    BASE_I_term = 0;
    BASE_lastD_term = 0;
    BASE_lastOutput = 0.0;
    BASE_P = 1.0;
    BASE_theta[0] = BASE_Kp;
    BASE_theta[1] = BASE_Ki;
    BASE_theta[2] = BASE_Kd;
    for (int i = 0; i < 3; i++) {
        BASE_theta_velocity[i] = 0;
    }
}

// ===============================
// Reset Function for FORT
// ===============================
void FORT_resetVariables() {
    FORT_encoderPos = 0;
    FORT_lastEncoded = 0;
    FORT_targetAngle = 0;
    FORT_filteredAngle = 0.0;
    FORT_lastError = 0;
    FORT_I_term = 0;
    FORT_lastD_term = 0;
    FORT_lastOutput = 0.0;
    FORT_P = 1.0;
    FORT_theta[0] = FORT_Kp;
    FORT_theta[1] = FORT_Ki;
    FORT_theta[2] = FORT_Kd;
    for (int i = 0; i < 3; i++) {
        FORT_theta_velocity[i] = 0;
    }
}


// ===============================
// Reset Motion for BASE
// ===============================
void BASE_resetMotions() {
    Status = 1;
    printStatus();
    while(1){
        analogWrite(BASE_m_A, 0);
        analogWrite(BASE_m_B, 0);
        int val = digitalRead(BASE_ms);
        if (val == LOW)
            break;
    }
    delay(1000);
    float c = 30;
    while(1){
        c = constrain(c,0,70);
        analogWrite(BASE_m_A, 0);
        analogWrite(BASE_m_B, c);
        int val = digitalRead(BASE_ms);
        if (val == LOW)
            break;
        c+=0.0005;
    }

    analogWrite(BASE_m_A, 0);
    analogWrite(BASE_m_B, 0);
    BASE_resetVariables();
    // Set a target angle after reset
    BASE_targetAngle = 71; // Set to the desired angle
    //Serial.println("Rotating to 90 degrees...");
    BASE_maxOutput = 45;
    // Continuously adjust the motor until it reaches the target angle
    while (abs(BASE_filteredAngle - BASE_targetAngle) > 1) {  // Adjust tolerance as needed
        float BASE_currentAngle = BASE_encoderPos * BASE_k;
        // Update the Kalman filter for angle estimation
        updateKalmanFilter_BASE(BASE_currentAngle);
        // Control the motor to move towards the target angle
        controlMotorToAngle_BASE(BASE_filteredAngle, BASE_targetAngle);
        delay(1); // Small delay to prevent excessive motor adjustment
    }
    BASE_maxOutput = 75;
    BASE_resetVariables();
    Status = 0;
    printStatus();
}

// ===============================
// Reset Motion for FORT
// ===============================
void FORT_resetMotions() {
    Status = 2;
    printStatus();
    while(1){
        analogWrite(FORT_m_A, 0);
        analogWrite(FORT_m_B, 0);
        int val = digitalRead(FORT_ms);
        if (val == LOW)
            break;
    }
    delay(1000);
    float c = 10;
    while(1){
        c = constrain(c,0,50);
        analogWrite(FORT_m_A, c);
        analogWrite(FORT_m_B, 0);
        int val = digitalRead(FORT_ms);
        if (val == LOW)
            break;
        c+=0.005;
    }

    analogWrite(FORT_m_A, 0);
    analogWrite(FORT_m_B, 0);
    FORT_resetVariables();
    // Set a target angle after reset
    FORT_targetAngle = -20; // Set to the desired angle for FORT
    //Serial.println("FORT rotating to 90 degrees...");
    // Continuously adjust the motor until it reaches the target angle
    while (abs(FORT_filteredAngle - FORT_targetAngle) > 3.5) {  // Adjust tolerance as needed
        float FORT_currentAngle = FORT_encoderPos * FORT_k;
//        Serial.println(abs(FORT_filteredAngle - FORT_targetAngle));
        // Update the Kalman filter for angle estimation
        updateKalmanFilter_FORT(FORT_currentAngle);
        // Control the motor to move towards the target angle
        controlMotorToAngle_FORT(FORT_filteredAngle, FORT_targetAngle);
        delay(1); // Small delay to prevent excessive motor adjustment
    }
    FORT_resetVariables();
    Status = 0;
    printStatus();
}


void Launch(){
   if (LaunchKeepTime>0){
      digitalWrite(LaunchPin,HIGH); 
      LaunchKeepTime-=1;
   }
   else{
    digitalWrite(LaunchPin,LOW);  
   }
}

// ===============================
// Main Loop
// ===============================
void loop() {
    now = micros();
    duration = (now-last)*1e-6;
    // Read Serial Input
    String input = readSerialInput();
    Launch();
  
    // Process Command
    if (input.length() > 0) {
        processSerialCommand(input);
    }
    FORT_targetAngle = constrain(FORT_targetAngle, FORT_MIN_ANGLE, FORT_MAX_ANGLE);
    BASE_targetAngle = constrain(BASE_targetAngle, BASE_MIN_ANGLE, BASE_MAX_ANGLE);
    // Update Angles and Control Motors
    BASE_currentAngle = BASE_encoderPos * BASE_k;
    FORT_currentAngle = FORT_encoderPos * FORT_k;

    // Kalman Filter Updates
    updateKalmanFilter_BASE(BASE_currentAngle);
    updateKalmanFilter_FORT(FORT_currentAngle);

    // Control Motors
    controlMotorToAngle_BASE(BASE_filteredAngle, BASE_targetAngle);
    controlMotorToAngle_FORT(FORT_filteredAngle, FORT_targetAngle);
    last = now;
    if (StatusCount>60){
        printStatus();
        StatusCount=0;
    }
    else
        StatusCount++;
    delay(1); // Small delay to prevent excessive serial communication
}

// ===============================
// Function to Read Serial Input
// ===============================
String readSerialInput() {
    String input = "";
    if (Serial.available() > 0) {
        input = Serial.readString();
        input.trim(); // Remove any trailing newline or spaces
    }
    return input;
}

// ===============================
// Function to Process Serial Command
// ===============================
void printStatus(){
    /*
      status,b_t,b_c,f_t,f_c,duration
    */
    Serial.print(Status);
    Serial.print(",");
    Serial.print(BASE_targetAngle);
    Serial.print(",");
    Serial.print(BASE_currentAngle);
    Serial.print(",");
    Serial.print(FORT_targetAngle);
    Serial.print(",");
    Serial.print(FORT_currentAngle);
    Serial.print(",");
    Serial.println(duration);
}
void processSerialCommand(String input) {
    if (input.startsWith("SET_BASE:")) {
        int angle = input.substring(9).toInt();
        BASE_targetAngle = constrain(angle, -720, 720);
        BASE_I_term = 0;
        BASE_lastOutput = 0;
        //Serial.println("BASE angle set to " + String(BASE_targetAngle));
    }
    else if (input.startsWith("RESET_BASE")) {
        BASE_resetVariables();
        //Serial.println("BASE system reset completed.");
    }
    else if (input.startsWith("SET_FORT:")) {
        int angle = input.substring(9).toInt();
        FORT_targetAngle = constrain(angle, -720, 720);
        FORT_lastOutput = 0;
        //Serial.println("FORT angle set to " + String(FORT_targetAngle));
    }
    else if (input.startsWith("RESET_FORT")) {
        FORT_resetVariables();
        //Serial.println("FORT system reset completed.");
    }
    else if (input == "RESET_MS_BASE") {
        BASE_resetMotions();  // Call the function to reset BASE motor
        //Serial.println("BASE motor reset motion completed.");
    }
    else if (input == "RESET_MS_FORT") {
        FORT_resetMotions();  // Call the function to reset FORT motor
        //Serial.println("FORT motor reset motion completed.");
    }
    else if (input == "LAUNCH") {
        LaunchKeepTime = 1000;
        //Serial.println("LAUNCH");
    }
}


// ===============================
// Kalman Filter Update Functions
// ===============================
void updateKalmanFilter_BASE(float currentAngle) {
    BASE_P += BASE_Q;
    BASE_K_gain = BASE_P / (BASE_P + BASE_R);
    BASE_filteredAngle += BASE_K_gain * (currentAngle - BASE_filteredAngle);
    BASE_P *= (1 - BASE_K_gain);
}

void updateKalmanFilter_FORT(float currentAngle) {
    FORT_P += FORT_Q;
    FORT_K_gain = FORT_P / (FORT_P + FORT_R);
    FORT_filteredAngle += FORT_K_gain * (currentAngle - FORT_filteredAngle);
    FORT_P *= (1 - FORT_K_gain);
}


// ===============================
// Encoder Interrupt Handlers
// ===============================

// BASE Encoder Interrupt
void BASE_updateEncoder() {
    int MSB = digitalRead(BASE_pinA);
    int LSB = digitalRead(BASE_pinB);
    int encoded = (MSB << 1) | LSB;
    int sum = (BASE_lastEncoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) BASE_encoderPos++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) BASE_encoderPos--;

    BASE_lastEncoded = encoded;
}

// FORT Encoder Interrupt
void FORT_updateEncoder() {
    int MSB = digitalRead(FORT_pinA);
    int LSB = digitalRead(FORT_pinB);
    int encoded = (MSB << 1) | LSB;
    int sum = (FORT_lastEncoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) FORT_encoderPos++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) FORT_encoderPos--;

    FORT_lastEncoded = encoded;
}

// ===============================
// Control Motor to Target Angle - BASE
// ===============================
void controlMotorToAngle_BASE(float currentAngle, float targetAngle) {
    BASE_error = targetAngle - currentAngle;
    
    float dError = (BASE_error - BASE_lastError)/duration;
    BASE_I_term += BASE_error*duration;
    BASE_I_term = constrain(BASE_I_term, -BASE_I_term_limit, BASE_I_term_limit);

    float output = BASE_theta[0] * BASE_error + BASE_I_term*BASE_theta[1] + dError*BASE_theta[2];

    output = constrain(output, -BASE_maxOutput, BASE_maxOutput);

    if (output > 0) {
        analogWrite(BASE_m_A, abs(output));
        digitalWrite(BASE_m_B, LOW);
    }
    else {
        digitalWrite(BASE_m_A, LOW);
        analogWrite(BASE_m_B, abs(output));
    }

    BASE_lastError = BASE_error;

    if (abs(BASE_error) > 5) {
        float phi[] = { -BASE_error, -BASE_I_term, -dError };
        for (int i = 0; i < 3; i++) {
            BASE_theta_velocity[i] = BASE_momentum * BASE_theta_velocity[i] + BASE_mu * BASE_error * phi[i];
            BASE_theta[i] += BASE_damping_factor * BASE_theta_velocity[i];
            BASE_theta[i] = constrain(BASE_theta[i],
                (i == 0) ? BASE_Kp_min : (i == 1) ? BASE_Ki_min : BASE_Kd_min,
                (i == 0) ? BASE_Kp_max : (i == 1) ? BASE_Ki_max : BASE_Kd_max);
        }
    }
}

// ===============================
// Control Motor to Target Angle - FORT
// ===============================
void controlMotorToAngle_FORT(float currentAngle, float targetAngle) {
    FORT_error = targetAngle - currentAngle;

    float dError = (FORT_error - FORT_lastError)/duration;
    FORT_I_term += FORT_error * duration;
    FORT_I_term = constrain(FORT_I_term, -FORT_I_term_limit, FORT_I_term_limit);

    float output = FORT_theta[0] * FORT_error + FORT_I_term*FORT_theta[1] + dError * FORT_theta[2];

    output = constrain(output, FORT_minOutput, FORT_maxOutput);
    
    if (output > 0) {
        analogWrite(FORT_m_A, abs(output));
        digitalWrite(FORT_m_B, LOW);
    }
    else {
        digitalWrite(FORT_m_A, LOW);
        analogWrite(FORT_m_B, abs(output));
    }

    FORT_lastError = FORT_error;
    if (abs(FORT_error) > 5) {
        float phi[] = { -FORT_error, -FORT_I_term, -dError };
        for (int i = 0; i < 3; i++) {
            FORT_theta_velocity[i] = FORT_momentum * FORT_theta_velocity[i] + FORT_mu * FORT_error * phi[i];
            FORT_theta[i] += FORT_damping_factor * FORT_theta_velocity[i];
            FORT_theta[i] = constrain(FORT_theta[i],
                (i == 0) ? FORT_Kp_min : (i == 1) ? FORT_Ki_min : FORT_Kd_min,
                (i == 0) ? FORT_Kp_max : (i == 1) ? FORT_Ki_max : FORT_Kd_max);
        }
    }
}

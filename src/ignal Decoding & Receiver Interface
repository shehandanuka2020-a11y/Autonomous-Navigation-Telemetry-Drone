
 * Component: Multi-Channel Signal Decoding & Receiver Interface
 * Purpose: Processing RC Inputs (SBUS/IBUS) with Failsafe Protection
 * project: Danuka Shehan
 */

#include <IBusBM.h> // Library for Digital Signal Decoding

// --- Object Initialization ---
IBusBM ibus; 

// --- RC Channel Variables ---
struct RC_Channels {
    int throttle; // Thrust Control
    int roll;     // Horizontal Tilt
    int pitch;    // Vertical Tilt
    int yaw;      // Rotation
    int arm_switch; // Safety Switch (AUX 1)
    int mode_switch; // Flight Mode (GPS/Manual)
};

RC_Channels remote;

// --- Signal Constants ---
const int MIN_PULSE = 1000;
const int MAX_PULSE = 2000;
const int MID_PULSE = 1500;

void setup() {
    // Standard Baud Rate for Digital Receivers
    Serial.begin(115200); 
    
    // Initializing the Receiver on Hardware Serial 1 (RX1)
    ibus.begin(Serial1); 
    
    Serial.println(">>> VECTOR-X RECEIVER ENGINE INITIALIZED...");
}

void loop() {
    // 1. DATA ACQUISITION
    // Reading 6 channels from the transmitter
    remote.roll       = ibus.readChannel(0);
    remote.pitch      = ibus.readChannel(1);
    remote.throttle   = ibus.readChannel(2);
    remote.yaw        = ibus.readChannel(3);
    remote.arm_switch = ibus.readChannel(4);
    remote.mode_switch = ibus.readChannel(5);

    // 2. SIGNAL VALIDATION & FAILSAFE
    // If signal is lost or arm switch is low, shut down everything
    if (isSignalLost() || remote.arm_switch < 1500) {
        applyEmergencyCutoff();
    } else {
        processFlightCommands();
    }

    // 3. TELEMETRY OUTPUT (For debugging in Serial Monitor)
    printTelemetry();

    delay(10); // Running at 100Hz (Industry standard for RX input)
}

// --- CORE SYSTEM FUNCTIONS ---

bool isSignalLost() {
    // If the pulse is outside the standard 1000-2000 range, 
    // it means the transmitter is disconnected.
    if (remote.throttle < 900 || remote.throttle > 2100) return true;
    return false;
}

void processFlightCommands() {
    // Normalize Raw PWM to Target Degrees for PID Loops
    float target_roll  = map(remote.roll, MIN_PULSE, MAX_PULSE, -35, 35);
    float target_pitch = map(remote.pitch, MIN_PULSE, MAX_PULSE, -35, 35);
    
    // Log the converted data for Flight Controller
    Serial.print("SYSTEM ARMED | TARGET_ROLL: ");
    Serial.println(target_roll);
}

void applyEmergencyCutoff() {
    // Force all motor speeds to ZERO immediately
    // This part links to the Master Controller's motor pins
    for (int i = 10; i <= 13; i++) {
        analogWrite(i, 0);
    }
    Serial.println("!!! WARNING: SYSTEM DISARMED / SIGNAL LOST !!!");
}

void printTelemetry() {
    // Professional formatting for real-time monitoring
    Serial.print("CH1:"); Serial.print(remote.roll);
    Serial.print(" | CH2:"); Serial.print(remote.pitch);
    Serial.print(" | CH3:"); Serial.println(remote.throttle);
}

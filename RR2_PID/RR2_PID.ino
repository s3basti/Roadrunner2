/* Roadrunner PID Controller
    ------------------------
   Eric Sanchez 9/26/2018
   -------------------------
   Communicates with RPi and gets sensor information. Locally
   runs PID loop using PID library from Brett Beaugard
*/

#include <PID_v1.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// Variables passed from RPi
double Setpoint, Input, Output;
bool setptOn = false;

// Loop Speed
static int pidFreq = 1; // ms 1000 Hz
static int ioFreq = 10; // ms 100 Hz

// PID Gains
double Kp = .4, Ki = 2.75, Kd = .04;

// Initialize vars
unsigned long prevTime = 0;
unsigned long currTime = 0;

double prevSetpoint = 0.0;
bool prevSetptOn = false;

bool write_;
float mapped_out = 0.0;
double esc_min = 0.0, esc_max = 15.0;

char buf[32];
int readSize;

// Initialize PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Functions ---------------
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  // Initialize vars
  prevTime = millis();

  // Set PID sample time
  myPID.SetSampleTime(pidFreq);
  myPID.SetOutputLimits(esc_min,esc_max);

  Serial.begin(9600);
}

void loop() {
  
  write_ = myPID.Compute();

  if (write_) {
    
  }

  // Grab/send data loop
  currTime = millis();

  if ((currTime - prevTime) >= ioFreq) {
    
    // Send Data
    Serial.write(mapped_out);
    
    // Grab Data
    if (Serial.available()) {
      // Read buffer until it reaches a new line character
      readSize = Serial.readBytesUntil('\n', buf, 32);
      if (readSize != 0) {
        bool eh = true;
      }
    }
  }

  currTime = prevTime;
}

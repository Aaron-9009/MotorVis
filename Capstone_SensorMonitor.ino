 
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
//Ensure Libraries Added
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <PulseSensorPlayground.h>

//Buad Rate 115200
 
//Pins
#define PULSE_SENSOR_PIN        34     
 
//Pulse Sensor
#define PULSE_THRESHOLD         550
 
//Heart Rate Safety Window
#define HR_MIN_SAFE             50      // Bradycardia alert
#define HR_MAX_SAFE             120     // Tachycardia alert
 
// Motion sensitivity
//   4.0  = moderate   (catches sharp jolts / falls)
//   8.0  = aggressive (only extreme impacts)
#define ACCEL_DELTA_THRESHOLD   4.0f
 
// Accelerometer Sample Rate
#define ACCEL_SAMPLE_MS         100
 
 
enum class AlertType
{
    HEART_RATE_LOW,         // BPM dropped below HR_MIN_SAFE
    HEART_RATE_HIGH,        // BPM rose above HR_MAX_SAFE
    MOTION_SUDDEN_CHANGE    // Acceleration delta exceeded ACCEL_DELTA_THRESHOLD
};
 
struct Alert
{
    AlertType type;
 
    // Heart-rate alerts: current BPM (0 for motion alerts)
    int   bpm;
 
    // Motion alerts: magnitude of the acceleration change vector
    float accelDelta;
 
    // Motion alerts: accelerometer snapshot at the moment of the alert
    float ax;
    float ay;
    float az;
};
 
 
//Sensor
PulseSensorPlayground    pulseSensor;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
 
 
// States
static float         s_prevAx          = 0.0f;
static float         s_prevAy          = 0.0f;
static float         s_prevAz          = 0.0f;
static bool          s_firstAccelRead  = true;
static unsigned long s_lastAccelTime   = 0;
 
 
// Declarations
void handleAlert(const Alert& alert);
void checkMotionAlert(float ax, float ay, float az);
void readAndCheckAccel();
void readAndCheckHeartRate();
 

void handleAlert(const Alert& alert)
{
    switch (alert.type)
    {
        // --------------------------------------------------
        case AlertType::HEART_RATE_LOW:
            Serial.println();
            Serial.println("Alert: Heart Rate Too Low");
            Serial.print  ("  BPM : ");
            Serial.println(alert.bpm);
            Serial.println();

            // ---- INSERT LOW-HEART-RATE RESPONSE CODE HERE ----
            break;
 
        // --------------------------------------------------
        case AlertType::HEART_RATE_HIGH:
            Serial.println();
            Serial.println("Alert: Heart Rate Too High");
            Serial.print  ("  BPM : ");
            Serial.println(alert.bpm);
            Serial.println();
 
            // ---- INSERT HIGH-HEART-RATE RESPONSE CODE HERE ----
            break;
 
        // --------------------------------------------------
        case AlertType::MOTION_SUDDEN_CHANGE:
            Serial.println();
            Serial.println("Alert: Crash Detected");
            Serial.println();
 
            // ---- INSERT MOTION ALERT RESPONSE CODE HERE ----
            break;
    }
}
 
void checkMotionAlert(float ax, float ay, float az)
{
    if (s_firstAccelRead)
    {
        s_prevAx         = ax;
        s_prevAy         = ay;
        s_prevAz         = az;
        s_firstAccelRead = false;
        return;
    }
 
    float dx    = ax - s_prevAx;
    float dy    = ay - s_prevAy;
    float dz    = az - s_prevAz;
    float delta = sqrtf(dx*dx + dy*dy + dz*dz);
 
    if (delta > ACCEL_DELTA_THRESHOLD)
    {
        Alert a;
        a.type       = AlertType::MOTION_SUDDEN_CHANGE;
        a.bpm        = 0;
        a.accelDelta = delta;
        a.ax         = ax;
        a.ay         = ay;
        a.az         = az;
        handleAlert(a);
    }
 
    s_prevAx = ax;
    s_prevAy = ay;
    s_prevAz = az;
}
 
 
// Accelerometer
void readAndCheckAccel()
{
    sensors_event_t event;
    accel.getEvent(&event);
 
    float ax = event.acceleration.x;
    float ay = event.acceleration.y;
    float az = event.acceleration.z;
 
    Serial.print("[ACCEL]  X: ");
    Serial.print(ax, 3);
    Serial.print("   Y: ");
    Serial.print(ay, 3);
    Serial.print("   Z: ");
    Serial.print(az, 3);
    Serial.println("  m/s²");
 
    checkMotionAlert(ax, ay, az);
}
 
 
// Heart Rate
 
void readAndCheckHeartRate()
{
    if (!pulseSensor.sawStartOfBeat())
        return;
 
    int bpm = pulseSensor.getBeatsPerMinute();
 
    Serial.print("[HEART]  BPM: ");
    Serial.println(bpm);
 
    if (bpm < HR_MIN_SAFE)
    {
        Alert a;
        a.type       = AlertType::HEART_RATE_LOW;
        a.bpm        = bpm;
        a.accelDelta = 0.0f;
        a.ax = a.ay = a.az = 0.0f;
        handleAlert(a);
    }
    else if (bpm > HR_MAX_SAFE)
    {
        Alert a;
        a.type       = AlertType::HEART_RATE_HIGH;
        a.bpm        = bpm;
        a.accelDelta = 0.0f;
        a.ax = a.ay = a.az = 0.0f;
        handleAlert(a);
    }
}
 
 
//Set Up
 
void setup()
{
    Serial.begin(115200);
    delay(500);
 
    pulseSensor.analogInput(PULSE_SENSOR_PIN);
    pulseSensor.setThreshold(PULSE_THRESHOLD);
 
    if (pulseSensor.begin())
        Serial.println("[INIT]  PulseSensor  OK");
    else
        Serial.println("[INIT]  PulseSensor  FAILED — check wiring on GPIO 34");
 
    if (!accel.begin())
    {
        Serial.println("[INIT]  ADXL345  NOT FOUND");
        Serial.println("        Verify: CS -> 3.3V,  SDO -> GND,  SDA -> GPIO21,  SCL -> GPIO22");
        while (true) { delay(10); }
    }
 
    accel.setRange(ADXL345_RANGE_2_G);
 
    Serial.println("[INIT]  ADXL345  OK");
    Serial.println("[INIT]  Monitoring started.\n");
}
 
// Main 
void loop()
{
    // Heart rate
    readAndCheckHeartRate();
 
    // Accelerometer
    if (millis() - s_lastAccelTime >= ACCEL_SAMPLE_MS)
    {
        s_lastAccelTime = millis();
        readAndCheckAccel();
    }
}

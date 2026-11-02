#include <Wire.h>
#include <Arduino.h>
#include "MAX30100_PulseOximeter.h"
#include <MPU6050_light.h>
#include <math.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
#define Serial SerialBT

const int ECG_PIN = 34;
const int NTC_PIN = 35;
const float NTC_NOMINAL_RESISTANCE = 100000.0;
const float FIXED_RESISTOR = 100000.0;
const float NTC_NOMINAL_TEMPERATURE = 25.0;
const float NTC_BETA = 3950.0;
const float ADC_VOLTAGE = 3.3;
const float ADC_MAX = 4095.0;

#define MAX_SDA 25
#define MAX_SCL 26
#define MPU_SDA 21
#define MPU_SCL 22

TwoWire MPUWire = TwoWire(1);
PulseOximeter pox;
MPU6050 mpu(MPUWire);

#define REPORTING_PERIOD_MS 1000
uint32_t lastSensorReport = 0;

float accelerationX = 0, accelerationY = 0, accelerationZ = 0, totalAcceleration = 0;
bool fallDetected = false;
const float FALL_ACCELERATION_THRESHOLD = 1.0;

const int SAMPLING_FREQ = 250;
const int SAMPLING_INTERVAL_US = 1000000 / SAMPLING_FREQ;
const int PEAK_DETECTION_WINDOW = 40;
const float MIN_HEART_RATE_BPM = 40.0;
const float MAX_HEART_RATE_BPM = 180.0;
const float R_PEAK_THRESHOLD_PERCENT = 0.65;
const float ALPHA_BASELINE = 0.01;
const int HR_AVG_WINDOW = 5;

float baseline = 2048, filteredSignal = 0, previousFilteredSignal = 0;
unsigned long lastSampleTime = 0, lastBeatTime = 0, beatInterval = 0;
float currentHeartRate = 0, filteredHeartRate = 0, peakAmplitude = 0, rPeakThreshold = 0;
unsigned long lastPeakTime = 0, refractoryPeriod = 0;
const unsigned long REFRACTORY_MS = 250;
float heartRateHistory[HR_AVG_WINDOW];
int hrHistoryIndex = 0, hrHistoryCount = 0;

enum PeakState { WAITING_FOR_RISING, DETECTING_PEAK, WAITING_FOR_REFRACTORY };
PeakState peakState = WAITING_FOR_RISING;
float peakValue = 0;
unsigned long peakStartTime = 0;

const float MEAN = 88.25;
const float STD = 52.56;
const float WEIGHTS[3] = { -7.78, -0.35, 7.84 };
const float INTERCEPTS[3] = { -5.70, -1.01, -1.77 };
const char* CLASSES[3] = { "BRADYCARDIA", "NORMAL", "TACHYCARDIA" };

float sigmoid(float x) { return 1.0 / (1.0 + exp(-x)); }

int predict(float heart_rate) {
  float normalized = (heart_rate - MEAN) / STD;
  float probs[3], total = 0;
  for (int i = 0; i < 3; i++) {
    probs[i] = sigmoid(WEIGHTS[i] * normalized + INTERCEPTS[i]);
    total += probs[i];
  }
  for (int i = 0; i < 3; i++) probs[i] /= total;
  int best = 0;
  for (int i = 1; i < 3; i++) if (probs[i] > probs[best]) best = i;
  return best;
}

void onBeatDetected() {}

void setupADC() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
}

float removeBaseline(int rawValue) {
  baseline = ALPHA_BASELINE * rawValue + (1 - ALPHA_BASELINE) * baseline;
  float filtered = rawValue - baseline;
  static float smoothingBuffer[5];
  static int smoothIndex = 0;
  smoothingBuffer[smoothIndex] = filtered;
  smoothIndex = (smoothIndex + 1) % 5;
  float smoothSum = 0;
  for (int i = 0; i < 5; i++) smoothSum += smoothingBuffer[i];
  return smoothSum / 5.0;
}

float calculateHeartRate(unsigned long interval) {
  if (interval > 0 && interval < 2000) return 60000.0 / interval;
  return 0;
}

void updateHeartRateAverage(float newHR) {
  if (newHR <= 0) return;
  heartRateHistory[hrHistoryIndex] = newHR;
  hrHistoryIndex = (hrHistoryIndex + 1) % HR_AVG_WINDOW;
  if (hrHistoryCount < HR_AVG_WINDOW) hrHistoryCount++;
  float sum = 0;
  for (int i = 0; i < hrHistoryCount; i++) sum += heartRateHistory[i];
  filteredHeartRate = sum / hrHistoryCount;
}

void detectRPeak(float value) {
  float absValue = abs(value);
  if (absValue > peakAmplitude) peakAmplitude = absValue;
  peakAmplitude *= 0.998;
  rPeakThreshold = peakAmplitude * R_PEAK_THRESHOLD_PERCENT;

  switch (peakState) {
    case WAITING_FOR_RISING:
      if (value > rPeakThreshold && value > previousFilteredSignal) {
        peakState = DETECTING_PEAK;
        peakValue = value;
        peakStartTime = millis();
      }
      break;
    case DETECTING_PEAK:
      if (value > peakValue) {
        peakValue = value;
        peakStartTime = millis();
      }
      if ((value < peakValue * 0.7 && millis() - peakStartTime > 20) || (millis() - peakStartTime > PEAK_DETECTION_WINDOW)) {
        unsigned long currentTime = millis();
        if (currentTime - refractoryPeriod > REFRACTORY_MS) {
          if (lastBeatTime > 0) {
            beatInterval = currentTime - lastBeatTime;
            float instantHR = calculateHeartRate(beatInterval);
            if (instantHR >= MIN_HEART_RATE_BPM && instantHR <= MAX_HEART_RATE_BPM) {
              currentHeartRate = instantHR;
              updateHeartRateAverage(currentHeartRate);
            }
          }
          lastBeatTime = currentTime;
          refractoryPeriod = currentTime;
        }
        peakState = WAITING_FOR_REFRACTORY;
      }
      break;
    case WAITING_FOR_REFRACTORY:
      if (millis() - refractoryPeriod > REFRACTORY_MS) peakState = WAITING_FOR_RISING;
      break;
  }
}

void initializeMPU() {
  MPUWire.begin(MPU_SDA, MPU_SCL);
  MPUWire.setClock(100000);
  byte status = mpu.begin();
  if (status != 0) while (1) delay(1000);
}

void checkFallDetection() {
  accelerationX = mpu.getAccX();
  accelerationY = mpu.getAccY();
  accelerationZ = mpu.getAccZ();
  totalAcceleration = sqrt((accelerationX * accelerationX) + (accelerationY * accelerationY) + (accelerationZ * accelerationZ));
  if (totalAcceleration > FALL_ACCELERATION_THRESHOLD) fallDetected = true;
}

float readTemperature() {
  int adcValue = analogRead(NTC_PIN);
  if (adcValue <= 0 || adcValue >= ADC_MAX) return -999.0;
  float voltage = (adcValue / ADC_MAX) * ADC_VOLTAGE;
  float resistance = FIXED_RESISTOR * voltage / (ADC_VOLTAGE - voltage);
  float steinhart = resistance / NTC_NOMINAL_RESISTANCE;
  steinhart = log(steinhart);
  steinhart /= NTC_BETA;
  steinhart += 1.0 / (NTC_NOMINAL_TEMPERATURE + 273.15);
  steinhart = 1.0 / steinhart;
  return steinhart - 273.15;
}

void serialPrintStatus() {}

void printSensorStatus() {}

void setup() {
  SerialBT.begin("ESP32_ECG_AI");
  delay(1000);
  Wire.begin(MAX_SDA, MAX_SCL);
  Wire.setClock(100000);
  setupADC();
  analogReadResolution(12);
  analogSetPinAttenuation(NTC_PIN, ADC_11db);
  for (int i = 0; i < HR_AVG_WINDOW; i++) heartRateHistory[i] = 0;
  if (!pox.begin()) while (1) delay(1000);
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);
  initializeMPU();
  lastSampleTime = micros();
  lastBeatTime = millis();
  lastSensorReport = millis();
}

void loop() {
  pox.update();
  mpu.update();
  checkFallDetection();

  unsigned long currentMicros = micros();
  if (currentMicros - lastSampleTime >= SAMPLING_INTERVAL_US) {
    lastSampleTime = currentMicros;
    int rawECG = analogRead(ECG_PIN);
    filteredSignal = removeBaseline(rawECG);
    detectRPeak(filteredSignal);
    previousFilteredSignal = filteredSignal;
  }

  if (millis() - lastSensorReport >= REPORTING_PERIOD_MS) {
    lastSensorReport = millis();
    printSensorStatus();
  }
}
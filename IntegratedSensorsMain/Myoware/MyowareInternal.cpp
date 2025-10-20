#include <Arduino.h>
#include <Wire.h>
namespace MyoInternal {
  using ::Wire;
  using ::TwoWire;
  using ::Serial;
  using ::micros;
  using ::millis;
  using ::delay;
  
const int MYO_PIN = A0;      // pin 14
const int LED_PIN = 13;      

// Sampling
const unsigned long SAMPLE_PERIOD_MS = 10; // 100 Hz
const float DT_SEC = SAMPLE_PERIOD_MS / 1000.0f;

// Smoothing 
const float TAU_SEC = 0.10f;             // ~100 ms smoothing
const float ALPHA   = TAU_SEC / (TAU_SEC + DT_SEC);

// Debounce & hold times
const int ENTER_SAMPLES = 5;             // 50 ms
const int EXIT_SAMPLES  = 12;            // 120 ms
const unsigned long LIFT_HOLD_MS = 250;  // need this long above thr_lift_emg

// Threshold model
const float K_STD        = 4.0f;         
const int   MARGIN_CNTS  = 60;           
const float HYST_FACTOR  = 0.6f;         

// Air-curl capture
const unsigned long AIRCURL_MS = 10000;  // 10 s capture
const int AIRCURL_MAX_SAMPLES  = 1200;   // 12 s 

// MVC capture (optional)
const unsigned long MVC_MS = 5000;       // 5 s capture
const int MVC_MAX_SAMPLES  = 600;

// ADC (for info only)
const float ADC_VREF = 3.3f;
const int   ADC_MAX  = 4095;

// -------- State Machine --------
enum Phase { CAL_REST, CAL_AIRCURL_READY, CAL_AIRCURL_RUN, CAL_MVC_QUERY, CAL_MVC_RUN, RUN };
Phase phase = CAL_REST;

// Timing
unsigned long lastSampleMs = 0;
unsigned long phaseStartMs = 0;

// Signal
int   raw = 0;
float smooth = 0.0f;

// REST stats (Welford)
unsigned long restN = 0;
double restMean = 0.0, restM2 = 0.0;

// AIRCURL buffer
float airBuf[AIRCURL_MAX_SAMPLES];
int   airN = 0;

// MVC buffer (optional)
float mvcBuf[MVC_MAX_SAMPLES];
int   mvcN = 0;
bool  doMVC = false;

// Thresholds
float mu_rest = 0.0f, std_rest = 0.0f;
float thr_on = 0.0f, thr_dn = 0.0f;
float P95_air = 0.0f, MVC95 = 0.0f;
float thr_lift_emg = 0.0f;

// Runtime flags
bool active = false;
int  enter_ctr = 0, exit_ctr = 0;

bool emg_lift = false;
unsigned long liftStartMs = 0;

// ---- Helpers ----
void resetRestStats() {
  restN = 0; restMean = 0.0; restM2 = 0.0;
}

void welfordAdd(double x) {
  restN++;
  double delta = x - restMean;
  restMean += delta / (double)restN;
  double delta2 = x - restMean;
  restM2 += delta * delta2;
}

float computeStd() {
  if (restN < 2) return 5.0f;
  double var = restM2 / (double)(restN - 1);
  if (var < 0) var = 0;
  return (float)sqrt(var);
}

float percentile95(float* arr, int n) {
  if (n <= 0) return 0.0f;
  static float tmp[AIRCURL_MAX_SAMPLES]; 
  for (int i=0; i<n; ++i) tmp[i] = arr[i];
  for (int i=1; i<n; ++i) {
    float key = tmp[i]; int j = i-1;
    while (j >= 0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
    tmp[j+1] = key;
  }
  int idx = (int)floorf(0.95f * (n - 1));
  if (idx < 0) idx = 0;
  if (idx >= n) idx = n-1;
  return tmp[idx];
}

void printDivider() {
  Serial.println(F("\n---------------------------------------------"));
}

void instruct(const __FlashStringHelper* s) {
  printDivider(); Serial.println(s); printDivider();
}

void setThresholds() {
  mu_rest = (float)restMean;
  std_rest = computeStd();

  thr_on = mu_rest + K_STD * std_rest;
  if (thr_on < mu_rest + MARGIN_CNTS) thr_on = mu_rest + MARGIN_CNTS;
  thr_dn = mu_rest + HYST_FACTOR * (thr_on - mu_rest);

  float base = 1.20f * P95_air;        // 20% above air-curl
  if (doMVC && MVC95 > 0.0f) {
    float mvcFloor = 0.25f * MVC95;    // 25% of MVC
    if (base < mvcFloor) base = mvcFloor;
  }

  // prevent  high thresholds
  float maxRise = 400.0f; 
  if (base > mu_rest + maxRise) base = mu_rest + maxRise;

  thr_lift_emg = base;

  // Summary
  Serial.println(F("\n=== Calibration Results ==="));
  Serial.print(F("Rest mean: ")); Serial.println(mu_rest, 1);
  Serial.print(F("Rest std : ")); Serial.println(std_rest, 1);
  Serial.print(F("P95_air  : ")); Serial.println(P95_air, 1);
  if (doMVC) { Serial.print(F("MVC95    : ")); Serial.println(MVC95, 1); }
  Serial.print(F("thr_on   : ")); Serial.println(thr_on, 1);
  Serial.print(F("thr_dn   : ")); Serial.println(thr_dn, 1);
  Serial.print(F("thr_lift : ")); Serial.println(thr_lift_emg, 1);
  Serial.println(F("===========================\n"));

  Serial.println(F("smooth\tthr_on\tthr_lift_emg\tactive\temg_lift")); // Plotter header
}

// ---- Calibration Flow ----
void startRestPhase() {
  phase = CAL_REST;
  phaseStartMs = millis();
  resetRestStats();
  instruct(F("REST CALIBRATION (4 s):\n"
             "Place forearm supported, fully relaxed.\n"
             "Don't move or talk. Collecting baseline..."));
}

void startAirReady() {
  phase = CAL_AIRCURL_READY;
  phaseStartMs = millis();
  instruct(F("AIR-CURL CAPTURE (~10 s):\n"
             "Get ready to do 2-3 slow 'air curls' (no load).\n"
             "Starts in 3 seconds..."));
}

void startAirRun() {
  phase = CAL_AIRCURL_RUN;
  phaseStartMs = millis();
  airN = 0;
  Serial.println(F("Go! Slow curls now..."));
}

void startMVCQuery() {
  phase = CAL_MVC_QUERY;
  phaseStartMs = millis();
  instruct(F("OPTIONAL MVC (max squeeze):\n"
             "Press 'm' within 5 s to include a 5 s MVC capture.\n"
             "Otherwise we'll skip."));
}

void startMVC() {
  doMVC = true;
  phase = CAL_MVC_RUN;
  phaseStartMs = millis();
  mvcN = 0;
  instruct(F("MVC CAPTURE (5 s):\n"
             "Give 2-3 strong squeezes, each ~1 s, then relax."));
}

void finishCalibration() {
  setThresholds();
  phase = RUN;
  // reset runtime state
  active = false; enter_ctr = exit_ctr = 0;
  emg_lift = false; liftStartMs = 0;
}

void myo_setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  analogReadResolution(12);   
  analogReadAveraging(16);

  // Prime filter
  raw = analogRead(MYO_PIN);
  smooth = (float)raw;

  instruct(F("MyoWare Guided Calibration\n"
             "- Signal on A0 (pin 14)\n"
             "- Press 'r' anytime to restart calibration\n"));

  startRestPhase();
}

void myo_loop() {
  // Commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      startRestPhase();
      return;
    }
    if (phase == CAL_MVC_QUERY && (c == 'm' || c == 'M')) {
      startMVC();
      return;
    }
  }

  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_PERIOD_MS) return;
  lastSampleMs = now;

  // Read & smooth
  raw = analogRead(MYO_PIN);
  smooth = ALPHA * smooth + (1.0f - ALPHA) * (float)raw;

  switch (phase) {
    case CAL_REST: {
      welfordAdd((double)smooth);
      if (now - phaseStartMs >= 4000) {   // 4 s
        // lock values
        mu_rest = (float)restMean;
        std_rest = computeStd();
        Serial.print(F("Rest mean=")); Serial.print(mu_rest,1);
        Serial.print(F(", std=")); Serial.println(std_rest,1);
        startAirReady();
      } else if (((now - phaseStartMs) % 1000) < SAMPLE_PERIOD_MS) {
        int left = 4 - (int)((now - phaseStartMs)/1000);
        Serial.print(F("...rest ")); Serial.print(left); Serial.println(F(" s"));
      }
    } break;

    case CAL_AIRCURL_READY: {
      if (now - phaseStartMs >= 3000) {
        startAirRun();
      } else if (((now - phaseStartMs) % 1000) < SAMPLE_PERIOD_MS) {
        int left = 3 - (int)((now - phaseStartMs)/1000);
        Serial.print(F("Starting in ")); Serial.print(left); Serial.println(F(" s"));
      }
    } break;

    case CAL_AIRCURL_RUN: {
      if (airN < AIRCURL_MAX_SAMPLES) airBuf[airN++] = smooth;
      if (now - phaseStartMs >= AIRCURL_MS) {
        P95_air = percentile95(airBuf, airN);
        Serial.print(F("Air-curl P95=")); Serial.println(P95_air,1);
        startMVCQuery();
      } else if (((now - phaseStartMs) % 1000) < SAMPLE_PERIOD_MS) {
        int left = (int)((AIRCURL_MS - (now - phaseStartMs) + 999)/1000);
        Serial.print(F("...air-curl ")); Serial.print(left); Serial.println(F(" s"));
      }
    } break;

    case CAL_MVC_QUERY: {
      if (now - phaseStartMs >= 5000) {
        doMVC = false;
        finishCalibration();
      } else if (((now - phaseStartMs) % 1000) < SAMPLE_PERIOD_MS) {
        int left = 5 - (int)((now - phaseStartMs)/1000);
        Serial.print(F("Press 'm' to include MVC (")); Serial.print(left); Serial.println(F(" s)"));
      }
    } break;

    case CAL_MVC_RUN: {
      if (mvcN < MVC_MAX_SAMPLES) mvcBuf[mvcN++] = smooth;
      if (now - phaseStartMs >= MVC_MS) {
        MVC95 = percentile95(mvcBuf, mvcN);
        Serial.print(F("MVC95=")); Serial.println(MVC95,1);
        finishCalibration();
      } else if (((now - phaseStartMs) % 1000) < SAMPLE_PERIOD_MS) {
        int left = (int)((MVC_MS - (now - phaseStartMs) + 999)/1000);
        Serial.print(F("...MVC ")); Serial.print(left); Serial.println(F(" s"));
      }
    } break;

    case RUN: {
      // --- 'active' detection with hysteresis + debounce ---
      if (!active) {
        if (smooth >= thr_on) {
          if (++enter_ctr >= ENTER_SAMPLES) {
            active = true;
            enter_ctr = 0;
          }
        } else enter_ctr = 0;
      } else {
        if (smooth <= thr_dn) {
          if (++exit_ctr >= EXIT_SAMPLES) {
            active = false;
            exit_ctr = 0;
          }
        } else exit_ctr = 0;
      }

      // --- 'emg_lift' detection ---
      if (smooth >= thr_lift_emg) {
        if (!emg_lift) {
          if (liftStartMs == 0) liftStartMs = now;
          if (now - liftStartMs >= LIFT_HOLD_MS) emg_lift = true;
        }
      } else {
        emg_lift = false;
        liftStartMs = 0;
      }

      // LED on when active
      digitalWrite(LED_PIN, active ? HIGH : LOW);

      // Plotter output
      Serial.print(smooth,1); Serial.print('\t');
      Serial.print(thr_on,1); Serial.print('\t');
      Serial.print(thr_lift_emg,1); Serial.print('\t');
      Serial.print(active ? 1 : 0); Serial.print('\t');
      Serial.println(emg_lift ? 1 : 0);
    } break;
  }
}


} 

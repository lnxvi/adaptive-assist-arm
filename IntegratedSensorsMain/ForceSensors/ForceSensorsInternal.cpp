#include <Arduino.h>
#include <Wire.h>

namespace ForceInternal {
  using ::Wire;
  using ::TwoWire;
  using ::Serial;
  using ::micros;
  using ::millis;
  using ::delay;

  // === Fit constants ===
  static float A_N     = 9.8f;
  static float B_per_S = 102460.0f;

  // ====== Pin map ======
  static const int PIN_T = A1;   // 15
  static const int PIN_I = A17;  // 41
  static const int PIN_M = A16;  // 40
  static const int PIN_R = A15;  // 39
  static const int PINS[4] = { PIN_T, PIN_I, PIN_M, PIN_R };

  // ====== ADC / divider config ======
  static const float VCC       = 3.3f;
  static const float RFIXED    = 150000.0f;   // 150 kOhm divider
  static const int   ADC_BITS  = 12;
  static const int   ADC_AVG   = 8;
  static const int   SAMPLE_AVG_N = 64;       // per reading averaging

  // ====== Filter / timing ======
  static const uint32_t STALE_MS = 250;

  // ====== Per-sensor calibration ======
  static float A_N_arr[4]    = { A_N, A_N, A_N, A_N };
  static float B_perS_arr[4] = { B_per_S, B_per_S, B_per_S, B_per_S };

  // ====== Per-sensor tare ======
  static float G0[4] = {0,0,0,0};     // tared conductance
  static bool  hasTare[4] = {false,false,false,false};

  // ====== Per-finger Weight percent ======
  static float c_eff[4] = { 0.15f, 1.05f, 1.10f, 1.00f };

  // Weight combiner coefficients
  static float alpha0 = 0.0f;
  static float alpha[4] = { 1.0f, 1.0f, 1.0f, 1.0f };

  // ====== State ======
  static uint32_t lastUpdateMs = 0;

  // ---------- Helpers ----------
  inline float clampf(float x, float lo, float hi) { return x<lo?lo:(x>hi?hi:x); }

  inline float readVoltsAvg(int pin, int N = SAMPLE_AVG_N) {
    long acc = 0;
    for (int i = 0; i < N; i++) acc += analogRead(pin);
    const float adc = float(acc) / float(N);
    return (adc / ((1<<ADC_BITS) - 1)) * VCC;
  }

  inline float voltsToR(float v) {
    if (v <= 0.001f || v >= VCC - 0.001f) return INFINITY;
    return RFIXED * (v / (VCC - v));
  }

  inline float RtoG(float R) {
    return (isfinite(R) && R > 0.0f) ? (1.0f / R) : 0.0f;
  }

  float readForceN_one(int idx) {
    const int pin = PINS[idx];
    float v = readVoltsAvg(pin, SAMPLE_AVG_N);
    float R = voltsToR(v);
    float G = RtoG(R);

    float Gt = hasTare[idx] ? max(G - G0[idx], 0.0f) : G;

    float arg = B_perS_arr[idx] * Gt;
    if (!isfinite(arg) || arg < 0.0f) arg = 0.0f;
    if (arg > 18.0f) arg = 18.0f; // optional safety; exp(18) ~ 6.6e7

    float F_N = A_N_arr[idx] * (expf(arg) - 1.0f);
    if (!isfinite(F_N) || F_N < 0.0f) F_N = 0.0f;
    return F_N;
  }

  // Tare all sensors
  void doTareAll(int samplesPerSensor = 300) {
    for (int i = 0; i < 4; ++i) {
      float acc = 0.0f;
      for (int n = 0; n < samplesPerSensor; ++n) {
        float v = readVoltsAvg(PINS[i], 32);
        float R = voltsToR(v);
        acc += RtoG(R);
      }
      G0[i] = acc / samplesPerSensor;
      hasTare[i] = true;
    }
    Serial.print("TARE SET. G0 = [");
    for (int i=0;i<4;++i){ Serial.print(G0[i],8); if(i<3) Serial.print(", "); }
    Serial.println("]");
  }

  // ====== Setup ======
  void force_setup() {
    Serial.begin(115200);
    analogReadResolution(ADC_BITS);
    analogReadAveraging(ADC_AVG);

    delay(250);
    Serial.println("finger_forces: ThumbN,IndexN,MiddleN,RingN | total_kg,total_lb");
    delay(250);

    doTareAll(300);  // auto-tare on boot
    lastUpdateMs = millis();
  }

  // ====== Main Loop ======
  void force_loop() {
    // Serial command: re-tare
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 't' || c == 'T') {
        doTareAll(300);
      }
    }

    // Read all four sensors
    float F_N[4] = {0,0,0,0};
    for (int i = 0; i < 4; ++i) {
      F_N[i] = readForceN_one(i);
    }

    // Estimate object weight from per-finger contributions
    const float g = 9.80665f;
    float W_kg = alpha0;
    for (int i = 0; i < 4; ++i) {
      float contribN = c_eff[i] * F_N[i];   
      W_kg += alpha[i] * (contribN / g);
    }
    W_kg = clampf(W_kg, 0.0f, 25.0f);      
    float W_lb = W_kg * 2.20462262f;

    Serial.print("finger_forces: ");
    for (int i=0;i<4;++i) {
      Serial.print(F_N[i], 2);
      if (i<3) Serial.print(",");
    }
    Serial.print(" | ");
    Serial.print(W_kg, 2); Serial.print(",");
    Serial.println(W_lb, 2);

    lastUpdateMs = millis();
    delay(150);
  }

} 

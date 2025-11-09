#include <Arduino.h>
#include <Wire.h>

namespace ForceInternal {
  using ::Wire;
  using ::TwoWire;
  using ::Serial;
  using ::micros;
  using ::millis;
  using ::delay;

  // Values for main
  static float last_sumN = 0.0f;
  static float last_W_kg = 0.0f;

  float force_getSumN();
  float force_getWeightKg();


  // Values from excel
  static float A_N     = 9.8f;
  static float B_per_S = 102460.0f;

  // pin map
  static const int PIN_I = A1;   
  static const int PIN_M = A17;  
  static const int PIN_R = A16;  
  static const int PIN_P = A15;  
  static int PINS[4] = { PIN_I, PIN_M, PIN_R, PIN_P };

  //   INDEX ->  0
  //   MIDDLE ->  1
  //   RING   ->  3
  //   PINKY  ->  2
  static int mapIdx[4] = { 0, 1, 3, 2 };

  static const float VCC          = 3.3f;
  static const float RFIXED       = 150000.0f;   // 150 kΩ 
  static const int   ADC_BITS     = 12;
  static const int   ADC_AVG      = 8;
  static const int   SAMPLE_AVG_N = 64;          // per reading average

  static float A_N_arr[4]    = { A_N, A_N, A_N, A_N };
  static float B_perS_arr[4] = { B_per_S, B_per_S, B_per_S, B_per_S };

  static float G0[4]     = {0,0,0,0};
  static bool  hasTare[4]= {false,false,false,false};

  static float alpha0            = 0.0f;
  static float handScaleK_base   = 2.20f;    // 55% palm
  // static float handScaleK_base = 2.86f; //35% fingers, may want to try this value 
  static float handScaleK_lift   = 2.20f;    // locked for current lift
  static const float K_LOCK_MIN  = 0.85f;   
  static const float K_LOCK_MAX  = 1.15f;    
  static const float g           = 9.80665f;

  // Lift detection / locking
  static bool     liftActive = false;
  static uint32_t tLiftStart = 0;
  static float    sumN_peak  = 0.0f;

  // Hysteresis + window (tune as needed)
  static const float     SUMN_LIFT_ON_N   = 8.0f;   // enter lift when smoothed sumN > 8 N
  static const float     SUMN_LIFT_OFF_N  = 4.0f;   // exit when smoothed sumN < 4 N
  static const uint32_t  LOCK_WINDOW_MS   = 1000;   // find peak in first ~1.0 s

  // Small EMA for stability
  static float sumN_ema = 0.0f;
  static const float SUMN_EMA_ALPHA = 0.25f;

  static uint32_t lastUpdateMs = 0;

  inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
  }

  inline float readVoltsAvg(int pin, int N = SAMPLE_AVG_N) {
    long acc = 0;
    for (int i = 0; i < N; i++) acc += analogRead(pin);
    const float adc = float(acc) / float(N);
    return (adc / ((1 << ADC_BITS) - 1)) * VCC;
  }

  inline float voltsToR(float v) {
    if (v <= 0.001f || v >= VCC - 0.001f) return INFINITY;
    return RFIXED * (v / (VCC - v));
  }

  inline float RtoG(float R) {
    return (isfinite(R) && R > 0.0f) ? (1.0f / R) : 0.0f;
  }

  float readForceN_rawIdx(int rawIdx) {
    const int pin = PINS[rawIdx];
    float v  = readVoltsAvg(pin, SAMPLE_AVG_N);
    float R  = voltsToR(v);
    float G  = RtoG(R);
    float Gt = hasTare[rawIdx] ? max(G - G0[rawIdx], 0.0f) : G;

    float arg = B_perS_arr[rawIdx] * Gt;
    if (!isfinite(arg) || arg < 0.0f) arg = 0.0f;
    if (arg > 18.0f) arg = 18.0f; // clamp exponent to keep exp() sane

    float F_N = A_N_arr[rawIdx] * (expf(arg) - 1.0f);
    if (!isfinite(F_N) || F_N < 0.0f) F_N = 0.0f;
    return F_N;
  }

  // Tare all channels
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

    Serial.print("TARE SET. G0(raw idx 0..3) = [");
    for (int i = 0; i < 4; ++i) {
      Serial.print(G0[i], 8);
      if (i < 3) Serial.print(", ");
    }
    Serial.println("]");

    Serial.print("Mapping complete. Logical (I,M,R,P) -> raw idx = [");
    for (int i=0;i<4;++i){ Serial.print(mapIdx[i]); if(i<3) Serial.print(", "); }
    Serial.println("]");

    // reset per-lift scale to base on tare
    handScaleK_lift = handScaleK_base;
  }

  static bool extLiftActive = false;
  void setLiftActive(bool active) { extLiftActive = active; }
  void force_setLiftActive(bool active) { setLiftActive(active); }

  void force_setup() {
    Serial.begin(115200);
    analogReadResolution(ADC_BITS);
    analogReadAveraging(ADC_AVG);

    delay(250);
    Serial.println("finger_forces: IndexN,MiddleN,RingN,PinkyN | total_kg,total_lb");
    Serial.println("[Keys] 't' = tare");
    delay(250);

    doTareAll(300);
    lastUpdateMs = millis();
  }

  void force_loop() {
    // Handle keys
    while (Serial.available()) {
      char c = Serial.read();
      if (c == 't' || c == 'T') {
        doTareAll(300);
      }
    }

    // Read all raw channels once
    float Fraw_N[4] = {0,0,0,0};
    for (int raw = 0; raw < 4; ++raw) Fraw_N[raw] = readForceN_rawIdx(raw);

    // Reorder to logical I,M,R,P using fixed map
    float F_I = Fraw_N[ mapIdx[0] ];
    float F_M = Fraw_N[ mapIdx[1] ];
    float F_R = Fraw_N[ mapIdx[2] ];
    float F_P = Fraw_N[ mapIdx[3] ];

    // Sum fingertip forces (neutral per-finger multipliers for now)
    float sumN = F_I + F_M + F_R + F_P;

    // Smooth a bit
    if (sumN_ema == 0.0f) sumN_ema = sumN;
    sumN_ema = SUMN_EMA_ALPHA * sumN + (1.0f - SUMN_EMA_ALPHA) * sumN_ema;

    // Lift detection (force-based, with optional EMG gating to force-on)
    bool liftNow = liftActive;
    if (!liftNow && (sumN_ema > SUMN_LIFT_ON_N)) liftNow = true;
    if ( liftNow && (sumN_ema < SUMN_LIFT_OFF_N)) liftNow = false;
    if (extLiftActive) liftNow = true; // EMG can only force it ON

    // On rising edge, start a new lock window
    static bool wasLift = false;
    if (liftNow && !wasLift) {
      tLiftStart     = millis();
      sumN_peak      = sumN_ema;
      handScaleK_lift= handScaleK_base; // reset toward base for each lift
    }

    // During the first short window
    if (liftNow) {
      uint32_t dt = millis() - tLiftStart;
      if (sumN_ema > sumN_peak) sumN_peak = sumN_ema;

      if (dt >= LOCK_WINDOW_MS) {
        float denom = sumN_ema / g;
        float numer = sumN_peak / g;
        if (denom > 0.05f && numer > 0.05f) {
          float ratio = numer / denom;                 
          float Klock = handScaleK_base * ratio;       
          float scale = clampf(Klock / handScaleK_base, K_LOCK_MIN, K_LOCK_MAX);
          handScaleK_lift = handScaleK_base * scale;   // freeze for the rest of this lift
        }
        // Push window in the past so we don't relock mid-lift
        tLiftStart = millis() - (LOCK_WINDOW_MS + 1);
      }
    } else {
      // Not lifting — reset between lifts
      handScaleK_lift = handScaleK_base;
      sumN_peak = 0.0f;
    }
    wasLift = liftNow;
    liftActive = liftNow;

    // Final weight estimate with per-lift locked K
    float W_kg = alpha0 + handScaleK_lift * (sumN_ema / g);
    W_kg = clampf(W_kg, 0.0f, 25.0f);
    float W_lb = W_kg * 2.20462262f;

    // Print line
    // Serial.print("finger_forces: ");
    // Serial.print(F_I,2); Serial.print(",");
    // Serial.print(F_M,2); Serial.print(",");
    // Serial.print(F_R,2); Serial.print(",");
    // Serial.print(F_P,2);
    // Serial.print(" | ");
    // Serial.print(W_kg,2); Serial.print(",");
    // Serial.println(W_lb,2);

    lastUpdateMs = millis();
    last_sumN = sumN;
    last_W_kg = W_kg;
    delay(150);
    
  }

  float force_getSumN() { return last_sumN; }
  float force_getWeightKg() { return last_W_kg; }
  float force_getWeightLb() { return last_W_kg * 2.20462262f;}
} 

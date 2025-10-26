#include <Arduino.h>

#include "Accelerometer/Accelerometer.h"
#include "Myoware/Myoware.h"
#include "ForceSensors/ForceSensors.h"
#include "MotorAssist/MotorAssist.h"

#include "Accelerometer/Accelerometer.cpp"
#include "Accelerometer/AccelerometerInternal.cpp"
#include "Myoware/Myoware.cpp"
#include "Myoware/MyowareInternal.cpp"
#include "ForceSensors/ForceSensors.cpp"
#include "ForceSensors/ForceSensorsInternal.cpp"
#include "MotorAssist/MotorAssist.cpp"
#include "MotorAssist/MotorAssistInternal.cpp"

namespace ForceInternal {
  void  force_setLiftActive(bool active);
  float force_getSumN();
  float force_getWeightKg();
}
namespace AccelInternal {
  float accel_getAmag();
}
namespace MotorInternal {
  void  motor_setWeightKg(float wkg);
  void  motor_setElbowAngleRad(float th);
  void  motor_setAssistFraction(float a);
  float motor_getOutputTorqueNm();
  float motor_getMotorCurrentA();
}

// Modules
Accel::Module       accel;
Myo::Module         myo;
Force::Module       force;
MotorTorque::Module motor;   

// Periods
static const uint32_t MYO_PERIOD_MS   = 10;   // ~100 Hz
static const uint32_t ACCEL_PERIOD_MS = 20;   // ~50 Hz
static const uint32_t FORCE_PERIOD_MS = 160;  // ~6 Hz (force delays ~150 ms internally)
static uint32_t lastMyoMs   = 0;
static uint32_t lastAccelMs = 0;
static uint32_t lastForceMs = 0;

// Force thresholds (N)
static const float SUMN_ON_N   = 8.0f;
static const float SUMN_OFF_N  = 1.5f;

// Motion thresholds (|amag-1|)
static const float AMAG_DELTA_ON  = 0.12f;
static const float AMAG_DELTA_OFF = 0.05f;

// FSM timing
static const uint32_t ENTER_MS   = 120;  // NO_LIFT -> LIFT
static const uint32_t EXIT_MS    = 300;  // LIFT -> HOLD
static const uint32_t RELEASE_MS = 250;  // HOLD -> NO_LIFT 
static const float    HOLD_MIN_N = 2.0f;

static inline bool hyst(bool prev, bool onCond, bool offCond) {
  if (!prev && onCond)  return true;
  if ( prev && offCond) return false;
  return prev;
}

enum class State : uint8_t { NO_LIFT, LIFT, HOLD };
static State state = State::NO_LIFT;

// timers
static uint32_t votesOnStartMs  = 0; // entering LIFT
static uint32_t votesOffStartMs = 0; // LIFT -> HOLD
static uint32_t releaseStartMs  = 0; // HOLD -> NO_LIFT

static float sumN_ema = 0.0f;
static const float SUMN_EMA_ALPHA = 0.25f;

static uint32_t lastAssistTickUs = 0;

// Using a constant for now
static const float DEFAULT_ASSIST_FRACTION = 0.30f;

static inline float getElbowAngleRadFallback() {
  return 1.5707963f; // π/2 rad (arm ~horizontal)
}

void setup() {
  Serial.begin(115200);
#if defined(ARDUINO_TEENSY41)
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) { /* wait */ }
#endif

  myo.setup();
  accel.setup();
  force.setup();
  motor.setup();   // Torque calculations

  lastAssistTickUs = micros();

  Serial.println();
  Serial.println(F("[Main] EMG + ACCEL + FORCE + MOTOR with FSM {NO_LIFT, LIFT, HOLD} + 3-way voting"));
  Serial.println(F("[Keys] 't' = tare force (handled inside force module)."));
  Serial.println();
}

void loop() {
  const uint32_t now = millis();

  // MyoWare
  if (now - lastMyoMs >= MYO_PERIOD_MS) {
    myo.runTestStep();
    lastMyoMs = now;
  }

  // Accelerometer
  if (now - lastAccelMs >= ACCEL_PERIOD_MS) {
    accel.runTestStep();
    lastAccelMs = now;
  }

  // Force
  if (now - lastForceMs >= FORCE_PERIOD_MS) {
    force.runTestStep();
    lastForceMs = now;
  }

  const bool v_emg = myo.isLiftActive();

  float sumN = ForceInternal::force_getSumN();
  if (sumN_ema == 0.0f) sumN_ema = sumN;
  sumN_ema = SUMN_EMA_ALPHA * sumN + (1.0f - SUMN_EMA_ALPHA) * sumN_ema;

  static bool v_force = false;
  const bool forceOn  = (sumN_ema > SUMN_ON_N);
  const bool forceOff = (sumN_ema < SUMN_OFF_N);
  v_force = hyst(v_force, forceOn, forceOff);

  const float amag = AccelInternal::accel_getAmag();
  const float dmag = fabsf(amag - 1.0f);
  static bool v_motion = false;
  const bool motionOn  = (dmag > AMAG_DELTA_ON);
  const bool motionOff = (dmag < AMAG_DELTA_OFF);
  v_motion = hyst(v_motion, motionOn, motionOff);

  const uint8_t votes = (uint8_t)v_emg + (uint8_t)v_force + (uint8_t)v_motion;

  // --- FSM ---
  switch (state) {
    case State::NO_LIFT: {
      if (votes >= 2) {
        if (votesOnStartMs == 0) votesOnStartMs = now;
        if ((now - votesOnStartMs) >= ENTER_MS) {
          state = State::LIFT;
          votesOnStartMs = 0;
          releaseStartMs = 0;
          ForceInternal::force_setLiftActive(true);
          Serial.println(F("[FSM] LIFT"));
        }
      } else {
        votesOnStartMs = 0;
      }
    } break;

    case State::LIFT: {
      if (votes <= 1) {
        if (votesOffStartMs == 0) votesOffStartMs = now;
        if ((now - votesOffStartMs) >= EXIT_MS) {
          state = State::HOLD;
          votesOffStartMs = 0;
          Serial.println(F("[FSM] HOLD"));
        }
      } else {
        votesOffStartMs = 0;
      }
    } break;

    case State::HOLD: {
      const bool emgQuiet    = !v_emg;
      const bool motionQuiet = !v_motion;
      const bool forceLow    = (sumN_ema < HOLD_MIN_N);
      const bool quiet       = emgQuiet && motionQuiet && forceLow;

      if (quiet) {
        if (releaseStartMs == 0) releaseStartMs = now;
        if ((now - releaseStartMs) >= RELEASE_MS) {
          state = State::NO_LIFT;
          releaseStartMs = 0;
          ForceInternal::force_setLiftActive(false);
          Serial.println(F("[FSM] NO_LIFT"));
        }
      } else {
        releaseStartMs = 0;
      }

      // Return to LIFT if ≥2 votes again
      const uint8_t votesNow = (uint8_t)v_emg + (uint8_t)v_force + (uint8_t)v_motion;
      if (votesNow >= 2) {
        state = State::LIFT;
        votesOffStartMs = 0;
        releaseStartMs  = 0;
        ForceInternal::force_setLiftActive(true);
        Serial.println(F("[FSM] HOLD, LIFT"));
      }
    } break;
  }

  // TORQUE STUFF
  const bool assistEnabled = (state != State::NO_LIFT);
  const uint32_t nowUs = micros();
  const float dtSec = max( (nowUs - lastAssistTickUs) * 1e-6f, 1e-3f );
  lastAssistTickUs = nowUs;

  if (assistEnabled) {
    const float weightKg      = ForceInternal::force_getWeightKg();
    const float elbowAngleRad = getElbowAngleRadFallback();
    const float assistFrac    = DEFAULT_ASSIST_FRACTION;

    // Feed inputs to motor internals
    MotorInternal::motor_setWeightKg(weightKg);
    MotorInternal::motor_setElbowAngleRad(elbowAngleRad);
    MotorInternal::motor_setAssistFraction(assistFrac);

    motor.runTestStep();

    const float tauOutNm = MotorInternal::motor_getOutputTorqueNm();
    const float motorA   = MotorInternal::motor_getMotorCurrentA();

    Serial.printf("[ASSIST] %s  τOut=%.2f Nm  I=%.2f A  W=%.2f kg  θ=%.2f rad\n",
                  (state == State::LIFT ? "LIFT" : "HOLD"),
                  tauOutNm, motorA, weightKg, elbowAngleRad);

    // TODO: Command driver
    // driverWritePWM(pwm);

  } else {
    //  set motor command to zero.
    // driverWritePWM(0);
    // Serial.println("[ASSIST] OFF");
  }

  /*
  const float W_kg = ForceInternal::force_getWeightKg();
  Serial.print("state="); Serial.print(state==State::NO_LIFT?"NO_LIFT":(state==State::LIFT?"LIFT":"HOLD"));
  Serial.print(", votes="); Serial.print((int)((uint8_t)v_emg + (uint8_t)v_force + (uint8_t)v_motion));
  Serial.print(", emg="); Serial.print(v_emg);
  Serial.print(", forceN="); Serial.print(sumN_ema, 2);
  Serial.print(", motion="); Serial.print(v_motion);
  Serial.print(", amag="); Serial.print(amag, 2);
  Serial.print(", W_kg="); Serial.println(W_kg, 2);
  */
}

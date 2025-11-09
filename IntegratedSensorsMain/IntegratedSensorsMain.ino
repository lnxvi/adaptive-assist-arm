#include <Arduino.h>
#include <TimerThree.h>
#include "Accelerometer/Accelerometer.h"
#include "Myoware/Myoware.h"
#include "ForceSensors/ForceSensors.h"
#include "MotorAssist/MotorAssist.h"

// These #includes of .cpps are unusual but keeping as you have it:
#include "Accelerometer/Accelerometer.cpp"
#include "Accelerometer/AccelerometerInternal.cpp"
#include "Myoware/Myoware.cpp"
#include "Myoware/MyowareInternal.cpp"
#include "ForceSensors/ForceSensors.cpp"
#include "ForceSensors/ForceSensorsInternal.cpp"
#include "MotorAssist/MotorAssist.cpp"
#include "MotorAssist/MotorAssistInternal.cpp"
#include "Control/controller.cpp"
#include "Control/controller.h"

namespace ForceInternal {
  void  force_setLiftActive(bool active);
  float force_getSumN();
  float force_getWeightKg();
  float force_getWeightLb();
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
  float motor_getElbowTorqueNm();
}

Accel::Module       accel;
Myo::Module         myo;
Force::Module       force;
MotorTorque::Module motor;
Controller controller;

// timing periods
static const uint32_t MYO_PERIOD_MS   = 10;   // 100 Hz
static const uint32_t ACCEL_PERIOD_MS = 20;   // 50 Hz
static const uint32_t FORCE_PERIOD_MS = 120;  
static uint32_t lastMyoMs   = 0;
static uint32_t lastAccelMs = 0;
static uint32_t lastForceMs = 0;

// force thresholds 
static const float SUMN_ON_N   = 5.0f;
static const float SUMN_OFF_N  = 1.5f;  // this will drop vote sooner when force falls

// motion thresholds
static const float AMAG_DELTA_ON  = 0.12f;  // enter moving
static const float AMAG_DELTA_OFF = 0.07f;  // exit moving 

// entry guard
static const float MIN_FORCE_FOR_EMG_MOTION_LIFT = 0.30f; 

// FSM timing
static const uint32_t ENTER_MS   = 90;  // kept but not used for PRELIFT entry now
static const uint32_t EXIT_MS    = 80;  // LIFT to HOLD faster
static const uint32_t RELEASE_MS = 350; // HOLD to NO_LIFT
static const float    HOLD_MIN_N = 2.0f; 

// PRELIFT requirements
static const uint8_t  PRELIFT_VOTES_REQ = 2;
static const uint32_t PRELIFT_DWELL_MS  = 3000; // 3 seconds

// startup, avoids immediate lift
static uint32_t bootMs = 0;
static const uint32_t STARTUP_INHIBIT_MS = 3000; // 3 s after power on

// Motor Values
const float r_spool = 0.02761;
const float l_forearm = 0.25;
const float Kt = 0.69;

// Motor Pins
#define nSLEEP 31
#define PMODE 30
#define IN2 6
#define IN1 9

static inline bool hyst(bool prev, bool onCond, bool offCond) {
  if (!prev && onCond)  return true;
  if ( prev && offCond) return false;
  return prev;
}

enum class State : uint8_t { NO_LIFT, PRELIFT, LIFT, HOLD };
static State state = State::NO_LIFT;

// timers
static uint32_t votesOnStartMs  = 0; // entering LIFT 
static uint32_t votesOffStartMs = 0; // LIFT to HOLD
static uint32_t releaseStartMs  = 0; // HOLD to NO_LIFT
static uint32_t preliftStartMs  = 0; // timing PRELIFT dwell

// voters
static bool v_force  = false;
static bool v_motion = false;

// asymmetric EMA for force 
static float sumN_ema = 0.0f;
static inline float ema_asym(float prev, float x) {
  const float alpha_up = 0.60f;  // reacts quickly
  const float alpha_dn = 0.25f;  
  const float a = (x > prev) ? alpha_up : alpha_dn;
  return a * x + (1.0f - a) * prev;
}

// torque tick
static uint32_t lastAssistTickUs = 0;

// Using a constant for now
static const float DEFAULT_ASSIST_FRACTION = 0.30f;

static inline float getElbowAngleRadFallback() {
  return 1.5707963f; 
}

void setup() {
  Serial.begin(115200);
#if defined(ARDUINO_TEENSY41)
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) { /* wait */ }
#endif

  // Set motor pins to output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(nSLEEP, OUTPUT);
  pinMode(PMODE, OUTPUT);
  digitalWrite(nSLEEP, HIGH);
  digitalWrite(PMODE, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, LOW);

  Timer3.initialize(40);
  Timer3.pwm(IN1, 0);

  myo.setup();
  accel.setup();
  force.setup();
  motor.setup();   // Torque calculations
  bootMs = millis();

  // voter/timers 
  v_motion = false;
  v_force  = false;
  votesOnStartMs  = votesOffStartMs = releaseStartMs = 0;
  preliftStartMs  = 0;
  state = State::NO_LIFT;

  lastAssistTickUs = micros();

  Serial.println();
  Serial.println(F("[Main] FSM Started"));
  Serial.println();
}

void loop() {
  const uint32_t now = millis();
  const bool armed = (now - bootMs) > STARTUP_INHIBIT_MS;

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

  // Voters
  const bool v_emg = myo.isLiftActive();

  float sumN = ForceInternal::force_getSumN();        // N
  if (sumN_ema == 0.0f) sumN_ema = sumN;
  sumN_ema = ema_asym(sumN_ema, sumN);

  // Force vote 
  const bool forceOn  = (sumN_ema > SUMN_ON_N);
  const bool forceOff = (sumN_ema < SUMN_OFF_N);
  v_force = hyst(v_force, forceOn, forceOff);

  // Motion vote 
  const float amag = AccelInternal::accel_getAmag();  // g
  const float dmag = fabsf(amag - 1.0f);              // 0 at rest
  const bool motionOn  = (dmag > AMAG_DELTA_ON);
  const bool motionOff = (dmag < AMAG_DELTA_OFF);
  v_motion = hyst(v_motion, motionOn, motionOff);

  const uint8_t votes = (uint8_t)v_emg + (uint8_t)v_force + (uint8_t)v_motion;

  // --- FSM ---
  switch (state) {
    case State::NO_LIFT: {
      const bool canEnterLift =
          (v_force && (v_emg || v_motion)) ||
          (v_emg && v_motion && sumN_ema > MIN_FORCE_FOR_EMG_MOTION_LIFT);

      if (armed && canEnterLift) {
        if (votes >= PRELIFT_VOTES_REQ) {
          state = State::PRELIFT;
          preliftStartMs = now;
          votesOnStartMs = 0;   
          releaseStartMs = 0;
          ForceInternal::force_setLiftActive(false);
          Serial.printf("[FSM] PRELIFT start; votes=%u [%s%s%s ] forceN=%.2f dmag=%.3f\n",
                        votes,
                        v_emg?"EMG ":"",
                        v_force?"FORCE ":"",
                        v_motion?"MOTION ":"",
                        sumN_ema, dmag);
        }
      } else {
        votesOnStartMs = 0;
        preliftStartMs = 0;
      }
    } break;

    case State::PRELIFT: {
      if (!armed || (votes < PRELIFT_VOTES_REQ)) {
        state = State::NO_LIFT;
        preliftStartMs = 0;
        ForceInternal::force_setLiftActive(false);
        Serial.printf("[FSM] PRELIFT to NO_LIFT; votes=%u forceN=%.2f\n", votes, sumN_ema);
        break;
      }

      if ((now - preliftStartMs) >= PRELIFT_DWELL_MS) {
        state = State::LIFT;
        preliftStartMs = 0;
        ForceInternal::force_setLiftActive(true);
        Serial.printf("[FSM] PRELIFT to LIFT; votes=%u forceN=%.2f\n", votes, sumN_ema);
      }
    } break;

    case State::LIFT: {
      const uint8_t votesNow = votes;
      const bool forceLow    = (sumN_ema < HOLD_MIN_N);
      const bool quietish    = (votesNow <= 1);
      const bool forceVeryLow = (sumN_ema < 1.5f);
      const bool emgInactive  = !v_emg;
      Serial.printf("[FSM] Made it here, beginning of LIFT\n");

      if (forceVeryLow && emgInactive) {
        state = State::HOLD;
        votesOffStartMs = 0;
        Serial.printf("[FSM] HOLD votes=%u [...]\n", votesNow);
        break;
      }

      // Exit LIFT faster when you set weight down, or it's quiet
      if (forceLow || quietish) {
        if (votesOffStartMs == 0) votesOffStartMs = now;
        if ((now - votesOffStartMs) >= EXIT_MS) {
          state = State::HOLD;
          votesOffStartMs = 0;
          Serial.printf("[FSM] HOLD votes=%u [%s%s%s ] forceN=%.2f dmag=%.3f\n",
                        votesNow,
                        v_emg?"EMG ":"",
                        v_force?"FORCE ":"",
                        v_motion?"MOTION ":"",
                        sumN_ema, dmag);
        }
      } else {
        votesOffStartMs = 0;
      }
    } break;

    case State::HOLD: {
      const uint8_t votesNow = votes;

      // HOLD to LIFT
      const bool canEnterLift =
          (v_force && (v_emg || v_motion)) ||
          (v_emg && v_motion && sumN_ema > MIN_FORCE_FOR_EMG_MOTION_LIFT);

      const bool forceLow   = (sumN_ema < HOLD_MIN_N);
      const bool votesQuiet = (votesNow <= 1);
      const bool quiet = (sumN_ema < 1.5f) && (votesNow <= 1);

      // quicker escape when basically zero contact
      static const float    FORCE_ESCAPE_N  = 0.30f;
      static const uint32_t ESCAPE_MS       = 250;
      static uint32_t forceEscapeStartMs    = 0;

      if (sumN_ema < FORCE_ESCAPE_N) {
        if (forceEscapeStartMs == 0) forceEscapeStartMs = now;
        if (now - forceEscapeStartMs >= ESCAPE_MS) {
          state = State::NO_LIFT;
          releaseStartMs = 0;
          forceEscapeStartMs = 0;
          ForceInternal::force_setLiftActive(false);

          Serial.printf("[FSM] NO_LIFT votes=%u [%s%s%s ] forceN=%.2f dmag=%.3f\n",
                        votesNow,
                        v_emg?"EMG ":"",
                        v_force?"FORCE ":"",
                        v_motion?"MOTION ":"",
                        sumN_ema, dmag);
          break;
        }
      } else {
        forceEscapeStartMs = 0;
      }

      if (quiet) {
        if (releaseStartMs == 0) releaseStartMs = now;
        if ((now - releaseStartMs) >= RELEASE_MS) {
          state = State::NO_LIFT;
          releaseStartMs = 0;
          ForceInternal::force_setLiftActive(false);

          Serial.printf("[FSM] NO_LIFT votes=%u [%s%s%s ] forceN=%.2f dmag=%.3f\n",
                        votesNow,
                        v_emg?"EMG ":"",
                        v_force?"FORCE ":"",
                        v_motion?"MOTION ":"",
                        sumN_ema, dmag);
        }
      } else {
        releaseStartMs = 0;
      }

      // re-enter LIFT 
      if (canEnterLift) {
        state = State::LIFT;
        votesOffStartMs = 0;
        releaseStartMs  = 0;
        ForceInternal::force_setLiftActive(true);

        Serial.printf("[FSM] HOLD, LIFT votes=%u [%s%s%s ] forceN=%.2f dmag=%.3f\n",
                      votesNow,
                      v_emg?"EMG ":"",
                      v_force?"FORCE ":"",
                      v_motion?"MOTION ":"",
                      sumN_ema, dmag);
      }
    } break;
  }

  // TORQUE STUFF
  const bool assistEnabled = (state == State::LIFT || state == State::HOLD);
  const uint32_t nowUs = micros();
  lastAssistTickUs = nowUs;

  if (assistEnabled) {
    Serial.printf("[FSM] Made it here, beginning of assistance\n");

    const float weightKg      = ForceInternal::force_getWeightKg();
    const float weightLb      = ForceInternal::force_getWeightLb();
    const float elbowAngleRad = getElbowAngleRadFallback();
    const float elbowTorque =   MotorInternal::motor_getElbowTorqueNm();
    const float assistFrac    = DEFAULT_ASSIST_FRACTION;

    MotorInternal::motor_setWeightKg(weightKg);
    MotorInternal::motor_setElbowAngleRad(elbowAngleRad);
    MotorInternal::motor_setAssistFraction(assistFrac);
    motor.runTestStep();

    Serial.printf("[FSM] Made it here, sending value\n");

    digitalWrite(PMODE, HIGH);

    const float I_set = controller.torqueToCurrent(elbowTorque, r_spool, l_forearm, Kt);
    const uint16_t duty = controller.currentToPWM(I_set);
    controller.sendMotorDuty(700);

  } else {
    
    const float weightKg      = ForceInternal::force_getWeightKg();
    const float weightLb      = ForceInternal::force_getWeightLb();
    const float elbowAngleRad = getElbowAngleRadFallback();
    const float elbowTorque = MotorInternal::motor_getElbowTorqueNm();
    const float assistFrac    = DEFAULT_ASSIST_FRACTION;

    MotorInternal::motor_setWeightKg(weightKg);
    MotorInternal::motor_setElbowAngleRad(elbowAngleRad);
    MotorInternal::motor_setAssistFraction(assistFrac);

    motor.runTestStep();
    //const float I_set = controller.torqueToCurrent(0, r_spool, l_forearm, Kt);
    //const uint16_t duty = controller.currentToPWM(I_set);
    controller.sendMotorDuty(0);
  }
}

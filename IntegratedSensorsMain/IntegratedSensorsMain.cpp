// // AdaptiveAssistMain.ino
// #include <Arduino.h>
// #include <math.h>  // for fabsf

// #include "Accelerometer/Accelerometer.h"
// #include "Myoware/Myoware.h"
// #include "ForceSensors/ForceSensors.h"
// #include "MotorAssist/MotorAssist.h"

// // Forward decls for internal functions provided by your *Internal.cpp units.
// // (If you prefer, export these from the public headers instead.)
// namespace ForceInternal {
//   void  force_setLiftActive(bool active);
//   float force_getSumN();
//   float force_getWeightKg();
// }
// namespace AccelInternal {
//   float accel_getAmag();
// }

// // Module instances
// Accel::Module       accel;
// Myo::Module         myo;
// Force::Module       force;
// MotorTorque::Module motorAssist;

// // Periods
// static const uint32_t MYO_PERIOD_MS   = 10;   // ~100 Hz
// static const uint32_t ACCEL_PERIOD_MS = 20;   // ~50 Hz
// static const uint32_t FORCE_PERIOD_MS = 160;  // ~6 Hz (force module also delays ~150 ms internally)

// static uint32_t lastMyoMs   = 0;
// static uint32_t lastAccelMs = 0;
// static uint32_t lastForceMs = 0;

// // Voting thresholds
// static const float SUMN_ON_N   = 8.0f;
// static const float SUMN_OFF_N  = 1.5f;

// static const float AMAG_DELTA_ON  = 0.12f;
// static const float AMAG_DELTA_OFF = 0.05f;

// // Timing thresholds (ms)
// static const uint32_t ENTER_MS   = 120;  // HOLD/NO_LIFT -> LIFT needs ≥2 votes for this long
// static const uint32_t EXIT_MS    = 300;  // LIFT -> HOLD when votes drop ≤1 for this long
// static const uint32_t RELEASE_MS = 250;  // HOLD -> NO_LIFT when quiet for this long

// static const float HOLD_MIN_N = 2.0f;

// // FSM
// enum class State : uint8_t { NO_LIFT, LIFT, HOLD };
// static State state = State::NO_LIFT;

// // timers
// static uint32_t votesOnStartMs  = 0; // entering LIFT
// static uint32_t votesOffStartMs = 0; // exiting LIFT to HOLD
// static uint32_t releaseStartMs  = 0; // declaring NO_LIFT

// // Force smoothing
// static float sumN_ema = 0.0f;
// static const float SUMN_EMA_ALPHA = 0.25f;

// // Simple hysteresis helper
// static inline bool hyst(bool prev, bool onCond, bool offCond) {
//   if (!prev && onCond)  return true;
//   if ( prev && offCond) return false;
//   return prev;
// }

// void setup() {
//   Serial.begin(115200);
// #if defined(ARDUINO_TEENSY41)
//   const uint32_t t0 = millis();
//   while (!Serial && (millis() - t0 < 2000)) { /* wait for USB */ }
// #endif

//   myo.setup();         // EMG
//   accel.setup();       // Accelerometer
//   force.setup();       // Force sensors
//   motorAssist.setup(); // Motor torque calculation

//   Serial.println();
//   Serial.println(F("[Main] EMG + ACCEL + FORCE with FSM {NO_LIFT, LIFT, HOLD} and 3-way voting."));
//   Serial.println(F("[Keys] 't' = tare force (handled inside force module)."));
//   Serial.println();
// }

// void loop() {
//   const uint32_t now = millis();

//   // MyoWare
//   if (now - lastMyoMs >= MYO_PERIOD_MS) {
//     myo.runTestStep();
//     lastMyoMs = now;
//   }

//   // Accelerometer
//   if (now - lastAccelMs >= ACCEL_PERIOD_MS) {
//     accel.runTestStep();
//     lastAccelMs = now;
//   }

//   // Force
//   if (now - lastForceMs >= FORCE_PERIOD_MS) {
//     force.runTestStep();
//     lastForceMs = now;
//   }

//   // EMG vote
//   const bool v_emg = myo.isLiftActive();

//   // Force vote (EMA + hysteresis)
//   float sumN = ForceInternal::force_getSumN();
//   if (sumN_ema == 0.0f) sumN_ema = sumN;
//   sumN_ema = SUMN_EMA_ALPHA * sumN + (1.0f - SUMN_EMA_ALPHA) * sumN_ema;

//   static bool v_force = false;
//   const bool forceOn  = (sumN_ema > SUMN_ON_N);
//   const bool forceOff = (sumN_ema < SUMN_OFF_N);
//   v_force = hyst(v_force, forceOn, forceOff);

//   // Motion vote (|amag - 1g| + hysteresis)
//   const float amag = AccelInternal::accel_getAmag();
//   const float dmag = fabsf(amag - 1.0f);

//   static bool v_motion = false;
//   const bool motionOn  = (dmag > AMAG_DELTA_ON);
//   const bool motionOff = (dmag < AMAG_DELTA_OFF);
//   v_motion = hyst(v_motion, motionOn, motionOff);

//   // Count votes
//   const uint8_t votes = static_cast<uint8_t>(v_emg) + static_cast<uint8_t>(v_force) + static_cast<uint8_t>(v_motion);

//   // FSM
//   switch (state) {
//     case State::NO_LIFT: {
//       // Enter LIFT when ≥2 votes sustained
//       if (votes >= 2) {
//         if (votesOnStartMs == 0) votesOnStartMs = now;
//         if ((now - votesOnStartMs) >= ENTER_MS) {
//           state = State::LIFT;
//           votesOnStartMs = 0;
//           releaseStartMs = 0;
//           ForceInternal::force_setLiftActive(true);
//           Serial.println(F("[FSM] LIFT"));
//         }
//       } else {
//         votesOnStartMs = 0;
//       }
//     } break;

//     case State::LIFT: {
//       // Exit to HOLD when votes drop to ≤1 sustained
//       if (votes <= 1) {
//         if (votesOffStartMs == 0) votesOffStartMs = now;
//         if ((now - votesOffStartMs) >= EXIT_MS) {
//           state = State::HOLD;
//           votesOffStartMs = 0;
//           Serial.println(F("[FSM] HOLD"));
//         }
//       } else {
//         votesOffStartMs = 0;
//       }
//     } break;

//     case State::HOLD: {
//       // Quiet if EMG & Motion are OFF and force very low, then go NO_LIFT
//       const bool emgQuiet    = !v_emg;
//       const bool motionQuiet = !v_motion;
//       const bool forceLow    = (sumN_ema < HOLD_MIN_N);
//       const bool quiet       = emgQuiet && motionQuiet && forceLow;

//       if (quiet) {
//         if (releaseStartMs == 0) releaseStartMs = now;
//         if ((now - releaseStartMs) >= RELEASE_MS) {
//           state = State::NO_LIFT;
//           releaseStartMs = 0;
//           ForceInternal::force_setLiftActive(false);
//           Serial.println(F("[FSM] NO_LIFT"));
//         }
//       } else {
//         releaseStartMs = 0;
//       }

//       // If ≥2 votes again, pop back to LIFT
//       const uint8_t votes2 = static_cast<uint8_t>(v_emg) + static_cast<uint8_t>(v_force) + static_cast<uint8_t>(v_motion);
//       if (votes2 >= 2) {
//         state = State::LIFT;
//         votesOffStartMs = 0;
//         releaseStartMs  = 0;
//         ForceInternal::force_setLiftActive(true);
//         Serial.println(F("[FSM] HOLD → LIFT"));
//       }
//     } break;
//   }

//   // Optional debug:
//   /*
//   const float W_kg = ForceInternal::force_getWeightKg();
//   Serial.print("state="); Serial.print(state==State::NO_LIFT?"NO_LIFT":(state==State::LIFT?"LIFT":"HOLD"));
//   Serial.print(", votes="); Serial.print(votes);
//   Serial.print(", emg="); Serial.print(v_emg);
//   Serial.print(", forceN="); Serial.print(sumN_ema, 2);
//   Serial.print(", motion="); Serial.print(v_motion);
//   Serial.print(", amag="); Serial.print(amag, 2);
//   Serial.print(", W_kg="); Serial.println(W_kg, 2);
//   */
// }

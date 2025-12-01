#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Need this for arduino to work
namespace AccelInternal {
  using ::Wire;
  using ::TwoWire;
  using ::Serial;
  using ::micros;
  using ::millis;
  using ::delay;

  const uint8_t MPU_ADDR = 0x68;
  const float   ACC_LSB_PER_G = 16384.0f;

  void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }

  void readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)6);
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
  }

  // Tuning
  const float DT_sec  = 0.02f;   // 50 Hz
  const float tau_sec = 0.25f;   // gravity LPF time constant
  const float alpha   = tau_sec / (tau_sec + DT_sec);

  // Motion thresholds on linear acceleration magnitude
  const float THRESH_HIGH = 0.121f; // enter moving
  const float THRESH_LOW  = 0.090f; // exit moving

  // Debounce samples
  const int ENTER_COUNT = 5;   // 100 ms at 50 Hz
  const int EXIT_COUNT  = 8;   // 160 ms

  // Intent parameters
  const unsigned long INTENT_MIN_MS = 350;  // must be moving this long for intent
  const unsigned long GAP_TOL_MS    = 60;
  const unsigned long REFRACTORY_MS = 400;
 
  static float ax = 0.0f, ay = 0.0f, az = 0.0f; // in g
  static float gx = 0.0f, gy = 0.0f, gz = 1.0f; // gravity estimate in g
  static float amag = 1.0f;                     // total accel magnitude in g

  enum MotionState { STILL, MOVING };
  static MotionState state = STILL;
  static int enter_ctr = 0, exit_ctr = 0;

  // Moving average for linear acceleration magnitude
  const int SMA_N = 5;
  static float ring[SMA_N] = {0};
  static int   ring_i = 0;
  static float sma_sum = 0;

  // Intent detection
  static bool intent = false;
  static unsigned long bout_start_ms = 0;
  static unsigned long last_moving_ms = 0;
  static unsigned long last_intent_ms = 0;

  void accel_setup() {
    Serial.begin(115200);
    while (!Serial) { }

    Wire.begin();
    Wire.setClock(400000);

    writeReg(0x6B, 0x00); // PWR_MGMT_1: wake up
    writeReg(0x1C, 0x00); // ACCEL_CONFIG: ±2g

    // Serial.println("ms,ax_g,ay_g,az_g,a_lin_raw,a_lin_smooth,state,intent");
  }

  void accel_loop() {
    int16_t ax_raw_i16, ay_raw_i16, az_raw_i16;
    readAccelRaw(ax_raw_i16, ay_raw_i16, az_raw_i16);

    // scale to g
    ax = (float)ax_raw_i16 / ACC_LSB_PER_G;
    ay = (float)ay_raw_i16 / ACC_LSB_PER_G;
    az = (float)az_raw_i16 / ACC_LSB_PER_G;

    // Gravity LPF
    gx = alpha * gx + (1.0f - alpha) * ax;
    gy = alpha * gy + (1.0f - alpha) * ay;
    gz = alpha * gz + (1.0f - alpha) * az;

    // Linear acceleration
    const float linx = ax - gx;
    const float liny = ay - gy;
    const float linz = az - gz;

    const float a_lin_raw = sqrtf(linx*linx + liny*liny + linz*linz);

    // SMA filter
    sma_sum -= ring[ring_i];
    ring[ring_i] = a_lin_raw;
    sma_sum += ring[ring_i];
    ring_i = (ring_i + 1) % SMA_N;
    const float a_lin_smooth = sma_sum / SMA_N;

    // Compute total accel magnitude
    amag = sqrtf(ax*ax + ay*ay + az*az);

    // Motion state machine 
    switch (state) {
      case STILL:
        if (a_lin_smooth > THRESH_HIGH) {
          if (++enter_ctr >= ENTER_COUNT) {
            state = MOVING;
            enter_ctr = 0;
            exit_ctr = 0;
            bout_start_ms = millis();
            last_moving_ms = bout_start_ms;
          }
        } else {
          enter_ctr = 0;
        }
        break;

      case MOVING:
        last_moving_ms = millis();
        if (a_lin_smooth < THRESH_LOW) {
          if (++exit_ctr >= EXIT_COUNT) {
            state = STILL;
            exit_ctr = 0;
            enter_ctr = 0;
          }
        } else {
          exit_ctr = 0;
        }
        break;
    }

    // Intent detection
    const unsigned long now = millis();
    const unsigned long effective_bout_ms =
      (state == MOVING)
        ? (now - bout_start_ms)
        : (((now - last_moving_ms) <= GAP_TOL_MS) ? (now - bout_start_ms) : 0UL);

    const bool sustained    = (effective_bout_ms >= INTENT_MIN_MS);
    const bool refractoryOK = (now - last_intent_ms) >= REFRACTORY_MS;

    if (!intent && sustained && refractoryOK) {
      intent = true;
      last_intent_ms = now;
    }

    // Reset intent when still
    if (state == STILL && (now - last_moving_ms) > GAP_TOL_MS) {
      intent = false;
      bout_start_ms = 0;
    }
  }

  // Getters
  float accel_getAmag() {
    return amag;
  }

  bool accel_getIntent() {
    return intent;
  }
}

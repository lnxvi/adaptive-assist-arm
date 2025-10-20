#include <Arduino.h>
#include <Wire.h>
namespace AccelInternal {
  using ::Wire;
  using ::TwoWire;
  using ::Serial;
  using ::micros;
  using ::millis;
  using ::delay;
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

const uint8_t MPU_ADDR = 0x68;         
const float   ACC_LSB_PER_G = 16384.0; 

// ---------- I2C helpers ----------
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)6);
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
}

// ---------- Tuning knobs ----------
const float DT_sec  = 0.02f;      
const float tau_sec = 0.25f;      
const float alpha   = tau_sec / (tau_sec + DT_sec);

// Motion thresholds (in g)
const float THRESH_HIGH = 0.121f;  // enter MOVING
const float THRESH_LOW  = 0.08f;  // exit MOVING

// Debounce samples
const int ENTER_COUNT = 5;        // 100 ms
const int EXIT_COUNT  = 15;       // 300 ms

// Sustained intent parameters
const unsigned long INTENT_MIN_MS = 550;   // must be moving at least this long
const unsigned long GAP_TOL_MS    = 120;   // brief stillness tolerated
const unsigned long REFRACTORY_MS = 800;   

// ---------- State ----------
enum MotionState { STILL, MOVING };
MotionState state = STILL;
int enter_ctr = 0, exit_ctr = 0;

// Gravity state
float gx = 0.0f, gy = 0.0f, gz = 1.0f;

// Simple moving average for linear-accel magnitude
const int SMA_N = 5;
float ring[SMA_N] = {0};
int   ring_i = 0;
float sma_sum = 0;

// Intent bookkeeping
bool intent = false;
unsigned long bout_start_ms = 0;
unsigned long last_moving_ms = 0;
unsigned long last_intent_ms = 0;

void accel_setup() {
  Serial.begin(115200);
  while (!Serial) { } 

  Wire.begin();          
  Wire.setClock(400000);  

  writeReg(0x6B, 0x00);   
  writeReg(0x1C, 0x00);   

  Serial.println("ms,ax_g,ay_g,az_g,a_lin_raw,a_lin_smooth,state,intent");
}

void accel_loop() {
  // ---- Read accelerometer ----
  int16_t ax_raw_i16, ay_raw_i16, az_raw_i16;
  readAccelRaw(ax_raw_i16, ay_raw_i16, az_raw_i16);

  float ax = ax_raw_i16 / ACC_LSB_PER_G;
  float ay = ay_raw_i16 / ACC_LSB_PER_G;
  float az = az_raw_i16 / ACC_LSB_PER_G;

  // ---- Gravity estimate ----
  gx = alpha * gx + (1.0f - alpha) * ax;
  gy = alpha * gy + (1.0f - alpha) * ay;
  gz = alpha * gz + (1.0f - alpha) * az;

  // ---- Linear acceleration ----
  float linx = ax - gx;
  float liny = ay - gy;
  float linz = az - gz;

  float a_lin_raw = sqrtf(linx*linx + liny*liny + linz*linz);

  sma_sum -= ring[ring_i];
  ring[ring_i] = a_lin_raw;
  sma_sum += ring[ring_i];
  ring_i = (ring_i + 1) % SMA_N;
  float a_lin_smooth = sma_sum / SMA_N;

  // ---- Motion state machine ----
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

  // ---- Sustained INTENT detection ----
  unsigned long now = millis();

  unsigned long effective_bout_ms =
    (state == MOVING)
      ? (now - bout_start_ms)
      : (((now - last_moving_ms) <= GAP_TOL_MS) ? (now - bout_start_ms) : 0UL);

  bool sustained    = (effective_bout_ms >= INTENT_MIN_MS);
  bool refractoryOK = (now - last_intent_ms) >= REFRACTORY_MS;

  if (!intent && sustained && refractoryOK) {
    intent = true;           
    last_intent_ms = now;
  }

  // Reset intent 
  if (state == STILL && (now - last_moving_ms) > GAP_TOL_MS) {
    intent = false;
    bout_start_ms = 0;
  }

  // ---- CSV logging ----
  Serial.print(now);                 Serial.print(",");
  Serial.print(ax, 3);               Serial.print(",");
  Serial.print(ay, 3);               Serial.print(",");
  Serial.print(az, 3);               Serial.print(",");
  Serial.print(a_lin_raw, 3);        Serial.print(",");
  Serial.print(a_lin_smooth, 3);     Serial.print(",");
  Serial.print(state == MOVING ? "MOVING" : "STILL");  Serial.print(",");
  Serial.println(intent ? "INTENT" : "NOINTENT");

  delay(20); 
  }
} 

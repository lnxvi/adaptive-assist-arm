#include <Arduino.h>
#include <Wire.h>

namespace MotorInternal {
  using ::Wire;
  using ::TwoWire;
  using ::Serial;
  using ::micros;
  using ::millis;
  using ::delay;

  static float g_mps2            = 9.80665f;
  static float r_hand_m          = 0.25f;    // elbow to hand (m)
  static float assist_ratio_0to1 = 0.60f;    // 0..1
  static float rho_elbow_m       = 0.035f;   // effective elbow moment arm (m)
  static float spool_radius_m    = 0.02761f;   // spool radius (m)
  static float mech_efficiency   = 0.80f;
  static float Kt_out_Nm_per_A   = 0.6934f;    // Nm/A 
  static float tau_cont_Nm       = 1.47f;    // continuous torque (Nm)
  static float I_cont_A          = 2.5f;     // A
  static float I_stall_A         = 5.0f;     // A

  static float in_weight_kg   = 0.0f;
  static float in_elbow_rad   = 1.570796f; // ~90°
  static float in_assist_frac = 0.60f;

  static float last_tauElbow_Nm = 0.0f;
  static float last_cable_N     = 0.0f;
  static float last_motor_I_A   = 0.0f;
  static float last_tauOut_Nm   = 0.0f;

  void  motor_setWeightKg(float wkg)     { in_weight_kg = (wkg < 0 ? 0 : wkg); }
  void  motor_setElbowAngleRad(float th) { in_elbow_rad = th; }
  void  motor_setAssistFraction(float a) { if (a<0)a=0; if(a>1)a=1; in_assist_frac = a; }
  float motor_getElbowTorqueNm() { return last_tauElbow_Nm; }
  float motor_getCableForceN()   { return last_cable_N; }
  float motor_getMotorCurrentA() { return last_motor_I_A; }
  float motor_getOutputTorqueNm(){ return last_tauOut_Nm; }

  void motor_setup() {
    Serial.begin(115200);
    delay(150);
    Serial.println("[Motor] ready.");
  }

  void motor_loop() {
    const float tau_obj_90 = in_weight_kg * g_mps2 * r_hand_m; 
    const float tau_grav   = tau_obj_90 * sinf(in_elbow_rad);

    // Assist at elbow
    float tau_elbow_cmd = tau_grav * in_assist_frac;
    if (!isfinite(tau_elbow_cmd) || tau_elbow_cmd < 0) tau_elbow_cmd = 0;

    // Elbow torque to cable force
    const float rho = (rho_elbow_m > 0.010f ? rho_elbow_m : 0.010f);
    float F_cable   = tau_elbow_cmd / rho;

    // Cable force to output torque
    float tau_out = (F_cable * spool_radius_m) / (mech_efficiency > 0.10f ? mech_efficiency : 0.10f);

    // Output torque to motor current, clamp to limits
    float I_A = (Kt_out_Nm_per_A > 0.0f) ? (tau_out / Kt_out_Nm_per_A) : 0.0f;
    if (!isfinite(I_A) || I_A < 0) I_A = 0.0f;
    if (I_A > I_stall_A) I_A = I_stall_A;

    if (tau_out > tau_cont_Nm) {
      tau_out = tau_cont_Nm;
      I_A     = tau_out / Kt_out_Nm_per_A;
      if (I_A > I_cont_A) I_A = I_cont_A;
    }

    last_tauElbow_Nm = tau_elbow_cmd;
    last_cable_N     = F_cable;
    last_tauOut_Nm   = tau_out;
    last_motor_I_A   = I_A;

    // Serial.print("[Motor] Wkg="); Serial.print(in_weight_kg,2);
    // Serial.print(" θ="); Serial.print(in_elbow_rad,3);
    // Serial.print(" τe="); Serial.print(last_tauElbow_Nm,2);
    // Serial.print(" Fc="); Serial.print(last_cable_N,1);
    // Serial.print(" τout="); Serial.print(last_tauOut_Nm,2);
    // Serial.print(" I="); Serial.println(last_motor_I_A,2);

    delay(150); 
  }
}

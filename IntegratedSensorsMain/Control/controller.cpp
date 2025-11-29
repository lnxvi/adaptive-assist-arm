#include "controller.h"

#include <stdint.h>
#include "Arduino.h"
#include <TimerThree.h>

// #define ENC_A 2
// #define ENC_B 3
#ifndef nSLEEP
#define nSLEEP 31
#endif

#ifndef PMODE
#define PMODE 30
#endif

#ifndef IN2
#define IN2 6
#endif

#ifndef IN1
#define IN1 9
#endif

#ifndef CS
#define CS A14  // pin 38
#endif

Controller::Controller(float Kp_init, float Ki_init, float Kd_init) 
  : Kp(Kp_init), Ki(Ki_init), Kd(Kd_init), I_setpoint(0.0f), 
  I_cmd(0.0f), error(0.0f), prev_error(0.0f), integral(0.0f) 
  {}

// torque-current conversion
float Controller::torqueToCurrent(float torque, float r_spool, float l_forearm, float Kt) {
  //Serial.println(F("[Controller] torqueToCurrent called"));
  return (torque*r_spool)/(l_forearm*Kt);
}

// PID loop on current
// measured current
// returns regulated current
float Controller::control(float i_meas, uint16_t dt) {
  float dt_s = (float)dt/1000.0;

  error = I_setpoint - i_meas;
  integral += 0.5*(error+prev_error)*dt_s;  // trapezoidal integration method
  // TODO windup check here

  // float derivative = (error-prev_error)/dt_s;

  // I_cmd = Kp*error;
  I_cmd = Kp*error + Ki*integral;
  // I_cmd = Kp*error + Ki*integral + Kd*derivative;

  prev_error = error;

  return I_cmd;
}

// current PWM conversion
uint16_t Controller::currentToPWM(float i, bool winding) {

  uint16_t duty;
  if (winding) duty = (i-b_wind)/m_wind;  // winding
  else duty = (i-b_unwind)/m_unwind;  // unwinding

  // duty = constrain(duty, MIN_PWM, MAX_PWM);  // should never go below 200 or above 1023
  
  // ramp increase decrease control
  if (prev_duty == 0) prev_duty = MIN_PWM;
  // if new duty is more than 25% increase from previous, cap at 25% increase
  if (duty > prev_duty+25) duty = prev_duty+25;  

  // if new duty is less than 25% decrease from previous, cap at 25% decrease
  else if (duty < prev_duty-25) duty = prev_duty-25;

  duty = constrain(duty, MIN_PWM, MAX_PWM);

  prev_duty = duty;
  return duty;
}

void Controller::sendMotorDuty(uint16_t duty, bool winding) {
  if (winding) Timer3.pwm(IN2, duty);
  else         Timer3.pwm(IN1, duty);  // send duty to timer
}

void Controller::reset() {
  error = 0.0f;
  integral = 0.0f;
  prev_error = 0.0f;
  I_cmd = 0.0f;
  prev_duty = 0;
}

void Controller::setKp(float new_Kp) {Kp = new_Kp;}
float Controller::getKp() {return Kp;}
void Controller::setKi(float new_Ki) {Ki = new_Ki;}
float Controller::getKi() {return Ki;}
void Controller::setKd(float new_Kd) {Kd = new_Kd;}
float Controller::getKd() {return Kd;}
void Controller::setIsetpoint(float new_i_setpoint) {I_setpoint = new_i_setpoint;}
float Controller::getIsetpoint() {return I_setpoint;}
void Controller::setIcmd(float new_i_cmd) {I_cmd = new_i_cmd;}  // I_cmd should really only change internally
float Controller::getIcmd() {return I_cmd;}
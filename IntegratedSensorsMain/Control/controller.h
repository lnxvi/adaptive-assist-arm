#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "Arduino.h"
#include <TimerThree.h>

class Controller {
    private:
        float Kp;  // proporional gain
        float Ki;
        float Kd;
        float I_setpoint;
        float I_cmd;
        float error;
        float prev_error;
        float integral;

        uint16_t prev_duty = 0;

        const float m_wind = 0.0062f;
        const float b_wind = 0.2483f;
        const float m_unwind = 0.0064f;
        const float b_unwind = 0.2094f;

        float const MAX_CURRENT = 3.0f;
        uint16_t MAX_PWM = 1023;
        uint16_t MIN_PWM = 500;

    public:
        Controller(float Kp = 0.0, float Ki = 0.0, float Kd = 0.0);  // constructor

        float torqueToCurrent(float torque, float r_spool, float l_forearm, float Kt);  // torque-current conversion
        float control(float i_meas, uint16_t dt);
        uint16_t currentToPWM(float i, bool winding=true);  // current PWM conversion
        void sendMotorDuty(uint16_t duty, bool winding=true); 

        void reset();

        // mutators, accessors
        void setKp(float new_Kp);
        float getKp();
        void setKi(float new_Ki);
        float getKi();
        void setKd(float new_Kd);
        float getKd();
        void setIsetpoint(float new_i_setpoint);
        float getIsetpoint();
        void setIcmd(float new_i_cmd);  // I_cmd should really only change internally
        float getIcmd();

};

#endif
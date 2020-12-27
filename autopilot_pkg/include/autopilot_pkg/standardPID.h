/*
* @file      StandardPID.h
* @date      11/6/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines a standard linear PID controller algorithm
*/
#ifndef __STANDARD_PID_H__
#define __STANDARD_PID_H__

// Included Libraries
#include <string>

// 3rd Party Libraries

// User Libraries
#include "controllers.h"

class StandardPID : public ControllerInterface
{
    public:
        StandardPID();
        void reset(void);
        void loadConfig(std::string & DOF);
        void setTarget(float val);
        void setStates(float val1, float val2, float val3);
        void process(void);
        float returnCmd(void);
        float returnTargetPosition(void);
        float returnTargetRate(void);
        float limit(float input, float limit);

    protected:
        float Kp, Ki, Kd;   // PID Gains For Positional Control
        float I;            // Integrator Sum
        float maxI, max;    // Integrator limit and maximum command
        float KpR, KiR, KdR;// PID Gains For Rate Control
        float IR;           // Integrator Sum (Rate)
        float maxIR, maxR;  // Integrator limit and maximum command (Rate)
        float target;       // Target state value
        float error;        // State error value
        float rate_error;   // State rate error value
        float state;        // Current state value
        float state_rate;   // Current state derivative (rate)
        float state_accel;  // Current state 2nd derivative (accel)
        float rate_cmd;     // Rate command from positional controller
        float cmd;          // Output command (0-100%)
        float dt;           // 1/Hz (controller frequency in seconds)
        float feed_forward; // Optional Feed forward term for the rate controller

};


#endif // __SUPER_TWISTING_SMC_H__
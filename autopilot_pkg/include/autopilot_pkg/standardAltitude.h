/*
* @file      StandardPID.h
* @date      11/6/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines a standard linear PID controller algorithm
*/
#ifndef __STANDARD_ALTITUDE_H__
#define __STANDARD_ALTITUDE_H__

// Included Libraries
#include <string>

// 3rd Party Libraries

// User Libraries
#include "controllers.h"

class StandardAltitude : public ControllerInterface
{
    public:
        StandardAltitude();
        void reset(void);
        void loadConfig(std::string & DOF);
        void setTarget(float val);
        void setStates(float val1, float val2, float val3);
        void process(void);
        float returnCmd(void);
        float returnTargetPosition(void);
        float returnTargetRate(void);
        void updateGravityThrottle(void);

    protected:
        float Kp, Ki, Kd;   // PID Gains For Positional Control
        float g_throttle;   // Throttle term to offset quad's mass*gravity
        float I;            // Integrator Sum
        float maxI, max;    // Integrator limit and maximum command
        float target;       // Target state value
        float error;        // State error value
        float state;        // Current state value
        float state_rate;   // Current state derivative (rate)
        float cmd;          // Output command (0-100%)
        float dt;           // 1/Hz (controller frequency in seconds)
        float filter_count; // Used to determine whether the quad is at hover

};


#endif // __STANDARD_ALTITUDE_H__
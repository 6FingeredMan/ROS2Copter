/*
* @file      SuperTwistingSMC.h
* @date      11/6/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the Super Twisting Slide Mode Control Algorithm Class
*/
#ifndef __SUPER_TWISTING_SMC_H__
#define __SUPER_TWISTING_SMC_H__

// Included Libraries
#include <string>

// 3rd Party Libraries

// User Libraries
#include "controllers.h"

class SuperTwistingSMC : public ControllerInterface
{
    public:
        SuperTwistingSMC();
        void reset(void);
        void loadConfig(std::string & DOF);
        void setTarget(float val);
        void setStates(float val1, float val2, float val3);
        void process(void);
        float returnCmd(void);
        float returnTargetPosition(void);
        float returnTargetRate(void);

    protected:
        float c, lambda, W; // Super Twisting Gains
        float maxI, max;    // Integrator limit and maximum command
        float target;       // Target state value
        float error;        // State error value
        float state;        // Current state value
        float state_rate;   // Current state derivative (rate)
        float s, u1, u2;    // Intermediate SMC values
        float cmd;          // Output command (0-100%)
        float dt;           // 1/Hz (controller frequency in seconds)

};


#endif // __SUPER_TWISTING_SMC_H__
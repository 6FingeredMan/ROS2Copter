/*
* @file      SuperTwistingSMC.cpp
* @date      11/6/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the Super Twisting Slide Mode Control Algorithm Class
*/

// Included Libraries
#include <math.h>

// 3rd Party Libraries
#include "INIReader.h"

// User Libraries
#include "superTwistingSMC.h"

SuperTwistingSMC::SuperTwistingSMC()
:
    c(0.0), lambda(0.0), W(0.0),
    maxI(0.0), max(0.0),
    target(0.0),
    error(0.0),
    state(0.0),
    state_rate(0.0),
    s(0.0), u1(0.0), u2(0.0),
    cmd(0.0),
    dt(0.0)
{
    reset();
}

void SuperTwistingSMC::reset(void)
{
    c = lambda = W = 0.0;
    maxI = max = 0.0;
    s = u1 = u2 = 0.0;
    target = state = state_rate = cmd = 0.0;
    dt = 0.02;
}

void SuperTwistingSMC::loadConfig(std::string & DOF)
{
    INIReader reader("/home/ubuntu/PiCopter/Config/picopter_config.ini");

    // First Read the Positional Controller Coefficients
    c = reader.GetReal(DOF, "c", 3.0);
    lambda = reader.GetReal(DOF, "lambda", 0.0);
    W = reader.GetReal(DOF, "W", 0.0);
    maxI = reader.GetReal(DOF, "maxI", 0.0);
    max = reader.GetReal(DOF, "max", 0.0);

}

void SuperTwistingSMC::setTarget(float val)
{
    target = val;
}

void SuperTwistingSMC::setStates(float val1, float val2, float val3)
{
    state = val1;
    state_rate = val2;
}

void SuperTwistingSMC::process(void)
{
    error = target - state;

    // Calculate the Sigma Function
    s = c * error - state_rate;

    // Calculate the Reaching Input
    u1 = lambda * sqrt(abs(s)) * copysign(1, s);

    // Calulate the non-linear integrator
    u2 += W * copysign(1, s) * dt;

    // Clamp the Integrator
    if(u2 > maxI)
    {
        u2 = maxI;
    }
    if(u2 < -maxI)
    {
        u2 = -maxI;
    }

    // Compute the command and clamp it
    cmd = u1 + u2;

    if(cmd > 100.0)
    {
        cmd = 100.0;
    }
    if(cmd < -max)
    {
        cmd = -max;
    }

}

float SuperTwistingSMC::returnCmd(void)
{
    return cmd;
}

float SuperTwistingSMC::returnTargetPosition(void)
{
    return cmd;
}

float SuperTwistingSMC::returnTargetRate(void)
{
    return 0.;
}




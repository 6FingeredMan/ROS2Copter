/*
* @file      StandardPID.cpp
* @date      11/6/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines a standard PID controller algorithm
*/

// Included Libraries
#include <iostream>
#include <math.h>
#include <string>

// 3rd Party Libraries
#include "INIReader.h"

// User Libraries
#include "standardAltitude.h"

StandardAltitude::StandardAltitude()
:
    Kp(0.0), Ki(0.0), Kd(0.0),
    g_throttle(0.0),
    I(0.0),
    maxI(0.0), max(0.0),
    target(0.0),
    error(0.0),
    state(0.0),
    state_rate(0.0),
    cmd(0.0),
    dt(0.02),
    filter_count(0.0)
{
    reset();
}

void StandardAltitude::reset(void)
{
    I = 0.0;
    target = state = state_rate = cmd = 0.0;
}

void StandardAltitude::loadConfig(std::string & DOF)
{
    INIReader reader("/home/ubuntu/PiCopter/Config/picopter_config.ini");

    // First Read the Positional Controller Coefficients
    Kp = reader.GetReal(DOF, "Kp", 3.0);
    Ki = reader.GetReal(DOF, "Ki", 0.0);
    Kd = reader.GetReal(DOF, "Kd", 0.0);
    maxI = reader.GetReal(DOF, "maxI", 0.0);
    max = reader.GetReal(DOF, "max", 0.0);

    // Then Read the Gravity Offeset Throttle
    g_throttle = reader.GetReal(DOF, "Gravity Throttle", 0.0);

}

void StandardAltitude::setTarget(float val)
{
    target = val;
}

void StandardAltitude::setStates(float val1, float val2, float val3)
{
    state = val1;
    state_rate = val2;
}


void StandardAltitude::process(void)
{

    ///////////////////////////////////////////////
    // POSITIONAL CONTROL 
    ///////////////////////////////////////////////
    error = target - state;

    I += Ki*error*dt;

    // Clamp the Integrator
    if(I > maxI)
    {
        I = maxI;
    }
    if(I < -maxI)
    {
        I = -maxI;
    }

    // Compute the command and clamp it
    cmd = Kp*error - Kd*state_rate + I + g_throttle;

    if(cmd > max)
    {
        cmd = max;
    }

    if(cmd < 0)
    {
        cmd = 0.0;
    }

    updateGravityThrottle();

}

float StandardAltitude::returnCmd(void)
{
    return cmd;
}

float StandardAltitude::returnTargetPosition(void)
{
    return target;
}

float StandardAltitude::returnTargetRate(void)
{
    return 0.0;
}

void StandardAltitude::updateGravityThrottle(void)
{
    // Check to see if the quad is hovering at the desired altitude (>0).
    // Check condition is altitude within +/- 0.25 meters for at least 4 seconds
    // If it is, zero out the Integrator term and reset the gravity throttle.
    // if(error < 0.25 && error > -0.25 && target > 0 && state_rate < 0.1 && state_rate > -0.1)
    // {
    //     filter_count += 1;
    // }
    // else
    // {
    //     filter_count = 0;
    // }

    // if(filter_count >= 200)
    // {
    //     I = 0.0;
    //     g_throttle = cmd;
    // }

}

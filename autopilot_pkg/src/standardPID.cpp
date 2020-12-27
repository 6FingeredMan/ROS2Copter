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
#include "standardPID.h"

StandardPID::StandardPID()
:
    Kp(0.0), Ki(0.0), Kd(0.0),
    I(0.0),
    maxI(0.0), max(0.0),
    KpR(0.0), KiR(0.0), KdR(0.0),
    IR(0.0),
    maxIR(0.0), maxR(0.0),
    target(0.0),
    error(0.0),
    rate_error(0.0),
    state(0.0),
    state_rate(0.0),
    state_accel(0.0),
    rate_cmd(0.0),
    cmd(0.0),
    dt(0.02),
    feed_forward(0.0)
{
    reset();
}

void StandardPID::reset(void)
{
    I = 0.0;
    IR = 0.0;
    target = state = state_rate = state_accel = rate_cmd = cmd = 0.0;
}

void StandardPID::loadConfig(std::string & DOF)
{
    INIReader reader("/home/ubuntu/PiCopter/Config/picopter_config.ini");

    // First Read the Positional Controller Coefficients
    Kp = reader.GetReal(DOF, "Kp", 0.0);
    Ki = reader.GetReal(DOF, "Ki", 0.0);
    Kd = reader.GetReal(DOF, "Kd", 0.0);
    maxI = reader.GetReal(DOF, "maxI", 0.0);
    max = reader.GetReal(DOF, "max", 0.0);

    // Then Read the Rate Controller Coefficients
    KpR = reader.GetReal(DOF, "Kp Rate", 0.0);
    KiR = reader.GetReal(DOF, "Ki Rate", 0.0);
    KdR = reader.GetReal(DOF, "Kd Rate", 0.0);
    maxIR = reader.GetReal(DOF, "maxI Rate", 0.0);
    maxR = reader.GetReal(DOF, "max Rate", 0.0);
    feed_forward = reader.GetReal(DOF, "feed forward", 0.0);

    // DEBUG ONLY
    // std::cout << "Created a " << DOF << " PID Controller." << std::endl;
    // std::cout << "Kp = " << Kp << " Ki = " << Ki << " Kd = " << Kd << std::endl;

}

void StandardPID::setTarget(float val)
{
    target = val;
}

void StandardPID::setStates(float val1, float val2, float val3)
{
    state = val1;
    state_rate = val2;
    state_accel = val3;
}


void StandardPID::process(void)
{

    ///////////////////////////////////////////////
    // POSITIONAL CONTROL 
    ///////////////////////////////////////////////
    error = target - state;

    I += Ki*error*dt;

    // Clamp the Integrator
    limit(I, maxI);

    // Compute the command and clamp it
    rate_cmd = Kp*error - Kd*state_rate + I;

    // Clamp the Rate command
    limit(rate_cmd, max);

    ///////////////////////////////////////////////
    // RATE CONTROL 
    ///////////////////////////////////////////////
    rate_error = rate_cmd - state_rate;

    IR += KiR*rate_error*dt;

    // Clamp the Integrator
    limit(IR, maxIR);

    // Compute the command and clamp it
    cmd = KpR*rate_error - KdR*state_accel + IR + feed_forward;

    // Clamp the Output command
    limit(cmd, maxR);
}

float StandardPID::returnCmd(void)
{
    return cmd;
}

float StandardPID::returnTargetPosition(void)
{
    return target;
}

float StandardPID::returnTargetRate(void)
{
    return rate_cmd;
}

float StandardPID::limit(float input, float limit)
{
    if(input > limit)
    {
        input = limit;
    }

    if(input < -limit)
    {
        input = -limit;
    }

    return input;
}

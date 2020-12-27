/*
* @file      Controllers.h
* @date      11/6/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the Controller Factory
*/

// Included Libraries
#include <map>
#include <string>

// 3rd Party Libraries

// User Libraries
#include "controllers.h"
#include "standardPID.h"
#include "standardAltitude.h"
#include "superTwistingSMC.h"

ControllerFactory *ControllerFactory::_instance = 0;

ControllerFactory::ControllerFactory() : _controllerNameToEnum() {
    _controllerNameToEnum[ "PID" ]      = STANDARD_PID;
    _controllerNameToEnum[ "ALTITUDE" ] = STANDARD_ALTITUDE;
    _controllerNameToEnum[ "STSMC" ]    = SUPER_TWISTING_SMC;
}

ControllerInterface *ControllerFactory::createController( std::string & type ) {
    std::map< std::string, enum ControllerType >::iterator iter = _controllerNameToEnum.find( type );
    switch( iter->second ) {
        case( STANDARD_PID ):
            return new StandardPID();
        case( STANDARD_ALTITUDE ):
            return new StandardAltitude();
        case( SUPER_TWISTING_SMC ):
            return new SuperTwistingSMC();
    }
    return NULL;
}
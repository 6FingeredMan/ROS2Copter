/*
* @file      Controllers.h
* @date      11/6/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the Controller Factory
*/
#ifndef __CONTROLLERS_H__
#define __CONTROLLERS_H__

// Included Libraries
#include <map>
#include <string>

// 3rd Party Libraries

// User Libraries

class ControllerInterface;

class ControllerFactory
{
    public:

	    // Enum containing the list of available controllers from the factory
        enum ControllerType
        {
            STANDARD_PID,
            STANDARD_ALTITUDE,
            SUPER_TWISTING_SMC
        };

        ControllerInterface *createController(std::string & controlType);

        static ControllerFactory *instance()
        {
            if (!_instance)
            {
                _instance = new ControllerFactory();
            }
            return _instance;
        }

    private:

	    explicit ControllerFactory();
        static ControllerFactory *_instance;
        std::map< std::string, enum ControllerType > _controllerNameToEnum;

};

class ControllerInterface
{
    public:
        virtual ~ControllerInterface() {}
        virtual void loadConfig(std::string & DOF) = 0;
        virtual void setTarget(float val) = 0;
        // Controllers get State, Rate, and Accel
        virtual void setStates(float val1, float val2, float val3) = 0;
        virtual void process(void) = 0;
        virtual void reset(void) = 0;
        virtual float returnCmd(void) = 0;
        virtual float returnTargetPosition(void) = 0;
        virtual float returnTargetRate(void) = 0;
        
};

#endif
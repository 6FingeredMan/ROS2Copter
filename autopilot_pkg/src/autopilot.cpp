/*
* @file      Autopilot.cpp
* @date      12/20/2020
* @copyright Brendan Martin
* @version   2.0.0
* @brief     Defines the Autopilot Class
*/

// Included Libraries
#include <chrono>
#include <iostream>
#include <map>
#include <math.h>
#include <string>

// 3rd Party Libraries
#include "INIReader.h"
#include "rclcpp/rclcpp.hpp"

// User Libraries
#include "autopilot.h"

// Interfaces
#include "picopter_interfaces/msg/autopilot_msg.hpp"
#include "picopter_interfaces/msg/elevation_msg.hpp"
#include "picopter_interfaces/msg/imu_msg.hpp"
#include "picopter_interfaces/msg/motors_msg.hpp"
#include "picopter_interfaces/msg/navigator_msg.hpp"

// Macros
using namespace std::chrono_literals;

Autopilot::Autopilot()
:
    Node("autopilot"),
    _controllerPrefix(),
    _controllerConfigType(),
    _controllers(),
    M1_cmd(0.0),
    M2_cmd(0.0),
    M3_cmd(0.0),
    M4_cmd(0.0),
    elevation_target(0.0),
    pitch_target(0.0),
    roll_target(0.0),
    yaw_target(0.0),
    pitch_val(0.0),
    roll_val(0.0),
    yaw_val(0.0),
    pitch_cmd(0.0),
    roll_cmd(0.0),
    yaw_cmd(0.0),
    z_cmd(0.0),
    idle_status(true),
    elevation(0.0),
    elevation_last(0.0),
    elevation_rate(0.0),
    elevation_rate_last(0.0),
    elevation_accel(0.0),
    R2D(57.2958),
    D2R(0.0174533),
    controller_frequency(10),
    navigator_override(false)
{
    _controllerPrefix[altitude] = "Altitude Control";
    _controllerPrefix[speed] = "Speed Control";
    _controllerPrefix[pitch] = "Pitch Control";
    _controllerPrefix[roll] = "Roll Control";
    _controllerPrefix[yaw] = "Yaw Control";

    loadConfig();
    startAutopilot();
}

void Autopilot::loadConfig(void)
{
    RCLCPP_INFO(this->get_logger(), "Autopilot is loading config.");
    // Declare an ini reader instance and pass it the location of the ini file
    INIReader reader("/home/ubuntu/PiCopter/Config/picopter_config.ini");

    // Get the update rate of the autopilot
    controller_frequency = reader.GetReal("Autopilot", "update rate (Hz)", 50);

    std::string temp; // Used as a temporary string to store an ini string

    // Loop through all of the controllers, read their congfig type, and return 
    // a new instance of that controller type. Then load the config of the controller.
    for (auto iter = _controllerPrefix.begin(); iter != _controllerPrefix.end(); ++iter)
    {
        temp = reader.Get(iter->second, "type", "UNKOWN");
        _controllerConfigType[iter->first] = temp;
    }

    for(auto iter = _controllerConfigType.begin(); iter != _controllerConfigType.end(); ++iter)
    {
        _controllers[iter->first] = ControllerFactory::instance()->createController(iter->second);
        _controllers[iter->first]->loadConfig(_controllerPrefix[iter->first]);
    }
    
}

void Autopilot::startAutopilot(void)
{

    // First, fire up the subscribers with their callbacks
    // Updates the attitude states - pitch, roll, yaw, pitch_rate, roll_rate, yaw_rate
    // OLD ROS1 METHOD - ahrs_sub = n.subscribe<picopter::Imu_msg>("imu_data", 1, &Autopilot::setAttitudeStates, this);
    ahrs_sub = this->create_subscription<picopter_interfaces::msg::ImuMSG>("imu_data", 1, std::bind(&Autopilot::setAttitudeStates, this, std::placeholders::_1));

    // Updates the idle status and zeroizes motor commands and integrators if true
    // OLD ROS1 METHOD - nav_sub = n.subscribe<picopter::Navigator_msg>("nav_data", 1, &Autopilot::setNavStates, this);
    nav_sub = this->create_subscription<picopter_interfaces::msg::NavigatorMSG>("nav_data", 1, std::bind(&Autopilot::setNavStates, this, std::placeholders::_1));

    // Updates the elevation status
    // OLD ROS1 METHOD - elevation_sub = n.subscribe<picopter::Elevation_msg>("elevation_data", 1, &Autopilot::setElevationState, this);
    elevation_sub = this->create_subscription<picopter_interfaces::msg::ElevationMSG>("elevation_data", 1, std::bind(&Autopilot::setElevationState, this, std::placeholders::_1));

    // Runs the main process
    // OLD ROS 1 METHOD - motor_pub = n.advertise<picopter::Motors_msg>("motor_cmds", 1);
    // OLD ROS 1 METHOD - pilot_pub = n.advertise<picopter::Autopilot_msg>("autopilot_data", 1);
    motor_pub = this->create_publisher<picopter_interfaces::msg::MotorsMSG>("motor_cmds", 1);
    pilot_pub = this->create_publisher<picopter_interfaces::msg::AutopilotMSG>("autopilot_data", 1);

    // ROS2 Timer Callback - Hardcoded to 50 Hz / 20 ms
    timer_ = this->create_wall_timer(20ms, std::bind(&Autopilot::runAutopilot, this));


    // TO DO - Get the code below working with a sleep instead of a timer callback

    // rclcpp::Rate loop_rate(controller_frequency);
    // while (rclcpp::ok())
    // {
    //     RCLCPP_INFO(this->get_logger(), "Autopilot is running.");
    //     // If the idle status is false, 
    //     // process the control loops and then 
    //     // publish the controller motor commands
    //     if(idle_status == false)
    //     {
    //         processLoops();
    //         motor_msg.m1 = M1_cmd;
    //         motor_msg.m2 = M2_cmd;
    //         motor_msg.m3 = M3_cmd;
    //         motor_msg.m4 = M4_cmd;
    //     }
    //     // If the idle status is true, zeroize the motor commands 
    //     // and prevent integrator windup by NOT processing the control loops.
    //     // NOT processing the control loops stil preserves the GOOD integrator.
    //     else 
    //     {
    //         motor_msg.m1 = 0.0;
    //         motor_msg.m2 = 0.0;
    //         motor_msg.m3 = 0.0;
    //         motor_msg.m4 = 0.0;
    //     }
    //     motor_pub->publish(motor_msg);
    //     pilot_pub->publish(pilot_msg);
    //     rclcpp::spin_some();
    //     loop_rate.sleep();
    // }

}

void Autopilot::runAutopilot(void)
{
    //RCLCPP_INFO(this->get_logger(), "Autopilot is running.");
    // If the idle status is false, 
    // process the control loops and then 
    // publish the controller motor commands
    if(idle_status == false)
    {
        processLoops();
        motor_msg.m1 = M1_cmd;
        motor_msg.m2 = M2_cmd;
        motor_msg.m3 = M3_cmd;
        motor_msg.m4 = M4_cmd;
    }
    // If the idle status is true, zeroize the motor commands 
    // and prevent integrator windup by NOT processing the control loops.
    // NOT processing the control loops stil preserves the GOOD integrator.
    else 
    {
        motor_msg.m1 = 0.0;
        motor_msg.m2 = 0.0;
        motor_msg.m3 = 0.0;
        motor_msg.m4 = 0.0;
    }
    motor_pub->publish(motor_msg);
    pilot_pub->publish(pilot_msg);
}

void Autopilot::setAttitudeStates(const picopter_interfaces::msg::ImuMSG::SharedPtr msg)
{
    _controllers[pitch]->setStates(msg->pitch, msg->pitch_rate, msg->pitch_rate_rate);
    _controllers[roll]->setStates(msg->roll, msg->roll_rate, msg->roll_rate_rate);
    _controllers[yaw]->setStates(msg->yaw, msg->yaw_rate, msg->yaw_rate_rate);
    pitch_val = msg->pitch;
    roll_val = msg->roll;
    yaw_val = msg->yaw;
}

void Autopilot::setNavStates(const picopter_interfaces::msg::NavigatorMSG::SharedPtr msg)
{
    elevation_target = msg->target_elevation;
    yaw_target = msg->target_course;
    idle_status = msg->idle;
    navigator_override = msg->override_attitude_targets;
    
    // If the navigator is trying to override the autopilot's
    // computed targets for the attitude controllers, then bypass the
    // conversion from Nav data to Attitude Targets.
    if(navigator_override == true)
    {
        roll_target = msg->roll_goal;
        pitch_target = msg->pitch_goal;
        yaw_target = msg->yaw_goal;
    }
    else // Let the autopilot determine the attitude states
    {
        convertNavToAttitude();
    }
    
}

void Autopilot::convertNavToAttitude(void)
{

    roll_target = 0.0;
    pitch_target = 0.0;
    yaw_target = 0.0;

}

void Autopilot::setElevationState(const picopter_interfaces::msg::ElevationMSG::SharedPtr msg)
{
    elevation = msg->elevation;
    elevation_rate = (elevation - elevation_last)*10.0; // TO DO - Add elevation rate to the elevation topic for direct computation in that publisher
    elevation_accel = (elevation_rate - elevation_rate_last)*10.0; // TO DO - Add elevation accel to the elevation topic for direct computation in that publisher
    elevation_last = elevation;
    elevation_rate_last = elevation_rate;
    
    _controllers[altitude]->setStates(elevation, elevation_rate, elevation_accel);
}

void Autopilot::setTargets(void)
{
    _controllers[altitude]->setTarget(elevation_target);
    _controllers[pitch]->setTarget(pitch_target);
    _controllers[roll]->setTarget(roll_target);
    _controllers[yaw]->setTarget(yaw_target); 
}

void Autopilot::processLoops(void)
{
    // Update the controller targets
    setTargets();

    // Process the positional controllers first
    _controllers[altitude]->process();
    _controllers[speed]->process();

    // Pass the output of the north and east controllers as input to the attitude controllers
    // TO DO

    // Process the control loops
    _controllers[pitch]->process();
    _controllers[roll]->process();
    _controllers[yaw]->process();

    // Get their output commands
    pitch_cmd = _controllers[pitch]->returnCmd();
    roll_cmd = _controllers[roll]->returnCmd();
    yaw_cmd = _controllers[yaw]->returnCmd();
    z_cmd = _controllers[altitude]->returnCmd();

    mix();
}

void Autopilot::mix(void)
{
    // Mixes the output commands from the attitude controllers
    // and updates the motor_cmd message before publishing to the topic.

    // Step 1 - compensate for pitch and roll to correct the altitude command...
    // in theory this lets the altitude controller integrators compensate for battery drain
    z_cmd = z_cmd / ( std::cos(pitch_val * D2R) * std::cos(roll_val * D2R) );
    z_cmd = limit(z_cmd, 100.0); // limit to 100% 
    z_cmd = limitZero(z_cmd);     // limit to 0 - 100%
    // ROS_ERROR_STREAM("z_cmd: " << z_cmd << " roll_cmd: " << roll_cmd << " pitch_cmd: " << pitch_cmd);

    M1_cmd = z_cmd;
    M2_cmd = z_cmd;
    M3_cmd = z_cmd;
    M4_cmd = z_cmd;

    // Step 2 - check for saturation
    // TO DO

    // Step 3 - add the pitch command
    M1_cmd += pitch_cmd;
    M2_cmd -= pitch_cmd;
    M3_cmd -= pitch_cmd;
    M4_cmd += pitch_cmd;

    // Step 4 -- add the roll command
    M1_cmd += roll_cmd;
    M2_cmd += roll_cmd;
    M3_cmd -= roll_cmd;
    M4_cmd -= roll_cmd;

    // Step 5 -- add the yaw command
    M1_cmd -= yaw_cmd;          // CW Prop, (-) yaw torque
    M2_cmd += yaw_cmd;          // CCW Prop, (+) yaw torque
    M3_cmd -= yaw_cmd;          // CW Prop, (-) yaw torque
    M4_cmd += yaw_cmd;          // CCW Prop, (+) yaw torque

    // Step 6 -- Check for Nan
    checkNaN();

    // Step 7 -- Limit Commands to be zero or above
    limitZero(M1_cmd);
    limitZero(M2_cmd);
    limitZero(M3_cmd);
    limitZero(M4_cmd);

    // Step 8 -- Prep Autopilot Data for publishing
    collectAutopilotData();

}

void Autopilot::saturate(void)
{
    // TO DO
}

void Autopilot::collectAutopilotData(void)
{
    // Publish the autopilot messages at the end of the chain
    pilot_msg.z_cmd = z_cmd;
    pilot_msg.pitch_cmd = pitch_cmd;
    pilot_msg.roll_cmd = roll_cmd;
    pilot_msg.yaw_cmd = yaw_cmd;
    pilot_msg.target_z_position = _controllers[altitude]->returnTargetPosition();
    pilot_msg.target_z_rate = _controllers[altitude]->returnTargetRate();
    pilot_msg.target_pitch_position = _controllers[pitch]->returnTargetPosition();
    pilot_msg.target_pitch_rate = _controllers[pitch]->returnTargetRate();
    pilot_msg.target_roll_position = _controllers[roll]->returnTargetPosition();
    pilot_msg.target_roll_rate = _controllers[roll]->returnTargetRate();
    pilot_msg.target_yaw_position = _controllers[yaw]->returnTargetPosition();
    pilot_msg.target_yaw_rate = _controllers[yaw]->returnTargetRate();

}

float Autopilot::limitZero(float input)
{
    if(input < 0)
    {
        input = 0.0;
    }

    return input;
}

float Autopilot::limit(float input, float limit)
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

void Autopilot::checkNaN(void)
{
    if (isnan(M1_cmd) || isnan(M2_cmd) || isnan(M3_cmd) || isnan(M4_cmd))
    {
        M1_cmd = 0.0;
        M2_cmd = 0.0;
        M3_cmd = 0.0;
        M4_cmd = 0.0;
    }
}
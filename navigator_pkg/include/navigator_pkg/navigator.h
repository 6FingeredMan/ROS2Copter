/*
* @file      Navigator.h
* @date      12/26//2020
* @copyright Brendan Martin
* @version   2.0.0
* @brief     Defines the Navigator Class
*/
#ifndef __NAVIGATOR_H__
#define __NAVIGATOR_H__

// Included Libraries
#include <chrono>
#include <ctime>
#include <cstdint>
#include <time.h>
#include <map>

// 3rd Party Librariess
#include "rclcpp/rclcpp.hpp"
#include "INIReader.h"

// User Libraries

// Interface Libraries
#include "picopter_interfaces/msg/gps_msg.hpp"
#include "picopter_interfaces/msg/altitude_msg.hpp"
#include "picopter_interfaces/msg/navigator_msg.hpp"

class Navigator : public rclcpp::Node
{
public:

	// Constructor
	Navigator();

    // Destructor
    //~Autopilot();

	// Checks the validity of the mission file
	void missionCheck(void);

	// Start the ROS processing loops
	void startNavigator(void);

    // Timer callback to run the navigator
    void runNavigator(void);

    // Sets the current lat/lon and northings/eastings
    void setLatLon(const picopter_interfaces::msg::GpsMSG::SharedPtr msg);

    // Sets the current altitude in meters
    void setAltitude(const picopter_interfaces::msg::AltitudeMSG::SharedPtr msg);

	// Processes the mission 
	void process(void);

    // Sets current target states for the controllers
	void setTargetStates(void);

    // Creates a basic mission file with all available behaviors
    void createMissionTemplate(void);

    // Verifies that the mission file is good for run
    void verifyMission(void);

    // Gets the system time and returns a time in seconds
    double getTime(void);

	// Public variable declarations
    bool RUN_OK;
    float duration_minutes;
    float duration_seconds;
    float course_target;
    float elevation_target;
    float speed_target;
    float altitude;
    float min_altitude;
    float heading_rate_target;
    float latitude;
    float longitude;
    float northings;
    float eastings;
    bool idle_status;
    int objective_number;
    int prev_objective_number;
    int end_objective_number;
    double obj_start_time;
    double obj_end_time;
    float navigator_frequency;
    bool nav_override_status;
    float roll_goal;
    float pitch_goal;
    float yaw_goal;
    std::string curObjHeader;
    std::string curObj;
    picopter_interfaces::msg::NavigatorMSG nav_msg;

    // Mission Reader
    INIReader reader;

    // Enum containing the list of available behaviors
    enum Objectives
    {
        IDLE,
        HOVER,
        HOVER_ROTATE,
        HOLD_STATION,
        HOLD_STATION_ROTATE,
        NAVIGATE,
        LAND
    };

private:

	// ROS hooks
	// ros::NodeHandle n;
    // ros::Publisher nav_pub;
    // ros::Subscriber gps_sub;
    // ros::Subscriber altitude_sub;

    // ROS2 hooks (Foxy)
    rclcpp::Subscription<picopter_interfaces::msg::GpsMSG>::SharedPtr gps_sub;
    rclcpp::Subscription<picopter_interfaces::msg::AltitudeMSG>::SharedPtr altitude_sub;
    rclcpp::Publisher<picopter_interfaces::msg::NavigatorMSG>::SharedPtr nav_pub;

    // Used for the ROS2 callback
	rclcpp::TimerBase::SharedPtr timer_;

    // Behaviors / Objectives
    std::map< std::string, enum Objectives > ObjectivesToEnum;

    // Idle behavior - no outputs and waits for transition command
    void idle(void);

    // Hover behavior - holds steady elevation and zero attitude, does
    // not attempt to maintain a lat/lon
    void hover(void);

    // Hover rotate behavior - holds steady elevation and rotates about the yaw axis,
    // does not attempt to maintain a lat/lon
    void hoverRotate(void);

    // Hold station behavior - holds steady elevation and maintains lat/lon
    void holdStation(void);

    // Hold station rotate behavior - holds steady elevation and maintains lat/lon
    // while rotating about the yaw axis
    void holdStationRotate(void);

    // Navigate behavior - guides the quad to a waypoint
    void navigate(void);

    // Land behavior - lands the quad
    void land(void);

    // Testing Objective
    void setStatesExplicit(void);
	
};

#endif // __NAVIGATOR_H__
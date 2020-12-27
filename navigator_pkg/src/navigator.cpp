/*
* @file      Navigator.cpp
* @date      12/26/2020
* @copyright Brendan Martin
* @version   2.0.0
* @brief     Defines the Navigator Class
*/

// Included Libraries
#include <chrono>
#include <ctime>
#include <iostream>
#include <fstream>
#include <map>
#include <math.h>
#include <string>
#include <time.h>

// 3rd Party Libraries
#include "INIReader.h"
#include "rclcpp/rclcpp.hpp"

// User Libraries
#include "navigator.h"

// Interfaces
#include "picopter_interfaces/msg/gps_msg.hpp"
#include "picopter_interfaces/msg/altitude_msg.hpp"
#include "picopter_interfaces/msg/navigator_msg.hpp"

// Macros
using namespace std::chrono_literals;

Navigator::Navigator()
:
    Node("navigator"),
    RUN_OK(false),
    duration_minutes(60.0),
    duration_seconds(3600.0),
    course_target(0.0),
    elevation_target(0.0),
    speed_target(0.0),
    altitude(0.0),
    min_altitude(0.0),
    heading_rate_target(0.0),
    latitude(0.0),
    longitude(0.0),
    northings(0.0),
    eastings(0.0),
    idle_status(true),
    objective_number(1),
    prev_objective_number(0),
    end_objective_number(0),
    nav_override_status(false),
    roll_goal(0.0),
    pitch_goal(0.0),
    yaw_goal(0.0),
    curObjHeader("Objective "),
    curObj("Idle"),
    reader("/home/ubuntu/PiCopter/Missions/mission.ini")
{
    missionCheck();
    startNavigator();
}

void Navigator::missionCheck(void)
{
    // Check if the mission file exists
    if (reader.ParseError() != 0)
    {
        RCLCPP_INFO(this->get_logger(), "Navigator could not find a mission...");
        RCLCPP_INFO(this->get_logger(), "Creating a mission template and aborting mission.");
        RUN_OK = false; // prevent the quad from running
        createMissionTemplate(); // create a basic mission file
    }

    // Verify Mission
    verifyMission();

}

void Navigator::startNavigator(void)
{
    RCLCPP_INFO(this->get_logger(), "Navigator is loading config.");
    // Get the update rate of the navigator for publishing
    INIReader config_reader("/home/ubuntu/PiCopter/Config/picopter_config.ini");
    navigator_frequency = config_reader.GetReal("Navigator", "update rate (Hz)", 50);

    // First, fire up the subscribers with their callbacks
    // OLD ROS1 METHOD - gps_sub = n.subscribe<picopter::Gps_msg>("gps_data", 1, &Navigator::setLatLon, this);
    // OLD ROS1 METHOD - altitude_sub = n.subscribe<picopter::Altitude_msg>("altitude_data", 1, &Navigator::setAltitude, this);
    gps_sub = this->create_subscription<picopter_interfaces::msg::GpsMSG>("gps_data", 1, std::bind(&Navigator::setLatLon, this, std::placeholders::_1));
    altitude_sub = this->create_subscription<picopter_interfaces::msg::AltitudeMSG>("altitude_data", 1, std::bind(&Navigator::setAltitude, this, std::placeholders::_1));
    
    // Runs the main process
    // OLD ROS1 METHOD - nav_pub = n.advertise<picopter::Navigator_msg>("nav_data", 1);
    nav_pub = this->create_publisher<picopter_interfaces::msg::NavigatorMSG>("nav_data", 1);

    // ROS2 Timer Callback - Hardcoded to 50 Hz / 20 ms
    timer_ = this->create_wall_timer(20ms, std::bind(&Navigator::runNavigator, this));

    // TO DO - Get code below working with a sleep instead of a timer callback
    // ros::Rate loop_rate(navigator_frequency);
    // while (ros::ok)
    // {
    //     process();
    //     nav_msg.target_course = course_target;
    //     nav_msg.target_elevation = elevation_target;
    //     nav_msg.target_speed = speed_target;
    //     nav_msg.idle = idle_status;
    //     nav_msg.current_objective = curObj;
    //     if(end_objective_number - objective_number >= 0)
    //     {
    //         nav_msg.objectives_remaining = end_objective_number - objective_number;
    //     }
    //     else
    //     {
    //         nav_msg.objectives_remaining = 0;
    //     }
    //     nav_msg.override_attitude_targets = nav_override_status;
    //     nav_msg.roll_goal = roll_goal;
    //     nav_msg.pitch_goal = pitch_goal;
    //     nav_msg.yaw_goal = yaw_goal;
        
    //     nav_pub.publish(nav_msg);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
}

void Navigator::runNavigator(void)
{
    process();
    nav_msg.target_course = course_target;
    nav_msg.target_elevation = elevation_target;
    nav_msg.target_speed = speed_target;
    nav_msg.idle = idle_status;
    nav_msg.current_objective = curObj;
    if(end_objective_number - objective_number >= 0)
    {
        nav_msg.objectives_remaining = end_objective_number - objective_number;
    }
    else
    {
        nav_msg.objectives_remaining = 0;
    }
    nav_msg.override_attitude_targets = nav_override_status;
    nav_msg.roll_goal = roll_goal;
    nav_msg.pitch_goal = pitch_goal;
    nav_msg.yaw_goal = yaw_goal;
    
    nav_pub->publish(nav_msg);

}

void Navigator::setLatLon(const picopter_interfaces::msg::GpsMSG::SharedPtr msg)
{
    latitude = msg->lat;
    longitude = msg->lon;
    northings = msg->northings;
    eastings = msg->eastings;
}

void Navigator::setAltitude(const picopter_interfaces::msg::AltitudeMSG::SharedPtr msg)
{
    altitude = msg->altitude;
}

void Navigator::process(void)
{
    // Check to see if the quad should be entering the next objective.
    // If it is, parse through the mission file set the curObj
    // string so that we call the appropriate objective function.
    if(objective_number > prev_objective_number && objective_number <= end_objective_number)
    {
        curObjHeader = "Objective " + std::to_string(objective_number);
        curObj = reader.Get(curObjHeader, "type", "UNKNOWN");
        prev_objective_number = objective_number;
        setTargetStates();
        obj_start_time = getTime();
        obj_end_time = obj_start_time + duration_seconds;
    }

    // Check to see if the quad is at the end of the mission
    // and force an idle state if it is
    if(objective_number > end_objective_number)
    {
        idle();
        return;
    }

    // This super hacky - TO DO - use a map instead
    if(curObj == "Idle")
    {
        idle();
        return;
    }
    if(curObj == "Hover")
    {
        hover();
        return;
    }
    if(curObj == "Hover Rotate")
    {
        hoverRotate();
        return;
    }
    if(curObj == "Hold Station")
    {
        holdStation();
        return;
    }
    if(curObj == "Hold Station Rotate")
    {
        holdStationRotate();
        return;
    }
    if(curObj == "Navigate")
    {
        navigate();
        return;
    }
    if(curObj == "Land")
    {
        land();
        return;
    }
    if(curObj == "Set States Explicit")
    {
        setStatesExplicit();
        return;
    }
}

void Navigator::setTargetStates(void)
{
    // Try to get a duration
    try
    {
        duration_minutes = reader.GetReal(curObjHeader, "Duration (minutes)", 60);
        duration_seconds = duration_minutes * 60.0;
    }
    catch (int e)
    {
        duration_minutes = 60.0;
        duration_seconds = duration_minutes * 60.0;
    }
    // Try to get an elevation target
    try
    {
        elevation_target = reader.GetReal(curObjHeader, "Elevation (meters)", 0);
    }
    catch (int e)
    {
        elevation_target = 0.0;
    }
    // Try to get a minimum altitude
    try
    {
        min_altitude = reader.GetReal(curObjHeader, "Minimum Altitude (meters)", 0);
    }
    catch (int e)
    {
        min_altitude = 0.0;
    }
    // Try to get a speed target
    try
    {
        speed_target = reader.GetReal(curObjHeader, "Speed (m/s)", 0);
    }
    catch (int e)
    {
        speed_target = 0.0;
    }
    // Try to get a heading rate target
    try
    {
        heading_rate_target = reader.GetReal(curObjHeader, "Rate (deg/s)", 0);
    }
    catch (int e)
    {
        heading_rate_target = 0.0;
    }

    roll_goal = reader.GetReal(curObjHeader, "Roll Goal (deg)", 0.0);
    pitch_goal = reader.GetReal(curObjHeader, "Pitch Goal (deg)", 0.0);
    yaw_goal = reader.GetReal(curObjHeader, "Yaw Goal (deg)", 0.0);


}

void Navigator::idle(void)
{
   idle_status = true;

   // Check to see if the elapsed time since the objective start has
   // reached the time limit set in the mission file. If it has, bump
   // up the current objective number to start the next objective
   if (getTime() >= obj_end_time)
   {
       objective_number += 1;
   }
   
}

void Navigator::hover(void)
{
    idle_status = false;

    // Check to see if the elapsed time since the objective start has
   // reached the time limit set in the mission file. If it has, bump
   // up the current objective number to start the next objective
   if (getTime() >= obj_end_time)
   {
       objective_number += 1;
   }
}

void Navigator::hoverRotate(void)
{
    // TO DO
}

void Navigator::holdStation(void)
{
    // TO DO
}

void Navigator::holdStationRotate(void)
{
    // TO DO
}

void Navigator::navigate(void)
{
    // TO DO
}

void Navigator::land(void)
{
    idle_status = false;
    if(altitude <= .05)
    {
        objective_number += 1;
    }

    // TO DO - create a flag for the autopilot to gracefully land
}

void Navigator::setStatesExplicit(void)
{
    // This Function Doesn't Need to do Anything Unique
    // The Autopilot will see the current objective
    // as Set States Explicit and set the objective 
    // values to the targets
    idle_status = false;
    nav_override_status = true;

    if (getTime() >= obj_end_time)
   {
       objective_number += 1;
       nav_override_status = false;
   }
}

void Navigator::createMissionTemplate(void)
{
    std::ofstream tempFile("/home/ubuntu/PiCopter/Missions/basic_mission.ini");
	if (tempFile.is_open())
	{
        tempFile << "[Objective 1]\n";
        tempFile << "Type = Idle\n";
        tempFile << "Duration (minutes) = 0.5\n";
        tempFile << "\n";

        tempFile << "[Objective 2]\n";
        tempFile << "Type = Hover\n";
        tempFile << "Elevation (meters) = 10\n";
        tempFile << "Minimum Altitude (meters) = 1.0\n";
        tempFile << "Duration (minutes) = 1.0\n";
        tempFile << "\n";

        tempFile << "[Objective 3]\n";
        tempFile << "Type = Hover Rotate\n";
        tempFile << "Rate (deg/s) = 10\n";
        tempFile << "Elevation (meters) = 10\n";
        tempFile << "Minimum Altitude (meters) = 1.0\n";
        tempFile << "Duration (minutes) = 1.0\n";
        tempFile << "\n";

        tempFile << "[Objective 4]\n";
        tempFile << "Type = Hold Station\n";
        tempFile << "Latitude = \n";
        tempFile << "Longitude = \n";
        tempFile << "Elevation (meters) = 10\n";
        tempFile << "Minimum Altitude (meters) = 1.0\n";
        tempFile << "Duration (minutes) = 1.0\n";
        tempFile << "\n";

        tempFile << "[Objective 5]\n";
        tempFile << "Type = Hold Station Rotate\n";
        tempFile << "Rate (deg/s) = 10\n";
        tempFile << "Latitude = \n";
        tempFile << "Longitude = \n";
        tempFile << "Elevation (meters) = 10\n";
        tempFile << "Minimum Altitude (meters) = 1.0\n";
        tempFile << "Duration (minutes) = 1.0\n";
        tempFile << "\n";

        tempFile << "[Objective 6]\n";
        tempFile << "Type = Navigate\n";
        tempFile << "Latitude = \n";
        tempFile << "Longitude = \n";
        tempFile << "Elevation (meters) = 10\n";
        tempFile << "Minimum Altitude (meters) = 1.0\n";
        tempFile << "Orientation = Any\n";
        tempFile << "Speed (m/s) = 1\n";
        tempFile << "\n";

        tempFile << "[Objective 7]\n";
        tempFile << "Type = Land\n";
        tempFile << "\n";

        tempFile << "[Objective 8]\n";
        tempFile << "Type = Idle\n";
        tempFile << "Duration (minutes) = 0.5\n";
        tempFile << "\n";

        tempFile.close();
    }
}

void Navigator::verifyMission(void)
{

    // First determine how many objectives there are
    std::set<std::string> sections = reader.Sections();
    for (std::set<std::string>::iterator it = sections.begin(); it != sections.end(); ++it)
    {
        end_objective_number +=1;
    }

    // TO DO - make sure mission file isn't f'd up

}

double Navigator::getTime(void)
{
    struct timeval t;
	gettimeofday(&t, NULL);
	double time = (t.tv_sec + (t.tv_usec / 1000000.0)); // returns the time of day in seconds
	return time;
}
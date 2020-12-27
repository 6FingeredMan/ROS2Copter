/*
* @file      Autopilot.h
* @date      12/20/2020
* @copyright Brendan Martin
* @version   2.0.0
* @brief     Defines the Autopilot Class
*/
#ifndef __AUTOPILOT_H__
#define __AUTOPILOT_H__

// Included Libraries
#include <cstdint>
#include <map>

// 3rd Party Libraries
#include "rclcpp/rclcpp.hpp"

// User Libraries
#include "controllers.h"

// Interface Libraries
#include "picopter_interfaces/msg/autopilot_msg.hpp"
#include "picopter_interfaces/msg/elevation_msg.hpp"
#include "picopter_interfaces/msg/imu_msg.hpp"
#include "picopter_interfaces/msg/motors_msg.hpp"
#include "picopter_interfaces/msg/navigator_msg.hpp"

class Autopilot : public rclcpp::Node
{
public:

	// Constructor
	Autopilot();

    // Destructor
    //~Autopilot();

	// Load Components
	void loadConfig(void);

	// Start the ROS processing loops
	void startAutopilot(void);

	// Timer callback to run the autopilot
	void runAutopilot(void);

	// Sets current attitude states for the controllers
	void setAttitudeStates(const picopter_interfaces::msg::ImuMSG::SharedPtr msg);

	// Sets the navigation states for the controllers
	void setNavStates(const picopter_interfaces::msg::NavigatorMSG::SharedPtr msg);

	// Converts a target course & speed to pitch & roll targets
	void convertNavToAttitude(void);

	// Sets elevation state for the controller
	void setElevationState(const picopter_interfaces::msg::ElevationMSG::SharedPtr msg);

	// Sets target states for the controllers
	void setTargets(void);

	// Processes all of the control loops
	void processLoops(void);

	// Handles controller mixing and priority of controls
	// and computes motor commands
	void mix(void);

	// Checks to see if any controllers are saturating the throttle
	// command below zero or above 100
	void saturate(void);

	// Checks for NaN output and sets motors to zero if true
	void checkNaN(void);

	// Collects Autopilot data to publish
	void collectAutopilotData(void);

	float limitZero(float input);

	float limit(float input, float limit);

	// Public variable declarations
	float M1_cmd;
	float M2_cmd;
	float M3_cmd;
	float M4_cmd;
	float elevation_target;
	float pitch_target;
	float roll_target;
	float yaw_target;
	float pitch_val;
	float roll_val;
	float yaw_val;
	float pitch_cmd;
	float roll_cmd;
	float yaw_cmd;
	float z_cmd;
	bool idle_status;
	float elevation;
	float elevation_last;
	float elevation_rate;
	float elevation_rate_last;
	float elevation_accel;
	float R2D;
	float D2R;
	float controller_frequency;
	bool navigator_override;
	picopter_interfaces::msg::MotorsMSG motor_msg;
	picopter_interfaces::msg::AutopilotMSG pilot_msg;


private:

	// ROS hooks
	// ros::NodeHandle n;
	// ros::Publisher motor_pub;
	// ros::Publisher pilot_pub;
	// ros::Subscriber ahrs_sub;
	// ros::Subscriber nav_sub;
	// ros::Subscriber elevation_sub;

	// ROS2 hooks (Foxy)
	rclcpp::Subscription<picopter_interfaces::msg::ImuMSG>::SharedPtr ahrs_sub;
	rclcpp::Subscription<picopter_interfaces::msg::NavigatorMSG>::SharedPtr nav_sub;
	rclcpp::Subscription<picopter_interfaces::msg::ElevationMSG>::SharedPtr elevation_sub;
	rclcpp::Publisher<picopter_interfaces::msg::MotorsMSG>::SharedPtr motor_pub;
	rclcpp::Publisher<picopter_interfaces::msg::AutopilotMSG>::SharedPtr pilot_pub;

	// Controller Factory Hooks
	enum {altitude, speed, pitch, roll, yaw};

	std::map< uint32_t, std::string > _controllerPrefix;
    std::map< uint32_t, std::string > _controllerConfigType;
    std::map< uint32_t, ControllerInterface *> _controllers;

	// Used for the ROS2 callback
	rclcpp::TimerBase::SharedPtr timer_;
	
};

#endif // __AUTOPILOT_H__
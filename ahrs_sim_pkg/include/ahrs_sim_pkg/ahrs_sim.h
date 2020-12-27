/*
* @file      ahrs_sim.h
* @date      12/26/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the AHRS simulator class
*/
#ifndef __AHRS_SIM_H__
#define __AHRS_SIM_H__

// Included Libraries
#include <cstdint>
#include <map>

// 3rd Party Libraries
#include "rclcpp/rclcpp.hpp"

// Interface Libraries
#include "picopter_interfaces/msg/imu_msg.hpp"
#include "picopter_interfaces/msg/simulator_msg.hpp"

// Macros

class AHRS_SIM : public rclcpp::Node
{
public:

	// Constructor
	AHRS_SIM();

    // Destructor
    //~Autopilot();

	// Load Components
	void loadConfig(void);

	// Start the ROS processing loops
	void startAhrs(void);

    // Run the AHRS loop
    void runAhrs(void);

	// Sets current attitude states for the controllers
	void setStates(const picopter_interfaces::msg::SimulatorMSG::SharedPtr msg);

	// Public variable declarations
	picopter_interfaces::msg::ImuMSG ahrs_msg;
    float pitch;
    float roll;
    float yaw;
    float pitch_noise;
    float roll_noise;
    float yaw_noise;
    float pitch_rate_noise;
    float roll_rate_noise;
    float yaw_rate_noise;
    float sim_rate;
	float R2D;
    float pitch_last;
    float roll_last;
    float yaw_last;
    float pitch_rate_last;
    float roll_rate_last;
    float yaw_rate_last;

private:

	// ROS hooks
	// ros::NodeHandle n;
	// ros::Publisher ahrs_pub;
	// ros::Subscriber sim_sub;

    // ROS2 hooks (Foxy)
    rclcpp::Subscription<picopter_interfaces::msg::SimulatorMSG>::SharedPtr sim_sub;
    rclcpp::Publisher<picopter_interfaces::msg::ImuMSG>::SharedPtr ahrs_pub;

    // Used for the ROS2 callback
	rclcpp::TimerBase::SharedPtr timer_;
	
};

#endif // __AHRS_SIM_H__
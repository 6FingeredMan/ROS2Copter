/*
* @file      ahrs_sim_node.cpp
* @date      12/26/2020
* @copyright Brendan Martin
* @version   2.0.0
* @brief     The ROS node that simulates the RazorAHRS object and publishes IMU data based on simulation data
*/

// Included Libraries
#include <string>

// 3rd Party Libraries
#include "rclcpp/rclcpp.hpp"

// User Libraries
#include "ahrs_sim.h"

// Interface Libraries
#include "picopter_interfaces/msg/imu_msg.hpp"
#include "picopter_interfaces/msg/simulator_msg.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the AHRS sim instance
    auto node = std::make_shared<AHRS_SIM>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
/*
* @file      autopilot_node.cpp
* @date      11/10/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     The ROS node that utilizes the Autopilot object and subscribes to imu and altitude data and publishes to motor commands.
*/

// Included Libraries
#include <string>

// 3rd Party Libraries
#include "rclcpp/rclcpp.hpp"

// User Libraries
#include "autopilot.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the autopilot instance
    auto node = std::make_shared<Autopilot>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
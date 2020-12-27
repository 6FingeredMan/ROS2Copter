/*
* @file      simulator_node.cpp
* @date      12/26/2020
* @copyright Brendan Martin
* @version   2.0.0
* @brief     The ROS node that utilizes the Simulator object and subscribes to motor commands and publishes simulated state data.
*/

// Included Libraries
#include <string>

// 3rd Party Libraries
#include "rclcpp/rclcpp.hpp"

// User Libraries
#include "simulator.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the Simulator instance
    auto node = std::make_shared<Simulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
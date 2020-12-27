/*
* @file      navigator_node.cpp
* @date      12/26/2020
* @copyright Brendan Martin
* @version   2.0.0
* @brief     The ROS node that utilizes the Navigator object.
*            Subcribes to the following topics:
*            - GPS
*            - Elevation
*            - Altitude
*            - Simulator
*            - 
*            Publishes to the following topics:
*            - TargetStates
*            
*/

// Included Libraries
#include <string>

// 3rd Party Libraries
#include "rclcpp/rclcpp.hpp"

// User Libraries
#include "navigator.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the AHRS instance
    auto node = std::make_shared<Navigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
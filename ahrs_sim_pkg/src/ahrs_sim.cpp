/*
* @file      ahrs_sim.cpp
* @date      11/4/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the Autopilot Class
*/

// Included Libraries
#include <iostream>
#include <map>
#include <math.h>
#include <string>

// 3rd Party Libraries
#include "INIReader.h"
#include "rclcpp/rclcpp.hpp"

// User Libraries
#include "ahrs_sim.h"

// Interface Libraries
#include "picopter_interfaces/msg/imu_msg.hpp"
#include "picopter_interfaces/msg/simulator_msg.hpp"

// Macros
using namespace std::chrono_literals;

AHRS_SIM::AHRS_SIM()
:
    Node("ahrs_sim"),
    pitch(0.0),
    roll(0.0),
    yaw(0.0),
    pitch_noise(0.0),
    roll_noise(0.0),
    yaw_noise(0.0),
    pitch_rate_noise(0.0),
    roll_rate_noise(0.0),
    yaw_rate_noise(0.0),
    sim_rate(100.0),
    R2D(57.2958),
    pitch_last(0.0),
    roll_last(0.0),
    yaw_last(0.0),
    pitch_rate_last(0.0),
    roll_rate_last(0.0),
    yaw_rate_last(0.0)
{
    loadConfig();
    startAhrs();
}

void AHRS_SIM::loadConfig(void)
{
    RCLCPP_INFO(this->get_logger(), "AHRS simulator is loading config.");
    // Declare an ini reader instance and pass it the location of the ini file
    INIReader reader("/home/ubuntu/PiCopter/Simulation/simulation.ini");
    pitch_noise = reader.GetReal("AHRS", "pitch noise amplitude (deg)", 0.0);
    pitch_noise = pitch_noise * 2.0; // To get proper amplitude value since noise is generated with rand & +/- 0.5
    roll_noise = reader.GetReal("AHRS", "roll noise amplitude (deg)", 0.0);
    roll_noise = roll_noise * 2.0; // To get proper amplitude value since noise is generated with rand & +/- 0.5
    yaw_noise = reader.GetReal("AHRS", "yaw noise amplitude (deg)", 0.0);
    yaw_noise = yaw_noise * 2.0; // To get proper amplitude value since noise is generated with rand & +/- 0.5
    sim_rate = reader.GetReal("AHRS", "update rate (Hz)", 100.0);
}

void AHRS_SIM::startAhrs(void)
{
    // First, fire up the subscriber to the sim data with a callback to update the attitude values
    // OLD ROS1 METHOD - sim_sub = n.subscribe<picopter::Sim_msg>("sim_data", 1, &AHRS_SIM::setStates, this);
    sim_sub = this->create_subscription<picopter_interfaces::msg::SimulatorMSG>("sim_data", 1, std::bind(&AHRS_SIM::setStates, this, std::placeholders::_1));

    // Run the main loop that publishes simulated AHRS data to the "imu_data" topic
    // OLD ROS1 METHOD - ahrs_pub = n.advertise<picopter::Imu_msg>("imu_data", 1);
    ahrs_pub = this->create_publisher<picopter_interfaces::msg::ImuMSG>("imu_data", 1);

    // ROS2 Timer Callback - Hardcoded to 100 Hz / 10 ms
    timer_ = this->create_wall_timer(10ms, std::bind(&AHRS_SIM::runAhrs, this));

    // TO DO - Get the code below working with a sleep instead of a timer callback
    // ros::Rate loop_rate(sim_rate);
    // while (ros::ok)
    // {
    //     // Since the sim data values are perfect without noise, use rand 
    //     // to generate a normally distributed random number between 0 and 1,
    //     // then shift it to +/- 0.5, and multiply by the 2x the noise amplitude
    //     ahrs_msg.pitch = pitch + ((((float) rand() / RAND_MAX)) - 0.5)*pitch_noise;
    //     ahrs_msg.roll = roll + ((((float) rand() / RAND_MAX)) - 0.5)*roll_noise;
    //     ahrs_msg.yaw = yaw + ((((float) rand() / RAND_MAX)) - 0.5)*yaw_noise;

    //     // Calculate the rates based on state measurements and update rate
    //     ahrs_msg.pitch_rate = (pitch - pitch_last) * sim_rate;
    //     ahrs_msg.roll_rate = (roll - roll_last) * sim_rate;
    //     ahrs_msg.yaw_rate = (yaw_last - yaw) * sim_rate;
    //     ahrs_msg.pitch_rate_rate = (ahrs_msg.pitch_rate - pitch_rate_last) * sim_rate;
    //     ahrs_msg.roll_rate_rate = (ahrs_msg.roll_rate - roll_rate_last) * sim_rate;
    //     ahrs_msg.yaw_rate_rate = (ahrs_msg.yaw_rate - yaw_rate_last) * sim_rate;
    //     pitch_last = pitch;
    //     roll_last = roll;
    //     yaw_last = yaw;
    //     pitch_rate_last = ahrs_msg.pitch_rate;
    //     roll_rate_last = ahrs_msg.roll_rate;
    //     yaw_rate_last = ahrs_msg.yaw_rate;

    //     ahrs_pub.publish(ahrs_msg);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

}

void AHRS_SIM::runAhrs(void)
{
    // Since the sim data values are perfect without noise, use rand 
    // to generate a normally distributed random number between 0 and 1,
    // then shift it to +/- 0.5, and multiply by the 2x the noise amplitude
    ahrs_msg.pitch = pitch + ((((float) rand() / RAND_MAX)) - 0.5)*pitch_noise;
    ahrs_msg.roll = roll + ((((float) rand() / RAND_MAX)) - 0.5)*roll_noise;
    ahrs_msg.yaw = yaw + ((((float) rand() / RAND_MAX)) - 0.5)*yaw_noise;

    // Calculate the rates based on state measurements and update rate
    ahrs_msg.pitch_rate = (pitch - pitch_last) * sim_rate;
    ahrs_msg.roll_rate = (roll - roll_last) * sim_rate;
    ahrs_msg.yaw_rate = (yaw_last - yaw) * sim_rate;
    ahrs_msg.pitch_rate_rate = (ahrs_msg.pitch_rate - pitch_rate_last) * sim_rate;
    ahrs_msg.roll_rate_rate = (ahrs_msg.roll_rate - roll_rate_last) * sim_rate;
    ahrs_msg.yaw_rate_rate = (ahrs_msg.yaw_rate - yaw_rate_last) * sim_rate;
    pitch_last = pitch;
    roll_last = roll;
    yaw_last = yaw;
    pitch_rate_last = ahrs_msg.pitch_rate;
    roll_rate_last = ahrs_msg.roll_rate;
    yaw_rate_last = ahrs_msg.yaw_rate;

    ahrs_pub->publish(ahrs_msg);
}

void AHRS_SIM::setStates(const picopter_interfaces::msg::SimulatorMSG::SharedPtr msg)
{
    pitch = msg->pitch * R2D;
    roll = msg->roll * R2D;
    yaw = msg->yaw * R2D;
}

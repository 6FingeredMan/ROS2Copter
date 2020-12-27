/*
* @file      Simulator.h
* @date      12/26/2020
* @copyright Brendan Martin
* @version   2.0.0
* @brief     Defines the Simulator Class
*/
#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

// Included Libraries
#include <cstdint>
#include <map>

// 3rd Party Libraries
#include "INIReader.h"
#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Dense>

// User Libraries

// Interface Libraries
#include "picopter_interfaces/msg/altitude_msg.hpp"
#include "picopter_interfaces/msg/elevation_msg.hpp"
#include "picopter_interfaces/msg/gps_msg.hpp"
#include "picopter_interfaces/msg/imu_msg.hpp"
#include "picopter_interfaces/msg/motors_msg.hpp"
#include "picopter_interfaces/msg/navigator_msg.hpp"
#include "picopter_interfaces/msg/simulator_msg.hpp"

using namespace Eigen;

class Simulator : public rclcpp::Node
{
    public:
        // Constructor
        Simulator();

        // Reads the simulator.ini and configures the simulation
        void loadConfig(void);

        // Starts the simulator
        void startSimulator(void);

        // Runs the simulator
        void runSimulator(void);

        void process(void);

        void rungeKutta(void);

        void calcDerivatives( VectorXf &input_array );

        void setMotorCmd(const picopter_interfaces::msg::MotorsMSG::SharedPtr msg);

        // void computeRPM(void);

        void propThrust(void);

        void propTorque(void);

        float limitZero(float input);

        // Motor Commands, %
        float M1_cmd;
        float M2_cmd;
        float M3_cmd;
        float M4_cmd;
        // Propeller RPM, RPM
        float M1_RPM;
        float M2_RPM;
        float M3_RPM;
        float M4_RPM;
        // Motor Forces, Newtons
        float M1_F;
        float M2_F;
        float M3_F;
        float M4_F;
        // Motor/Prop Induced Torques about yaw, N-M
        float M1_T;
        float M2_T;
        float M3_T;
        float M4_T;
        // State variables
        float u_dot;            // Body relative acceleration, x, m/s/s
        float v_dot;            // Body relative acceleration, y, m/s/s
        float w_dot;            // Body relative acceleration, z, m/s/s
        float u;                // Body relative velocity, x, m/s
        float v;                // Body relative velocity, y, m/s
        float w;                // Body relative velocity, z, m/s
        float p_dot;            // Body relative angluar acceleration, roll, rad/s/s
        float q_dot;            // Body relative angluar acceleration, pitch, rad/s/s
        float r_dot;            // Body relative angluar acceleration, yaw, rad/s/s
        float p;                // Body relative angluar velocity, roll, rad/s
        float q;                // Body relative angluar velocity, pitch, rad/s
        float r;                // Body relative angluar velocity, yaw, rad/s
        float phi_dot;          // Earth relative angluar velocity, roll, rad/s
        float theta_dot;        // Earth relative angluar velocity, pitch, rad/s
        float psi_dot;          // Earth relative angluar velocity, yaw, rad/s
        float phi;              // Earth relative euler angle, roll, rad
        float theta;            // Earth relative euler angle, pitch, rad
        float psi;              // Earth relative euler angle, yaw, rad
        float X_dot;            // Earth relative velocity, x, m/s
        float Y_dot;            // Earth relative velocity, y, m/s
        float Z_dot;            // Earth relative velocity, z, m/s
        float northings;        // Earth relative location, m
        float eastings;         // Earth relative location, m
        float elevation;        // Earth relative location, m
        float X;                // Body realtive force, N
        float Y;                // Body realtive force, N
        float Z;                // Body realtive force, N
        float K;                // Body realtive torque, N-m
        float M;                // Body realtive torque, N-m
        float N;                // Body realtive torque, N-m
        // Robot Parameters
        float mass;             // Mass, kg
        float Ixx;              // Moment of Inertia, kg-m^2
        float Iyy;              // Moment of Inertia, kg-m^2
        float Izz;              // Moment of Inertia, kg-m^2        
        float LA;               // Asbolute lever arm from quad motors to cg, m
        float Km;               // Prop Thrust Constant
        float Tm;               // Prop Drag Constant to compute torque
        float Voltage;          // Initial battery voltage
        // Environment Parameters
        float g;                // Gravity, m/s/s
        float rho;              // Air Density, kg/m^3
        float lat;              // Latitude, degree decimal
        float lon;              // Longitude, degree decimal
        float pi;               // 3.14
        float D2R;              // degrees to rads
        // Runge-Kutta Solver Variables and Arrays
        double sim_freq;         // Frequency of solver
        float dt;                // Time step of solver
        VectorXf return_array;  // An array updated only by the calcDerivatives Function
        VectorXf state_array;   // Current Array of 6 velocities and 6 positions
        VectorXf temp_array;    // Intermediate Array of 6 velocities & 6 positions
        VectorXf K1;            // Runge Kutta Array of 6 acclerations and 6 velocities
        VectorXf K2;            // ""
        VectorXf K3;            // ""
        VectorXf K4;            // ""
        // Simulation time
        float sim_time;

        // Simulation ini Reader
        INIReader reader;

        // ROS topic messages
        picopter_interfaces::msg::SimulatorMSG sim_data_msg;

    private:
        // ROS hooks
        // ros::NodeHandle n;
        // ros::Publisher sim_pub;
        // ros::Subscriber motors_sub;

        // ROS2 hooks (Foxy)
        rclcpp::Subscription<picopter_interfaces::msg::MotorsMSG>::SharedPtr motors_sub;
        rclcpp::Publisher<picopter_interfaces::msg::SimulatorMSG>::SharedPtr sim_pub;
        
        // Used for the ROS2 callback
	    rclcpp::TimerBase::SharedPtr timer_;
};


#endif // __SIMULATOR_H__

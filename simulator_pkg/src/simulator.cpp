/*
* @file      Simulator.cpp
* @date      11/21/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the Simulator Class
*/

// Included Libraries
#include <cstdint>
#include <map>
#include <math.h>

// 3rd Party Libraries
#include "INIReader.h"
#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Dense>

// User Libraries
#include "simulator.h"

// Interface Libraries
#include "picopter_interfaces/msg/altitude_msg.hpp"
#include "picopter_interfaces/msg/elevation_msg.hpp"
#include "picopter_interfaces/msg/gps_msg.hpp"
#include "picopter_interfaces/msg/imu_msg.hpp"
#include "picopter_interfaces/msg/motors_msg.hpp"
#include "picopter_interfaces/msg/navigator_msg.hpp"
#include "picopter_interfaces/msg/simulator_msg.hpp"

// Macros
using namespace std::chrono_literals;
using namespace Eigen;

Simulator::Simulator()
:
    Node("simulator"),
    M1_cmd(0.0),
    M2_cmd(0.0),
    M3_cmd(0.0),
    M4_cmd(0.0),
    M1_RPM(0.0),
    M2_RPM(0.0),
    M3_RPM(0.0),
    M4_RPM(0.0),
    M1_F(0.0),
    M2_F(0.0),
    M3_F(0.0),
    M4_F(0.0),
    M1_T(0.0),
    M2_T(0.0),
    M3_T(0.0),
    M4_T(0.0),
    u_dot(0.0),
    v_dot(0.0),
    w_dot(0.0),
    u(0.0),
    v(0.0),
    w(0.0),
    p_dot(0.0),
    q_dot(0.0),
    r_dot(0.0),
    p(0.0),
    q(0.0),
    r(0.0),
    phi_dot(0.0),
    theta_dot(0.0),
    psi_dot(0.0),
    phi(0.0),
    theta(0.0),
    psi(0.0),
    X_dot(0.0),
    Y_dot(0.0),
    Z_dot(0.0),
    northings(0.0),
    eastings(0.0),
    elevation(0.0),
    X(0.0),
    Y(0.0),
    Z(0.0),
    K(0.0),
    M(0.0),
    N(0.0),
    sim_freq(200.0),
    dt(0.005),
    sim_time(0.0),
    pi(3.1415926535),
    D2R(0.0174533),
    reader("/home/ubuntu/PiCopter/Simulation/simulation.ini")
{
    loadConfig();
    startSimulator();
}

void Simulator::loadConfig(void)
{
    RCLCPP_INFO(this->get_logger(), "Simulator is loading config.");

    // Set Eigen VectorXf vectors size to 12 with values equal to 0
    return_array.setZero(12);
    state_array.setZero(12);
    temp_array.setZero(12);
    K1.setZero(12);
    K2.setZero(12);
    K3.setZero(12);
    K4.setZero(12); 

    // Get Robot Parameters
    mass = float(reader.GetReal("Robot", "mass", 0.4377));
    Ixx = float(reader.GetReal("Robot", "Ixx", 0.0047));
    Iyy = float(reader.GetReal("Robot", "Iyy", 0.0047));
    Izz = float(reader.GetReal("Robot", "Izz", 0.9998));
    LA = float(reader.GetReal("Robot", "Lever Arm", .125));
    Km = float(reader.GetReal("Robot", "Km", 0.0));
    Tm = float(reader.GetReal("Robot", "Tm", 0.0));
    Voltage = float(reader.GetReal("Robot", "Voltage", 0.0));

    // Get Inital Conditions
    state_array(0)  = float(reader.GetReal("Initial Conditions", "u (m/s)", 0.0));
    state_array(1)  = float(reader.GetReal("Initial Conditions", "v (m/s)", 0.0));
    state_array(2)  = float(reader.GetReal("Initial Conditions", "w (m/s)", 0.0));
    state_array(3)  = float(reader.GetReal("Initial Conditions", "p (deg/s)", 0.0) * D2R);
    state_array(4)  = float(reader.GetReal("Initial Conditions", "q (deg/s)", 0.0) * D2R);
    state_array(5)  = float(reader.GetReal("Initial Conditions", "r (deg/s)", 0.0) * D2R);
    state_array(6)  = float(reader.GetReal("Initial Conditions", "northings", 0.0));
    state_array(7)  = float(reader.GetReal("Initial Conditions", "eastings", 0.0));
    state_array(8)  = float(reader.GetReal("Initial Conditions", "altitude", 0.0));
    state_array(9)  = float(reader.GetReal("Initial Conditions", "phi (deg)", 0.0) * D2R);
    state_array(10) = float(reader.GetReal("Initial Conditions", "theta (deg)", 0.0) * D2R);
    state_array(11) = float(reader.GetReal("Initial Conditions", "psi (deg)", 0.0) * D2R);

    // Get Environment Parameters
    g = float(reader.GetReal("Environment", "gravity", 9.81));
    rho = float(reader.GetReal("Environment", "rho", 1.225));

    // Get Simulation Parameters
    sim_freq = double(reader.GetReal("Simulator", "simulation frequency (hz)", 200));
    dt = 1.0 / sim_freq;
}

void Simulator::startSimulator(void)
{
    // First, fire up the subscriber to the motor commands
    // OLD ROS1 METHOD - motors_sub = n.subscribe<picopter::Motors_msg>("motor_cmds", 1, &Simulator::setMotorCmd, this);
    motors_sub = this->create_subscription<picopter_interfaces::msg::MotorsMSG>("motor_cmds", 1, std::bind(&Simulator::setMotorCmd, this, std::placeholders::_1));

    // Then fire up the simulation loop
    // OLD ROS1 METHOD - sim_pub = n.advertise<picopter::Sim_msg>("sim_data", 1);
    sim_pub = this->create_publisher<picopter_interfaces::msg::SimulatorMSG>("sim_data", 1);

    // ROS2 Timer Callback - Hardcoded to 200 Hz / 5 ms
    timer_ = this->create_wall_timer(5ms, std::bind(&Simulator::runSimulator, this));

    // TO DO - Get the code below working with a sleep instead of a timer callback
    // ros::Rate loop_rate(sim_freq);
    // while (ros::ok)
    // {
    //     //ROS_INFO("Looping");
    //     process();
    //     sim_time += dt;
    //     sim_data_msg.pitch = theta;
    //     sim_data_msg.roll = phi;
    //     sim_data_msg.yaw = psi;
    //     sim_data_msg.pitch_rate = theta_dot;
    //     sim_data_msg.roll_rate = phi_dot;
    //     sim_data_msg.yaw_rate = psi_dot;
    //     sim_data_msg.elevation = elevation;
    //     sim_data_msg.altitude = elevation;
    //     sim_data_msg.northings = northings;
    //     sim_data_msg.eastings = eastings;
    //     sim_data_msg.latitude = 0.0;
    //     sim_data_msg.longitude = 0.0;
    //     sim_data_msg.motor_1_force = M1_F;
    //     sim_data_msg.motor_2_force = M2_F;
    //     sim_data_msg.motor_3_force = M3_F;
    //     sim_data_msg.motor_4_force = M4_F;
    //     sim_data_msg.X_Force = X;
    //     sim_data_msg.Y_Force = Y;
    //     sim_data_msg.Z_Force = Z;
    //     sim_data_msg.K_Moment = K;
    //     sim_data_msg.M_Moment = M;
    //     sim_data_msg.N_Moment = N;
    //     sim_data_msg.u_dot = u_dot;
    //     sim_data_msg.v_dot = v_dot;
    //     sim_data_msg.w_dot = w_dot;
    //     sim_data_msg.p_dot = p_dot;
    //     sim_data_msg.q_dot = q_dot;
    //     sim_data_msg.r_dot = r_dot;
    //     sim_data_msg.u = u;
    //     sim_data_msg.v = v;
    //     sim_data_msg.w = w;
    //     sim_data_msg.p = p;
    //     sim_data_msg.q = q;
    //     sim_data_msg.r = r;
    //     sim_data_msg.sim_time = sim_time;
    //     sim_pub.publish(sim_data_msg);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
}

void Simulator::runSimulator(void)
{
    process();
    sim_time += dt;
    sim_data_msg.pitch = theta;
    sim_data_msg.roll = phi;
    sim_data_msg.yaw = psi;
    sim_data_msg.pitch_rate = theta_dot;
    sim_data_msg.roll_rate = phi_dot;
    sim_data_msg.yaw_rate = psi_dot;
    sim_data_msg.elevation = elevation;
    sim_data_msg.altitude = elevation;
    sim_data_msg.northings = northings;
    sim_data_msg.eastings = eastings;
    sim_data_msg.latitude = 0.0;
    sim_data_msg.longitude = 0.0;
    sim_data_msg.motor_1_force = M1_F;
    sim_data_msg.motor_2_force = M2_F;
    sim_data_msg.motor_3_force = M3_F;
    sim_data_msg.motor_4_force = M4_F;
    sim_data_msg.x_force = X;
    sim_data_msg.y_force = Y;
    sim_data_msg.z_force = Z;
    sim_data_msg.k_moment = K;
    sim_data_msg.m_moment = M;
    sim_data_msg.n_moment = N;
    sim_data_msg.u_dot = u_dot;
    sim_data_msg.v_dot = v_dot;
    sim_data_msg.w_dot = w_dot;
    sim_data_msg.p_dot = p_dot;
    sim_data_msg.q_dot = q_dot;
    sim_data_msg.r_dot = r_dot;
    sim_data_msg.u = u;
    sim_data_msg.v = v;
    sim_data_msg.w = w;
    sim_data_msg.p = p;
    sim_data_msg.q = q;
    sim_data_msg.r = r;
    sim_data_msg.sim_time = sim_time;
    sim_pub->publish(sim_data_msg);

}

void Simulator::process(void)
{
    // Call the Runge-Kutta Method
    rungeKutta();

    // Unpack the Runge-Kutta State Array for correct publishing
    u         = state_array(0);
    v         = state_array(1);
    w         = state_array(2);
    p         = state_array(3);
    q         = state_array(4);
    r         = state_array(5);
    northings = state_array(6);
    eastings  = state_array(7);
    elevation = state_array(8);
    phi       = state_array(9);
    theta     = state_array(10);
    psi       = state_array(11);


    ///////////////////////////////////////////////
    // OLD METHOD BELOW - BAD
    //////////////////////////////////////////////

    // // Check for NaN and reset to values to 0.0
    // if (isnan(theta)) theta = 0.0;
    // if (isnan(phi)) phi = 0.0;
    // if (isnan(psi)) psi = 0.0;

    // // Calculate the body relative forces - mechanics
    // X = -mass*g*std::sin(theta); // theta
    // Y = mass*g*std::sin(phi)*std::cos(theta); // phi & theta
    // Z = -M1_F - M2_F - M3_F - M4_F; // Prop forces are negative due to NED reference frame
    // Z = Z + mass*g*std::cos(phi)*std::cos(theta);
    // // Calculate the body relative forces - add aerodynamics
    // // TBD

    // // Calculate the body relative torques
    // K = (M1_F + M2_F - M3_F - M4_F) * LA;          // Roll
    // M = (M1_F + M4_F - M2_F - M3_F) * LA;          // Pitch
    // N = (M1_T + M2_T + M3_T + M4_T);               // Yaw
    // // Calculate the body relative torques - add aerodynamics

    // // Calculate Accelerations

    // // Run Solver
    // // calcAccels();
    // // calcVelocities();
    // // calcPositions();
}

void Simulator::rungeKutta(void)
{
    // Call calcDerivatives 4 times and then recompute the state array
    calcDerivatives(state_array);
    K1 = return_array;
    temp_array = state_array + 0.5*dt*K1;
    calcDerivatives(temp_array);
    K2 = return_array;
    temp_array = state_array + 0.5*dt*K2;
    calcDerivatives(temp_array);
    K3 = return_array;
    temp_array = state_array + dt*K3;
    calcDerivatives(temp_array);
    K4 = return_array;

    // Calculate the new state array
    state_array = state_array + (dt/6)*(K1 + 2*K2 + 2*K3 + K4);

    // Calculate the acceleration values
    temp_array = (1/6)*(K1 + K2 + K3 + K4);
    u_dot = temp_array(0);
    v_dot = temp_array(1);
    w_dot = temp_array(2);
    p_dot = temp_array(3);
    q_dot = temp_array(4);
    r_dot = temp_array(5);

    // Correct for a ground condition
    if(state_array(8) < 0)
    {
        // Force minimum altitude to be 0
        state_array(8) = 0.0;
        
        // Force velocities to be = 0 if the quad elevation velocity is less than 0.0
        if(Z_dot < 0)
        {
            state_array(0) = 0.0;
            state_array(1) = 0.0;
            state_array(2) = 0.0;
            state_array(3) = 0.0;
            state_array(4) = 0.0;
            state_array(5) = 0.0;
        }
    }

    // Limit Roll to +/- 180 degrees (+/- PI Radians)
    if(state_array(9) > pi)
    {
        state_array(9) = state_array(9) - 2*pi;
    }
    if(state_array(9) < -pi)
    {
        state_array(9) = state_array(9) + 2*pi;
    }

    // Limit Pitch to +/- 89 degrees to avoid gimbal lock in calc (TO DO - USE QUATERNIONS)
    if(state_array(10) > (pi/2) - .017543)
    {
        state_array(10) = (pi/2) - .017543;
    }
    if(state_array(10) < -(pi/2) + .017543)
    {
        state_array(10) = -(pi/2) + .017543;
    }


}

void Simulator::calcDerivatives( VectorXf &input_array )
{

    // Unpack the Variables First
    u         = input_array(0);
    v         = input_array(1);
    w         = input_array(2);
    p         = input_array(3);
    q         = input_array(4);
    r         = input_array(5);
    northings = input_array(6);
    eastings  = input_array(7);
    elevation = input_array(8);
    phi       = input_array(9);
    theta     = input_array(10);
    phi       = input_array(11);

    // Calculate the Body Relative Forces
    X = -mass*g*std::sin(theta); // theta
    Y = mass*g*std::sin(phi)*std::cos(theta); // phi & theta
    Z = -M1_F - M2_F - M3_F - M4_F; // Prop forces are negative due to NED reference frame
    Z = Z + mass*g*std::cos(phi)*std::cos(theta);
    // Calculate the body relative forces - add aerodynamics
    // TBD

    // Calculate the Body Relative Torques
    K = (M1_F + M2_F - M3_F - M4_F) * LA;          // Roll
    M = (M1_F + M4_F - M2_F - M3_F) * LA;          // Pitch
    N = (M1_T + M2_T + M3_T + M4_T);               // Yaw

    // Calculate the Accelerations, which can only occur if the elevation is above 0
    if(elevation >= 0)
    {
        u_dot = -g*std::sin(theta) + r*v - q*w;
        v_dot = g*std::sin(phi)*std::cos(theta) - r*u + p*w;
        w_dot = (1/mass)*(Z) + q*u - p*v;
        p_dot = (1/Ixx)*(K + (Iyy-Izz)*q*r);
        q_dot = (1/Iyy)*(M + (Izz-Ixx)*p*r);
        r_dot = (1/Izz)*(N + (Ixx-Iyy)*p*q);
    }
    else
    {
        u_dot = 0.0;
        v_dot = 0.0;
        w_dot = 0.0;
        p_dot = 0.0;
        q_dot = 0.0;
        r_dot = 0.0;
    }

    // Calculate the World Positional Velocities
    X_dot = std::cos(theta)*std::cos(psi)*u +
            (-std::cos(phi)*std::sin(psi) + std::sin(phi)*std::sin(theta)*std::cos(psi))*v +
            (std::sin(phi)*std::sin(psi) + std::cos(phi)*std::sin(theta)*std::cos(psi))*w;
    Y_dot = std::cos(theta)*std::sin(psi)*u + 
            (std::cos(phi)*std::cos(psi) + std::sin(phi)*std::sin(theta)*std::sin(psi))*v +
            (-std::sin(phi)*std::cos(psi) + std::cos(phi)*std::sin(theta)*std::sin(psi))*w;
    Z_dot = -1.0*(-std::sin(theta)*u + std::sin(phi)*std::cos(theta)*v + std::cos(phi)*std::cos(theta)*w);

    // Calculate the World Attitude Rates
    phi_dot = p + ( q*std::sin(phi) + r*std::cos(phi) )*std::tan(theta);
    theta_dot = q*std::cos(phi) - r*std::sin(phi);
    psi_dot = (q*std::sin(phi) + r*std::cos(phi))*(1/std::cos(theta));

    return_array(0) = u_dot;
    return_array(1) = v_dot;
    return_array(2) = w_dot;
    return_array(3) = p_dot;
    return_array(4) = q_dot;
    return_array(5) = r_dot;
    return_array(6) = X_dot;
    return_array(7) = Y_dot;
    return_array(8) = Z_dot;
    return_array(9) = phi_dot;
    return_array(10) = theta_dot;
    return_array(11) = psi_dot;

}

void Simulator::setMotorCmd(const picopter_interfaces::msg::MotorsMSG::SharedPtr msg)
{
    M1_cmd = msg->m1;
    M2_cmd = msg->m2;
    M3_cmd = msg->m3;
    M4_cmd = msg->m4;
    propThrust();
    propTorque();
}

// void Simulator::computeRPM(void)
// {
//     // Convert command to RPM value (instantaneous right now..)
//     M1_RPM = M1_cmd*0.9*Voltage;
//     M2_RPM = M2_cmd*0.9*Voltage;
//     M3_RPM = M3_cmd*0.9*Voltage;
//     M4_RPM = M4_cmd*0.9*Voltage;
// }

void Simulator::propThrust(void)
{
    // Convert throttle command to micro seconds
    float M1_DC = ( M1_cmd * 9.30 + 1070 );
    float M2_DC = ( M2_cmd * 9.30 + 1070 );
    float M3_DC = ( M3_cmd * 9.30 + 1070 );
    float M4_DC = ( M4_cmd * 9.30 + 1070 );

    // Covert throttle in micro seconds to a force in grams
    // using best fit empirical data
    if (M1_DC <= 1070) 
    {
        M1_F = 0.0;
    }
    else
    {
        M1_F = 0.0007 * M1_DC * M1_DC - 0.6577 * M1_DC;
    }

    if (M2_DC <= 1070) 
    {
        M2_F = 0.0;
    }
    else
    {
        M2_F = 0.0007 * M2_DC * M2_DC - 0.6577 * M2_DC;
    }

    if (M3_DC <= 1070) 
    {
        M3_F = 0.0;
    }
    else
    {
        M3_F = 0.0007 * M3_DC * M3_DC - 0.6577 * M3_DC;
    }

    if (M4_DC <= 1070) 
    {
        M4_F = 0.0;
    }
    else
    {
        M4_F = 0.0007 * M4_DC * M4_DC - 0.6577 * M4_DC;
    }

    // Convert Motor forces from grams to Newtons
    M1_F = (M1_F / 1000.0) * 9.81;
    M2_F = (M2_F / 1000.0) * 9.81;
    M3_F = (M3_F / 1000.0) * 9.81;
    M4_F = (M4_F / 1000.0) * 9.81;

}

void Simulator::propTorque(void)
{
    // Calculate Induced Yaw Torque, Newton-meters
    M1_T = 0.0;
    M2_T = 0.0;
    M3_T = 0.0;
    M4_T = 0.0;
    //M1_T = -1.0 * (M1_RPM * M1_RPM * Tm);   // CW Prop,  (-1) Torque
    //M2_T = (M2_RPM * M2_RPM * Tm);          // CCW Prop, (+1) Torque
    //M3_T = -1.0 * (M3_RPM * M3_RPM * Tm);   // CW Prop,  (-1) Torque
    //M4_T = (M4_RPM * M4_RPM * Tm);          // CCW Prop, (+1) Torque
}

float Simulator::limitZero(float input)
{
    if(input < 0)
    {
        input = 0.0;
    }

    return input;
}


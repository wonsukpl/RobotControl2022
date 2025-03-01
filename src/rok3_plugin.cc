/*
 * RoK-3 Gazebo Simulation Code 
 * 
 * Robotics & Control Lab.
 * 
 * Master : BKCho
 * First developer : Yunho Han
 * Second developer : Minho Park
 * 
 * ======
 * Update date : 2022.03.16 by Yunho Han
 * ======
 */
//* Header file for C++
#include <stdio.h>
#include <iostream>
#include <boost/bind.hpp>

//* Header file for Gazebo and Ros
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>

//* Header file for RBDL and Eigen
#include <rbdl/rbdl.h> // Rigid Body Dynamics Library (RBDL)
#include <rbdl/addons/urdfreader/urdfreader.h> // urdf model read using RBDL
#include <Eigen/Dense> // Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.

#define PI      3.141592
#define D2R     PI/180.
#define R2D     180./PI

//Print color
#define C_BLACK   "\033[30m"
#define C_RED     "\x1b[91m"
#define C_GREEN   "\x1b[92m"
#define C_YELLOW  "\x1b[93m"
#define C_BLUE    "\x1b[94m"
#define C_MAGENTA "\x1b[95m"
#define C_CYAN    "\x1b[96m"
#define C_RESET   "\x1b[0m"

//Eigen//
using Eigen::MatrixXd;
using Eigen::VectorXd;

//RBDL//
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace std;

//Global parameters *WsJi*
/*
 VectorXd q(6);
    //
    //double deg2rad = PI / 180;
    //double rad2deg = 180 / PI;
    q << 10*D2R, 20*D2R, 30*D2R, 40*D2R, 50*D2R, 60*D2R;
 */

VectorXd q_left_des(6), r_left_des(3), r_left_init(3), r_left_prev(3), r_left_home(3), r_right_home(3);
VectorXd q_right_des(6), r_right_des(3), r_right_init(3), r_right_prev(3);

MatrixXd C_left_des(3,3), C_right_des(3,3);
double ori_left_des[3] = {0., 0., 0.};
double ori_left_prev[3] = {0., 0., 0.};
double ori_right_des[3] = {0., 0., 0.};
double ori_right_prev[3] = {0., 0., 0.};


double myTemp = 1.;
double prev_time = 0.;

int practice6_subQuestion = 3;  // 0 to 3: Practice 6-0 to 3.

bool walk_ready_posture = false;
bool walk_start_or_finish = false;

bool walk_stop = false;
bool walk_start = false;

double t_num = 0.; // 
//double swing_period = 1.0;  // [s]

double time_walkready = 5.;


double swing_height = 0.2;
double swing_length = 0.;
//double swing_height = 0.;

double swing_length_yaxis = 0.12;

//double swing_period = 2.0;  // [s]
//double walk_ready_height = 0.2;  // 0.3912
//double CoM_x_dist = 0.05;

double swing_period = 2.0;  // [s]
//double CoM_x_dist = 0.05;
double CoM_x_dist = 0.07;
//double CoM_x_dist = 0.08;
double walk_ready_height = 0.2;  // 0.3912

//walk_ready_height = 0.2;


enum
{
    LEFT_SWING_UP = 0, LEFT_SWING_DOWN, RIGHT_SWING_UP, RIGHT_SWING_DOWN, COM_MOVE_LEFT, COM_MOVE_RIGHT, STOP_MOVING, TURN_RIGHT
};

enum
{
    TURN_RIGHT_PHASE1 = 0, TURN_RIGHT_PHASE2, TURN_RIGHT_PHASE3, TURN_RIGHT_PHASE4,
    TURN_RIGHT_PHASE5, TURN_RIGHT_PHASE6, TURN_RIGHT_PHASE7, TURN_RIGHT_PHASE8
};

unsigned int leg_phase = COM_MOVE_RIGHT;

//unsigned int leg_phase = TURN_RIGHT;
unsigned int turn_phase = TURN_RIGHT_PHASE1;
double turn_theta = 0.;
std::string myOrder = "straight";

namespace gazebo
{

    class rok3_plugin : public ModelPlugin
    {
        //*** Variables for RoK-3 Simulation in Gazebo ***//
        //* TIME variable
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;

        //* Model & Link & Joint Typedefs
        physics::ModelPtr model;

        physics::JointPtr L_Hip_yaw_joint;
        physics::JointPtr L_Hip_roll_joint;
        physics::JointPtr L_Hip_pitch_joint;
        physics::JointPtr L_Knee_joint;
        physics::JointPtr L_Ankle_pitch_joint;
        physics::JointPtr L_Ankle_roll_joint;

        physics::JointPtr R_Hip_yaw_joint;
        physics::JointPtr R_Hip_roll_joint;
        physics::JointPtr R_Hip_pitch_joint;
        physics::JointPtr R_Knee_joint;
        physics::JointPtr R_Ankle_pitch_joint;
        physics::JointPtr R_Ankle_roll_joint;
        physics::JointPtr torso_joint;

        physics::JointPtr LS, RS;

        //* Index setting for each joint
        
        enum
        {
            WST = 0, LHY, LHR, LHP, LKN, LAP, LAR, RHY, RHR, RHP, RKN, RAP, RAR
        };

        //* Joint Variables
        int nDoF; // Total degrees of freedom, except position and orientation of the robot

        typedef struct RobotJoint //Joint variable struct for joint control 
        {
            double targetDegree; //The target deg, [deg]
            double targetRadian; //The target rad, [rad]

            double targetVelocity; //The target vel, [rad/s]
            double targetTorque; //The target torque, [N·m]

            double actualDegree; //The actual deg, [deg]
            double actualRadian; //The actual rad, [rad]
            double actualVelocity; //The actual vel, [rad/s]
            double actualRPM; //The actual rpm of input stage, [rpm]
            double actualTorque; //The actual torque, [N·m]

            double Kp;
            double Ki;
            double Kd;

        } ROBO_JOINT;
        ROBO_JOINT* joint;

    public:
        //*** Functions for RoK-3 Simulation in Gazebo ***//
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/); // Loading model data and initializing the system before simulation 
        void UpdateAlgorithm(); // Algorithm update while simulation

        void jointController(); // Joint Controller for each joint

        void GetJoints(); // Get each joint data from [physics::ModelPtr _model]
        void GetjointData(); // Get encoder data of each joint

        void initializeJoint(); // Initialize joint variables for joint control
        void SetJointPIDgain(); // Set each joint PID gain for joint control
    };
    GZ_REGISTER_MODEL_PLUGIN(rok3_plugin);
}

//

//* getTransformIo()
MatrixXd getTransformI0()
{
    MatrixXd tmp_m(4,4);
    
    
    tmp_m = MatrixXd::Identity(4, 4);
    
    /*
     tmp_m << 1, 0, 0, 0, \
     *        0, 1, 0, 0, \
     *        0, 0, 1, 0, \
     *        0, 0, 0, 1;
     */
    
    /*
     tmp_m(0, 0) = 1; tmp(0, 1) = 0; ...
     * ...
     * ...
     */
    
    
    return tmp_m;
}

/*
MatrixXd getTransform3E()
{
    MatrixXd tmp_m(4,4);
    tmp_m << 1, 0, 0, 0, \
             0, 1, 0, 0, \
             0, 0, 1, 1, \
             0, 0, 0, 1;
    
    return tmp_m;    
}
 * */

MatrixXd jointToTransform01(VectorXd q, std::string dir = "left") // Hip Yaw
{
    //* q: generalized coordinates. q = [q1; q2; q3];
    MatrixXd tmp_m(4,4);
    double tmp_q = q(0);
    
    std::string left_or_right = dir;
    
    if(left_or_right == "left"){
        tmp_m << cos(tmp_q), -sin(tmp_q), 0, 0, \
                 sin(tmp_q), cos(tmp_q), 0, 0.105, \
                 0, 0, 1, -0.1512, \
                 0, 0, 0, 1;
    }
    
    else if(left_or_right == "right"){
        tmp_m << cos(tmp_q), -sin(tmp_q), 0, 0, \
                 sin(tmp_q), cos(tmp_q), 0, -0.105, \
                 0, 0, 1, -0.1512, \
                 0, 0, 0, 1;
    }
    
    else{
        std::cout << "wrong direction. just use left or right." << std::endl;
    }
       
    return tmp_m;
}

MatrixXd jointToTransform12(VectorXd q) // Hip Roll
{
    //* q: generalized coordinates. q = [q1; q2; q3];
    MatrixXd tmp_m(4,4);
    double tmp_q = q(1);
    
    tmp_m << 1, 0, 0, 0, \
             0, cos(tmp_q), -sin(tmp_q), 0, \
             0, sin(tmp_q), cos(tmp_q), 0, \
             0, 0, 0, 1;
    
       
    
    return tmp_m;
}

MatrixXd jointToTransform23(VectorXd q) // Hip Pitch
{
    //* q: generalized coordinates. q = [q1; q2; q3];
    MatrixXd tmp_m(4,4);
    double tmp_q = q(2);
    
    tmp_m << cos(tmp_q), 0, sin(tmp_q), 0, \
             0, 1, 0, 0, \
             -sin(tmp_q), 0, cos(tmp_q), 0, \
             0, 0, 0, 1;
    
       
    
    return tmp_m;
}

MatrixXd jointToTransform34(VectorXd q) // Knee Pitch
{
    //* q: generalized coordinates. q = [q1; q2; q3];
    MatrixXd tmp_m(4,4);
    double tmp_q = q(3);
    
    tmp_m << cos(tmp_q), 0, sin(tmp_q), 0, \
             0, 1, 0, 0, \
             -sin(tmp_q), 0, cos(tmp_q), -0.35, \
             0, 0, 0, 1;
    
       
    
    return tmp_m;
}

MatrixXd jointToTransform45(VectorXd q) // Ankle Pitch
{
    //* q: generalized coordinates. q = [q1; q2; q3];
    MatrixXd tmp_m(4,4);
    double tmp_q = q(4);
    
    tmp_m << cos(tmp_q), 0, sin(tmp_q), 0, \
             0, 1, 0, 0, \
             -sin(tmp_q), 0, cos(tmp_q), -0.35, \
             0, 0, 0, 1;
    
       
    
    return tmp_m;
}

MatrixXd jointToTransform56(VectorXd q) // Ankle Roll
{
    //* q: generalized coordinates. q = [q1; q2; q3];
    MatrixXd tmp_m(4,4);
    double tmp_q = q(5);
    
    tmp_m << 1, 0, 0, 0, \
             0, cos(tmp_q), -sin(tmp_q), 0, \
             0, sin(tmp_q), cos(tmp_q), 0, \
             0, 0, 0, 1;
    
       
    
    return tmp_m;
}

MatrixXd rot_z(double q)
{
    MatrixXd tmp_m(3,3);
    tmp_m << cos(q), -sin(q), 0,\
             sin(q), cos(q), 0,\
             0, 0, 1;
    
    return tmp_m;
            
}

MatrixXd getTransform6E() // Ankle Roll
{
    //* q: generalized coordinates. q = [q1; q2; q3];
    MatrixXd tmp_m(4,4);
    
    tmp_m << 1, 0, 0, 0, \
             0, 1, 0, 0, \
             0, 0, 1, -0.09, \
             0, 0, 0, 1;
    
       
    
    return tmp_m;
}



VectorXd jointToPosition(VectorXd q, std::string dir = "left")
{
    MatrixXd tmp_m(4,4);
    Vector3d tmp_p;
    
    MatrixXd TI0(4,4), T01(4,4), T12(4,4), T23(4,4), T34(4,4), T45(4,4), T56(4,4), T6E(4,4);
    
    TI0 = getTransformI0();
    T01 = jointToTransform01(q, dir);
    T12 = jointToTransform12(q);
    T23 = jointToTransform23(q);
    T34 = jointToTransform34(q);
    T45 = jointToTransform45(q);
    T56 = jointToTransform56(q);
    T6E = getTransform6E();
    
    tmp_m = TI0*T01*T12*T23*T34*T45*T56*T6E;
    
    tmp_p = tmp_m.block(0, 3, 3, 1);
    
    return tmp_p;
    
    
}

MatrixXd jointToRotMat(VectorXd q, std::string dir = "left")
{
    MatrixXd tmp_m(4,4);
    MatrixXd tmp_return(3,3);
        
    MatrixXd TI0(4,4), T01(4,4), T12(4,4), T23(4,4), T34(4,4), T45(4,4), T56(4,4), T6E(4,4);
    
    TI0 = getTransformI0();
    T01 = jointToTransform01(q, dir);
    T12 = jointToTransform12(q);
    T23 = jointToTransform23(q);
    T34 = jointToTransform34(q);
    T45 = jointToTransform45(q);
    T56 = jointToTransform56(q);
    T6E = getTransform6E();
    
    tmp_m = TI0*T01*T12*T23*T34*T45*T56*T6E;
    
    tmp_return = tmp_m.block(0, 0, 3, 3);
    
    return tmp_return;
}

VectorXd rotToEuler(MatrixXd rotMat)
{
    double z = atan2(rotMat(1,0), rotMat(0,0));
    double y = atan2(-rotMat(2,0), sqrt(rotMat(2,1)*rotMat(2,1) + rotMat(2,2)*rotMat(2,2)));
    double x = atan2(rotMat(2,1), rotMat(2,2));
    
    Vector3d tmp_v(z, y, x);
    return tmp_v;
}

// 220502 practice: Geometric Jacobian Programming
MatrixXd jointToPosJac(VectorXd q, std::string dir = "left")
{
    // Input: vector of generalized coordinates (joint angles)
    // Output: J_P, Jacobian of the end-effector translation which maps joint velocities to end-effector linear velocities in I frame.
    MatrixXd J_P = MatrixXd::Zero(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4);
    MatrixXd T_IE(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    VectorXd r_I_I1(3), r_I_I2(3), r_I_I3(3), r_I_I4(3), r_I_I5(3), r_I_I6(3);
    VectorXd n_1(3), n_2(3), n_3(3), n_4(3), n_5(3), n_6(3);
    Vector3d n_I_1, n_I_2, n_I_3, n_I_4, n_I_5, n_I_6;
    VectorXd r_I_IE(3);


    //* Compute the relative homogeneous transformation matrices.
    T_I0 = getTransformI0();
    T_01 = jointToTransform01(q, dir);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();
       

    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0 * T_01;
    T_I2 = T_I1 * T_12;
    T_I3 = T_I2 * T_23;
    T_I4 = T_I3 * T_34;
    T_I5 = T_I4 * T_45;
    T_I6 = T_I5 * T_56;
    
    T_IE = T_I6 * T_6E;

    //* Extract the rotation matrices from each homogeneous transformation matrix. Use sub-matrix of EIGEN. https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
    R_I1 = T_I1.block(0,0,3,3);
    R_I2 = T_I2.block(0,0,3,3);
    R_I3 = T_I3.block(0,0,3,3);
    R_I4 = T_I4.block(0,0,3,3);
    R_I5 = T_I5.block(0,0,3,3);
    R_I6 = T_I6.block(0,0,3,3);

    //* Extract the position vectors from each homogeneous transformation matrix. Use sub-matrix of EIGEN.
    r_I_I1 = T_I1.block(0,3,3,1);
    r_I_I2 = T_I2.block(0,3,3,1);
    r_I_I3 = T_I3.block(0,3,3,1);
    r_I_I4 = T_I4.block(0,3,3,1);
    r_I_I5 = T_I5.block(0,3,3,1);
    r_I_I6 = T_I6.block(0,3,3,1);

    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;

    //* Compute the unit vectors for the inertial frame I.
    n_I_1 = R_I1 * n_1;
    n_I_2 = R_I2 * n_2;
    n_I_3 = R_I3 * n_3;
    n_I_4 = R_I4 * n_4;
    n_I_5 = R_I5 * n_5;
    n_I_6 = R_I6 * n_6;

    //* Compute the end-effector position vector.
    r_I_IE = T_IE.block(0,3,3,1);


    //* Compute the translational Jacobian. Use cross of EIGEN.
    
    //std::cout << (r_I_IE-r_I_I1) << std::endl;
    //std::cout << n_I_1 << std::endl;
    
    
    // The important point is that cross func should be used for vector3d, not vectorxd.
    // (the Eigen lib don't use static_cast..just my opinion.)
    
    Vector3d rI1E = r_I_IE-r_I_I1;
    Vector3d rI2E = r_I_IE-r_I_I2;
    Vector3d rI3E = r_I_IE-r_I_I3;
    Vector3d rI4E = r_I_IE-r_I_I4;
    Vector3d rI5E = r_I_IE-r_I_I5;
    Vector3d rI6E = r_I_IE-r_I_I6;
    
    /*J_R.col(0) << n_I_1;
    J_R.col(1) << n_I_2;
    J_R.col(2) << n_I_3;
    J_R.col(3) << n_I_4;
    J_R.col(4) << n_I_5;
    J_R.col(5) << n_I_6;
    */
    //
    /*Vector3d nI1 = n_I_1;
    Vector3d nI2 = n_I_2;
    Vector3d nI3 = n_I_3;
    Vector3d nI4 = n_I_4;
    Vector3d nI5 = n_I_5;
    Vector3d nI6 = n_I_6;
    */
    
    //std::cout << nI1.cross(rI11) << std::endl;
    
    //std::cout << nI1.cross(rI1E) << std::endl;
    
    //
    
    
    
    //std::cout << nI1.cross(rI11) << std::endl;
    
    //std::cout << nI1.cross(rI1E) << std::endl;
    J_P.col(0) << n_I_1.cross(rI1E);
    J_P.col(1) << n_I_2.cross(rI2E);
    J_P.col(2) << n_I_3.cross(rI3E);
    J_P.col(3) << n_I_4.cross(rI4E);
    J_P.col(4) << n_I_5.cross(rI5E);
    J_P.col(5) << n_I_6.cross(rI6E);
    
    /*
    J_P.col(0) << n_I_1.cross(r_I_IE-r_I_I1);
    J_P.col(1) << n_I_2.cross(r_I_IE-r_I_I2);
    J_P.col(2) << n_I_3.cross(r_I_IE-r_I_I3);
    J_P.col(3) << n_I_4.cross(r_I_IE-r_I_I4);
    J_P.col(4) << n_I_5.cross(r_I_IE-r_I_I5);
    J_P.col(5) << n_I_6.cross(r_I_IE-r_I_I6);
    */
    
    //std::cout << "Test, JP:" << std::endl << J_P << std::endl;

    return J_P;
}

MatrixXd jointToRotJac(VectorXd q, std::string dir = "left")
{
    // Input: vector of generalized coordinates (joint angles)
    // Output: J_P, Jacobian of the end-effector translation which maps joint velocities to end-effector linear velocities in I frame.
    MatrixXd J_R = MatrixXd::Zero(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4);
    MatrixXd T_IE(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    VectorXd r_I_I1(3), r_I_I2(3), r_I_I3(3), r_I_I4(3), r_I_I5(3), r_I_I6(3);
    VectorXd n_1(3), n_2(3), n_3(3), n_4(3), n_5(3), n_6(3);
    Vector3d n_I_1, n_I_2, n_I_3, n_I_4, n_I_5, n_I_6;
    VectorXd r_I_IE(3);


    //* Compute the relative homogeneous transformation matrices.
    T_I0 = getTransformI0();
    T_01 = jointToTransform01(q, dir);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();
       

    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0 * T_01;
    T_I2 = T_I1 * T_12;
    T_I3 = T_I2 * T_23;
    T_I4 = T_I3 * T_34;
    T_I5 = T_I4 * T_45;
    T_I6 = T_I5 * T_56;
    
    T_IE = T_I6 * T_6E;

    //* Extract the rotation matrices from each homogeneous transformation matrix. Use sub-matrix of EIGEN. https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
    R_I1 = T_I1.block(0,0,3,3);
    R_I2 = T_I2.block(0,0,3,3);
    R_I3 = T_I3.block(0,0,3,3);
    R_I4 = T_I4.block(0,0,3,3);
    R_I5 = T_I5.block(0,0,3,3);
    R_I6 = T_I6.block(0,0,3,3);

    //* Extract the position vectors from each homogeneous transformation matrix. Use sub-matrix of EIGEN.
    r_I_I1 = T_I1.block(0,3,3,1);
    r_I_I2 = T_I2.block(0,3,3,1);
    r_I_I3 = T_I3.block(0,3,3,1);
    r_I_I4 = T_I4.block(0,3,3,1);
    r_I_I5 = T_I5.block(0,3,3,1);
    r_I_I6 = T_I6.block(0,3,3,1);

    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;

    //* Compute the unit vectors for the inertial frame I.
    n_I_1 = R_I1 * n_1;
    n_I_2 = R_I2 * n_2;
    n_I_3 = R_I3 * n_3;
    n_I_4 = R_I4 * n_4;
    n_I_5 = R_I5 * n_5;
    n_I_6 = R_I6 * n_6;

    //* Compute the end-effector position vector.
    r_I_IE = T_IE.block(0,3,3,1);


    //* Compute the translational Jacobian. Use cross of EIGEN.
    
    //std::cout << (r_I_IE-r_I_I1) << std::endl;
    //std::cout << n_I_1 << std::endl;
    
    Vector3d rI1E = r_I_IE-r_I_I1;
    Vector3d rI2E = r_I_IE-r_I_I2;
    Vector3d rI3E = r_I_IE-r_I_I3;
    Vector3d rI4E = r_I_IE-r_I_I4;
    Vector3d rI5E = r_I_IE-r_I_I5;
    Vector3d rI6E = r_I_IE-r_I_I6;
    
    /*Vector3d nI1 = n_I_1;
    Vector3d nI2 = n_I_2;
    Vector3d nI3 = n_I_3;
    Vector3d nI4 = n_I_4;
    Vector3d nI5 = n_I_5;
    Vector3d nI6 = n_I_6;
    */
    
    //std::cout << nI1.cross(rI11) << std::endl;
    
    //std::cout << nI1.cross(rI1E) << std::endl;
    J_R.col(0) << n_I_1;
    J_R.col(1) << n_I_2;
    J_R.col(2) << n_I_3;
    J_R.col(3) << n_I_4;
    J_R.col(4) << n_I_5;
    J_R.col(5) << n_I_6;
    
    /*
    J_P.col(0) << n_I_1.cross(r_I_IE-r_I_I1);
    J_P.col(1) << n_I_2.cross(r_I_IE-r_I_I2);
    J_P.col(2) << n_I_3.cross(r_I_IE-r_I_I3);
    J_P.col(3) << n_I_4.cross(r_I_IE-r_I_I4);
    J_P.col(4) << n_I_5.cross(r_I_IE-r_I_I5);
    J_P.col(5) << n_I_6.cross(r_I_IE-r_I_I6);
    */
    
    //std::cout << "Test, JR:" << std::endl << J_R << std::endl;

    return J_R;
}

MatrixXd pseudoInverseMat(MatrixXd A, double lambda = 0)
{
    // Input: Any m-by-n matrix
    // Output: An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula
    MatrixXd pinvA;

    // In this tutorial, i assumed that rank of the Jacobian is full-rank.

    // 1. The input matrix A is m*n matrix. compare m with n.
    unsigned int m = A.rows();
    unsigned int n = A.cols();
    
    
    //typedef Matrix<double, 3, 3>  MatrixMxN;
   // Eigen::FullPivLU<MatrixMxN> lu(A);
    
    MatrixXd transposeA = A.transpose();
    MatrixXd temp;
    
    
    //identityMat = Identity(n);
    
    // 2. when m >= n and rank(A) = n, left pseudo-inverse pinvA should be:
    if(m >= n){
    
        //MatrixXd identityMat(n,n);
        //identityMat = MatrixXd::Identity(n,n);
        
        //temp = transposeA * A + lambda*lambda*identityMat;
        temp = transposeA * A + lambda*lambda*(MatrixXd::Identity(n,n));
        pinvA = temp.inverse() * transposeA;
    }
    
    // 3. when n > m and rank(A) = m, right pseudo-inverse pinA should be: 
    else{
        temp = A * transposeA + lambda*lambda*(MatrixXd::Identity(m,m));
        pinvA = transposeA * temp.inverse();
    }
    
    return pinvA;
}

VectorXd rotMatToRotVec(MatrixXd C)
{
    // Input: a rotation matrix C
    // Output: the rotational vector which describes the rotation C
    Vector3d phi,n;
    double th;
    
    th = acos((C(0,0)+C(1,1)+C(2,2)-1)/2);
    
    if(fabs(th)<0.001){
         n << 0,0,0;
    }
    else{
        n << 1/(2*sin(th))*(C(2,1)-C(1,2)),
             1/(2*sin(th))*(C(0,2)-C(2,0)),   
             1/(2*sin(th))*(C(1,0)-C(0,1));
    }
        
    phi = th*n;
    
   
    return phi;
}

MatrixXd jointToGeoJac(VectorXd q)
{
    MatrixXd Jacobian = MatrixXd::Zero(6,6);
    Jacobian << jointToPosJac(q), jointToRotJac(q);

    return Jacobian;
}

VectorXd inverseKinematics(Vector3d r_des, MatrixXd C_des, VectorXd q0, double tol, std::string dir = "left")
{
    // Input: desired end-effector position, desired end-effector orientation, initial guess for joint angles, threshold for the stopping-criterion
    // Output: joint angles which match desired end-effector position and orientation
    double num_it = 0;
    MatrixXd J_P(3,6), J_R(3,6), J(6,6), pinvJ(6,6), C_err(3,3), C_IE(3,3);
    VectorXd q(6),dq(6),dXe(6);
    Vector3d dr, dph;
    double lambda;
    
    //* Set maximum number of iterations
    double max_it = 200;
    
    //* Initialize the solution with the initial guess
    
    MatrixXd T_I0 = getTransformI0();
    MatrixXd T_01 = jointToTransform01(q, dir);
    MatrixXd T_12 = jointToTransform12(q);
    MatrixXd T_23 = jointToTransform23(q);
    MatrixXd T_34 = jointToTransform34(q);
    MatrixXd T_45 = jointToTransform45(q);
    MatrixXd T_56 = jointToTransform56(q);
    MatrixXd T_6E = getTransform6E();
    
    
    
    
    q = q0;
    C_IE = (T_I0*T_01*T_12*T_23*T_34*T_45*T_56*T_6E).block(0,0,3,3);
    C_err = C_des * C_IE.transpose();
    
    //* Damping factor
    lambda = 0.01;
    
    //* Initialize error
    dr = r_des -  jointToPosition(q, dir);
    dph = rotMatToRotVec(C_err) ;
    dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
    
    ////////////////////////////////////////////////
    //** Iterative inverse kinematics
    ////////////////////////////////////////////////
    
    //* Iterate until terminating condition
    while (num_it<max_it && dXe.norm()>tol)
    {
        
        //Compute Inverse Jacobian
        J_P = jointToPosJac(q, dir);
        J_R = jointToRotJac(q, dir);

        J.block(0,0,3,6) = J_P;
        J.block(3,0,3,6) = J_R; // Geometric Jacobian
        
        // Convert to Geometric Jacobian to Analytic Jacobian
        dq = pseudoInverseMat(J,lambda)*dXe;
        
        // Update law
        q += 0.5*dq;
        
        // Update error
        
        T_I0 = getTransformI0();
        T_01 = jointToTransform01(q, dir);
        T_12 = jointToTransform12(q);
        T_23 = jointToTransform23(q);
        T_34 = jointToTransform34(q);
        T_45 = jointToTransform45(q);
        T_56 = jointToTransform56(q);
        T_6E = getTransform6E();
    
        
    
        C_IE = (T_I0*T_01*T_12*T_23*T_34*T_45*T_56*T_6E).block(0,0,3,3);
        C_err = C_des * C_IE.transpose();
        
        dr = r_des -  jointToPosition(q, dir);
        dph = rotMatToRotVec(C_err) ;
        dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
        
                   
        num_it++;
    }
    //std::cout << "iteration: " << num_it << ", value: " << 180./M_PI*q << std::endl;
    
    return q;
}

double func_1_cos(double t, double init, double fin, double T = 1.)
{
        double des;
        
        double omega = PI/T;
        
        des = (fin - init) * 0.5 * (1 - cos(omega*t)) + init;
        
        
        //...
        
        return des;
}

//* Preparing RobotControl Practice
void Practice()
{
    //double deg2rad = PI / 180;
    //double rad2deg = 180 / PI;
    MatrixXd TI0(4,4), T01(4,4), T12(4,4), T23(4,4), T34(4,4), T45(4,4), T56(4,4), T6E(4,4), TIE(4,4);
    //VectorXd q(10*deg2rad, 20*deg2rad, 30*deg2rad, 40*deg2rad, 50*deg2rad, 60*deg2rad);
    VectorXd q(6);
    //
    q << 10*D2R, 20*D2R, 30*D2R, 40*D2R, 50*D2R, 60*D2R;
    //q << 0, 0, 0, 0, 0, 0;
    //Vector3d q1(10, 20, 30);
    //Vector3d q2(40, 50, 60);
    Vector3d pos, euler;
    MatrixXd CIE(3,3);
    
    //Vector3d q(30, 30, 30);
    TI0 = getTransformI0();
    T01 = jointToTransform01(q);
    T12 = jointToTransform12(q);
    T23 = jointToTransform23(q);
    T34 = jointToTransform34(q);
    T45 = jointToTransform45(q);
    T56 = jointToTransform56(q);
    T6E = getTransform6E();
    
    TIE = TI0*T01*T12*T23*T34*T45*T56*T6E;
    
    pos = jointToPosition(q);
    CIE = jointToRotMat(q);
    euler = rotToEuler(CIE) * R2D;
    
    
    std::cout << "Hello World!" << std::endl;
    std::cout << "TI0 = " << TI0 << std::endl;
    std::cout << "T01 = " << T01 << std::endl;
    std::cout << "T12 = " << T12 << std::endl;
    std::cout << "T23 = " << T23 << std::endl;
    std::cout << "T34 = " << T34 << std::endl;
    std::cout << "T45 = " << T45 << std::endl;
    std::cout << "T45 = " << T56 << std::endl;
    std::cout << "T6E = " << T6E << std::endl;
    std::cout << "TIE = " << TIE << std::endl;
    
    std::cout << std::endl;
    std::cout << "Position = " << pos << std::endl;
    std::cout << "CIE = " << CIE << std::endl;
    std::cout << "Euler(ZYX) = " << euler << std::endl;
    
    
    
    //* Practice 4
    VectorXd q_init(6);
    q_init = q * 0.5;
    
    MatrixXd C_init(3, 3);
    C_init = jointToRotMat(q_init);
    
    MatrixXd C_des(3,3);
    C_des = jointToRotMat(q);
    
    MatrixXd C_err(3, 3);
    C_err = C_des * C_init.transpose();
    
    Vector3d dph;
    dph = rotMatToRotVec(C_err);
    
    std::cout << "dph: \n" << dph << std::endl;
    
    //* Practice 5
    q << 0, 0, -30, 60, -30, 0;
    q *= D2R;
    
    //q_init = q*0.5;
    
    VectorXd q_cal(6);
    Vector3d r_des, r_init;
    
    r_des << 0, 0.105, -0.55;
    C_des = MatrixXd::Identity(3,3);
    
    q_cal = inverseKinematics(r_des, C_des, q, 0.001);
    
    std::cout << "q_cal: \n" << q_cal << std::endl;

}

//


void gazebo::rok3_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    /*
     * Loading model data and initializing the system before simulation 
     */

    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    model = _model;

    //* [physics::ModelPtr model] based model update
    GetJoints();



    //* RBDL API Version Check
    int version_test;
    version_test = rbdl_get_api_version();
    printf(C_GREEN "RBDL API version = %d\n" C_RESET, version_test);

    //* model.urdf file based model data input to [Model* rok3_model] for using RBDL
    Model* rok3_model = new Model();
    Addons::URDFReadFromFile("/home/wonsukji/.gazebo/models/rok3_model/urdf/rok3_model.urdf", rok3_model, true, true);
    //Addons::URDFReadFromFile("/home/wonsukjihomepc/.gazebo/models/rok3_model/urdf/rok3_model.urdf", rok3_model, true, true);
    //↑↑↑ Check File Path ↑↑↑
    nDoF = rok3_model->dof_count - 6; // Get degrees of freedom, except position and orientation of the robot
    joint = new ROBO_JOINT[nDoF]; // Generation joint variables struct

    //* initialize and setting for robot control in gazebo simulation
    initializeJoint();
    SetJointPIDgain();


    //* setting for getting dt
    #if GAZEBO_MAJOR_VERSION >= 8
        last_update_time = model->GetWorld()->SimTime();
    #else
        last_update_time = model->GetWorld()->GetSimTime();
    #endif
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&rok3_plugin::UpdateAlgorithm, this));
    
    //** Init time
    q_left_des << 0, 0, 0, 0, 0, 0;
    //r_left_des = jointToPosition(q_left_des);
    r_left_des = jointToPosition(q_left_des, "left");
    r_left_init = r_left_des;
    
    q_right_des << 0, 0, 0, 0, 0, 0;
    //r_left_des = jointToPosition(q_left_des);
    r_right_des = jointToPosition(q_right_des, "right");
    r_right_init = r_right_des;
    
    
    std::cout << "r_left_init: \n" << r_left_init << std::endl;
    std::cout << "r_right_init: \n" << r_right_init << std::endl;
    
    C_left_des = MatrixXd::Identity(3,3);
    C_right_des = MatrixXd::Identity(3,3);
    
    /*VectorXd q(6);
    //
    //double deg2rad = PI / 180;
    //double rad2deg = 180 / PI;
    q << 10*D2R, 20*D2R, 30*D2R, 40*D2R, 50*D2R, 60*D2R;
    
    jointToPosJac(q);
    jointToRotJac(q);
    
    Practice();*/

}

void gazebo::rok3_plugin::UpdateAlgorithm()
{
    /*
     * Algorithm update while simulation
     */

    //* UPDATE TIME : 1ms
    //common::Time current_time = model->GetWorld()->GetSimTime();
    #if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = model->GetWorld()->SimTime();
    #else
        common::Time current_time = model->GetWorld()->GetSimTime();
    #endif    
    
    dt = current_time.Double() - last_update_time.Double();
    //    cout << "dt:" << dt << endl;
    time = time + dt;
    //    cout << "time:" << time << endl;

    //* setting for getting dt at next step
    last_update_time = current_time;


    //* Read Sensors data
    GetjointData();
    
    //
    //* Algorithm loop
//start of practice 6
#if 0
    switch(practice6_subQuestion){
        case 0:
            if(time <= 5.){
                r_left_des(2) = func_1_cos(time, r_left_init(2), r_left_init(2)+0.3912, 5.);
                //r_left_des(2) = func_1_cos(time, r_left_init(2), r_left_init(2)+0.4, 5.);
                q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);

                //myTemp = func_1_cos(time, 1., 7., 5.);        

                if(time == 1. || time == 2. || time == 3. || time == 4. || time == 5. ){
                    r_left_prev = r_left_des;
                    //std::cout << "time: " << time << "\nmyTemp:" << myTemp << std::endl;
                    prev_time = time;
                }
            }
            
            break;
        
        case 1:
            if(time <= 5.){
                //r_left_des(2) = func_1_cos(time, r_left_init(2), r_left_init(2)+0.3912, 5.);
                r_left_des(2) = func_1_cos(time, r_left_init(2), r_left_init(2)+0.2, 5.);
                q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);

                //myTemp = func_1_cos(time, 1., 7., 5.);        

                if(time == 1. || time == 2. || time == 3. || time == 4. || time == 5. ){
                    r_left_prev = r_left_des;
                    //std::cout << "time: " << time << "\nmyTemp:" << myTemp << std::endl;
                    prev_time = time;
                }
            }
            
            break;
            
        case 2:
            if(time <= 5.){
                //r_left_des(2) = func_1_cos(time, r_left_init(2), r_left_init(2)+0.3912, 5.);
                r_left_des(2) = func_1_cos(time, r_left_init(2), r_left_init(2)+0.2, 5.);
                q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);

                //myTemp = func_1_cos(time, 1., 7., 5.);        

                if(time == 1. || time == 2. || time == 3. || time == 4. || time == 5. ){
                    r_left_prev = r_left_des;
                    //std::cout << "time: " << time << "\nmyTemp:" << myTemp << std::endl;
                    prev_time = time;
                }
            }

            else if(time <= 10.){
                r_left_des(2) = func_1_cos(time-prev_time, r_left_prev(2), r_left_prev(2)-0.2, 5.);
                q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);

                if(time == 10.) {
                    prev_time = time;
                    r_left_prev = r_left_des;
                }
            }
            
            break;
            
        case 3:
            if(time <= 5.){
                //r_left_des(2) = func_1_cos(time, r_left_init(2), r_left_init(2)+0.3912, 5.);
                r_left_des(2) = func_1_cos(time, r_left_init(2), r_left_init(2)+0.2, 5.);
                q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001, "right");

                //myTemp = func_1_cos(time, 1., 7., 5.);        

                if(time == 1. || time == 2. || time == 3. || time == 4. || time == 5. ){
                    r_left_prev = r_left_des;
                    //std::cout << "time: " << time << "\nmyTemp:" << myTemp << std::endl;
                    prev_time = time;
                }
            }

            else if(time <= 10.){
                ori_left_des[2] = func_1_cos(time-prev_time, ori_left_prev[2], ori_left_prev[2]+(90.*D2R), 5.);
                C_left_des = rot_z(ori_left_des[2]);
                q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001, "right");

                if(time == 10.){
                    prev_time = time;
                    r_left_prev = r_left_des;            
                }

            }
            
            break;
        
        
    }
    
    // end of Practice 6
#endif

    // start of Practice 7
    if(!walk_ready_posture){
        if(time <= time_walkready){
            //r_left_des(2) = func_1_cos(time, r_left_init(2), r_left_init(2)+0.3912, 5.);
            
            
            /*r_left_des(2) = func_1_cos(time, r_left_init(2), r_left_init(2)+walk_ready_height, time_walkready);
            q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
            
            r_right_des(2) = func_1_cos(time, r_right_init(2), r_right_init(2)+walk_ready_height, time_walkready);
            q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");*/
            
            q_left_des(0) = 0;
            q_left_des(1) = 0;
            q_left_des(2) = func_1_cos(time, 0, -30.*D2R, time_walkready);
            q_left_des(3) = func_1_cos(time, 0, 60.*D2R, time_walkready);
            q_left_des(4) = func_1_cos(time, 0, -30.*D2R, time_walkready);
            q_left_des(5) = 0;
            
            q_right_des(0) = 0;
            q_right_des(1) = 0;
            q_right_des(2) = func_1_cos(time, 0, -30.*D2R, time_walkready);
            q_right_des(3) = func_1_cos(time, 0, 60.*D2R, time_walkready);
            q_right_des(4) = func_1_cos(time, 0, -30.*D2R, time_walkready);
            q_right_des(5) = 0;
            //q_left_des << 0, 0, -0.05, 0.1, -0.05, 0;
            
            r_left_des = jointToPosition(q_left_des, "left");
            r_right_des = jointToPosition(q_right_des, "right");

            //myTemp = func_1_cos(time, 1., 7., 5.);        

            /*if(time == 1. || time == 2. || time == 3. || time == 4. || time == 5. ){
                r_left_prev = r_left_des;
                r_right_prev = r_right_des;
                //std::cout << "time: " << time << "\nmyTemp:" << myTemp << std::endl;
                prev_time = time;
                if(time >= time_walkready){
                    r_left_prev = r_left_des;
                    r_right_prev = r_right_des;
                    prev_time = time;
                    walk_ready_posture = 1;
                }
            }*/
            
            
            if(time >= time_walkready){
                r_left_prev = r_left_des;
                r_right_prev = r_right_des;
                prev_time = time;
                walk_ready_posture = true;
                
                r_left_home = r_left_des;
                r_right_home = r_right_des;
            }
        }
    }

    // walk ready!!
    else{
                
        // Joystick
        if(time == 23.){
            walk_stop = true;
            
        }
        
        else if(time == 35.){
            //leg_phase = COM_MOVE_RIGHT;
            walk_start = true;
            //leg_phase = TURN_RIGHT;
            myOrder = "turn";
            
        }
        
        else if(time == 53.){
            walk_start = true;
            myOrder = "straight";
        }
        
        if(time == 65.){
            walk_stop = true;
        }
        
        
        
        switch(leg_phase){
            
            
            
            case LEFT_SWING_UP:
                
                // CoM_x_dist
                // swing_length
                std::cout << "case: LEFT_SWING_UP" << std::endl;                
                
                if(time <= swing_period * (t_num+1.) + time_walkready){
                    
                    if(!walk_start_or_finish){
                        swing_length = 0.5*CoM_x_dist;
                        
                        r_left_des(2) = func_1_cos(time-prev_time, r_left_prev(2), r_left_prev(2)+swing_height, swing_period);                      
                        
                        r_left_des(0) = func_1_cos(time-prev_time, r_left_prev(0), r_left_prev(0)+swing_length, swing_period);                      
                        
                        
                        r_right_des(0) = func_1_cos(time-prev_time, r_right_prev(0), r_right_prev(0)-swing_length, swing_period);                      
                        
                    }
                    
                    else{
                        swing_length = CoM_x_dist;
                        /*r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_prev(1)+2*swing_length_yaxis, swing_period);
                        q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                       
                        r_right_des(1) = func_1_cos(time-prev_time, r_right_prev(1), r_right_prev(1)+2*swing_length_yaxis, swing_period);
                        q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");*/
                        
                        r_left_des(2) = func_1_cos(time-prev_time, r_left_prev(2), r_left_prev(2)+swing_height, swing_period);
                        
                        r_left_des(0) = func_1_cos(time-prev_time, r_left_prev(0), r_left_prev(0)+swing_length, swing_period);                      
                        r_right_des(0) = func_1_cos(time-prev_time, r_right_prev(0), r_right_prev(0)-swing_length, swing_period);
                    }
                    
                    q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                    q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");
                    
                    if(time == swing_period * (t_num+1.) + time_walkready){
                            leg_phase = LEFT_SWING_DOWN;
                            t_num++;

                            r_left_prev = r_left_des;
                            r_right_prev = r_right_des;
                            prev_time = time;
                            
                            
                    }
                }
                
                
                break;
                
            case LEFT_SWING_DOWN:
                std::cout << "case: LEFT_SWING_DOWN" << std::endl;                
                
                if(time <= swing_period * (t_num+1.) + time_walkready){
                    
                    
                    if(!walk_start_or_finish){
                        swing_length = 0.5*CoM_x_dist;
                        
                        r_left_des(2) = func_1_cos(time-prev_time, r_left_prev(2), r_left_prev(2)-swing_height, swing_period);
                        
                        r_left_des(0) = func_1_cos(time-prev_time, r_left_prev(0), r_left_prev(0)+swing_length, swing_period);                      
                        r_right_des(0) = func_1_cos(time-prev_time, r_right_prev(0), r_right_prev(0)-swing_length, swing_period);
                        
                    }
                    
                    else{                        
                        swing_length = CoM_x_dist;
                        
                        r_left_des(2) = func_1_cos(time-prev_time, r_left_prev(2), r_left_prev(2)-swing_height, swing_period);
                        
                        r_left_des(0) = func_1_cos(time-prev_time, r_left_prev(0), r_left_prev(0)+swing_length, swing_period);                      
                        r_right_des(0) = func_1_cos(time-prev_time, r_right_prev(0), r_right_prev(0)-swing_length, swing_period);
                        
                    }
                    
                    q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                    q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");
                    
                    if(time == swing_period * (t_num+1.) + time_walkready){
                        
                        if(walk_stop == false){
                            leg_phase = COM_MOVE_LEFT;
                            
                            if(!walk_start_or_finish){
                            walk_start_or_finish = true;
                        }
                            
                        }
                        else{
                            if(walk_start_or_finish){
                                walk_start_or_finish = false;
                                leg_phase = COM_MOVE_LEFT;
                            }
                            else{
                                leg_phase = STOP_MOVING;                                
                            }
                        }
                        
                        t_num++;
                        
                        r_left_prev = r_left_des;
                        r_right_prev = r_right_des;
                        prev_time = time;
                        
                        std::cout << "time: " << time << std::endl;
                        std::cout << "t_num: " << t_num << std::endl;
                        std::cout << "time_walkready: " << time_walkready << "\n" << std::endl;
                        
                        
                    }
                }
                
                
                
                break;
                
            // Right leg
            case RIGHT_SWING_UP:
                // Original
                std::cout << "case: RIGHT_SWING_UP" << std::endl;                
                                             
                if(time <= swing_period * (t_num+1.) + time_walkready){
                    
                    
                    
                    if(!walk_start_or_finish){
                        swing_length = 0.5*CoM_x_dist;
                        
                        r_right_des(2) = func_1_cos(time-prev_time, r_right_prev(2), r_right_prev(2)+swing_height, swing_period);
                        
                        r_left_des(0) = func_1_cos(time-prev_time, r_left_prev(0), r_left_prev(0)-swing_length, swing_period);                      
                        r_right_des(0) = func_1_cos(time-prev_time, r_right_prev(0), r_right_prev(0)+swing_length, swing_period);
                        
                    }
                    
                    else{
                        swing_length = CoM_x_dist;
                        
                        r_right_des(2) = func_1_cos(time-prev_time, r_right_prev(2), r_right_prev(2)+swing_height, swing_period);
                        
                        r_left_des(0) = func_1_cos(time-prev_time, r_left_prev(0), r_left_prev(0)-swing_length, swing_period);                      
                        r_right_des(0) = func_1_cos(time-prev_time, r_right_prev(0), r_right_prev(0)+swing_length, swing_period);
                        
                        
                    }
                    
                    q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                    q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");
                    
                    if(time == swing_period * (t_num+1.) + time_walkready){
                        leg_phase = RIGHT_SWING_DOWN; // original
                        
                        t_num++;
                        
                        r_left_prev = r_left_des;
                        r_right_prev = r_right_des;
                        prev_time = time;
                        
                        
                    }
                }
                
                
                
                break;
                
            case RIGHT_SWING_DOWN:
                
                std::cout << "case: RIGHT_SWING_DOWN" << std::endl;                
                                
                if(time <= swing_period * (t_num+1.) + time_walkready){
                    
                    
                    if(!walk_start_or_finish){
                        swing_length = 0.5*CoM_x_dist;
                        
                        r_right_des(2) = func_1_cos(time-prev_time, r_right_prev(2), r_right_prev(2)-swing_height, swing_period);
                        
                        r_left_des(0) = func_1_cos(time-prev_time, r_left_prev(0), r_left_prev(0)-swing_length, swing_period);                      
                        r_right_des(0) = func_1_cos(time-prev_time, r_right_prev(0), r_right_prev(0)+swing_length, swing_period);
                        
                        
                    }
                    else{
                        swing_length = CoM_x_dist;
                        
                        r_right_des(2) = func_1_cos(time-prev_time, r_right_prev(2), r_right_prev(2)-swing_height, swing_period);
                        
                        r_left_des(0) = func_1_cos(time-prev_time, r_left_prev(0), r_left_prev(0)-swing_length, swing_period);                      
                        r_right_des(0) = func_1_cos(time-prev_time, r_right_prev(0), r_right_prev(0)+swing_length, swing_period);
                        
                        
                    }
                    
                    q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                    q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");
                    
                    if(time == swing_period * (t_num+1.) + time_walkready){
                        //leg_phase = LEFT_SWING_UP; // original
                        
                        if(walk_stop == false){
                            leg_phase = COM_MOVE_RIGHT;
                            
                            if(!walk_start_or_finish){
                            walk_start_or_finish = true;
                        }
                            
                        }
                        else{
                            if(walk_start_or_finish){
                                walk_start_or_finish = false;
                                leg_phase = COM_MOVE_RIGHT;
                            }
                            else{
                                leg_phase = STOP_MOVING;                                
                            }
                        }
                        t_num++;
                        
                        r_left_prev = r_left_des;
                        r_right_prev = r_right_des;
                        prev_time = time;
                        
                        /*if(!walk_start_or_finish){
                            walk_start_or_finish = true;
                        }*/
                    }
                }
                
                break;
                
                
                
                
            case COM_MOVE_RIGHT:
                std::cout << "case: COM_MOVE_RIGHT" << std::endl;                
                                                
                if(time <= swing_period * (t_num+1.) + time_walkready){
                    
                    if(!walk_start_or_finish){
                        if(!walk_stop){
                            r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_prev(1)+swing_length_yaxis, swing_period);                        
                            r_right_des(1) = func_1_cos(time-prev_time, r_right_prev(1), r_right_prev(1)+swing_length_yaxis, swing_period);
                        }
                        else{
                            r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_prev(1)+2*swing_length_yaxis, swing_period);
                            r_right_des(1) = func_1_cos(time-prev_time, r_right_prev(1), r_right_prev(1)+2*swing_length_yaxis, swing_period);
                        
                        }   
                        
                    }
                    
                    else{
                        r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_prev(1)+2*swing_length_yaxis, swing_period);
                       
                        r_right_des(1) = func_1_cos(time-prev_time, r_right_prev(1), r_right_prev(1)+2*swing_length_yaxis, swing_period);
                        
                    }
                    
                    q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                    q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");
                    
                    
                    if(time == swing_period * (t_num+1.) + time_walkready){
                            leg_phase = LEFT_SWING_UP;
                            t_num++;

                            r_left_prev = r_left_des;
                            r_right_prev = r_right_des;
                            prev_time = time;                            
                            
                    }
                }
                
                
                break;
                
                
                
            case COM_MOVE_LEFT:
                std::cout << "case: COM_MOVE_LEFT" << std::endl;                
                                                
                if(time <= swing_period * (t_num+1.) + time_walkready){
                    
                    if(!walk_start_or_finish){
                        if(!walk_stop){
                            r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_prev(1)-swing_length_yaxis, swing_period);
                            r_right_des(1) = func_1_cos(time-prev_time, r_right_prev(1), r_right_prev(1)-swing_length_yaxis, swing_period);
                        }
                        else{
                            r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_prev(1)-2*swing_length_yaxis, swing_period);
                            r_right_des(1) = func_1_cos(time-prev_time, r_right_prev(1), r_right_prev(1)-2*swing_length_yaxis, swing_period);
                        
                        }                        
                        
                    }
                    
                    else{
                        r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_prev(1)-2*swing_length_yaxis, swing_period);
                        
                       
                        r_right_des(1) = func_1_cos(time-prev_time, r_right_prev(1), r_right_prev(1)-2*swing_length_yaxis, swing_period);
                        
                        
                    }
                    
                    q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                    q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");
                    
                    if(time == swing_period * (t_num+1.) + time_walkready){
                            leg_phase = RIGHT_SWING_UP;
                            t_num++;

                            r_left_prev = r_left_des;
                            r_right_prev = r_right_des;
                            prev_time = time;                            
                            
                    }
                }
                
                
                break;
                
                
            case STOP_MOVING:
                std::cout << "case: STOP_MOVING" << std::endl;                
                if(time <= swing_period * (t_num+1.) + time_walkready){
                    
                    if(!walk_start_or_finish){
                        //r_left_home
                        r_left_des(0) = func_1_cos(time-prev_time, r_left_prev(0), r_left_home(0), swing_period);
                        r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_home(1), swing_period);
                        r_left_des(2) = func_1_cos(time-prev_time, r_left_prev(2), r_left_home(2), swing_period);
                        r_right_des(0) = func_1_cos(time-prev_time, r_right_prev(0), r_right_home(0), swing_period);
                        r_right_des(1) = func_1_cos(time-prev_time, r_right_prev(1), r_right_home(1), swing_period);
                        r_right_des(2) = func_1_cos(time-prev_time, r_right_prev(2), r_right_home(2), swing_period);
                        
                    }
                    
                    else{
                        r_left_des(0) = func_1_cos(time-prev_time, r_left_prev(0), r_left_home(0), swing_period);
                        r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_home(1), swing_period);
                        r_left_des(2) = func_1_cos(time-prev_time, r_left_prev(2), r_left_home(2), swing_period);
                        r_right_des(0) = func_1_cos(time-prev_time, r_right_prev(0), r_right_home(0), swing_period);
                        r_right_des(1) = func_1_cos(time-prev_time, r_right_prev(1), r_right_home(1), swing_period);
                        r_right_des(2) = func_1_cos(time-prev_time, r_right_prev(2), r_right_home(2), swing_period);
                        
                    }
                    
                    q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                    q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");
                    
                    if(time == swing_period * (t_num+1.) + time_walkready){
                            //leg_phase = RIGHT_SWING_UP;
                        walk_stop = false;
                        t_num++;

                        r_left_prev = r_left_des;
                        r_right_prev = r_right_des;
                        prev_time = time;      
                        
                        if(walk_start){
                            if(myOrder == "straight"){
                                leg_phase = COM_MOVE_RIGHT;
                                myOrder = "";
                            }
                            else if(myOrder == "turn"){
                                leg_phase = TURN_RIGHT;
                                myOrder = "";
                            }
                        }
                            
                    }
                }
                
                
                break;
                
            case TURN_RIGHT:
                std::cout << "case: TURN RIGHT" << std::endl;
                //TURN_RIGHT_PHASE1
                
                switch(turn_phase){
                    case TURN_RIGHT_PHASE1:  // move com left
                        //
                        if(time <= swing_period * (t_num+1.) + time_walkready){                    
                            r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_prev(1)-0.105, swing_period);
                            r_right_des(1) = func_1_cos(time-prev_time, r_right_prev(1), r_right_prev(1)-0.105, swing_period);
                        }

                        q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                        q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");

                        if(time == swing_period * (t_num+1.) + time_walkready){
                                //leg_phase = RIGHT_SWING_UP;
                            turn_phase = TURN_RIGHT_PHASE2;
                            t_num++;

                            r_left_prev = r_left_des;
                            r_right_prev = r_right_des;
                            prev_time = time;                            
                        }
                        
                        //                   
                        break;
                        
                    case TURN_RIGHT_PHASE2: // up right leg and spin left leg ankle
                        if(time <= swing_period * (t_num+1.) + time_walkready){                    
                            //r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_prev(1)-swing_length_yaxis, swing_period);
                            r_right_des(2) = func_1_cos(time-prev_time, r_right_prev(2), r_right_prev(2)+0.1, swing_period);
                            turn_theta = func_1_cos(time-prev_time, 0., 90.*D2R, swing_period);                            
                            C_left_des = rot_z(turn_theta);
                        }

                        q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                        q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");

                        if(time == swing_period * (t_num+1.) + time_walkready){
                                //leg_phase = RIGHT_SWING_UP;
                            turn_phase = TURN_RIGHT_PHASE3;
                            t_num++;

                            r_left_prev = r_left_des;
                            r_right_prev = r_right_des;
                            prev_time = time;                            
                        }
                        
                        break;
                        
                    case TURN_RIGHT_PHASE3: // down right leg with little -y distance
                        
                        if(time <= swing_period * (t_num+1.) + time_walkready){                    
                            //r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_prev(1)-swing_length_yaxis, swing_period);
                            r_right_des(1) = func_1_cos(time-prev_time, r_right_prev(1), r_right_prev(1)-0.05, swing_period);
                            r_right_des(2) = func_1_cos(time-prev_time, r_right_prev(2), r_right_prev(2)-0.1, swing_period);
                            //turn_theta = func_1_cos(time-prev_time, 0., -90.*D2R, swing_period);                            
                            //C_left_des = rot_z(turn_theta);
                        }

                        q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                        q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");

                        if(time == swing_period * (t_num+1.) + time_walkready){
                                //leg_phase = RIGHT_SWING_UP;
                            turn_phase = TURN_RIGHT_PHASE4;
                            t_num++;

                            r_left_prev = r_left_des;
                            r_right_prev = r_right_des;
                            prev_time = time;                            
                        }
                        
                        break;
                                           
                    case TURN_RIGHT_PHASE4: // move CoM to left
                        
                        if(time <= swing_period * (t_num+1.) + time_walkready){                    
                            //r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_prev(1)-swing_length_yaxis, swing_period);
                            
                            r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_prev(1)+2*0.105+(0.05), swing_period);
                            r_right_des(1) = func_1_cos(time-prev_time, r_right_prev(1), r_right_prev(1)+2*0.105+0.05+(0.05), swing_period);
                            
                            //r_right_des(2) = func_1_cos(time-prev_time, r_right_prev(2), r_right_prev(2)-0.1, swing_period);
                            //turn_theta = func_1_cos(time-prev_time, 0., -90.*D2R, swing_period);                            
                            //C_left_des = rot_z(turn_theta);
                        }

                        q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                        q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");

                        if(time == swing_period * (t_num+1.) + time_walkready){
                                //leg_phase = RIGHT_SWING_UP;
                            turn_phase = TURN_RIGHT_PHASE5;
                            t_num++;

                            r_left_prev = r_left_des;
                            r_right_prev = r_right_des;
                            prev_time = time;                            
                        }
                        
                        break;   
                    
                    case TURN_RIGHT_PHASE5: // up left leg
                        
                        if(time <= swing_period * (t_num+1.) + time_walkready){                    
                            //r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_prev(1)-swing_length_yaxis, swing_period);
                            
                            r_left_des(2) = func_1_cos(time-prev_time, r_left_prev(2), r_left_prev(2)+0.1, swing_period);
                            
                            /*if(time >= swing_period * (t_num) + time_walkready + 0.5)
                            {
                                turn_theta = func_1_cos(time-prev_time, 90.*D2R, 0.*D2R, swing_period-0.5);
                                C_left_des = rot_z(turn_theta);
                            }*/
                                
                            //r_right_des(2) = func_1_cos(time-prev_time, r_right_prev(2), r_right_prev(2)-0.1, swing_period);
                            //turn_theta = func_1_cos(time-prev_time, 0., -90.*D2R, swing_period);                            
                            //C_left_des = rot_z(turn_theta);
                        }

                        q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                        q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");

                        if(time == swing_period * (t_num+1.) + time_walkready){
                                //leg_phase = RIGHT_SWING_UP;
                            turn_phase = TURN_RIGHT_PHASE6;
                            t_num++;

                            r_left_prev = r_left_des;
                            r_right_prev = r_right_des;
                            prev_time = time;                            
                        }
                        
                        break;
                        
                        
                    case TURN_RIGHT_PHASE6: // spin left hip yaw
                        
                        if(time <= swing_period * (t_num+1.) + time_walkready){                    
                            //r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_prev(1)-swing_length_yaxis, swing_period);
                            
                            //r_left_des(2) = func_1_cos(time-prev_time, r_left_prev(2), r_left_prev(2)+0.1, swing_period);
                            
                            turn_theta = func_1_cos(time-prev_time, 90.*D2R, 0.*D2R, swing_period);
                            C_left_des = rot_z(turn_theta);
  
                        }

                        q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                        q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");

                        if(time == swing_period * (t_num+1.) + time_walkready){
                                //leg_phase = RIGHT_SWING_UP;
                            turn_phase = TURN_RIGHT_PHASE7;
                            t_num++;

                            r_left_prev = r_left_des;
                            r_right_prev = r_right_des;
                            prev_time = time;                            
                        }
                        
                        break;   
                    
                    case TURN_RIGHT_PHASE7: // left leg down
                        
                        if(time <= swing_period * (t_num+1.) + time_walkready){                    
                            r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_prev(1)+0.05, swing_period);                            
                            r_left_des(2) = func_1_cos(time-prev_time, r_left_prev(2), r_left_home(2), swing_period);
                            
                           
  
                        }

                        q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                        q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");

                        if(time == swing_period * (t_num+1.) + time_walkready){
                                //leg_phase = RIGHT_SWING_UP;
                            turn_phase = TURN_RIGHT_PHASE8;
                            t_num++;

                            r_left_prev = r_left_des;
                            r_right_prev = r_right_des;
                            prev_time = time;                            
                        }
                       
                        break;
                        
                        
                        
                    case TURN_RIGHT_PHASE8: // finish
                        
                        
                        if(time <= swing_period * (t_num+1.) + time_walkready){                    
                            //r_left_des(1) = func_1_cos(time-prev_time, r_left_prev(1), r_left_home(1), swing_period);                            
                            //r_left_des(2) = func_1_cos(time-prev_time, r_left_prev(2), r_left_home(2), swing_period);
                            
                            for(size_t i = 0; i < 3; ++i){
                                r_left_des(i) = func_1_cos(time-prev_time, r_left_prev(i), r_left_home(i), swing_period);
                                r_right_des(i) = func_1_cos(time-prev_time, r_right_prev(i), r_right_home(i), swing_period);
                            }
                            //turn_theta = func_1_cos(time-prev_time, 90.*D2R, 0.*D2R, swing_period);
                            //C_left_des = rot_z(turn_theta);
  
                        }

                        q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
                        q_right_des = inverseKinematics(r_right_des, C_right_des, q_right_des, 0.001, "right");

                        if(time == swing_period * (t_num+1.) + time_walkready){
                                //leg_phase = RIGHT_SWING_UP;
                            turn_phase = TURN_RIGHT_PHASE1;
                            
                            leg_phase = STOP_MOVING;
                            walk_stop = true;
                            walk_start = false;
                            //leg_phase = COM_MOVE_RIGHT;
                            
                            t_num++;

                            r_left_prev = r_left_des;
                            r_right_prev = r_right_des;
                            prev_time = time;                            
                        }
                        
                        break;    
                }
                
                               
                
                break;
        }
        
    }
    
    /*
    if(time <= 5.){
        //r_left_des(2) = func_1_cos(time, r_left_init(2), r_left_init(2)+0.3912, 5.);
        r_left_des(2) = func_1_cos(time, r_left_init(2), r_left_init(2)+0.4, 5.);
        q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
        
        //myTemp = func_1_cos(time, 1., 7., 5.);        
        
        if(time == 1. || time == 2. || time == 3. || time == 4. || time == 5. ){
            r_left_prev = r_left_des;
            //std::cout << "time: " << time << "\nmyTemp:" << myTemp << std::endl;
            prev_time = time;
        }
    }
    
    else if(time <= 10.){
        r_left_des(2) = func_1_cos(time-prev_time, r_left_prev(2), r_left_prev(2)-0.2, 5.);
        q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);

        if(time == 10.) {
            prev_time = time;
            r_left_prev = r_left_des;
        }
    }
    
    else if(time <= 15.){
        ori_left_des[2] = func_1_cos(time-prev_time, ori_left_prev[2], ori_left_prev[2]+(90.*D2R), 5.);
        C_left_des = rot_z(ori_left_des[2]);
        q_left_des = inverseKinematics(r_left_des, C_left_des, q_left_des, 0.001);
        
        if(time == 15.){
            prev_time = time;
            r_left_prev = r_left_des;            
        }
        
    }
    */
    
    
    //* End of Algorithm loop
    
    /*joint[LHY].targetRadian = 10*3.14/180;
    joint[LHR].targetRadian = 20*3.14/180;
    joint[LHP].targetRadian = 30*3.14/180;
    joint[LKN].targetRadian = 40*3.14/180;
    joint[LAP].targetRadian = 50*3.14/180;
    joint[LAR].targetRadian = 60*3.14/180;*/
    for(size_t i = 0; i < 6; ++i){
        joint[i+1].targetRadian = q_left_des(i);
        joint[i+1+6].targetRadian = q_right_des(i);
    }
    //
    
    /*if(static_cast<int>(time) % 1 == 0.){
        std::cout << "r_left_des: " << r_left_des << std::endl; 
        std::cout << "r_right_des: " << r_right_des << std::endl;
    }*/
    
    //* Joint Controller
    jointController();
}

void gazebo::rok3_plugin::jointController()
{
    /*
     * Joint Controller for each joint
     */

    // Update target torque by control
    for (int j = 0; j < nDoF; j++) {
        joint[j].targetTorque = joint[j].Kp * (joint[j].targetRadian-joint[j].actualRadian)\
                              + joint[j].Kd * (joint[j].targetVelocity-joint[j].actualVelocity);
    }

    // Update target torque in gazebo simulation     
    L_Hip_yaw_joint->SetForce(0, joint[LHY].targetTorque);
    L_Hip_roll_joint->SetForce(0, joint[LHR].targetTorque);
    L_Hip_pitch_joint->SetForce(0, joint[LHP].targetTorque);
    L_Knee_joint->SetForce(0, joint[LKN].targetTorque);
    L_Ankle_pitch_joint->SetForce(0, joint[LAP].targetTorque);
    L_Ankle_roll_joint->SetForce(0, joint[LAR].targetTorque);

    R_Hip_yaw_joint->SetForce(0, joint[RHY].targetTorque);
    R_Hip_roll_joint->SetForce(0, joint[RHR].targetTorque);
    R_Hip_pitch_joint->SetForce(0, joint[RHP].targetTorque);
    R_Knee_joint->SetForce(0, joint[RKN].targetTorque);
    R_Ankle_pitch_joint->SetForce(0, joint[RAP].targetTorque);
    R_Ankle_roll_joint->SetForce(0, joint[RAR].targetTorque);

    torso_joint->SetForce(0, joint[WST].targetTorque);
}

void gazebo::rok3_plugin::GetJoints()
{
    /*
     * Get each joints data from [physics::ModelPtr _model]
     */

    //* Joint specified in model.sdf
    L_Hip_yaw_joint = this->model->GetJoint("L_Hip_yaw_joint");
    L_Hip_roll_joint = this->model->GetJoint("L_Hip_roll_joint");
    L_Hip_pitch_joint = this->model->GetJoint("L_Hip_pitch_joint");
    L_Knee_joint = this->model->GetJoint("L_Knee_joint");
    L_Ankle_pitch_joint = this->model->GetJoint("L_Ankle_pitch_joint");
    L_Ankle_roll_joint = this->model->GetJoint("L_Ankle_roll_joint");
    R_Hip_yaw_joint = this->model->GetJoint("R_Hip_yaw_joint");
    R_Hip_roll_joint = this->model->GetJoint("R_Hip_roll_joint");
    R_Hip_pitch_joint = this->model->GetJoint("R_Hip_pitch_joint");
    R_Knee_joint = this->model->GetJoint("R_Knee_joint");
    R_Ankle_pitch_joint = this->model->GetJoint("R_Ankle_pitch_joint");
    R_Ankle_roll_joint = this->model->GetJoint("R_Ankle_roll_joint");
    torso_joint = this->model->GetJoint("torso_joint");

    //* FTsensor joint
    LS = this->model->GetJoint("LS");
    RS = this->model->GetJoint("RS");
}

void gazebo::rok3_plugin::GetjointData()
{
    /*
     * Get encoder and velocity data of each joint
     * encoder unit : [rad] and unit conversion to [deg]
     * velocity unit : [rad/s] and unit conversion to [rpm]
     */

 #if GAZEBO_MAJOR_VERSION >= 8

    
    joint[LHY].actualRadian = L_Hip_yaw_joint->Position(0);
    joint[LHR].actualRadian = L_Hip_roll_joint->Position(0);
    joint[LHP].actualRadian = L_Hip_pitch_joint->Position(0);
    joint[LKN].actualRadian = L_Knee_joint->Position(0);
    joint[LAP].actualRadian = L_Ankle_pitch_joint->Position(0);
    joint[LAR].actualRadian = L_Ankle_roll_joint->Position(0);

    joint[RHY].actualRadian = R_Hip_yaw_joint->Position(0);
    joint[RHR].actualRadian = R_Hip_roll_joint->Position(0);
    joint[RHP].actualRadian = R_Hip_pitch_joint->Position(0);
    joint[RKN].actualRadian = R_Knee_joint->Position(0);
    joint[RAP].actualRadian = R_Ankle_pitch_joint->Position(0);
    joint[RAR].actualRadian = R_Ankle_roll_joint->Position(0);

    joint[WST].actualRadian = torso_joint->Position(0);
    
  #else
    joint[LHY].actualRadian = L_Hip_yaw_joint->GetAngle(0).Radian();
    joint[LHR].actualRadian = L_Hip_roll_joint->GetAngle(0).Radian();
    joint[LHP].actualRadian = L_Hip_pitch_joint->GetAngle(0).Radian();
    joint[LKN].actualRadian = L_Knee_joint->GetAngle(0).Radian();
    joint[LAP].actualRadian = L_Ankle_pitch_joint->GetAngle(0).Radian();
    joint[LAR].actualRadian = L_Ankle_roll_joint->GetAngle(0).Radian();

    joint[RHY].actualRadian = R_Hip_yaw_joint->GetAngle(0).Radian();
    joint[RHR].actualRadian = R_Hip_roll_joint->GetAngle(0).Radian();
    joint[RHP].actualRadian = R_Hip_pitch_joint->GetAngle(0).Radian();
    joint[RKN].actualRadian = R_Knee_joint->GetAngle(0).Radian();
    joint[RAP].actualRadian = R_Ankle_pitch_joint->GetAngle(0).Radian();
    joint[RAR].actualRadian = R_Ankle_roll_joint->GetAngle(0).Radian();

    joint[WST].actualRadian = torso_joint->GetAngle(0).Radian();
  #endif


    for (int j = 0; j < nDoF; j++) {
        joint[j].actualDegree = joint[j].actualRadian*R2D;
    }


    joint[LHY].actualVelocity = L_Hip_yaw_joint->GetVelocity(0);
    joint[LHR].actualVelocity = L_Hip_roll_joint->GetVelocity(0);
    joint[LHP].actualVelocity = L_Hip_pitch_joint->GetVelocity(0);
    joint[LKN].actualVelocity = L_Knee_joint->GetVelocity(0);
    joint[LAP].actualVelocity = L_Ankle_pitch_joint->GetVelocity(0);
    joint[LAR].actualVelocity = L_Ankle_roll_joint->GetVelocity(0);

    joint[RHY].actualVelocity = R_Hip_yaw_joint->GetVelocity(0);
    joint[RHR].actualVelocity = R_Hip_roll_joint->GetVelocity(0);
    joint[RHP].actualVelocity = R_Hip_pitch_joint->GetVelocity(0);
    joint[RKN].actualVelocity = R_Knee_joint->GetVelocity(0);
    joint[RAP].actualVelocity = R_Ankle_pitch_joint->GetVelocity(0);
    joint[RAR].actualVelocity = R_Ankle_roll_joint->GetVelocity(0);

    joint[WST].actualVelocity = torso_joint->GetVelocity(0);


    //    for (int j = 0; j < nDoF; j++) {
    //        cout << "joint[" << j <<"]="<<joint[j].actualDegree<< endl;
    //    }

}

void gazebo::rok3_plugin::initializeJoint()
{
    /*
     * Initialize joint variables for joint control
     */
    
    for (int j = 0; j < nDoF; j++) {
        joint[j].targetDegree = 0;
        joint[j].targetRadian = 0;
        joint[j].targetVelocity = 0;
        joint[j].targetTorque = 0;
        
        joint[j].actualDegree = 0;
        joint[j].actualRadian = 0;
        joint[j].actualVelocity = 0;
        joint[j].actualRPM = 0;
        joint[j].actualTorque = 0;
    }
}

void gazebo::rok3_plugin::SetJointPIDgain()
{
    /*
     * Set each joint PID gain for joint control
     */
    joint[LHY].Kp = 2000;
    joint[LHR].Kp = 9000;
    joint[LHP].Kp = 2000;
    joint[LKN].Kp = 5000;
    joint[LAP].Kp = 3000;
    joint[LAR].Kp = 3000;

    joint[RHY].Kp = joint[LHY].Kp;
    joint[RHR].Kp = joint[LHR].Kp;
    joint[RHP].Kp = joint[LHP].Kp;
    joint[RKN].Kp = joint[LKN].Kp;
    joint[RAP].Kp = joint[LAP].Kp;
    joint[RAR].Kp = joint[LAR].Kp;

    joint[WST].Kp = 2.;

    joint[LHY].Kd = 2.;
    joint[LHR].Kd = 2.;
    joint[LHP].Kd = 2.;
    joint[LKN].Kd = 4.;
    joint[LAP].Kd = 2.;
    joint[LAR].Kd = 2.;

    joint[RHY].Kd = joint[LHY].Kd;
    joint[RHR].Kd = joint[LHR].Kd;
    joint[RHP].Kd = joint[LHP].Kd;
    joint[RKN].Kd = joint[LKN].Kd;
    joint[RAP].Kd = joint[LAP].Kd;
    joint[RAR].Kd = joint[LAR].Kd;

    joint[WST].Kd = 2.;
}


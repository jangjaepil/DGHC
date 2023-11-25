#include "WHQP.h"
#include "GHCProjections.hpp"

#include <human_factor/LinkStateArray.h>
#include <human_factor/LinkState.h>
#include <human_factor/RiskFactor.h>
#include <std_msgs/Float64MultiArray.h>

#include <std_msgs/Int32.h>
#include "hrii_ra_interface/robot_interface/RoboticArmInterface.h"
#include "hrii_mor_interface/MobileRobotInterface.h"
#define MIN_LINEAR_WRENCH 2
#define MAX_LINEAR_WRENCH 100
#define MIN_ANGULAR_WRENCH 0.1
#define MAX_ANGULAR_WRENCH 50
#define LINEAR 1
#define ANGULAR 2
#define MAX_COP_DISPLACEMENT 0.03
#define MAX_COP_TORQUES 15

#pragma once

//순서대로, taskNumber, a_ii, a_ij
typedef std::vector<std::tuple<int,double,double>> priorityTuples;

class dghc_controller : public GHCProjections{
public:
    dghc_controller();
    
     void obstacle_states_callback(const human_factor::LinkStateArray::ConstPtr& obstacleStateMsg);

    void desired_pose_callback(const geometry_msgs::Pose& desired_pose);

    void mode_input_callback(const std_msgs::Bool::ConstPtr& mode_input);
    void priority_input_callback(const std_msgs::String::ConstPtr& priority_input_string);
    void mass_callback(const geometry_msgs::Inertia& mass_matrix);
    void riskfactor_callback(const human_factor::RiskFactor::ConstPtr& riskfactor);
    void cooperator_callback(const std_msgs::Int32& target);
    void mobile_target_callback(const std_msgs::Int32& mobile_target);
    void optitrack_callback(const geometry_msgs::Pose::ConstPtr& opti_pose);
    void d_dot_callback(const std_msgs::Float64MultiArray::ConstPtr& d_dot_msg);


    std::vector<double> getDesiredAlphas(priorityTuples priority);
    void getModel();
    void getJacobian();
    void getWrench();
    void setInertia();
    void setPriority();
    void setTrackingPriority(const int manipulatorTaskNum, const int mobileTaskNum, const int poseTaskNum);
    void setObstaclePrirority(const std::vector<int> obstacleTaskNums);
    void setJointLimitPriority(const std::vector<int> jointLimitTaskNums);
    void priorityFilter();
    void changeAlphas(std::vector<double>& alphas,double t, double dt, double duration);
    void getProjectionM();
    void getProjectedToq();
    bool alphasSetDone(const std::vector<double>& vec1, const std::vector<double>& vec2, double epsilon);
    void energy_filter();
    int run();
    void checkDeadzone(double &value, int wrenchType);
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated,const Eigen::Matrix<double, 7, 1> &tau_J_d);

private:
   
    HRII::MORInterface::MobileRobotInterface::Ptr mobile_base_interface_;
    HRII::RAInterface::RoboticArmInterface::Ptr arm_interface_;
    Eigen::VectorXd arm_tau_J_d;
    std::string interface_type_str;
    std::string robot_ns;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd q_past = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(7);
    Eigen::MatrixXd M_j_, M_j_up,C_j_;
    Eigen::VectorXd G_j_;
    Eigen::MatrixXd M_h_, C_h_;
    Eigen::VectorXd G_h_;
    Eigen::MatrixXd wholeM = Eigen::MatrixXd::Zero(10,10);
    Eigen::MatrixXd j;
    Eigen::MatrixXd j_dot;
    Eigen::VectorXd vir_force = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd toq = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd toqT = Eigen::VectorXd::Zero(10);

    ros::NodeHandle nh;

    ros::Subscriber obstacle_states;
    ros::Subscriber d_pose_sub;

    ros::Subscriber priority_input_sub;
    ros::Subscriber mode_sub;
    ros::Subscriber mass_sub;
    ros::Subscriber riskfactor_sub;
    ros::Subscriber cooperator_sub;
    ros::Subscriber mobile_target_sub;
    ros::Subscriber d_dot_sub;

    ros::Subscriber optitrack_sub; // optitrack data from natnet

    ros::Publisher Vir_torque;
    ros::Publisher modelPosepublisher; // for human factor  
    ros::Publisher modelVelpublisher;


    double dt = 0.001;
    double current_sec = 0;
    Eigen::MatrixXd jt;
    Eigen::MatrixXd jt_dot;
    Eigen::MatrixXd j1 = Eigen::MatrixXd::Zero(6,10);
    Eigen::MatrixXd j2 = Eigen::MatrixXd::Zero(6,10);
    Eigen::MatrixXd j3 = Eigen::MatrixXd::Zero(6,10);
    Eigen::MatrixXd j4 = Eigen::MatrixXd::Zero(6,10);
    Eigen::MatrixXd j5 = Eigen::MatrixXd::Zero(6,10);
    Eigen::MatrixXd j6 = Eigen::MatrixXd::Zero(6,10);
    Eigen::MatrixXd j7 = Eigen::MatrixXd::Zero(6,10);

    Eigen::MatrixXd jt1 = Eigen::MatrixXd::Zero(3,10);
    Eigen::MatrixXd jt2 = Eigen::MatrixXd::Zero(3,10);
    Eigen::MatrixXd jt3 = Eigen::MatrixXd::Zero(3,10);
    Eigen::MatrixXd jt4 = Eigen::MatrixXd::Zero(3,10);
    Eigen::MatrixXd jt5 = Eigen::MatrixXd::Zero(3,10);
    Eigen::MatrixXd jt6 = Eigen::MatrixXd::Zero(3,10);
    Eigen::MatrixXd jt7 = Eigen::MatrixXd::Zero(3,10);

    Eigen::MatrixXd j_dot1 = Eigen::MatrixXd::Zero(6,10);
    Eigen::MatrixXd j_dot2 = Eigen::MatrixXd::Zero(6,10);
    Eigen::MatrixXd j_dot3 = Eigen::MatrixXd::Zero(6,10);
    Eigen::MatrixXd j_dot4 = Eigen::MatrixXd::Zero(6,10);
    Eigen::MatrixXd j_dot5 = Eigen::MatrixXd::Zero(6,10);
    Eigen::MatrixXd j_dot6 = Eigen::MatrixXd::Zero(6,10);
    Eigen::MatrixXd j_dot7 = Eigen::MatrixXd::Zero(6,10);



    Eigen::MatrixXd j0 = Eigen::MatrixXd::Zero(7,10);
    Eigen::MatrixXd jmt = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd jm = Eigen::MatrixXd::Zero(6,3);
    Eigen::MatrixXd jmt2d = Eigen::MatrixXd::Zero(2,3);  
    Eigen::MatrixXd jimp = Eigen::MatrixXd::Zero(6,10);
    Eigen::MatrixXd jrimp = Eigen::MatrixXd::Zero(6,10);
    Eigen::MatrixXd jrimpT = Eigen::MatrixXd::Zero(5,10);
    Eigen::MatrixXd jobs0 = Eigen::MatrixXd::Zero(1,10);
    Eigen::MatrixXd jobs1 = Eigen::MatrixXd::Zero(1,10);
    Eigen::MatrixXd jobs4 = Eigen::MatrixXd::Zero(1,10);
    Eigen::MatrixXd jobs7 = Eigen::MatrixXd::Zero(1,10);
    Eigen::MatrixXd jobs_tmp = Eigen::MatrixXd::Zero(3,10);
    Eigen::MatrixXd jobs_tmp2d = Eigen::MatrixXd::Zero(2,10);
    Eigen::MatrixXd jmimp = Eigen::MatrixXd::Zero(3,10);
    Eigen::MatrixXd jwT = Eigen::MatrixXd::Zero(10,6);
    Eigen::MatrixXd jw = Eigen::MatrixXd::Zero(6,10);
    Eigen::MatrixXd jimp_pinv = Eigen::MatrixXd::Zero(10,6);
    Eigen::MatrixXd tmp_inv = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd jmt3d = Eigen::MatrixXd::Identity(3,3);

    Eigen::MatrixXd jjl1 = Eigen::MatrixXd::Zero(1,10);
    Eigen::MatrixXd jjl2 = Eigen::MatrixXd::Zero(1,10); 
    Eigen::MatrixXd jjl3 = Eigen::MatrixXd::Zero(1,10); 
    Eigen::MatrixXd jjl4 = Eigen::MatrixXd::Zero(1,10); 
    Eigen::MatrixXd jjl5 = Eigen::MatrixXd::Zero(1,10); 
    Eigen::MatrixXd jjl6 = Eigen::MatrixXd::Zero(1,10); 
    Eigen::MatrixXd jjl7 = Eigen::MatrixXd::Zero(1,10); 

    Eigen::MatrixXd Aw = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(10,10);
    Eigen::MatrixXd I10 = Eigen::MatrixXd::Identity(10,10);




    Eigen::VectorXd wrench0; 
    double wrenchObs0;
    double wrenchObs1;
    double wrenchObs4;
    double wrenchObs7;
    Eigen::VectorXd wrenchImp = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd wrenchImpT = Eigen::VectorXd::Zero(5);
    Eigen::VectorXd wrenchmImp = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd wrenchjl = Eigen::VectorXd::Zero(7);

    std::array<Eigen::Vector3d, 11> obstacleStates;
    int coop_tar = 0;
    int mobile_target = 4;
    bool coop_mode = false;
    std::array<double,44> riskFactorArray;
    double factor_lv1=2;
    double factor_lv2=3;

    Eigen::VectorXd d_position = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd d_position_tmp = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd d_pose = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd d_position2d = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd d_position3d = Eigen::VectorXd::Zero(3);
    double margin = 0.7; //0.8
    Eigen::Vector2d p2m = Eigen::VectorXd::Zero(2);
    Eigen::Vector2d u_p2m = Eigen::VectorXd::Zero(2);
    Eigen::Vector2d d_position2d_tmp = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd d_position3d_tmp = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd u_p2mani = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd p2mani = Eigen::VectorXd::Zero(3);
    double margin_end = 0.12;
    Eigen::VectorXd d_position_m = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd d_euler = Eigen::VectorXd::Zero(3);


    Eigen::MatrixXd k = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd k_tmp = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(6,6);
    Eigen::VectorXd dq2dot;
    Eigen::VectorXd xdd;

    Eigen::Vector3d arm_tmp_position = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d position;
    Eigen::Vector3d position1;
    Eigen::Vector3d position2;
    Eigen::Vector3d position3;
    Eigen::Vector3d position4;
    Eigen::Vector3d position5;
    Eigen::Vector3d position6;
    Eigen::Vector3d position7;
    Eigen::Vector2d position72d;
    Eigen::VectorXd position76d = Eigen::VectorXd::Zero(6,1);

    Eigen::Vector3d vel;
    Eigen::Vector3d vel1;
    Eigen::Vector3d vel2;
    Eigen::Vector3d vel3;
    Eigen::Vector3d vel4;
    Eigen::Vector3d vel5;
    Eigen::Vector3d vel6;
    Eigen::Vector3d vel7;

    Eigen::VectorXd twist;
    Eigen::VectorXd twist1;
    Eigen::VectorXd twist2;
    Eigen::VectorXd twist3;
    Eigen::VectorXd twist4;
    Eigen::VectorXd twist5;
    Eigen::VectorXd twist6;
    Eigen::VectorXd twist7;


    Eigen::Matrix3d mRe = Eigen::Matrix3Xd::Identity(3,3);
    Eigen::Matrix3d wRe = Eigen::Matrix3Xd::Identity(3,3);
    Eigen::Matrix3d eRd = Eigen::Matrix3Xd::Identity(3,3);
    Eigen::Matrix3d wRd = Eigen::Matrix3Xd::Identity(3,3);
    Eigen::MatrixXd R_m_e = Eigen::MatrixXd::Identity(6,6);
    Eigen::Matrix3d R_d = Eigen::Matrix3d::Identity(3,3);
    Eigen::MatrixXd w_T_e_tmp = Eigen::MatrixXd::Identity(4,4);
    Eigen::MatrixXd w_T_e = Eigen::MatrixXd::Identity(4,4);
    Eigen::Vector3d optitrack_p = Eigen::Vector3d::Zero();
    Eigen::Quaterniond optitrack_q;
    Eigen::Quaterniond arm_tmp_quat;
    Eigen::MatrixXd w_T_m_raw = Eigen::MatrixXd::Identity(4,4);
    Eigen::Matrix3d w_R_m = Eigen::Matrix3Xd::Identity(3,3);
    Eigen::Matrix3d w_R_o = Eigen::Matrix3Xd::Identity(3,3);
    Eigen::Quaterniond mobile_quat_r;
    Eigen::VectorXd mobile_pose_r = Eigen::VectorXd::Zero(3);
    Eigen::Quaterniond mobile_quat_filtered;
    nav_msgs::Odometry mobile_base_odom_filtered;



Eigen::Matrix3d R = Eigen::Matrix3d::Identity(3,3);
Eigen::Matrix3d R_tmp = Eigen::Matrix3d::Identity(3,3);









    // Eigen::Matrix3d optitrack_r = Eigen::Matrix3d::Identity();
    Eigen::Vector3d init_distance = Eigen::Vector3d::Zero();
    Eigen::Quaterniond init_quarternion;
    Eigen::Vector3d slip_distance = Eigen::Vector3d::Zero();
    Eigen::Quaterniond slip_quaternion;
    Eigen::Matrix4d init_o_T_M;
    Eigen::MatrixXd arm_tmp_r = Eigen::MatrixXd::Identity(3,3);

    Eigen::VectorXd init_q =  Eigen::VectorXd::Zero(7);
    Eigen::VectorXd lbq =  Eigen::VectorXd::Zero(7);
    Eigen::VectorXd ubq =  Eigen::VectorXd::Zero(7);
    Eigen::VectorXd init_position = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd k_j  = Eigen::MatrixXd::Zero(7,7);
    Eigen::MatrixXd b_j  = Eigen::MatrixXd::Zero(7,7);
    Eigen::MatrixXd k_j_m1  = Eigen::MatrixXd::Zero(7,7);
    Eigen::MatrixXd b_j_m1  = Eigen::MatrixXd::Zero(7,7);
    Eigen::MatrixXd k_m  = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd b_m  = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd k_m_m1  = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd b_m_m1  = Eigen::MatrixXd::Zero(3,3);
    Eigen::VectorXd euler_m = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd euler_e = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(7);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7,7);
    Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd Z37 = Eigen::MatrixXd::Zero(3,7);
    Eigen::MatrixXd Z33 = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd Z63 = Eigen::MatrixXd::Zero(6,3);
    Eigen::MatrixXd Z27 = Eigen::MatrixXd::Zero(2,7);
    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(7,7);
    Eigen::MatrixXd rot_z = Eigen::MatrixXd::Zero(3,3);
    Eigen::Quaterniond mobile_quat;


    Eigen::VectorXd mobile_pose = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd mobile_pose2d = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd mobile_pose3d = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd mobile_vel = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd mobile_vel2d = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd mobile_vel3d = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd mobile_twist = Eigen::VectorXd::Zero(6);
    Eigen::Matrix3d mobile_mass;
    Eigen::Matrix3d R_m = Eigen::Matrix3d::Identity(3,3);
    Eigen::VectorXd R_distance = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd m_distance = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd obstacle = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd obstacle2d = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd u2d = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd P2d = Eigen::MatrixXd::Identity(2,3);



    Eigen::VectorXd f02d = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd f0 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd f1 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd f4 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd f7 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd f_tmp = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd u1 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd u4 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd u7 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd D02d = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd D0v = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd D0 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd D1 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd D3 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd D5 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd D01 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd D11 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd D31 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd D51 = Eigen::VectorXd::Zero(3);

    Eigen::Vector3d quatV = Eigen::Vector3d::Zero(3,1);
    Eigen::VectorXd jl_k = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd jl_b = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd jl_alpha = Eigen::VectorXd::Zero(7);
    double distance; 
    short min_i = 0;
    double d_min = 0;
    double proj_margin = 0;

    double d[4][11] = {};
    float d_dot[44] = {};
    double d_past[4][11] = {};
    double dref[4] = {};
    double alpha[4] = {};

    double inner_space = 0.8; //0.698
    double work_space = 1.0; //0.7
    std::vector<Eigen::MatrixXd> allProjections;

    int numberOfTasks;
    std::string prevPriorityInputString;
    priorityTuples priorityInput;
    priorityTuples filteredPriority;
    priorityTuples prevFilteredPriority;

    std::vector<double> desiredAlphas;
    std::vector<double> scaleValues;

    bool priorityChanged;
    bool alphaChangeDone;
    bool modeInput;

    bool optitrack_sign;
    int count = 0;

    Eigen::Vector3d relativePosition;
    ros::Time previousTime;

    double transitionTime;
    const double delta_tau_max_{0.5};


    double play_time = 0;

    
    //mode 0
    // double a00=0;
    // double a77=0;
    // double a07=0;

    //mode 1
    double a00=0;
    double a11=0;
    double a01=0;

    double a55=0;
    double a05=0;
    double a15=0;

    double a66=0;
    double a06=0;
    double a16=0;

    double a77=0;
    double a07=0;
    double a17=0;

    double a44=0;
    double a24=0;

    double a45 = 0;
    double a46 = 0;
    double a47 = 0;
    double a56 = 0;
    double a57 = 0;
    double a67 = 0;

    Eigen::Matrix4d init_w_T_M, init_w_T_o;
    Eigen::Matrix4d real_mobile_Tf = Eigen::MatrixXd::Identity(4,4);
    Eigen::Matrix4d new_w_T_o = Eigen::MatrixXd::Identity(4,4);

};
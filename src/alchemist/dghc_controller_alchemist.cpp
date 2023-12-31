#include "dghc_controller_alchemist.hpp"
#include "GHCProjections.hpp"
#include "matlogger2/matlogger2.h"

dghc_controller::dghc_controller()
{

    priority_input_sub = nh.subscribe("/priorityInput", 1000, &dghc_controller::priority_input_callback, this);
    mode_sub = nh.subscribe("/modeInput", 1000, &dghc_controller::mode_input_callback, this);
    mass_sub = nh.subscribe("/mass_matrix", 1000, &dghc_controller::mass_callback, this);
    Vir_torque = nh.advertise<geometry_msgs::Wrench>("/vir_torque", 1000);
    alpha_pub = nh.advertise<geometry_msgs::Wrench>("/alphas", 1000);
    externel_wrench_sub = nh.subscribe("/hrii/ati_wrench", 1000, &dghc_controller::externel_wrench_callback, this);
    ch_pose_sub = nh.subscribe("/head_pose", 1000, &dghc_controller::ch_pose_callback, this);
    net_force_sub = nh.subscribe("/net_force", 1000, &dghc_controller::net_force_callback, this);
    ur_pose_sub = nh.subscribe("/ur5/unity/current_pose", 1000, &dghc_controller::ur_pose_callback, this);
    ur_pose_pub = nh.advertise<geometry_msgs::Pose>("/ur5/unity/desired_pose", 1000);
    //ur_current_pose_pub = nh.advertise<geometry_msgs::Pose>("/ur5/current_pose", 1000);

    ubq << 2.71344, 1.70281, 2.89407, -0.164777, 2.79709, 4.10882, 0.988027;
    lbq << -2.73363, -1.77708, -2.88609, -3.03556, -2.79978, 0.575631, -2.0; //-2.6895(joint7)
    R_distance << 0.2575, 0, 0.52;
    eRh<< 0.7071055,0.7071081,0,
          -0.7071081,0.7071055,0,
          0         ,0        ,1;

    transitionTime = 0;
    previousTime = ros::Time::now();
    modeInput = false;
    priorityInput.emplace_back(std::make_tuple(3, 0, 0));
    priorityInput.emplace_back(std::make_tuple(4, 0, 0));
    priorityInput.emplace_back(std::make_tuple(5, 0, 0));
    priorityInput.emplace_back(std::make_tuple(6, 0, 0));
    priorityInput.emplace_back(std::make_tuple(7, 0, 0));
    priorityInput.emplace_back(std::make_tuple(8, 0, 0));
    priorityInput.emplace_back(std::make_tuple(9, 0, 0));
    priorityInput.emplace_back(std::make_tuple(2, 0, 0));
    priorityInput.emplace_back(std::make_tuple(0, 0, 0));

    priorityChanged = true;
    alphaChangeDone = true;

    std::string interface_config_file_path = "/home/robot/master_harco_ws/src/hrii_moca/hrii_moca_interface/config/moca_no_gripper.yaml";

    // Create and initialize robot interface

    if (!nh.getParam(ros::this_node::getNamespace() + "/interface_type", interface_type_str))
    {
        std::cout << "Error : Interface Type" << std::endl;
    }

    // Get Arm Interface
    arm_interface_ = HRII::RAInterface::RoboticArmInterface::GenerateRoboticArmInterface(interface_config_file_path, interface_type_str);
    if (arm_interface_.get() == nullptr)
        std::cout << "Error : arm_interface_.get() == nullptr" << std::endl;

    if (!arm_interface_->init())
        std::cout << "Error : arm_interface_->init()" << std::endl;

    // Get Mobile Interface
    mobile_base_interface_ = HRII::MORInterface::MobileRobotInterface::GenerateMobileRobotInterface(interface_config_file_path);
    if (!mobile_base_interface_->init())
        std::cout << "Error : mobile_base_interface_->init()" << std::endl;

    // Update robot state
    if (!mobile_base_interface_->updateRobotState())
        std::cout << "Fail to Update Mobile Robot State" << std::endl;

    if (!arm_interface_->updateRobotState(mobile_base_interface_->getOdometry()))
        std::cout << "Fail to Update Manipulator State" << std::endl;

    sensor_msgs::JointState arm_joint_state = *(arm_interface_->getJointsStates());
    init_q = Eigen::Map<Eigen::VectorXd>(arm_joint_state.position.data(), arm_interface_->getJointsNum());

    chRb << 0, -1, 0,
        1, 0, 0,
        0, 0, 1;

    chRb_e.block(0, 0, 3, 3) = chRb;
    chRb_e.block(3, 3, 3, 3) = chRb;

    chTb.block(0, 3, 3, 1) << 0.3, -0.3, 0.5;
    chTb.block(0, 0, 3, 3) << chRb;
}
void dghc_controller::ch_pose_callback(const geometry_msgs::Pose &ch_pose)
{
    wTch.block(0, 3, 3, 1) << ch_pose.position.x, ch_pose.position.y, ch_pose.position.z;
    Eigen::Quaterniond quat;
    quat.x() = ch_pose.orientation.x;
    quat.y() = ch_pose.orientation.y;
    quat.z() = ch_pose.orientation.z;
    quat.w() = ch_pose.orientation.w;
    wTch.block(0, 0, 3, 3) << quat.toRotationMatrix();
    wRch = wTch.block(0, 0, 3, 3);
    flag1 = 1;
}

void dghc_controller::net_force_callback(const geometry_msgs::Vector3 &net_force)
{
    double scale = 1;
    Net_force_tmp[0] = scale * net_force.x; // ref : world frame
    Net_force_tmp[1] = scale * net_force.y;
    Net_force_tmp[2] = scale * net_force.z;
}

void dghc_controller::mass_callback(const geometry_msgs::Inertia &mass_matrix)
{
    mobile_mass << mass_matrix.ixx, mass_matrix.ixy, mass_matrix.ixz,
        0, mass_matrix.iyy, mass_matrix.iyz,
        0, 0, mass_matrix.izz;

    // std::cout<<"mobile_mass: "<<std::endl<<mobile_mass<<std::endl;
}

void dghc_controller::ur_pose_callback(const geometry_msgs::Pose &ur_pose)
{
    wTur.block(0, 3, 3, 1) << ur_pose.position.x, ur_pose.position.y, ur_pose.position.z;
    Eigen::Quaterniond quat;
    quat.x() = ur_pose.orientation.x;
    quat.y() = ur_pose.orientation.y;
    quat.z() = ur_pose.orientation.z;
    quat.w() = ur_pose.orientation.w;
    wTur.block(0, 0, 3, 3) << quat.toRotationMatrix();
    flag2 = 1;
}

void dghc_controller::externel_wrench_callback(const geometry_msgs::WrenchStamped &externel_wrench)
{
    wrenchExt_tmp(0) = externel_wrench.wrench.force.x;
    wrenchExt_tmp(1) = externel_wrench.wrench.force.y;
    wrenchExt_tmp(2) = externel_wrench.wrench.force.z;
    wrenchExt_tmp(3) = externel_wrench.wrench.torque.x;
    wrenchExt_tmp(4) = externel_wrench.wrench.torque.y;
    wrenchExt_tmp(5) = externel_wrench.wrench.torque.z;
    flag5 = 1;
}

void dghc_controller::priority_input_callback(const std_msgs::String::ConstPtr &priority_input_string)
{
    std::string input_str = priority_input_string->data;
    std::stringstream ss(input_str);
    std::unordered_set<char> seen_digits; // 중복된 숫자를 검사하기 위한 집합
    if (prevPriorityInputString == priority_input_string->data)
    {
        return;
    }
    priorityInput.clear();
    prevPriorityInputString = priority_input_string->data;

    int numTask = 10;
    double value;
    while (ss >> value)
    {
        if (value < 0 || value >= numTask)
        {
            std::cout << "Priority input value must be between 0 and " << numTask - 1 << std::endl;
            return;
        }

        if (seen_digits.find(value) != seen_digits.end())
        {
            std::cout << "Duplicate digit(" << value << ") found in input_str." << std::endl;
            return;
        }

        seen_digits.insert(value);
    }

    // mode_pub에서 받은 priority를 priorityInput 벡터에서 순서대로 저장
    // alpha값은 0으로 설정하여, strict하게 시작
    std::stringstream ss1(input_str);
    while (ss1 >> value)
    {
        priorityInput.emplace_back(std::make_tuple(value, 0, 0));
    }
}

void dghc_controller::mode_input_callback(const std_msgs::Bool::ConstPtr &mode_input)
{
    if (mode_input->data == modeInput)
    {
    }
    else if (modeInput == 0 && mode_input->data == 1)
    {

        chTh_init = chTb * bTh;
        wTh_init = wTch * chTh_init;
        h_initTch = chTh_init.inverse();
        wTur_init = wTur;
        std::cout << "mode 0 -> mode 1" << std::endl;
    }
    else if (modeInput == 1 && mode_input->data == 0)
    {

        wTur_past = wTur;
        sensor_msgs::JointState arm_joint_state = *(arm_interface_->getJointsStates());
        init_q = Eigen::Map<Eigen::VectorXd>(arm_joint_state.position.data(), arm_interface_->getJointsNum());

        std::cout << "mode 1 -> mode 0" << std::endl;
    }

    modeInput = mode_input->data;
    // std::cout<<"modeInput: "<<modeInput<<std::endl;
    flag4 = 1;
}

void dghc_controller::getModel()
{

    // Update robot state
    if (!mobile_base_interface_->updateRobotState())
        std::cout << "Fail to Update Mobile Robot State" << std::endl;

    if (!arm_interface_->updateRobotState(mobile_base_interface_->getOdometry()))
        std::cout << "Fail to Update Manipulatro State" << std::endl;

    // Get arm joint state
    sensor_msgs::JointState arm_joint_state = *(arm_interface_->getJointsStates());

    q = Eigen::Map<Eigen::VectorXd>(arm_joint_state.position.data(), arm_interface_->getJointsNum());
    q_dot = Eigen::Map<Eigen::VectorXd>(arm_joint_state.velocity.data(), arm_interface_->getJointsNum());
    // std::cout<<"q_dot: "<<q_dot<<std::endl;

    nav_msgs::Odometry mobile_base_odom = mobile_base_interface_->getOdometry();

    mobile_quat.w() = mobile_base_odom.pose.pose.orientation.w;
    mobile_quat.x() = mobile_base_odom.pose.pose.orientation.x;
    mobile_quat.y() = mobile_base_odom.pose.pose.orientation.y;
    mobile_quat.z() = mobile_base_odom.pose.pose.orientation.z;

    R_m = mobile_quat.normalized().toRotationMatrix();
    R_m_e.block(0, 0, 3, 3) = R_m;
    R_m_e.block(3, 3, 3, 3) = R_m;

    euler_m = R_m.eulerAngles(2, 1, 0); // zyx

    mobile_pose(0) = mobile_base_odom.pose.pose.position.x;
    mobile_pose(1) = mobile_base_odom.pose.pose.position.y;
    mobile_pose(2) = mobile_base_odom.pose.pose.position.z;
    // std::cout<<"mobile_pose"<<mobile_pose.transpose()<<std::endl;
    mobile_pose2d(0) = mobile_pose(0);
    mobile_pose2d(1) = mobile_pose(1);

    mobile_pose3d(0) = mobile_pose(0);
    mobile_pose3d(1) = mobile_pose(1);
    mobile_pose3d(2) = 0; // using quaternion

    mobile_vel(0) = mobile_base_odom.twist.twist.linear.x;
    mobile_vel(1) = mobile_base_odom.twist.twist.linear.y;
    mobile_vel(2) = mobile_base_odom.twist.twist.linear.z;
    mobile_vel2d(0) = mobile_vel(0);
    mobile_vel2d(1) = mobile_vel(1);

    mobile_vel3d(0) = mobile_vel(0);
    mobile_vel3d(1) = mobile_vel(1);
    mobile_vel3d(2) = mobile_base_odom.twist.twist.angular.z;

    mobile_twist << mobile_vel, mobile_base_odom.twist.twist.angular.x, mobile_base_odom.twist.twist.angular.y, mobile_base_odom.twist.twist.angular.z;

    M_j_ = arm_interface_->getInertiaMatrix().block(6, 6, arm_interface_->getJointsNum(), arm_interface_->getJointsNum());

    wholeM << mobile_mass, Z37,
        Z37.transpose(), M_j_;

    robot_ns = ros::this_node::getNamespace();
    if (!robot_ns.empty())
    {
        // Erase the first character (element) from the string
        robot_ns.erase(0, 1);
    }

    std::string frame_name[7];
    frame_name[6] = robot_ns + "_franka_EE";
    frame_name[5] = robot_ns + "_franka_link6";
    frame_name[4] = robot_ns + "_franka_link5";
    frame_name[3] = robot_ns + "_franka_link4";
    frame_name[2] = robot_ns + "_franka_link3";
    frame_name[1] = robot_ns + "_franka_link2";
    frame_name[0] = robot_ns + "_franka_link1";

    for (int i = 0; i < 7; i++)
    {
        geometry_msgs::Pose arm_pose = arm_interface_->getFramePose(frame_name[i], HRII::RAInterface::ModelInterfaceBase::ReferenceFrame::LOCAL_WORLD_ALIGNED);

        arm_tmp_position(0) = arm_pose.position.x;
        arm_tmp_position(1) = arm_pose.position.y;
        arm_tmp_position(2) = arm_pose.position.z;

        position = arm_tmp_position;

        if (i == 0)
        {
            j1 = arm_interface_->getJacobian(frame_name[i]).block(0, 6, 6, arm_interface_->getJointsNum());
            jt1 = j1.block(0, 0, 3, 7);
            position1 = position;
            vel1 = jt1 * q_dot;
            twist1 = j1 * q_dot + mobile_twist;
        }

        else if (i == 1)
        {
            j2 = arm_interface_->getJacobian(frame_name[i]).block(0, 6, 6, arm_interface_->getJointsNum());
            jt2 = j2.block(0, 0, 3, 7);
            position2 = position;
            vel2 = jt2 * q_dot;
            twist2 = j2 * q_dot + mobile_twist;
        }

        else if (i == 2)
        {
            j3 = arm_interface_->getJacobian(frame_name[i]).block(0, 6, 6, arm_interface_->getJointsNum());
            jt3 = j3.block(0, 0, 3, 7);
            position3 = position;
            vel3 = jt3 * q_dot;
            twist3 = j3 * q_dot + mobile_twist;
        }

        else if (i == 3)
        {
            j4 = arm_interface_->getJacobian(frame_name[i]).block(0, 6, 6, arm_interface_->getJointsNum());
            jt4 = j4.block(0, 0, 3, 7);
            position4 = position;
            vel4 = jt4 * q_dot;
            twist4 = j4 * q_dot + mobile_twist;
        }

        else if (i == 4)
        {
            j5 = arm_interface_->getJacobian(frame_name[i]).block(0, 6, 6, arm_interface_->getJointsNum());
            jt5 = j5.block(0, 0, 3, 7);
            position5 = position;
            vel5 = jt5 * q_dot;
            twist5 = j5 * q_dot + mobile_twist;
        }

        else if (i == 5)
        {
            j6 = arm_interface_->getJacobian(frame_name[i]).block(0, 6, 6, arm_interface_->getJointsNum());
            jt6 = j6.block(0, 0, 3, 7);
            position6 = position;
            vel6 = jt6 * q_dot;
            twist6 = j6 * q_dot + mobile_twist;
        }

        else if (i == 6)
        {
            j7 = arm_interface_->getJacobian(frame_name[i]).block(0, 6, 6, arm_interface_->getJointsNum());
            jt7 = j7.block(0, 0, 3, 7);

            position7 = position;
            position76d << position7, 0, 0, 0;
            position72d << position7(0),
                position7(1);
            vel7 = jt7 * q_dot;
            twist7 = j7 * q_dot + mobile_twist;
        }
    }

    Eigen::Affine3d b_T_ee = arm_interface_->getO_T_EE();
    wRe = b_T_ee.rotation();
    bRe = wRe;
    chRe = chRb * bRe;

    bTh.block(0, 3, 3, 1) = position - R_distance;
    bTh.block(0, 0, 3, 3) = bRe;

    if (count == 0)
    {
        std::cout << "wTur_init: " << std::endl
                  << wTur << std::endl;

        if (modeInput == 1)
        {
            chTh_init = chTb * bTh;
            wTh_init = wTch * chTh_init;
            h_initTch = chTh_init.inverse();
            wTur_past = wTur;
            wTur_last = wTur;
            desired_wTur = wTur;
        }
        else
        {
            wTur_past = wTur;
            wTur_last = wTur;
            desired_wTur = wTur;
        }
    }
    else
    {

        if (modeInput == 1)
        {
            chTh = chTb * bTh;           // done
            h_initTh = h_initTch * chTh; // done

            ur_pastTh_init = wTur_past.inverse() * wTh_init;
            ur_pastRh_init = ur_pastTh_init.block(0, 0, 3, 3);

            wRh_init = wTh_init.block(0, 0, 3, 3);
            ur_pastPrel = ur_pastRh_init * h_initTh.block(0, 3, 3, 1);
            h_initRh = h_initTh.block(0, 0, 3, 3);

            Eigen::AngleAxisd angleAxis(h_initRh);
            axis_h_initRh = angleAxis.axis();
            double angle = angleAxis.angle();

            Eigen::Vector3d axis_ur_pastRrel;
            axis_ur_pastRrel = ur_pastRh_init * axis_h_initRh;

            axis_ur_pastRrel.normalize();

            // Create a rotation matrix from axis-angle
            Eigen::AngleAxisd angleAxis2(angle, axis_ur_pastRrel);
            Eigen::Matrix3d ur_pastRrel = angleAxis2.toRotationMatrix();

            ur_pastTh_rel.block(0, 3, 3, 1) = ur_pastPrel;
            ur_pastTh_rel.block(0, 0, 3, 3) = ur_pastRrel;

            desired_wTur = wTur_past * ur_pastTh_rel;
            wTur_last = desired_wTur;
        }
        else
        {
            desired_wTur = wTur_last;
        }
    }

    // std::cout<<"wTur_past: "<<std::endl<<wTur_past<<std::endl;
    // std::cout<<"h_initTh: "<<std::endl<<h_initTh<<std::endl;
    // std::cout<<"ur_pastTh_rel: "<<std::endl<<ur_pastTh_rel<<std::endl;

    desired_position = desired_wTur.block(0, 3, 3, 1);
    desired_matrix = desired_wTur.block(0, 0, 3, 3);
    Eigen::Quaterniond desired_quat(desired_matrix);

    desired_ur_pose.position.x = desired_position(0);
    desired_ur_pose.position.y = desired_position(1);
    desired_ur_pose.position.z = desired_position(2);
    desired_ur_pose.orientation.x = desired_quat.x();
    desired_ur_pose.orientation.y = desired_quat.y();
    desired_ur_pose.orientation.z = desired_quat.z();
    desired_ur_pose.orientation.w = desired_quat.w();

    ur_pose_pub.publish(desired_ur_pose);
      
    

    if (count == 0)
        count = 1;

    jmt <<  1, 0, 0,
            0, 1, 0,
            0, 0, 0;

    jm << 1, 0, 0,
          0, 1, 0,
          0, 0, 0,
          0, 0, 0,
          0, 0, 0,
          0, 0, 1;

    jmt3d << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;

    jmt2d << 1, 0, 0,
             0, 1, 0;

    j0 << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
          0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    jjl1 << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
    jjl2 << 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
    jjl3 << 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
    jjl4 << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    jjl5 << 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
    jjl6 << 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
    jjl7 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    jimp << jm, j7; // joint7 frame
    jmimp << jmt3d, Z37;
    eRd = wRe.transpose() * wRd;
    Eigen::Quaterniond quat(eRd);

    // current_position = wTur.block(0, 3, 3, 1);
    // current_matrix = wTur.block(0, 0, 3, 3);
    // Eigen::Quaterniond current_quat(current_matrix);

    // current_ur_pose.position.x = current_position(0);
    // current_ur_pose.position.y = current_position(1);
    // current_ur_pose.position.z = current_position(2);
    // current_ur_pose.orientation.x = current_quat.x();
    // current_ur_pose.orientation.y = current_quat.y();
    // current_ur_pose.orientation.z = current_quat.z();
    // current_ur_pose.orientation.w = current_quat.w();

    // ur_current_pose_pub.publish(current_ur_pose);
}

void dghc_controller::getJacobian()
{
    std::vector<Eigen::MatrixXd> allJacobians;
    allJacobians.push_back(j0);
    allJacobians.push_back(jimp);
    allJacobians.push_back(jmimp);
    allJacobians.push_back(jjl1);
    allJacobians.push_back(jjl2);
    allJacobians.push_back(jjl3);
    allJacobians.push_back(jjl4);
    allJacobians.push_back(jjl5);
    allJacobians.push_back(jjl6);
    allJacobians.push_back(jjl7);

    setJacobianMatrices(allJacobians);
    //    std::cout<<"alljacobians:"<<std::endl<<allJacobians[0]<<std::endl;
}

void dghc_controller::getWrench()

{
    
    
    bRe_e.block(0, 0, 3, 3) = bRe*eRh;
    bRe_e.block(3, 3, 3, 3) = bRe*eRh;

    wrenchExt = bRe_e * wrenchExt_tmp; // hand guidacne

    if (modeInput == 1)
    {
        Net_force = chRb.inverse() * wRch.inverse() * Net_force_tmp;
        // std::cout<<"net_force: "<<std::endl<<Net_force_tmp.transpose()<<std::endl;
    }
    else
    {
        Net_force = Eigen::Vector3d::Zero(3);
    }

    wrenchHaptic[0] = wrenchExt[0] + Net_force[0];
    wrenchHaptic[1] = wrenchExt[1] + Net_force[1];
    wrenchHaptic[2] = wrenchExt[2] + Net_force[2];
    wrenchHaptic[3] = wrenchExt[3];
    wrenchHaptic[4] = wrenchExt[4];
    wrenchHaptic[5] = wrenchExt[5];
    //std::cout<<"wrenchHaptic: "<<std::endl<<wrenchHaptic.transpose()<<std::endl;
    k_m << 200, 0, 0,
        0, 200, 0,
        0, 0, 150;

    b_m << 20, 0, 0,
        0, 20, 0,
        0, 0, 5;

    R_d = Eigen::Matrix3d::Identity(3, 3);

    Eigen::Quaterniond quat0(R_m.transpose() * R_d);
    quatV << quat0.x(), quat0.y(), quat0.z();
    d_position_m << 0, 0, R_m.block(2, 0, 1, 3) * quatV;

    wrenchmImp = k_m * (d_position_m - mobile_pose3d) - b_m * mobile_vel3d;

    k_j << 18, 0, 0, 0, 0, 0, 0,
        0, 15, 0, 0, 0, 0, 0,
        0, 0, 15, 0, 0, 0, 0,
        0, 0, 0, 20, 0, 0, 0,
        0, 0, 0, 0, 10, 0, 0,
        0, 0, 0, 0, 0, 10, 0,
        0, 0, 0, 0, 0, 0, 5;

    b_j << 9, 0, 0, 0, 0, 0, 0,
        0, 9, 0, 0, 0, 0, 0,
        0, 0, 6, 0, 0, 0, 0,
        0, 0, 0, 6, 0, 0, 0,
        0, 0, 0, 0, 6, 0, 0,
        0, 0, 0, 0, 0, 6, 0,
        0, 0, 0, 0, 0, 0, 2.5;

    wrench0 = k_j * (init_q - q) - b_j * q_dot;

    // jl_alpha = 0.174533;//10deg
    jl_alpha << 0.26, 0.26, 0.26, 0.26, 0.26, 0.26, 0.26;
    jl_k << 50, 50, 50, 50, 50, 10, 5;
    jl_b << 10, 10, 10, 10, 10, 10, 5;

    for (int j = 0; j < 7; j++)
    {

        if (q(j) < lbq(j) + jl_alpha(j))
        {
            wrenchjl(j) = jl_k(j) * (lbq(j) + jl_alpha(j) - q(j)) - jl_b(j) * q_dot(j);
        }

        else if (q(j) > ubq(j) - jl_alpha(j))
        {
            wrenchjl(j) = jl_k(j) * (ubq(j) - jl_alpha(j) - q(j)) - jl_b(j) * q_dot(j);
        }

        else
        {
            wrenchjl(j) = 0;
        }
    }
}

void dghc_controller::setInertia()
{
    Eigen::MatrixXd I10 = Eigen::MatrixXd::Identity(10, 10);
    Eigen::MatrixXd Weighted_wholeM = wholeM;
    Weighted_wholeM.block(0, 0, 3, 3) = 1000 * mobile_mass;

    setInertiaMatrix(wholeM.inverse());
}

void dghc_controller::setPriority()
{
    // 현재 시간 업데이트
    ros::Time currentTime = ros::Time::now();
    // 시간 간격 계산
    double deltaT = (currentTime - previousTime).toSec();
    previousTime = currentTime;

    // priority가 변했는지 확인
    priorityFilter();

    // priority가 변하지 않았다면, 함수종료
    if (!priorityChanged)
    {
        // std::cout << "Priority not changed" <<std::endl;
        transitionTime = 0;
        return;
    };

    desiredAlphas = getDesiredAlphas(filteredPriority);

    // 현재의 alpha값들을 통해, priorityVector 생성
    std::vector<double> prioritiesVector;
    for (int row = 0; row < numberOfTasks; row++)
    {
        for (int col = row; col < numberOfTasks; col++)
        {
            prioritiesVector.emplace_back(getAlphaIJ(row, col));
        }
    }
    // std::cout << "a00 :\t" << prioritiesVector[0] << std::endl;
    // std::cout << "a11 :\t" << prioritiesVector[11] << std::endl;
    // std::cout << "a22 :\t" << prioritiesVector[22] << std::endl;
    // std::cout << "a01 :\t" << prioritiesVector[1] << std::endl;
    // std::cout << "a02 :\t" << prioritiesVector[2] << std::endl;
    // std::cout << "a12 :\t" << prioritiesVector[12] << std::endl;
    a00 = prioritiesVector[0];  // a00
    a11 = prioritiesVector[10]; // a11
    a22 = prioritiesVector[19]; // a22
    a01 = prioritiesVector[1];  // a01
    a02 = prioritiesVector[2];  // a02
    a12 = prioritiesVector[11]; // a12

    const double transitionDuration = 0.5;
    changeAlphas(prioritiesVector, transitionTime, deltaT, transitionDuration);

    transitionTime += deltaT;

    int counter = 0;
    for (unsigned int i = 0; i < numberOfTasks; i++)
    {
        for (unsigned int j = i; j < numberOfTasks; j++)
        {
            if (prioritiesVector[counter] < 0)
            {
                std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector[counter] << " < 0" << std::endl;
                return;
            }
            if (prioritiesVector[counter] > 1)
            {
                std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector[counter] << " > 1" << std::endl;
                return;
            }
            setAlphaIJ(i, j, prioritiesVector[counter]);
            counter++;
        }
    }
    assert(counter == prioritiesVector.size());
}

std::vector<double> dghc_controller::getDesiredAlphas(priorityTuples tupleVector)
{

    std::vector<std::vector<double>> matrix(numberOfTasks, std::vector<double>(numberOfTasks, -1));

    for (int i = 0; i < tupleVector.size(); i++)
    {
        int taskNumber = std::get<0>(tupleVector[i]);
        // diagonal term update
        matrix[taskNumber][taskNumber] = std::get<1>(tupleVector[i]);
        // relative term update
        if (tupleVector[i] != tupleVector.back())
        {
            int nextTaskNumber = std::get<0>(tupleVector[i + 1]);

            if (taskNumber < nextTaskNumber)
            {
                matrix[taskNumber][nextTaskNumber] = std::get<2>(tupleVector[i]);
            }

            else
            {
                matrix[nextTaskNumber][taskNumber] = 1 - std::get<2>(tupleVector[i]);
            }
        }

        // taskNumber의 행을 0으로 업데이트 (자기 자신의 alpha값 0, 다른 task들보다 높은 우선순위)
        for (int col = taskNumber + 1; col < numberOfTasks; col++)
        {
            // 이전에 값이 변경된 적이 있다면, 다른 task가 이미 선점한 값
            if (matrix[taskNumber][col] == -1)
            {
                matrix[taskNumber][col] = 0.0;
            }
        }
        // taskNumber의 열을 1로 업데이트 (다른 task들보다 높은 우선순위)
        for (int row = 0; row < taskNumber; row++)
        {
            if (matrix[row][taskNumber] == -1)
            {
                matrix[row][taskNumber] = 1.0;
            }
        }
    }

    // there is no taskNumber in tupleVector
    for (int i = 0; i < numberOfTasks; ++i)
    {

        if (matrix[i][i] == -1)
        {
            matrix[i][i] = 1.0;
        }

        for (int j = i + 1; j < numberOfTasks; j++)
        {
            if (matrix[i][j] == -1)
            {
                matrix[i][j] = 1.0;
            }
        }

        for (int j = i + 1; j < numberOfTasks; j++)
        {
            if (matrix[j][i] == -1)
            {
                matrix[j][i] = 0.0;
            }
        }
    }

    // 대칭된 위치의 아랫삼각행렬 값 업데이트
    for (int row = 0; row < numberOfTasks; row++)
    {
        for (int col = row + 1; col < numberOfTasks; col++)
        {
            matrix[col][row] = 1 - matrix[row][col];
        }
    }

    std::vector<double> desiredPriority;
    // matrix에서 vector추출
    for (int i = 0; i < numberOfTasks; ++i)
    {
        for (int j = i; j < numberOfTasks; ++j)
        {
            desiredPriority.emplace_back(matrix[i][j]);
        }
    }

    return desiredPriority;
}

// priorityInput에 저장된 값들을 변경하여, DesiredPriority를 재조정
void dghc_controller::setTrackingPriority(const int manipulatorTaskNum, const int mobileTaskNum, const int poseTaskNum)
{

    filteredPriority = priorityInput;
}

void dghc_controller::setObstaclePrirority(const std::vector<int> obstacleTaskNums)
{
    // d_ref보다 d_min이 크면, Task에서 Obstacle Avoidance 삭제
    if (f02d.norm() == 0)
    {
        for (auto it = filteredPriority.begin(); it != filteredPriority.end(); ++it)
        {
            if (std::get<0>(*it) == obstacleTaskNums[0])
            {
                it = filteredPriority.erase(it);
                break;
            }
        }
    }
    if (f1.norm() == 0)
    {
        for (auto it = filteredPriority.begin(); it != filteredPriority.end(); ++it)
        {
            if (std::get<0>(*it) == obstacleTaskNums[1])
            {
                it = filteredPriority.erase(it);
                break;
            }
        }
    }
    if (f3.norm() == 0)
    {
        for (auto it = filteredPriority.begin(); it != filteredPriority.end(); ++it)
        {
            if (std::get<0>(*it) == obstacleTaskNums[2])
            {
                it = filteredPriority.erase(it);
                break;
            }
        }
    }
    if (f5.norm() == 0)
    {
        for (auto it = filteredPriority.begin(); it != filteredPriority.end(); ++it)
        {
            if (std::get<0>(*it) == obstacleTaskNums[3])
            {
                it = filteredPriority.erase(it);
                break;
            }
        }
    }
}

void dghc_controller::setJointLimitPriority(const std::vector<int> jointLimitTaskNums)
{

    for (int i = 0; i < 7; i++)
    {
        if (lbq(i) + jl_alpha(i) < q(i) && ubq(i) - jl_alpha(i) > q(i))
        {
            for (auto it = filteredPriority.begin(); it != filteredPriority.end(); ++it)
            {
                if (std::get<0>(*it) == jointLimitTaskNums[i])
                {
                    it = filteredPriority.erase(it);
                    break;
                }
            }
        }
    }
}

void dghc_controller::priorityFilter()
{

    // filteredPriority = priorityInput;

    // priorityInput(사용자 지정) 값을 바탕으로, 환경 고려하여 filteredPriority 생성
    const int mani = 1;
    const int pose = 0;
    const int mobile = 2;
    const std::vector<int> jointLimitTaskNums = {3, 4, 5, 6, 7, 8, 9};

    // const std::vector<int> obstacle = {4,5,6,7}; // 0, 1, 3, 5 순서대로

    setTrackingPriority(mani, mobile, pose);
    // setObstaclePrirority(obstacle);
    setJointLimitPriority(jointLimitTaskNums);
    // std::cout << "prevFilteredPriority : ";
    // for(auto& task : prevFilteredPriority){
    //     std::cout << std::get<0>(task) << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "FilteredPriority : ";
    // for(auto& task : filteredPriority){
    //     std::cout << std::get<0>(task) << " ";
    // }
    // std::cout << std::endl;

    if (prevFilteredPriority != filteredPriority)
    {
        if (!filteredPriority.empty())
        {
            std::cout << "filtered priority : ";
            auto lastTask = filteredPriority.end() - 1;
            std::for_each(filteredPriority.begin(), lastTask, [](const auto &task)
                          { std::cout << "[" << std::get<0>(task) << "(" << std::get<1>(task) << ")] --" << std::get<2>(task) << "-->> "; });
            // Special handling for the last task
            std::cout << "[" << std::get<0>(*lastTask) << "(" << std::get<1>(*lastTask) << ")]" << std::endl;
        }
        else
        {
            // std::cout << "filteredPriority is empty (no task)" << std::endl;
        }

        priorityChanged = true;
        transitionTime = 0;
        prevFilteredPriority = filteredPriority;
    }
}

void dghc_controller::changeAlphas(std::vector<double> &currentValues, double t, double dt, double duration)
{

    // 변경 시작할 때를 기준으로, scale값들 설정
    if (alphaChangeDone)
    {
        for (int i = 0; i < desiredAlphas.size(); i++)
        {

            // increase alpha
            if (desiredAlphas[i] > currentValues[i])
            {
                scaleValues[i] = desiredAlphas[i] - currentValues[i];
            }

            // decrease alpha
            else if (desiredAlphas[i] < currentValues[i])
            {
                scaleValues[i] = currentValues[i] - desiredAlphas[i];
            }
            else
            {
            }
        }
        alphaChangeDone = false;
    }
    if (t <= duration)
    {
        for (int i = 0; i < desiredAlphas.size(); i++)
        {

            // increase alpha
            if (desiredAlphas[i] > currentValues[i])
            {

                // currentValues[i] = prevDesiredAlphas[i] +(0.5*(M_PI/duration)*sin((M_PI/duration)*t))*dt*scale;
                currentValues[i] += (0.5 * (M_PI / duration) * sin((M_PI / duration) * t)) * dt * scaleValues[i];

                if (currentValues[i] > desiredAlphas[i])
                {
                    currentValues[i] = desiredAlphas[i];
                }
            }

            // decrease alpha
            else if (desiredAlphas[i] < currentValues[i])
            {

                // currentValues[i] = prevDesiredAlphas[i] - (0.5*(M_PI/duration)*sin((M_PI/duration)*t))*dt*scale;
                currentValues[i] -= (0.5 * (M_PI / duration) * sin((M_PI / duration) * t)) * dt * scaleValues[i];

                if (currentValues[i] < desiredAlphas[i])
                {
                    currentValues[i] = desiredAlphas[i];
                }
            }
            else
            {
            }
        }
    }

    const double epsilon = 0.001;
    // 모든 alpha값에 대한 변경이 완료됐는지 확인
    if (alphasSetDone(currentValues, desiredAlphas, epsilon))
    {
        // printf("\n************** change is done !! **************\n");

        currentValues = desiredAlphas;
        transitionTime = 0;
        priorityChanged = false;
        alphaChangeDone = true;
    }
    // std::cout << "current Alphas" << std::endl;
    // int index = 0;
    // for (int row = 0; row < numberOfTasks; row++) {
    //     for (int i = 0; i < row; i++) {
    //         std::cout << std::setw(4) << " ";
    //     }
    //     for (int col = row; col < numberOfTasks; col++) {
    //         std::cout << std::setw(4) << std::fixed << std::setprecision(1)
    //                   << currentValues[index++];
    //     }
    //     std::cout << std::endl;
    // }
}

void dghc_controller::getProjectionM()
{
    int numTasks = getNumTasks();
    int DOFsize = getDOFsize();
    // std::cout<<"numTasks: "<<numTasks<<std::endl;
    // std::cout<<"DOFsize: "<<DOFsize<<std::endl;
    allProjections.clear();
    for (unsigned int i = 0; i < numTasks; i++)
    {
        allProjections.push_back(Eigen::MatrixXd::Zero(DOFsize, DOFsize));
    }

    Eigen::VectorXd ranks;
    ranks = Eigen::VectorXd::Zero(numTasks);
    //  std::cout<<"ranks: " <<std::endl<< ranks.transpose()<<std::endl;

    bool ok = getAllGeneralizedProjectors(allProjections, ranks);
}

void dghc_controller::getProjectedToq()
{

    toqT = allProjections[0] * j0.transpose() * wrench0 + allProjections[1] * jimp.transpose() * wrenchHaptic + allProjections[2] * jmimp.transpose() * wrenchmImp + allProjections[3] * jjl1.transpose() * wrenchjl[0] + allProjections[4] * jjl2.transpose() * wrenchjl[1] + allProjections[5] * jjl3.transpose() * wrenchjl[2] + allProjections[6] * jjl4.transpose() * wrenchjl[3] + allProjections[7] * jjl5.transpose() * wrenchjl[4] + allProjections[8] * jjl6.transpose() * wrenchjl[5] + allProjections[9] * jjl7.transpose() * wrenchjl[6];

    // toqT =  allProjections[0]*jrimp.transpose()*wrenchImp;
    //         +allProjections[1]*jmimp.transpose()*wrenchmImp;

    if (isnan(toqT(0)))
        toqT = Eigen::VectorXd::Zero(10, 1);
    //  std::cout<<"porjected toq jl7: " <<std::endl<<allProjections[9]*jjl7.transpose()*wrenchjl[6]<<std::endl;
    //  std::cout<<"porjected toq jl6: " <<std::endl<<allProjections[8]*jjl6.transpose()*wrenchjl[5]<<std::endl;
    //  std::cout<<"porjected toq jl5: " <<std::endl<<allProjections[7]*jjl5.transpose()*wrenchjl[4]<<std::endl;
    //  std::cout<<"porjected toq jl4: " <<std::endl<<allProjections[6]*jjl4.transpose()*wrenchjl[3]<<std::endl;
    //  std::cout<<"porjected toq jl3: " <<std::endl<<allProjections[5]*jjl3.transpose()*wrenchjl[2]<<std::endl;
    //  std::cout<<"porjected toq jl2: " <<std::endl<<allProjections[4]*jjl2.transpose()*wrenchjl[1]<<std::endl;
    //  std::cout<<"porjected toq jl1: " <<std::endl<<allProjections[3]*jjl1.transpose()*wrenchjl[0]<<std::endl;
    // std::cout<<"projected toq j0: " <<std::endl<<allProjections[0]*j0.transpose()*wrench06<<std::endl;
    // std::cout<<"allprojection1: "<<std::endl<<allProjections[1]<<std::endl;
    // std::cout<<"jrimpT: "<<std::endl<<jrimpT<<std::endl;
    // std::cout<<"wrenchImp: "<<std::endl<<wrenchImp.block(0,0,5,1).transpose()<<std::endl;
    // std::cout<<"projected toq jrimpT: " <<std::endl<<allProjections[1]*jrimpT.transpose()*wrenchImp<<std::endl;
    // std::cout<<"toq jrimpT: " <<std::endl<<jrimpT.transpose()*wrenchImp.block(0,0,5,1)<<std::endl;
    // std::cout<<"projected toq jmimp: " <<std::endl<<allProjections[2]*jmimp.transpose()*wrenchmImp<<std::endl;
}

bool dghc_controller::alphasSetDone(const std::vector<double> &vec1, const std::vector<double> &vec2, double epsilon)
{
    if (vec1.size() != vec2.size())
    {
        return false; // 벡터의 크기가 다르면 같지 않음
    }

    for (size_t i = 0; i < vec1.size(); ++i)
    {
        if (std::abs(vec1[i] - vec2[i]) > epsilon)
        {
            return false; // 원소가 epsilon보다 큰 차이가 있으면 같지 않음
        }
    }
    return true; // 모든 원소가 epsilon 이내에 있는 경우 같음
}

void dghc_controller::energy_filter()
{
}

void dghc_controller::checkDeadzone(double &value, int wrenchType)
{
    double minWrench, maxWrench;

    switch (wrenchType)
    {
    case 1:
        minWrench = MIN_LINEAR_WRENCH;
        maxWrench = MAX_LINEAR_WRENCH;
        break;
    case 2:
        minWrench = MIN_ANGULAR_WRENCH;
        maxWrench = MAX_ANGULAR_WRENCH;
        break;
    }

    if (std::fabs(value) < minWrench)
        value = 0;
    //     else if (value > maxWrench)
    //         value = maxWrench;
    //     else if (value < -maxWrench)
    //         value = -maxWrench;
}

Eigen::Matrix<double, 7, 1> dghc_controller::saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
                                                                const Eigen::Matrix<double, 7, 1> &tau_J_d)
{
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
}

void dghc_controller::test()
{
    //     Eigen::VectorXd prioritiesVector;
    //     int numTasks = getNumTasks();
    //     prioritiesVector = Eigen::VectorXd::Zero(0.5*(numTasks*numTasks+numTasks));

    //     prioritiesVector[0] = 0;
    //     prioritiesVector[1] = 0;
    //     prioritiesVector[2] = 0;

    //     int counter=0;
    //     for(unsigned int i=0; i<numTasks; i++){
    //       for(unsigned int j=i; j<numTasks; j++)
    //        {
    //          if(prioritiesVector(counter) < 0){
    //             std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector(counter) << " < 0" << std::endl;
    //             return;
    //            }
    //          if(prioritiesVector(counter) > 1){
    //             std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector(counter) << " > 1" << std::endl;
    //             return;
    //            }
    //         setAlphaIJ(i,j,prioritiesVector(counter));
    //         counter++;
    //        }
    //     }
    // assert(counter==prioritiesVector.size());
}
int dghc_controller::run()
{
    numberOfTasks = getNumTasks();
    scaleValues.assign(numberOfTasks * (numberOfTasks + 1) * 0.5, 1);
    // auto logger = XBot::MatLogger2::MakeLogger("/home/yushin/log/log");
    // logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    count = 0;

    ros::Time current_time, last_time;

    ros::Rate loop_rate(1000);
    std::cout << "waiting callbacks..." << std::endl;
    while (ros::ok())
    {
        ros::spinOnce();

        if (/*flag1 == 1 && flag2 == 1 && flag3 == 1 &&*/ flag4 == 1 && flag5 == 1)
        {
            current_time = ros::Time::now();
            dt = (current_time - last_time).toSec();

            getModel();
            if (count == 1)
            {
                std::cout << "start!" << std::endl;
                count = 2;
            }
            getWrench();

            getJacobian();

            //  test();
            setPriority();

            geometry_msgs::Wrench alphas_pub_data;

            alphas_pub_data.force.x = a00;
            alphas_pub_data.force.y = a11;
            alphas_pub_data.force.z = a22;
            alphas_pub_data.torque.x = a01;
            alphas_pub_data.torque.y = a02;
            alphas_pub_data.torque.z = a12;
            alpha_pub.publish(alphas_pub_data);

            geometry_msgs::Pose ur_current_pose;

            
            
            setInertia();

            getProjectionM();

            getProjectedToq();

            //  logger->add("a00",a00);
            //  logger->add("a01",a01);
            //  logger->add("a02",a02);
            //  logger->add("a03",a03);
            //  logger->add("a11",a11);
            //  logger->add("a12",a12);
            //  logger->add("a13",a13);
            //  logger->add("a22",a22);
            //  logger->add("a23",a23);
            //  logger->add("a33",a33);

            vir_force = R_m.transpose() * toqT.block(0, 0, 3, 1);
            //  std::cout <<"vir_force:"<<std::endl <<vir_force << std::endl;
            //  std::cout <<"toq: "<<std::endl <<toqT.block(3,0,7,1) << std::endl;

            toq = toqT.block(3, 0, 7, 1); //+ arm_interface_->getCoriolisCompensation();

            //   std::cout<<"toq: "<<toq.transpose()<<std::endl;

            arm_tau_J_d = arm_interface_->getDesiredJointCommands();
            toq << saturateTorqueRate(toq, arm_tau_J_d);

            checkDeadzone(vir_force(0), LINEAR);
            checkDeadzone(vir_force(1), LINEAR);
            checkDeadzone(vir_force(2), ANGULAR);

            // std::cout<<"toq_saturated: "<<toq.transpose()<<std::endl;
            geometry_msgs::Wrench vir_torque;

            vir_torque.force.x = vir_force(0);
            vir_torque.force.y = vir_force(1);
            vir_torque.torque.z = vir_force(2);
            Vir_torque.publish(vir_torque);

            // Limiting maximun torque values of the ARM
            for (size_t i = 3; i < 7; ++i)
            {
                if (toq(i) > 87)
                {
                    toq(i) = 87;
                }
            }
            for (size_t i = 7; i < 10; ++i)
            {
                if (toq(i) > 12)
                {
                    toq(i) = 12;
                }
            }
            //  std::cout<<"toq_limited: "<<toq.transpose()<<std::endl;
            // Set joint commands and publish virtual forces

            arm_interface_->setJointCommands(toq);
        }
        loop_rate.sleep();
        last_time = current_time;
    }
    return 0;
}
#include "dghc_controller_RAL.hpp"
#include "GHCProjections.hpp"
#include "matlogger2/matlogger2.h"


dghc_controller::dghc_controller(){
    
    d_pose_sub = nh.subscribe("/desired_pose", 1000, &dghc_controller::desired_pose_callback,this);
    c_pose_pub = nh.advertise<geometry_msgs::Pose>("/current_pose", 1000);
    hand_pose_sub = nh.subscribe("/natnet/hand_pose", 1000, &dghc_controller::hand_pose_callback,this);
    obstacle_states = nh.subscribe("/human_factor/human_joint_positions", 1000, &dghc_controller::obstacle_states_callback,this);
    riskfactor_sub = nh.subscribe("/human_factor/risk_factor", 500, &dghc_controller::riskfactor_callback,this);
    optitrack_sub = nh.subscribe("/natnet/robot_pose", 500, &dghc_controller::optitrack_callback,this);
    modelPosepublisher = nh.advertise<geometry_msgs::PoseArray>("/human_factor/robot_joint_positions",500);
    modelVelpublisher = nh.advertise<geometry_msgs::PoseArray>("/human_factor/robot_joint_velocities",500);
    d_dot_sub = nh.subscribe("/human_factor/rel_vel",500 ,&dghc_controller::d_dot_callback,this);
    

    distance_flag_pub = nh.advertise<std_msgs::Bool>("/distance_flag",1000);
    force_flag_pub = nh.advertise<std_msgs::Bool>("/force_flag",1000);
    current_step_sub = nh.subscribe("/current_step", 1000,&dghc_controller::curruent_step_callback, this);
    reset_sub = nh.subscribe("/reset",1000, &dghc_controller::reset_callback, this);
    
    priority_input_sub = nh.subscribe("/priorityInput", 1000, &dghc_controller::priority_input_callback,this);
    mass_sub  = nh.subscribe("/mass_matrix", 1000, &dghc_controller::mass_callback,this);
    Vir_torque = nh.advertise<geometry_msgs::Wrench>("/vir_torque", 1000);
    alpha_pub = nh.advertise<geometry_msgs::Wrench>("/alphas", 1000);
    externel_wrench_sub = nh.subscribe("/hrii/ati_wrench", 1000, &dghc_controller::externel_wrench_callback, this);


    d_euler<<0,M_PI,M_PI;
    // lbq << -2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973;
    // ubq << 2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973;
    ubq << 2.71344,1.70281,2.89407,-0.164777,2.79709,4.10882,0.988027;
    lbq << -2.73363,-1.77708,-2.88609,-3.03556,-2.79978,0.575631,-2.0;  //-2.6895(joint7)

    eRft<< 0.7071055,0.7071081,0,
          -0.7071081,0.7071055,0,
          0         ,0        ,1;

    R_distance<< 0.2575,0,0.52;
    m_distance<< 0,0,0.26;
    
    transitionTime =0;
    previousTime=ros::Time::now();
    
    priorityInput.emplace_back(std::make_tuple(3,0,0));
    priorityInput.emplace_back(std::make_tuple(4,0,0));
    priorityInput.emplace_back(std::make_tuple(5,0,0));
    priorityInput.emplace_back(std::make_tuple(6,0,0));
    priorityInput.emplace_back(std::make_tuple(7,0,0));
    priorityInput.emplace_back(std::make_tuple(8,0,0));
    priorityInput.emplace_back(std::make_tuple(9,0,0));
    priorityInput.emplace_back(std::make_tuple(2,0,0));
    priorityInput.emplace_back(std::make_tuple(0,0,0));
    prevFilteredPriority = priorityInput;
    priorityChanged = true;
    alphaChangeDone = true;

    distance_flag.data = false;
    force_flag.data = false;
    currentStep.data = -1;
    
    std::string interface_config_file_path = "/home/robot/master_harco_ws/src/hrii_moca/hrii_moca_interface/config/moca_no_gripper.yaml";
    
    // Create and initialize robot interface
    
    if(!nh.getParam(ros::this_node::getNamespace()+"/interface_type", interface_type_str))
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
    if(!mobile_base_interface_->updateRobotState())
        std::cout << "Fail to Update Mobile Robot State" << std::endl;

    if(!arm_interface_->updateRobotState(mobile_base_interface_->getOdometry()))
        std::cout << "Fail to Update Manipulator State" << std::endl;

    sensor_msgs::JointState arm_joint_state = *(arm_interface_->getJointsStates());
    init_q = Eigen::Map<Eigen::VectorXd>(arm_joint_state.position.data(), arm_interface_->getJointsNum());
    q=init_q;
    desired_q = init_q;
    q_past = init_q;
}    
void dghc_controller::obstacle_states_callback(const human_factor::LinkStateArray::ConstPtr& obstacleStateMsg)
{
    //   std::vector<std::string> modelNames =  obstacleStateMsg->states ;
    int obstacleIndex = 0;
    int dPositionIndex = 0;
    obstacleStates.fill(Eigen::Vector3d::Zero(3));
    //   obstacleStates.s
    // pelvis, t12, head, right_forarm, right_hand, left_forearm, left_hand, right_lower_leg, right_foot, left_lower_leg, left_foot
    for(int i=0; i < 11;i++){
    obstacleStates[i](0) = obstacleStateMsg->states[i].pose.position.x;
    obstacleStates[i](1) = obstacleStateMsg->states[i].pose.position.y;
    obstacleStates[i](2) = obstacleStateMsg->states[i].pose.position.z;
    }
}
void dghc_controller::hand_pose_callback(const geometry_msgs::Pose::ConstPtr& hand_pose){
    hand_position(0) = hand_pose->position.x;
    hand_position(1) = hand_pose->position.y;
    hand_position(2) = hand_pose->position.z;
    
   
            
}
void dghc_controller::riskfactor_callback(const human_factor::RiskFactor::ConstPtr& riskfactor){
    if (riskfactor->factor.size() == 44) 
    {
        // Access the elements of the received array
        for (size_t i = 0; i < riskfactor->factor.size(); ++i) {
            riskFactorArray[i] = riskfactor->factor[i];
            //ROS_INFO("Received value[%zu]: %f", i, riskFactorArray[i]);
        }
    } 
    else
    {
        ROS_WARN("Received array size is not 44!");
    }
}
void dghc_controller::optitrack_callback(const geometry_msgs::Pose::ConstPtr& opti_pose){
    // mobile pose data from optitrack natnet
    optitrack_p(0) = opti_pose->position.x;
    optitrack_p(1) = opti_pose->position.y;
    optitrack_p(2) = opti_pose->position.z;
    // opotitrack_r = Eigen::Quaterniond(opti_pose->orientation.x, opti_pose->orientation.y, opti_pose->orientation.z, opti_pose->orientation.w).normalized().toRotationMatrix();
    optitrack_q.x() = opti_pose->orientation.x;
    optitrack_q.y() = opti_pose->orientation.y;
    optitrack_q.z() = opti_pose->orientation.z;
    optitrack_q.w() = opti_pose->orientation.w;
    optitrack_sign = true;
      // std::cout << "optitrack_p: " << optitrack_p << std::endl;
    //  std::cout << "optitrack_sign: " << optitrack_sign << std::endl;
}
void dghc_controller::d_dot_callback(const std_msgs::Float64MultiArray::ConstPtr& d_dot_msg){
    for(int i=0; i<44; i++){
        d_dot[i] = d_dot_msg->data[i];
    }
    
}
void dghc_controller::mass_callback(const geometry_msgs::Inertia& mass_matrix)
{
    mobile_mass<<mass_matrix.ixx,mass_matrix.ixy,mass_matrix.ixz,
                               0,mass_matrix.iyy,mass_matrix.iyz,
                               0,              0,mass_matrix.izz;

    
    // std::cout<<"mobile_mass: "<<std::endl<<mobile_mass<<std::endl;
    
}

void dghc_controller::externel_wrench_callback(const geometry_msgs::WrenchStamped &externel_wrench)
{
    wrenchExt_tmp(0) = externel_wrench.wrench.force.x;
    wrenchExt_tmp(1) = externel_wrench.wrench.force.y;
    wrenchExt_tmp(2) = externel_wrench.wrench.force.z;
    wrenchExt_tmp(3) = externel_wrench.wrench.torque.x;
    wrenchExt_tmp(4) = externel_wrench.wrench.torque.y;
    wrenchExt_tmp(5) = externel_wrench.wrench.torque.z;
 
}

void dghc_controller::reset_callback(const std_msgs::Bool &reset){
    // initialize all signal
    if(reset.data){
        distance_flag.data = false;
        force_flag.data = false;
    }
}

void dghc_controller::curruent_step_callback(const std_msgs::Int16 &step){
    std::cout<<"currentStep: "<<currentStep.data<<std::endl;
    std::cout<<"step: "<<step.data<<std::endl;

    if(currentStep.data != step.data)
    {
         
        desired_q = q;
        std::cout<<"desired_q in callback: "<<desired_q.transpose()<<std::endl;
        

    }
    currentStep.data = step.data;
}

void dghc_controller::desired_pose_callback(const geometry_msgs::Pose& desired_pose){
    d_position(0) = desired_pose.position.x;
    d_position(1) = desired_pose.position.y;
    d_position(2) = desired_pose.position.z;
    
   
    
    Eigen::Quaterniond d_quat;
    d_quat.x() = desired_pose.orientation.x;
    d_quat.y() = desired_pose.orientation.y;    
    d_quat.z() = desired_pose.orientation.z;  
    d_quat.w() = desired_pose.orientation.w;
    
    wRd  = d_quat.normalized().toRotationMatrix();
            
}


void dghc_controller::priority_input_callback(const std_msgs::String::ConstPtr& priority_input_string){
    std::string input_str = priority_input_string->data;
    std::stringstream ss(input_str);
    std::unordered_set<char> seen_digits; // 중복된 숫자를 검사하기 위한 집합
    if(prevPriorityInputString == priority_input_string->data){
        return;
    }
    priorityInput.clear();
    prevPriorityInputString= priority_input_string->data;
    
    int numTask = 15;
    double value;
    while(ss>> value){
        if(value < 0 || value >= numTask){
                std::cout << "Priority input value must be between 0 and " << numTask-1 << std::endl;
                return;
            }

        if (seen_digits.find(value) != seen_digits.end()) {
        std::cout << "Duplicate digit(" << value << ") found in input_str." << std::endl;
        return;
        }

        seen_digits.insert(value);
    }

    //mode_pub에서 받은 priority를 priorityInput 벡터에서 순서대로 저장
    //alpha값은 0으로 설정하여, strict하게 시작 
    std::stringstream ss1(input_str);
    while(ss1>> value){
        priorityInput.emplace_back(std::make_tuple(value, 0,0)); 
    }
}


void dghc_controller::getModel(){
    // Update robot state
    if(!mobile_base_interface_->updateRobotState())
        std::cout << "Fail to Update Mobile Robot State" << std::endl;

    nav_msgs::Odometry mobile_base_odom = mobile_base_interface_->getOdometry();
    
    mobile_pose_r(0) = mobile_base_odom.pose.pose.position.x;
    mobile_pose_r(1) = mobile_base_odom.pose.pose.position.y;
    mobile_pose_r(2) = mobile_base_odom.pose.pose.position.z;
   
    mobile_quat_r.w() = mobile_base_odom.pose.pose.orientation.w;
    mobile_quat_r.x() = mobile_base_odom.pose.pose.orientation.x;
    mobile_quat_r.y() = mobile_base_odom.pose.pose.orientation.y;
    mobile_quat_r.z() = mobile_base_odom.pose.pose.orientation.z;
   
    w_R_m = mobile_quat_r.normalized().toRotationMatrix();
    w_R_o = optitrack_q.normalized().toRotationMatrix();

    w_T_m_raw.block(0,0,3,3) = w_R_m;
    w_T_m_raw.block(0,3,3,1) = mobile_pose_r;

    if(!marker_flag && optitrack_sign){
        marker_flag = true;

        init_w_T_m.setIdentity();
        init_w_T_m.block(0,0,3,3) = w_R_m;
        init_w_T_m.block(0,3,3,1) = mobile_pose_r;

        init_w_T_o.setIdentity();
        init_w_T_o.block(0,0,3,3) = w_R_o;
        init_w_T_o.block(0,3,3,1) = optitrack_p;

        init_o_T_m = init_w_T_o.inverse()*init_w_T_m;
    }
    
    if(optitrack_sign){

        new_w_T_o.block(0,0,3,3) = w_R_o;
        new_w_T_o.block(0,3,3,1) = optitrack_p;

        real_mobile_Tf = new_w_T_o*init_o_T_m;

        mobile_pose = real_mobile_Tf.block(0,3,3,1);

        R_m = real_mobile_Tf.block(0,0,3,3);

        mobile_quat_filtered = R_m;

        optitrack_sign = false;
    }

    mobile_base_odom_filtered = mobile_base_odom;
    mobile_base_odom_filtered.pose.pose.position.x = mobile_pose(0);
    mobile_base_odom_filtered.pose.pose.position.y = mobile_pose(1);
    mobile_base_odom_filtered.pose.pose.position.z = mobile_pose(2);
    mobile_base_odom_filtered.pose.pose.orientation.w = mobile_quat_filtered.w();
    mobile_base_odom_filtered.pose.pose.orientation.x = mobile_quat_filtered.x();
    mobile_base_odom_filtered.pose.pose.orientation.y = mobile_quat_filtered.y();
    mobile_base_odom_filtered.pose.pose.orientation.z = mobile_quat_filtered.z();
    //mobile_base_interface_->getOdometry()
    std::cout<<"filtered_odom: "<<mobile_pose.transpose()<<std::endl;
    std::cout << "filtered rotation : " << mobile_quat_filtered.toRotationMatrix() << std::endl;
    if(!arm_interface_->updateRobotState(mobile_base_odom_filtered))
        std::cout << "Fail to Update Manipulatro State" << std::endl;
    // Get arm joint state
    sensor_msgs::JointState arm_joint_state = *(arm_interface_->getJointsStates());
    q = Eigen::Map<Eigen::VectorXd>(arm_joint_state.position.data(), arm_interface_->getJointsNum());
    q_dot = Eigen::Map<Eigen::VectorXd>(arm_joint_state.velocity.data(), arm_interface_->getJointsNum());

    R_m_e.block(0,0,3,3) = R_m;
    R_m_e.block(3,3,3,3) = R_m;  
    
    euler_m = R_m.eulerAngles(2,1,0); //zyx

    mobile_pose2d(0) = mobile_pose(0);
    mobile_pose2d(1) = mobile_pose(1); 

    mobile_pose3d(0) = mobile_pose(0);
    mobile_pose3d(1) = mobile_pose(1); 
    mobile_pose3d(2) = 0; //using quaternion 

    mobile_vel(0) = mobile_base_odom.twist.twist.linear.x;
    mobile_vel(1) = mobile_base_odom.twist.twist.linear.y;
    mobile_vel(2) = mobile_base_odom.twist.twist.linear.z;
    mobile_vel2d(0) = mobile_vel(0);
    mobile_vel2d(1) = mobile_vel(1);
    
    mobile_vel3d(0) = mobile_vel(0);
    mobile_vel3d(1) = mobile_vel(1);
    mobile_vel3d(2) = mobile_base_odom.twist.twist.angular.z;

    mobile_twist<<mobile_vel,mobile_base_odom.twist.twist.angular.x,mobile_base_odom.twist.twist.angular.y,mobile_base_odom.twist.twist.angular.z;
    
    M_j_ = arm_interface_->getInertiaMatrix().block(6, 6, arm_interface_->getJointsNum(), arm_interface_->getJointsNum());

    

    wholeM<<mobile_mass, Z37,
           Z37.transpose(),M_j_;
    
    robot_ns = ros::this_node::getNamespace();
    if (!robot_ns.empty()) {
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
    
    Eigen::Affine3d b_T_ee = arm_interface_->getO_T_EE();
  
    for(int i = 0;i<7;i++)
    {
      geometry_msgs::Pose arm_pose = arm_interface_->getFramePose(frame_name[i],HRII::RAInterface::ModelInterfaceBase::ReferenceFrame::LOCAL_WORLD_ALIGNED);
     
      arm_tmp_position(0) = arm_pose.position.x;
      arm_tmp_position(1) = arm_pose.position.y;
      arm_tmp_position(2) = arm_pose.position.z;

      position = arm_tmp_position;
      
      if(i==0){
        j1 = arm_interface_->getJacobian(frame_name[i]).block(0,6,6,arm_interface_->getJointsNum());
        jt1 =j1.block(0,0,3,7);
        position1 = position;
        vel1 =jt1*q_dot;
        twist1 = j1*q_dot + mobile_twist;
        
      }

      else if(i==1){
        j2 = arm_interface_->getJacobian(frame_name[i]).block(0,6,6,arm_interface_->getJointsNum());
        jt2 =j2.block(0,0,3,7);
        position2 = position;
        vel2 =jt2*q_dot;
        twist2 = j2*q_dot + mobile_twist;
        
      }

      else if(i==2){
        j3 = arm_interface_->getJacobian(frame_name[i]).block(0,6,6,arm_interface_->getJointsNum());
        jt3 =j3.block(0,0,3,7);
        position3 = position;
        vel3 =jt3*q_dot;
        twist3 = j3*q_dot + mobile_twist;
      }

      else if(i==3){
        j4 = arm_interface_->getJacobian(frame_name[i]).block(0,6,6,arm_interface_->getJointsNum());
        jt4 =j4.block(0,0,3,7);
        position4 = position;
        vel4 =jt4*q_dot;
        twist4 = j4*q_dot + mobile_twist;
      }

      else if(i==4){
        j5 = arm_interface_->getJacobian(frame_name[i]).block(0,6,6,arm_interface_->getJointsNum());
        jt5 =j5.block(0,0,3,7);
        position5 = position;
        vel5 =jt5*q_dot;
        twist5 = j5*q_dot + mobile_twist;
      }

      else if(i==5){
        j6 = arm_interface_->getJacobian(frame_name[i]).block(0,6,6,arm_interface_->getJointsNum());
        jt6 =j6.block(0,0,3,7);
        position6 = position;
        vel6 =jt6*q_dot;
        twist6 =j6*q_dot + mobile_twist;
      }

      else if(i==6){
        // j7 = arm_interface_->getJacobian().block(0,6,6,arm_interface_->getJointsNum());
        // std::cout<<"good jacobian: "<<j7<<std::endl;
        j7 = arm_interface_->getJacobian(frame_name[i]).block(0,6,6,arm_interface_->getJointsNum());
        jt7 = j7.block(0,0,3,7);
        
        
        
        position7 = position;
        position76d<<position7,0,0,0;
        position72d<<position7(0),
                     position7(1);
        vel7 =jt7*q_dot;
        twist7 = j7*q_dot + mobile_twist;
        
        wRe = b_T_ee.rotation();
        wRe_e.block(0,0,3,3) = wRe;
        wRe_e.block(3,3,3,3) = wRe;
        
        if(count == 0)
        {
            d_position = position7;
            d_position2d_tmp(0) = d_position(0);
            d_position2d_tmp(1) = d_position(1);
            wRd = wRe;
            count =1;
        }
        else
        {
            d_position2d_tmp(0) = position7(0);
            d_position2d_tmp(1) = position7(1);
        }
        
        
        euler_e = wRe.eulerAngles(2,1,0); // zyx
        Eigen::Quaterniond current(wRe);

        geometry_msgs::Pose current_pose;
        current_pose.position.x = position7(0);
        current_pose.position.y = position7(1);
        current_pose.position.z = position7(2);
        current_pose.orientation.x = current.x();
        current_pose.orientation.y = current.y();
        current_pose.orientation.z = current.z();
        current_pose.orientation.w = current.w();
        
        c_pose_pub.publish(current_pose);
        // std::cout<<"position7: "<<position7<<std::endl;
        
      }
    }

    jmt <<1,0,0,
          0,1,0,
          0,0,0;

    jm  <<1,0,0,
          0,1,0,
          0,0,0,
          0,0,0,
          0,0,0,
          0,0,1;

    jmt3d <<1,0,0,
            0,1,0,
            0,0,1;

    jmt2d <<1,0,0,
            0,1,0;

    
    
    j0 <<0,0,0,1,0,0,0,0,0,0,
         0,0,0,0,1,0,0,0,0,0,
         0,0,0,0,0,1,0,0,0,0,
         0,0,0,0,0,0,1,0,0,0,
         0,0,0,0,0,0,0,1,0,0,
         0,0,0,0,0,0,0,0,1,0,
         0,0,0,0,0,0,0,0,0,1;
    //std::cout<<"at getmodel j0: "<<std::endl<<j0<<std::endl;
    jjl1 <<0,0,0,1,0,0,0,0,0,0;
    jjl2 <<0,0,0,0,1,0,0,0,0,0;
    jjl3 <<0,0,0,0,0,1,0,0,0,0;
    jjl4 <<0,0,0,0,0,0,1,0,0,0;
    jjl5 <<0,0,0,0,0,0,0,1,0,0;
    jjl6 <<0,0,0,0,0,0,0,0,1,0;
    jjl7 <<0,0,0,0,0,0,0,0,0,1;

    
    

    jimp << jm , j7; //joint7 frame

    jrimpT  = jimp.block(0,0,5,10);

    jhand = jimp;

    jmimp <<jmt3d ,Z37;

    p2m = mobile_pose2d - d_position2d_tmp;
    u_p2m = p2m/(sqrt(p2m.transpose()*p2m));
    d_position2d = margin*u_p2m + d_position2d_tmp;  //mobile position

    
    //std::cout<<"wRd: "<<wRd<<std::endl;
    eRd = wRe.transpose()*wRd;
    Eigen::Quaterniond quat(eRd);
    
    d_pose<<d_position,quat.x(),quat.y(),quat.z();
    
  

    Eigen::VectorXd mobile2targetV =  R_m.transpose()*(position7 - mobile_pose);
    if(abs(mobile2targetV(0))>0.05 && abs(mobile2targetV(1)) > 0.05)
    {
        R_d =R_m *Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(atan2(mobile2targetV(1),mobile2targetV(0)) , Eigen::Vector3d::UnitZ());
    }
    else
    {
        R_d = R_m;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    std::array<Eigen::Vector3d, 4> robotStates = {mobile_pose + R_m*m_distance, position1, position4, position7};
    std::vector<Eigen::VectorXd> alldistance; //distance direction
    for(int human=0 ; human<11 ; human++){
        for(int robot=0 ; robot<4 ; robot++){
            Eigen::Vector3d relativeDistance = robotStates[robot] - obstacleStates[human];
            d[robot][human] = sqrt(relativeDistance.transpose()*relativeDistance);//[frame][obstacle number]
            alldistance.push_back(relativeDistance/d[robot][human]);// [[human1][robot1 ... robot5] ... [human11][robot1 ... robot5]]
            //std::cout<<"frame : "<<robot<<"\t\t human : "<<human<<"\t\t distance : "<<d[robot][human]<<std::endl;
        }
    }
    
  
//////////////////////////////////////////OBSTACLE AVOIDANCE ////////////////////////////////////////
    P2d<<1,0,0,
         0,1,0;
    dref[0] = 0.7; 
    dref[1] = 0.2;
    dref[2] = 0.2;
    dref[3] = 0.3;

    alpha[0] = 0.3;
    alpha[1] = 0.3;
    alpha[2] = 0.3;
    alpha[3] = 0.2;

    int k_avd = 0;
    int b_avd = 10;

    std::vector<Eigen::VectorXd> allf;
    // i : frame 
    for(int i = 0;i<4;i++){
        f_tmp <<0,0,0;
        // j : obstacle
        for(int j = 0; j<11;j++){
            
            if(d[i][j]< dref[i]+ alpha[i]){
                //std::cout<<"riskFactor index No. : "<< 4*i+j << std::endl;
                //std::cout<<"riskFactor size : "<< riskFactorArray.size() << std::endl;
                if(riskFactorArray[11*i+j] >= factor_lv2){ // add riskfactor condition
                    
                    if(i==0){k_avd = 500;}

                    else if(i==1){k_avd = 100;}

                    else if(i==2){k_avd = 300;}

                    else {k_avd = 100;}

                    f_tmp = f_tmp + alldistance[i+j*4]*(k_avd*(dref[i] + alpha[i] - d[i][j]) - b_avd*d_dot[i*11+j]);
                }
            }
            d_past[i][j] = d[i][j];
        }
        
       
       if(isnan(f_tmp(0))||isnan(f_tmp(1))||isnan(f_tmp(2))) f_tmp<<0,0,0;
       
       allf.push_back(f_tmp);
    }   

    f02d = P2d*allf[0];
    f1 = allf[1];
    f4 = allf[2];
    f7 = allf[3];

    if(sqrt(f02d.transpose()*f02d)!=0){
        u2d = f02d/sqrt(f02d.transpose()*f02d);
    }
    else{
        u2d<<0,0;
    }

    if(sqrt(f1.transpose()*f1)!=0){
        u1 = f1/sqrt(f1.transpose()*f1);
    }
    else{
        u1<<0,0,0;
    }

    if(sqrt(f4.transpose()*f4)!=0){
        u4 = f4/sqrt(f4.transpose()*f4);
    }
    else{
        u4<<0,0,0;
    }

    if(sqrt(f7.transpose()*f7)!=0){
        u7 = f7/sqrt(f7.transpose()*f7);
    }
    else{
        u7<<0,0,0;
    }

    jobs_tmp2d << jmt2d ,Z27;
    jobs0 = u2d.transpose()*jobs_tmp2d;

    jobs_tmp << jmt ,jt1;
    jobs1 = u1.transpose()*jobs_tmp;

    jobs_tmp << jmt ,jt4;
    jobs4 = u4.transpose()*jobs_tmp;

    jobs_tmp << jmt ,jt7;
    jobs7 = u7.transpose()*jobs_tmp;


}

void dghc_controller::getJacobian()
{
   std::vector<Eigen::MatrixXd> allJacobians;
   //std::cout<<"at getjacobian j0: "<<std::endl<<j0<<std::endl;
   allJacobians.push_back(j0); 
   allJacobians.push_back(jimp); 
   allJacobians.push_back(jmimp); 
   allJacobians.push_back(jhand); 
   allJacobians.push_back(jjl1); 
   allJacobians.push_back(jjl2); 
   allJacobians.push_back(jjl3); 
   allJacobians.push_back(jjl4); 
   allJacobians.push_back(jjl5); 
   allJacobians.push_back(jjl6); 
   allJacobians.push_back(jjl7);  
   allJacobians.push_back(jobs0);
   allJacobians.push_back(jobs1);
   allJacobians.push_back(jobs4);
   allJacobians.push_back(jobs7);

   setJacobianMatrices(allJacobians);
//    std::cout<<"alljacobians:"<<std::endl<<allJacobians[0]<<std::endl;

}

void dghc_controller::getWrench()
{
      
    
   k_tmp<<I3,Z33,
           Z33,wRe; //0.1

   k<< 250,0,0,0,0,0,
       0,250,0,0,0,0,
       0,0,250,0,0,0,
       0,0,0,10,0,0,
       0,0,0,0,10,0,
       0,0,0,0,0,10; //0.1
    k= k*k_tmp;

    b<< 80,0,0,0,0,0,
        0,80,0,0,0,0,
        0,0,80,0,0,0,
        0,0,0,8,0,0,
        0,0,0,0,8,0,
        0,0,0,0,0,8; //0.1

    k_m<<200,0,0,
         0,200,0,
         0,0,180;

    // k_m<<50,0,0,
        //  0,50,0,
        //  0,0,20;

    b_m<<20,0,0,
         0,20,0,
         0,0,10;

    k_j<<18,0,0,0,0,0,0,
         0,15,0,0,0,0,0,
         0,0,15,0,0,0,0,
         0,0,0,20,0,0,0,
         0,0,0,0,10,0,0,
         0,0,0,0,0,10,0,
         0,0,0,0,0,0,5;

    b_j<<18,0,0,0,0,0,0,
         0,15,0,0,0,0,0,
         0,0,15,0,0,0,0,
         0,0,0,20,0,0,0,
         0,0,0,0,10,0,0,
         0,0,0,0,0,10,0,
         0,0,0,0,0,0,5;
    // b_j<<9,0,0,0,0,0,0,
    //      0,9,0,0,0,0,0,
    //      0,0,6,0,0,0,0,
    //      0,0,0,6,0,0,0,
    //      0,0,0,0,6,0,0,
    //      0,0,0,0,0,6,0,
    //      0,0,0,0,0,0,2.5;  

    // jl_alpha = 0.174533;//10deg 
    jl_alpha <<0.26,0.26,0.26,0.26,0.26,0.26,0.13;
    jl_k<<50,50,50,50,50,10,10;
    jl_b<<10,10,10,10,10,10,5;
   
    // jl_alpha = 0.08;//rad

    for(int j=0;j<7;j++){

        if(q(j)<=(lbq(j) + jl_alpha(j)))
        {
            wrenchjl(j) = jl_k(j)*(lbq(j) + jl_alpha(j)-q(j)) - jl_b(j)*q_dot(j);
            if(wrenchjl(j)<=0) wrenchjl(j) = 0;
        }

        else if(q(j)>=(ubq(j) - jl_alpha(j)))
        {
            wrenchjl(j) = jl_k(j)*(ubq(j) - jl_alpha(j)-q(j)) - jl_b(j)*q_dot(j);
            if(wrenchjl(j)>=0) wrenchjl(j) = 0;
        }

        else
        {
            wrenchjl(j) = 0;
        }

    }
 
    wrenchImp = k*(d_pose - position76d) - b*twist7;   // whole body end-effector impedance
    
        
    wRft = wRe*eRft;
    wRft_e.block(0,0,3,3) = wRft;
    wRft_e.block(3,3,3,3) = wRft; 
    wrenchHand = wRft_e * wrenchExt_tmp; // hand guidacne
    

    // initial state or holding pose state 
    if(currentStep.data==3){
        Eigen::Quaterniond quat0(R_m.transpose()*R_m);
        quatV << quat0.x(),quat0.y(),quat0.z();
        d_position_m <<  mobile_pose2d(0), mobile_pose2d(1), R_m.block(2,0,1,3)*quatV;
        wrenchmImp = k_m*(d_position_m -mobile_pose3d) - b_m*mobile_vel3d; //mobile impedance home position
    }

    else{
        Eigen::Quaterniond quat1(R_m.transpose()*R_d);
        quatV << quat1.x(),quat1.y(),quat1.z();
        d_position_m << d_position2d,R_m.block(2,0,1,3)*quatV;
        wrenchmImp = k_m*(d_position_m -mobile_pose3d) - b_m*mobile_vel3d; //mobile impedance impedance
        
    }
    wrench0 = k_j*(desired_q -q) - b_j*q_dot;  
    //std::cout<<"desired_q: "<<desired_q.transpose()<<std::endl;
    //std::cout<<"init_q: "<<init_q.transpose()<<std::endl;
    wrench06 = wrench0.block(0,0,6,1);
    wrenchObs0 = sqrt(f02d.transpose()*f02d);         //obstacle avoidance
    wrenchObs1 = sqrt(f1.transpose()*f1);
    wrenchObs4 = sqrt(f4.transpose()*f4);       //obstacle avoidance
    wrenchObs7 = sqrt(f7.transpose()*f7);  
}

void dghc_controller::setInertia()
{
   Eigen::MatrixXd I10 = Eigen::MatrixXd::Identity(10,10);
   Eigen::MatrixXd Weighted_wholeM = wholeM;
   Weighted_wholeM.block(0,0,3,3) = 1000*mobile_mass;
   
   setInertiaMatrix(wholeM.inverse());
    // std::cout<<"W: "<<std::endl<<Weighted_wholeM<<std::endl;

}

void dghc_controller::setPriority()
{
    //현재 시간 업데이트
    ros::Time currentTime = ros::Time::now();
    //시간 간격 계산 
    double deltaT = (currentTime-previousTime).toSec();
    previousTime = currentTime;

    //priority가 변했는지 확인
    priorityFilter();

    //priority가 변하지 않았다면, 함수종료
    if(!priorityChanged){
        transitionTime=0;
        return;
    };
    
    desiredAlphas = getDesiredAlphas(filteredPriority);


    //현재의 alpha값들을 통해, priorityVector 생성 
    std::vector<double> prioritiesVector;
    for(int row=0; row<numberOfTasks; row++){
        for(int col=row; col<numberOfTasks;col++){
            prioritiesVector.emplace_back(getAlphaIJ(row,col));
        }
    }
    // std::cout << "a00 :\t" << prioritiesVector[0] << std::endl;
    // std::cout << "a11 :\t" << prioritiesVector[11] << std::endl;
    // std::cout << "a22 :\t" << prioritiesVector[22] << std::endl;
    // std::cout << "a01 :\t" << prioritiesVector[1] << std::endl;
    // std::cout << "a02 :\t" << prioritiesVector[2] << std::endl;
    // std::cout << "a12 :\t" << prioritiesVector[12] << std::endl;
    a00 = prioritiesVector[0]; //a00
    a11 = prioritiesVector[11]; //a11
    a22 = prioritiesVector[21]; //a22
    a01 = prioritiesVector[1]; //a01
    a02 = prioritiesVector[2]; //a02
    a12 = prioritiesVector[12]; //a12
    
    const double transitionDuration = 0.3;
    changeAlphas(prioritiesVector,transitionTime,deltaT,transitionDuration);

    transitionTime += deltaT;

    int counter=0;
    for(unsigned int i=0; i<numberOfTasks; i++){
        for(unsigned int j=i; j<numberOfTasks; j++){
            if(prioritiesVector[counter] < 0){
                prioritiesVector[counter] = 0;
                // std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector[counter] << " < 0" << std::endl;
                return;
            }
            if(prioritiesVector[counter] > 1){
                prioritiesVector[counter] = 1;
                // std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector[counter] << " > 1" << std::endl;
                return;
            }
            setAlphaIJ(i,j,prioritiesVector[counter]);
            counter++;
        }
    }
    assert(counter==prioritiesVector.size());
}

std::vector<double> dghc_controller::getDesiredAlphas(priorityTuples tupleVector){

    std::vector<std::vector<double>> matrix(numberOfTasks, std::vector<double>(numberOfTasks,-1));
    
    for (int i = 0; i < tupleVector.size(); i++) {
        int taskNumber = std::get<0>(tupleVector[i]);
        //diagonal term update
        matrix[taskNumber][taskNumber] = std::get<1>(tupleVector[i]);
        //relative term update
        if(tupleVector[i] != tupleVector.back()){
            int nextTaskNumber = std::get<0>(tupleVector[i+1]);

            if(taskNumber < nextTaskNumber ){
                matrix[taskNumber][nextTaskNumber] = std::get<2>(tupleVector[i]);
            }

            else{
                matrix[nextTaskNumber][taskNumber] = 1 - std::get<2>(tupleVector[i]);
            }
        }

        // taskNumber의 행을 0으로 업데이트 (자기 자신의 alpha값 0, 다른 task들보다 높은 우선순위)
        for (int col = taskNumber+1; col < numberOfTasks; col++) {
            // 이전에 값이 변경된 적이 있다면, 다른 task가 이미 선점한 값
            if (matrix[taskNumber][col] == -1) {
                matrix[taskNumber][col] = 0.0;
            }
        }
        // taskNumber의 열을 1로 업데이트 (다른 task들보다 높은 우선순위)
        for (int row = 0; row < taskNumber; row++) {
            if (matrix[row][taskNumber] == -1) {
                matrix[row][taskNumber] = 1.0;
            }
        }
    }

    // there is no taskNumber in tupleVector
    for (int i = 0; i <numberOfTasks; ++i) {
        
        if (matrix[i][i] == -1) {
            matrix[i][i] = 1.0;
        }
        
        for (int j = i + 1; j <numberOfTasks; j++) {
            if (matrix[i][j] == -1) {
                matrix[i][j] = 1.0;
            }
        }
        
        for (int j = i + 1; j <numberOfTasks; j++) {
            if (matrix[j][i] == -1) {
                matrix[j][i] = 0.0;
            }
        }
    }
    
    // 대칭된 위치의 아랫삼각행렬 값 업데이트   
    for(int row=0;row<numberOfTasks;row++){
        for(int col=row+1;col<numberOfTasks;col++){
            matrix[col][row] = 1 -  matrix[row][col];
        }
    }
    
    std::vector<double> desiredPriority;
    //matrix에서 vector추출
    for (int i = 0; i < numberOfTasks; ++i) {
        for (int j = i; j < numberOfTasks; ++j) {
            desiredPriority.emplace_back(matrix[i][j]);
        }
    }

    return desiredPriority;
}

//priorityInput에 저장된 값들을 변경하여, DesiredPriority를 재조정 
void dghc_controller::setTrackingPriority(const int manipulatorTaskNum , const int mobileTaskNum, const int poseTaskNum){
    filteredPriority = priorityInput;

}

void dghc_controller::setObstaclePrirority(const std::vector<int> obstacleTaskNums){
    
    //d_ref보다 d_min이 크면, Task에서 Obstacle Avoidance 삭제 
    if(f02d.norm()==0){
        for (auto it = filteredPriority.begin(); it != filteredPriority.end();++it){
            if (std::get<0>(*it) == obstacleTaskNums[0]) {
                it = filteredPriority.erase(it);
                break;
            }
        }
    }
    if(f1.norm()==0){
        for (auto it = filteredPriority.begin(); it != filteredPriority.end();++it){
            if (std::get<0>(*it) == obstacleTaskNums[1]) {
                it = filteredPriority.erase(it);
                break;
            }
        }
    }
    if(f4.norm()==0){
        for (auto it = filteredPriority.begin(); it != filteredPriority.end();++it){
            if (std::get<0>(*it) == obstacleTaskNums[2]) {
                it = filteredPriority.erase(it);
                break;
            }
        }
    }
    if(f7.norm()==0){
        for (auto it = filteredPriority.begin(); it != filteredPriority.end();++it){
            if (std::get<0>(*it) == obstacleTaskNums[3]) {
                it = filteredPriority.erase(it);
                break;
            }
        }
    }
}

void dghc_controller::setJointLimitPriority(const std::vector<int> jointLimitTaskNums){
    double buffer = 1; //0.2rad = 11.45deg or Nm
    for(int i=0; i<7;i++){
        //lbq(i) + jl_alpha(i) - buffer < q(i) && ubq(i) - jl_alpha(i) + buffer > q(i)
        //std::cout<<"wrenchjl7: "<<std::endl<<wrenchjl(6)<<std::endl;
        if(abs(wrenchjl(i))<=buffer){
            for (auto it = filteredPriority.begin(); it != filteredPriority.end();++it){
              if (std::get<0>(*it) == jointLimitTaskNums[i]) {
                    it = filteredPriority.erase(it);
                    break;
                }
            }
        }

    }
}

void dghc_controller::priorityFilter(){

    // filteredPriority = priorityInput;

    //priorityInput(사용자 지정) 값을 바탕으로, 환경 고려하여 filteredPriority 생성
    const int mani = 1;
    const int pose = 0;
    const int mobile = 2;
    const std::vector<int> jointLimitTaskNums = {4,5,6,7,8,9,10};

    const std::vector<int> obstacle = {11,12,13,14}; // 0, 1, 4, 7 순서대로 

    setTrackingPriority(mani,mobile,pose);
    setObstaclePrirority(obstacle);
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


    if(prevFilteredPriority!=filteredPriority){
         if(!filteredPriority.empty()){
            std::cout<<"filtered priority : ";
            auto lastTask = filteredPriority.end() - 1;
            std::for_each(filteredPriority.begin(), lastTask, [](const auto& task) {
                std::cout << "[" << std::get<0>(task) << "(" << std::get<1>(task) << ")] --" << std::get<2>(task) << "-->> ";
            });
            // Special handling for the last task
            std::cout << "["<< std::get<0>(*lastTask) << "(" << std::get<1>(*lastTask) << ")]" << std::endl;
        }
        else{
            // std::cout << "filteredPriority is empty (no task)" << std::endl;
        }
        
        priorityChanged = true;
        transitionTime=0;
        prevFilteredPriority = filteredPriority;
    }
    
}

void dghc_controller::changeAlphas(std::vector<double>& currentValues,double t, double dt, double duration){
    // 변경 시작할 때를 기준으로, scale값들 설정
    if(alphaChangeDone){
        for(int i=0; i<desiredAlphas.size(); i++){
        
            //increase alpha 
            if(desiredAlphas[i]>currentValues[i]){
                scaleValues[i] = desiredAlphas[i]-currentValues[i];
            }

            //decrease alpha
            else if(desiredAlphas[i]<currentValues[i]){
                scaleValues[i] = currentValues[i] - desiredAlphas[i];
            }
            else{}
        }
        alphaChangeDone=false;
    }
    const double epsilon = 1.0e-06;


    for(int i=0; i<desiredAlphas.size(); i++){
        double currentValue_tmp;
        
        //increase alpha 
        if(desiredAlphas[i]>currentValues[i]){
            
            // currentValues[i] = prevDesiredAlphas[i] +(0.5*(M_PI/duration)*sin((M_PI/duration)*t))*dt*scale;
            currentValue_tmp = currentValues[i] + (0.5*(M_PI/duration)*sin((M_PI/duration)*t))*dt*scaleValues[i];

            if((currentValue_tmp - desiredAlphas[i]) > epsilon || currentValue_tmp > 1){
                currentValues[i] = desiredAlphas[i];
            }
            else{
                currentValues[i] = currentValue_tmp;
            }
        }

        //decrease alpha
        else if(desiredAlphas[i]<currentValues[i]){

            // currentValues[i] = prevDesiredAlphas[i] - (0.5*(M_PI/duration)*sin((M_PI/duration)*t))*dt*scale;
            currentValue_tmp = currentValues[i] - (0.5*(M_PI/duration)*sin((M_PI/duration)*t))*dt*scaleValues[i];

            if(desiredAlphas[i] - currentValue_tmp > epsilon || currentValue_tmp < 0){
                currentValues[i] = desiredAlphas[i];
            }
            else{
                currentValues[i] = currentValue_tmp;
            }
        }
        else{}
    }


    //모든 alpha값에 대한 변경이 완료됐는지 확인
    if(alphasSetDone(currentValues,desiredAlphas,epsilon)){
        printf("\n************** change is done !! **************\n");    
        
        currentValues=desiredAlphas;
        transitionTime=0;
        priorityChanged = false;
        alphaChangeDone=true;
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
    for(unsigned int i=0; i<numTasks; i++){
        allProjections.push_back(Eigen::MatrixXd::Zero(DOFsize,DOFsize));
    }
  
    Eigen::VectorXd ranks;
    ranks = Eigen::VectorXd::Zero(numTasks);
    //  std::cout<<"ranks: " <<std::endl<< ranks.transpose()<<std::endl;
     
    bool ok = getAllGeneralizedProjectors(allProjections, ranks);
}


void dghc_controller::getProjectedToq()
{
//    std::cout<<"allprojection0: " <<std::endl<< allProjections[0]<<std::endl;
    //std::cout<<"at projected toq j0T: "<<std::endl<<j0.transpose()<<std::endl;
    //std::cout<<"at projected toq j0: "<<std::endl<<j0<<std::endl;
    toqT = allProjections[0]*j0.transpose()*wrench0
    +allProjections[1]*jimp.transpose()*wrenchImp
    +allProjections[2]*jmimp.transpose()*wrenchmImp
    +allProjections[3]*jhand.transpose()*wrenchHand
    +allProjections[3]*jjl1.transpose()*wrenchjl[0]
    +allProjections[4]*jjl2.transpose()*wrenchjl[1]
    +allProjections[5]*jjl3.transpose()*wrenchjl[2]
    +allProjections[6]*jjl4.transpose()*wrenchjl[3]
    +allProjections[7]*jjl5.transpose()*wrenchjl[4]
    +allProjections[8]*jjl6.transpose()*wrenchjl[5]
    +allProjections[9]*jjl7.transpose()*wrenchjl[6]
    +allProjections[10]*jobs0.transpose()*wrenchObs0
    +allProjections[11]*jobs1.transpose()*wrenchObs1
    +allProjections[12]*jobs4.transpose()*wrenchObs4
    +allProjections[13]*jobs7.transpose()*wrenchObs7
    ; 

    // toqT =  allProjections[0]*jrimp.transpose()*wrenchImp;
    //         +allProjections[1]*jmimp.transpose()*wrenchmImp;

     
     if(isnan(toqT(0))) toqT = Eigen::VectorXd::Zero(10,1);
     
   
   
}

bool dghc_controller::alphasSetDone(const std::vector<double>& vec1, const std::vector<double>& vec2, double epsilon){
    if (vec1.size() != vec2.size()) {
        return false; // 벡터의 크기가 다르면 같지 않음
    }

    for (size_t i = 0; i < vec1.size(); ++i) {
        if (std::abs(vec1[i] - vec2[i]) > epsilon) {
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

void dghc_controller::set_fsm_args(){
    if(currentStep.data == 3 || currentStep.data == 4){
        
        d[3][4] = (position7 - hand_position).norm(); // right hand to EE
        std::cout << "d[3][4]: "<< d[3][4] << std::endl;
        //set distance flag , d[3][4] : right hand to EE 
        if(d[3][4] < DISTANCE_THRESHOLD){
            distance_flag.data = true;
        }
        else{
            distance_flag.data = false;
        }

        //set force flag
        Eigen::Vector3d FORCE  = wrenchHand.block(0,0,3,1);
        Eigen::Vector3d TORQUE  = wrenchHand.block(0,3,3,1);
        if(FORCE.norm() > FORCE_THRESHOLD || TORQUE.norm() > TORQUE_THRESHOLD){
            force_flag.data = true;
        }
        else{
            force_flag.data = false;
        }
    }
}




int dghc_controller::run(){
    numberOfTasks = getNumTasks();
    scaleValues.assign(numberOfTasks*(numberOfTasks+1)*0.5,1);
    // auto logger = XBot::MatLogger2::MakeLogger("/home/yushin/log/log");
    // logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);


    ros::Time current_time, last_time;


    ros::Rate loop_rate(1000);
    while(ros::ok())
    {    
         
        //  logger->add("mobile_position", mobile_pose2d);
        //  logger->add("mobile_pose", euler_m);
        //  logger->add("q", q);
        //  logger->add("d_position", d_position);
        //  logger->add("position7", position7);
        //  logger->add("d_euler", d_euler);
        //  logger->add("euler_e", euler_e);

        current_time = ros::Time::now();
        dt = (current_time - last_time).toSec();
        
         getModel();

         getWrench();

         getJacobian();

         set_fsm_args();
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

        vir_force = R_m.transpose()*toqT.block(0,0,3,1); 
        //  std::cout <<"vir_force:"<<std::endl <<vir_force << std::endl;
        //  std::cout <<"toq: "<<std::endl <<toqT.block(3,0,7,1) << std::endl;
        
     
        toq = toqT.block(3,0,7,1); //+ arm_interface_->getCoriolisCompensation();
         
    
           
                // if( interface_type_str == robot_ns + "HARDWARE")
                // {
                //     toq = toqT.block(3,0,7,1) + arm_interface_->getCoriolisCompensation(); //coriolis changed , gravity removed
                //      std::cout<<"INTERFACE_TYPE:HARDWARE "<<std::endl;     
                // }
                // else if(interface_type_str == robot_ns + "SIMULATION")
                // {
                //     toq = toqT.block(3,0,7,1) + arm_interface_->getCoriolisCompensation() + arm_interface_->getGravityCompensation();
                //     std::cout<<"INTERFACE_TYPE:SIMULATOIN "<<std::endl;     
                // }


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

        distance_flag_pub.publish(distance_flag);
        force_flag_pub.publish(force_flag);


        // Limiting maximun torque values of the ARM
        for (size_t i = 0; i < 4; ++i) {
            if(toq(i) > 87 ){
                toq(i) = 87;
            }
            else if(toq(i) < -87){
                toq(i) = -87;
            }
        }
        for (size_t i = 4; i < 7; ++i) {
            if(toq(i) > 8){
                toq(i) = 8;
            }
            else if(toq(i) < -8){
                toq(i) = -8;
            }
        }
        //std::cout<<"toq_limited: "<<toq.transpose()<<std::endl;
        // Set joint commands and publish virtual forces
        
        arm_interface_->setJointCommands(toq);
        q_past = q; 


        // for human factor
        geometry_msgs::PoseArray modelsPosemsg;
        geometry_msgs::PoseArray modelsVelmsg;

        // Create a vector of Vector3d to store positions
        std::vector<Eigen::Vector3d> positions = {
        mobile_pose + R_m*m_distance, position1, position2, position3,
        position4, position5, position6, position7
        };
        std::vector<Eigen::VectorXd> twists = {
        mobile_twist, twist1, twist2, twist3,
        twist4, twist5, twist6, twist7
        };

        // Resize the poses field to have the right size
        modelsPosemsg.poses.resize(positions.size());
        modelsVelmsg.poses.resize(positions.size());

        // Iterate through positions and fill modelsPosemsg.poses
        for (size_t i = 0; i < positions.size(); ++i) {
            modelsPosemsg.poses[i].position.x = positions[i](0);
            modelsPosemsg.poses[i].position.y = positions[i](1);
            modelsPosemsg.poses[i].position.z = positions[i](2);
            modelsVelmsg.poses[i].position.x = twists[i](0);
            modelsVelmsg.poses[i].position.y = twists[i](1);
            modelsVelmsg.poses[i].position.z = twists[i](2);
        }

        modelPosepublisher.publish(modelsPosemsg);
        modelVelpublisher.publish(modelsVelmsg);

        
        ros::spinOnce();
        loop_rate.sleep();
        last_time = current_time;
    }
    return 0;
}
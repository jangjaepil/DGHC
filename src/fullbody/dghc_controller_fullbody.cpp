#include "dghc_controller_fullbody.hpp"
#include "GHCProjections.hpp"
#include "matlogger2/matlogger2.h"
#include "matlogger2/utils/mat_appender.h"

dghc_controller::dghc_controller(){
    obstacle_states = nh.subscribe("/human_factor/human_joint_positions", 500, &dghc_controller::obstacle_states_callback,this);
    d_pose_sub = nh.subscribe("/desired_pose", 500, &dghc_controller::desired_pose_callback,this);
    priority_input_sub = nh.subscribe("/priorityInput", 500, &dghc_controller::priority_input_callback,this);
    mode_sub  = nh.subscribe("/modeInput", 500, &dghc_controller::mode_input_callback,this);
    mass_sub  = nh.subscribe("/mass_matrix", 500, &dghc_controller::mass_callback,this);
    riskfactor_sub = nh.subscribe("/human_factor/risk_factor", 500, &dghc_controller::riskfactor_callback,this);
    cooperator_sub = nh.subscribe("/human_factor/cooper_target", 500, &dghc_controller::cooperator_callback,this);
    mobile_target_sub = nh.subscribe("/human_factor/mobile_target", 500, &dghc_controller::mobile_target_callback,this);
    optitrack_sub = nh.subscribe("/natnet/robot_pose", 500, &dghc_controller::optitrack_callback,this);
    Vir_torque = nh.advertise<geometry_msgs::Wrench>("/vir_torque", 500);
    modelPosepublisher = nh.advertise<geometry_msgs::PoseArray>("/human_factor/robot_joint_positions",500);
    modelVelpublisher = nh.advertise<geometry_msgs::PoseArray>("/human_factor/robot_joint_velocities",500);
    d_dot_sub = nh.subscribe("/human_factor/rel_vel",500 ,&dghc_controller::d_dot_callback,this);
    
    init_q << 0,-0.8,0,-2.1,0,1.4,-0.5;
    q=init_q;
    ubq << 2.71344,1.70281,2.89407,-0.164777,2.79709,4.10882,0.988027;
    lbq << -2.73363,-1.77708,-2.88609,-3.03556,-2.79978,0.575631,-2.0;  //-2.6895(joint7)

    R_distance<< 0.2575,0,0.52;
    m_distance<< 0,0,0.26;
    
    transitionTime =0;
    previousTime=ros::Time::now();
    modeInput = false;
    priorityInput.emplace_back(std::make_tuple(8,0,0));
    priorityInput.emplace_back(std::make_tuple(9,0,0));
    priorityInput.emplace_back(std::make_tuple(10,0,0));
    priorityInput.emplace_back(std::make_tuple(11,0,0));
    priorityInput.emplace_back(std::make_tuple(12,0,0));
    priorityInput.emplace_back(std::make_tuple(13,0,0));
    priorityInput.emplace_back(std::make_tuple(14,0,0));
    priorityInput.emplace_back(std::make_tuple(6,0,0));
  
    priorityInput.emplace_back(std::make_tuple(0,0,0));
    priorityInput.emplace_back(std::make_tuple(2,0,0));
    priorityChanged = true;
    alphaChangeDone = true;

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
    // init_q = Eigen::Map<Eigen::VectorXd>(arm_joint_state.position.data(), arm_interface_->getJointsNum());
    
}    
void dghc_controller::cooperator_callback(const std_msgs::Int32& target){
    coop_tar = target.data; //4(right hand) or 6(left hand)

    
}

void dghc_controller::mobile_target_callback(const std_msgs::Int32& mobile_target){
    // pelvis, t12, head, right_forarm, right_hand, left_forearm, left_hand, right_lower_leg, right_foot, left_lower_leg, left_foot

    if (mobile_target.data < 0 || mobile_target.data > 10) {
        std::cout << "Invalid Value, please enter 0~10." <<std::endl;
        return;
        }   

    this->mobile_target = mobile_target.data; //4(right hand) or 6(left hand)

    ROS_INFO("Mobile Target is %d", mobile_target);

}

void dghc_controller::riskfactor_callback(const human_factor::RiskFactor::ConstPtr& riskfactor){
    if (riskfactor->factor.size() == 44) {
        // Access the elements of the received array
        for (size_t i = 0; i < riskfactor->factor.size(); ++i) {
            riskFactorArray[i] = riskfactor->factor[i];
            //ROS_INFO("Received value[%zu]: %f", i, riskFactorArray[i]);
        }
        if(coop_mode){
            // riskFactorArray[33 + coop_tar] = -1;
        }
    } else {
        ROS_WARN("Received array size is not 44!");
    }
}

void dghc_controller::mass_callback(const geometry_msgs::Inertia& mass_matrix)
{
    mobile_mass<<mass_matrix.ixx,mass_matrix.ixy,mass_matrix.ixz,
                               0,mass_matrix.iyy,mass_matrix.iyz,
                               0,              0,mass_matrix.izz;

    
    // std::cout<<"mobile_mass: "<<std::endl<<mobile_mass<<std::endl;
    
}



void dghc_controller::desired_pose_callback(const geometry_msgs::Pose& desired_pose){
    // d_euler(0) = desired_pose.orientation.x;
    // d_euler(1) = desired_pose.orientation.y;
    // d_euler(2) = desired_pose.orientation.z; //zyx convention

    // d_position_tmp(0) = desired_pose.position.x;
    // d_position_tmp(1) = desired_pose.position.y;
    // d_position_tmp(2) = desired_pose.position.z;
}

// void dghc_controller::obstacle_states_callback(const gazebo_msgs::ModelStates::ConstPtr& obstacleState)
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
 
    
    if(coop_mode){
        dPositionIndex = coop_tar;
        d_position3d_tmp(0) =  obstacleStateMsg->states[dPositionIndex].pose.position.x; 
        d_position3d_tmp(1) =  obstacleStateMsg->states[dPositionIndex].pose.position.y;
        d_position3d_tmp(2) =  obstacleStateMsg->states[dPositionIndex].pose.position.z;
        
        p2mani = position7 - d_position3d_tmp;
        u_p2mani = p2mani/(sqrt(p2mani.transpose()*p2mani));
        d_position = margin_end*u_p2mani + d_position3d_tmp;  ;  //manipulator position
        
        wRd<< 0.7,0.7,0, //manipulator rotation
              0.7,-0.7,0,
              0,0,-1;

        d_position2d_tmp(0) = obstacleStateMsg->states[mobile_target].pose.position.x;  
        d_position2d_tmp(1) = obstacleStateMsg->states[mobile_target].pose.position.y;
        p2m = mobile_pose2d - d_position2d_tmp;
        u_p2m = p2m/(sqrt(p2m.transpose()*p2m));
        d_position2d = margin*u_p2m + d_position2d_tmp;  //mobile position
        
        d_position3d(0) = obstacleStateMsg->states[mobile_target].pose.position.x; //mobile rotation
        d_position3d(1) = obstacleStateMsg->states[mobile_target].pose.position.y;
        d_position3d(2) = obstacleStateMsg->states[mobile_target].pose.position.z;
        
        Eigen::VectorXd mobiel2targetV =  R_m.transpose()*(d_position3d - mobile_pose);
        R_d =R_m *Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(atan2(mobiel2targetV(1),mobiel2targetV(0)) , Eigen::Vector3d::UnitZ());
    }
    else{
       
        wRd<< 0.7,0.7,0, //manipulator rotation
              0.7,-0.7,0,
              0,0,-1;

        d_position3d_tmp(0) =  -0.6; //manipulator position 
        d_position3d_tmp(1) =  1*sin((2*M_PI/50)*current_sec);
        d_position3d_tmp(2) =  0.1*sin((2*M_PI/50)*current_sec)+1;
        
        d_position = d_position3d_tmp;  
        

        d_position2d_tmp(0) = d_position3d_tmp(0);  
        d_position2d_tmp(1) = d_position3d_tmp(1);
        p2m = mobile_pose2d - d_position2d_tmp;
        u_p2m = p2m/(sqrt(p2m.transpose()*p2m));
        d_position2d = margin*u_p2m + d_position2d_tmp;  //mobile position
        
        d_position3d = d_position3d_tmp; //mobile rotation
      
        Eigen::VectorXd mobiel2targetV =  R_m.transpose()*(d_position3d - mobile_pose);
        R_d =R_m *Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(atan2(mobiel2targetV(1),mobiel2targetV(0)) , Eigen::Vector3d::UnitZ());
  
    }  
    
    
    eRd = wRe.transpose()*wRd;
    Eigen::Quaterniond quat(eRd);
    
    d_pose<<d_position,quat.x(),quat.y(),quat.z();


   
    //   std::cout<<"tracking: "<<std::endl<<d_position.transpose()<<std::endl;
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
    //   std::cout << "optitrack_p: " << optitrack_p << std::endl;
    //  std::cout << "optitrack_sign: " << optitrack_sign << std::endl;
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

    int numTask = getNumTasks();
    double value;
    while(ss>> value){
        if(value < 0 || value >= numTask){
                // std::cout << "Priority input value must be between 0 and " << numTask-1 << std::endl;
                return;
            }

        if (seen_digits.find(value) != seen_digits.end()) {
        // std::cout << "Duplicate digit(" << value << ") found in input_str." << std::endl;
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

void dghc_controller::mode_input_callback(const std_msgs::Bool::ConstPtr& mode_input){
    if(mode_input->data == modeInput){
        return;
    }   
    if(mode_input->data == true){
        modeInput = true;
    }
    else{
        modeInput = false;
    }
}

void dghc_controller::d_dot_callback(const std_msgs::Float64MultiArray::ConstPtr& d_dot_msg){
    for(int i=0; i<44; i++){
        d_dot[i] = d_dot_msg->data[i];
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
    

    //R_m = mobile_quat_r.normalized().toRotationMatrix();
    w_R_m = mobile_quat.normalized().toRotationMatrix();
    w_R_o = optitrack_q.normalized().toRotationMatrix();
    
    w_T_m_raw.block(0,0,3,3) = w_R_m;
    w_T_m_raw.block(0,3,3,1) = mobile_pose_r;

    if((count == 0) && optitrack_sign){ 
        count = 1; 
        
        
        
        init_w_T_M.setIdentity(); 
        init_w_T_M.block<3,3>(0,0) = w_R_m;
        init_w_T_M.block<3,1>(0,3) = mobile_pose_r;

        init_w_T_o.setIdentity();
        init_w_T_o.block<3,3>(0,0) = w_R_o;
        init_w_T_o.block<3,1>(0,3) = optitrack_p;

        init_o_T_M.setIdentity();
        init_o_T_M = init_w_T_o.inverse() * init_w_T_M; // = Rhat_T_Odot
        // std::cout<<std::endl<<"init_o_T_M"<<init_o_T_M<<std::endl;
    }

    if(optitrack_sign){        
        
        //new ver.
        new_w_T_o.block(0,0,3,3) = w_R_o; 
        new_w_T_o.block(0,3,3,1) = optitrack_p;
       
        // just use optitrack ver.
        real_mobile_Tf = new_w_T_o * init_o_T_M;
         
        
        // std::cout<<"init_w_T_o inverse"<<std::endl<<init_w_T_o.inverse()<<std::endl;
        // std::cout<<"init_w_T_M"<<std::endl<<init_w_T_M<<std::endl;
        mobile_pose = real_mobile_Tf.block(0,3,3,1);
        // std::cout<<"mobile_pose"<<std::endl<<mobile_pose<<std::endl;

        R_m = real_mobile_Tf.block(0,0,3,3);
        
        mobile_quat_filtered = R_m;


        optitrack_sign = false;
        // ROS_WARN_STREAM("Optitrack data is updated!");
    }
    
    mobile_base_odom_filtered = mobile_base_odom;
    mobile_base_odom_filtered.pose.pose.position.x = mobile_pose(0);
    mobile_base_odom_filtered.pose.pose.position.y = mobile_pose(1);
    mobile_base_odom_filtered.pose.pose.position.z = mobile_pose(2);
    mobile_base_odom_filtered.pose.pose.orientation.x = mobile_quat_filtered.x();
    mobile_base_odom_filtered.pose.pose.orientation.y = mobile_quat_filtered.y();
    mobile_base_odom_filtered.pose.pose.orientation.z = mobile_quat_filtered.z();
    mobile_base_odom_filtered.pose.pose.orientation.w = mobile_quat_filtered.w();




    if(!arm_interface_->updateRobotState(mobile_base_odom_filtered))
        std::cout << "Fail to Update Manipulatro State" << std::endl;

    // Get arm joint state
    sensor_msgs::JointState arm_joint_state = *(arm_interface_->getJointsStates());
    
    q = Eigen::Map<Eigen::VectorXd>(arm_joint_state.position.data(), arm_interface_->getJointsNum());
    q_dot = Eigen::Map<Eigen::VectorXd>(arm_joint_state.velocity.data(), arm_interface_->getJointsNum());
    // std::cout<<"q_dot: "<<q_dot<<std::endl;
    arm_tau_J_d = arm_interface_->getDesiredJointCommands();

    
       
    
    //std::cout<<"R_m"<<R_m<<std::endl;

    // new ver.    
    R_m_e.block(0,0,3,3) = R_m;
    R_m_e.block(3,3,3,3) = R_m;  
    euler_m = R_m.eulerAngles(2,1,0); //zyx
    // std::cout<<mobile_pose.transpose()<<std::endl;

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
      position(0) = arm_pose.position.x;
      position(1) = arm_pose.position.y;
      position(2) = arm_pose.position.z;
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
       // std::cout << "position4 : " <<std::endl << position4 << std::endl;
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
        euler_e = wRe.eulerAngles(2,1,0); // zyx
        // std::cout<<"euler_e: "<<euler_e<<std::endl;
        // std::cout<<"position1: "<<position1<<std::endl;
        // std::cout<<"position2: "<<position2<<std::endl;
        // std::cout<<"position3: "<<position3<<std::endl;
        // std::cout<<"position4: "<<position4<<std::endl;
        // std::cout<<"position5: "<<position5<<std::endl;
        // std::cout<<"position6: "<<position6<<std::endl;
        std::cout<<"position7: "<<position7<<std::endl;
        // std::cout<<"R_m: "<<R_m<<std::endl;
        

        
      }
    }

    P2d<<1,0,0,
         0,1,0;
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
    
  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    dref[0] = 0.7; //
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

    j0.block(0,3,7,7) = Eigen::MatrixXd::Identity(7,7);

    jjl1 <<0,0,0,1,0,0,0,0,0,0;
    jjl2 <<0,0,0,0,1,0,0,0,0,0;
    jjl3 <<0,0,0,0,0,1,0,0,0,0;
    jjl4 <<0,0,0,0,0,0,1,0,0,0;
    jjl5 <<0,0,0,0,0,0,0,1,0,0;
    jjl6 <<0,0,0,0,0,0,0,0,1,0;
    jjl7 <<0,0,0,0,0,0,0,0,0,1;

    

    jimp << jm , j7; //joint7 frame
    jrimp << Z63,j7;
    jrimpT = jrimp.block(0,0,5,10);
    jmimp <<jmt3d ,Z37;

    tmp_inv = jimp*jimp.transpose();
    jimp_pinv = jimp.transpose()*tmp_inv.inverse();

    W << 10,0,0,0,0,0,0,0,0,0,
         0,10,0,0,0,0,0,0,0,0,
         0,0,10000,0,0,0,0,0,0,0,
         0,0,0,100,0,0,0,0,0,0,
         0,0,0,0,100,0,0,0,0,0,
         0,0,0,0,0,100,0,0,0,0,
         0,0,0,0,0,0,100,0,0,0,
         0,0,0,0,0,0,0,1000,0,0,
         0,0,0,0,0,0,0,0,1000,0,
         0,0,0,0,0,0,0,0,0,1000;

    A = jimp_pinv.transpose()*wholeM*jimp_pinv;
    Aw = jimp_pinv.transpose()*wholeM*W*wholeM*jimp_pinv;

    jwT = W.inverse()*wholeM.inverse()*jimp.transpose()*Aw*A.inverse();

    jw = jwT.transpose();

    if (d[3][4]<0.4 && riskFactorArray[37]<=2) coop_mode = true;
    
    if(d[3][4]>=0.4 || riskFactorArray[37]>2)
    {    
        coop_mode = false;
    }
    // pelvis, t12, head, right_forarm, right_hand, left_forearm, left_hand, right_lower_leg, right_foot, left_lower_leg, left_foot

    std::cout<<"coop_mode: "<<coop_mode<<std::endl;

}



void dghc_controller::getJacobian()
{
   std::vector<Eigen::MatrixXd> allJacobians;
   
   allJacobians.push_back(j0); 
   allJacobians.push_back(jrimpT); 
   allJacobians.push_back(jmimp); 
   allJacobians.push_back(jimp); 
   allJacobians.push_back(jobs0);
   allJacobians.push_back(jobs1);
   allJacobians.push_back(jobs4);
   allJacobians.push_back(jobs7);
   allJacobians.push_back(jjl1); 
   allJacobians.push_back(jjl2); 
   allJacobians.push_back(jjl3); 
   allJacobians.push_back(jjl4); 
   allJacobians.push_back(jjl5); 
   allJacobians.push_back(jjl6); 
   allJacobians.push_back(jjl7);  
//    allJacobians.push_back(j0);
//    allJacobians.push_back(jmimp);
   setJacobianMatrices(allJacobians);
//    std::cout<<"alljacobians:"<<std::endl<<allJacobians[0]<<std::endl;

}


void dghc_controller::getWrench()

{
    k_tmp<<I3,Z33,
           Z33,wRe; //0.1

    if(coop_mode ==false)
    {
        k<< 250,0,0,0,0,0,
            0,250,0,0,0,0,
            0,0,250,0,0,0,
            0,0,0,80,0,0,
            0,0,0,0,80,0,
            0,0,0,0,0,0; //0.1
        k= k*k_tmp;
        b<< 80,0,0,0,0,0,
            0,80,0,0,0,0,
            0,0,80,0,0,0,
            0,0,0,30,0,0,
            0,0,0,0,30,0,
            0,0,0,0,0,0; //0.1
    }
    else
    {
        k<< 250,0,0,0,0,0,
            0,250,0,0,0,0,
            0,0,250,0,0,0,
            0,0,0,80,0,0,
            0,0,0,0,80,0,
            0,0,0,0,0,0; //0.1
        k= k*k_tmp;
        b<< 80,0,0,0,0,0,
            0,80,0,0,0,0,
            0,0,80,0,0,0,
            0,0,0,30,0,0,
            0,0,0,0,30,0,
            0,0,0,0,0,0; //0.1
    }
    k_m<<300,0,0,
         0,300,0,
         0,0,200;

    b_m<<100,0,0,
         0,100,0,
         0,0,15;



    
    k_j<<18,0,0,0,0,0,0,
         0,15,0,0,0,0,0,
         0,0,15,0,0,0,0,
         0,0,0,20,0,0,0,
         0,0,0,0,10,0,0,
         0,0,0,0,0,10,0,
         0,0,0,0,0,0,5;

     b_j<<9,0,0,0,0,0,0,
         0,9,0,0,0,0,0,
         0,0,6,0,0,0,0,
         0,0,0,6,0,0,0,
         0,0,0,0,6,0,0,
         0,0,0,0,0,6,0,
         0,0,0,0,0,0,4;  

    

    jl_alpha <<0.26,0.26,0.26,0.26,0.26,0.26,1;
    jl_k<<50,50,50,50,50,10,5;
    jl_b<<10,10,10,10,10,10,5;
   



    for(int j=0;j<7;j++){

        if(q(j)<lbq(j) + jl_alpha(j))
        {
            wrenchjl(j) = jl_k(j)*(lbq(j) + jl_alpha(j)-q(j)) - jl_b(j)*q_dot(j);
        }

        else if(q(j)>ubq(j) - jl_alpha(j))
        {
            wrenchjl(j) = jl_k(j)*(ubq(j) - jl_alpha(j)-q(j)) - jl_b(j)*q_dot(j);
        }

        else
        {
            wrenchjl(j) = 0;
        }

    }
    // std::cout << "wrenchjl : " << wrenchjl.transpose() << std::endl;
    // std::cout << "q : " << q.transpose() << std::endl;

    wrenchObs0 = sqrt(f02d.transpose()*f02d);         //obstacle avoidance
    wrenchObs1 = sqrt(f1.transpose()*f1);
    wrenchObs4 = sqrt(f4.transpose()*f4);       //obstacle avoidance
    wrenchObs7 = sqrt(f7.transpose()*f7);  

    wrenchImp = k*(d_pose - position76d) - b*twist7;                // whole body end-effector impedance
    // std::cout << "d_pose : " << d_pose.transpose() << std::endl;
    // std::cout << "position76d : " << position76d.transpose() << std::endl;
    wrenchImpT = wrenchImp.block(0,0,5,1);
    //std::cout << "wrenchImpT : " << wrenchImpT.transpose() << std::endl;
    if(!modeInput){
        
        R<<1,0,0,
           0,1,0,
           0,0,1;
        Eigen::Quaterniond quat0(R_m.transpose()*R);
        quatV << quat0.x(),quat0.y(),quat0.z();
        d_position_m << -1.2,0, R_m.block(2,0,1,3)*quatV;
        wrenchmImp = k_m*(d_position_m -mobile_pose3d) - b_m*mobile_vel3d; //mobile impedance home position
        wrench0 = k_j*(init_q -q) - b_j*q_dot;  
    }
    else{
        Eigen::Quaterniond quat1(R_m.transpose()*R_d);
        quatV << quat1.x(),quat1.y(),quat1.z();
        d_position_m << d_position2d,R_m.block(2,0,1,3)*quatV;
        wrenchmImp = k_m*(d_position_m -mobile_pose3d) - b_m*mobile_vel3d; //mobile impedance impedance
        wrench0 = k_j*(init_q -q) - b_j*q_dot;
    }
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
        // std::cout << "Priority not changed" <<std::endl;
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
    a00 = prioritiesVector[0];
    a11 = prioritiesVector[15];
    a01 = prioritiesVector[1];

    a55 = prioritiesVector[65];
    a05 = prioritiesVector[5];
    a15 = prioritiesVector[19];

    a66 = prioritiesVector[75];
    a06 = prioritiesVector[6];
    a16 = prioritiesVector[20];

    a77 = prioritiesVector[84];
    a07 = prioritiesVector[7];
    a17 = prioritiesVector[21];
    
    
    a44 = prioritiesVector[54];
    a24 = prioritiesVector[31];

    a45 = prioritiesVector[55]; //a45
    a46 = prioritiesVector[56]; //a46
    a47 = prioritiesVector[57]; //a47
    a56 = prioritiesVector[66]; //a56
    a57 = prioritiesVector[67]; //a57
    a67 = prioritiesVector[76]; //a67

    const double transitionDuration = 0.5;
    changeAlphas(prioritiesVector,transitionTime,deltaT,transitionDuration);

    transitionTime += deltaT;

    int counter=0;
    for(unsigned int i=0; i<numberOfTasks; i++){
        for(unsigned int j=i; j<numberOfTasks; j++){
            if(prioritiesVector[counter] < 0){
                std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector[counter] << " < 0" << std::endl;
                return;
            }
            if(prioritiesVector[counter] > 1){
                std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector[counter] << " > 1" << std::endl;
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


    //Find remained Obstacle Tasks
    std::vector<double> obs_tasks;

    if(desiredPriority[54]==0){obs_tasks.emplace_back(4);}
    if(desiredPriority[65]==0){obs_tasks.emplace_back(5);}
    if(desiredPriority[75]==0){obs_tasks.emplace_back(6);}
    if(desiredPriority[84]==0){obs_tasks.emplace_back(7);}
    desiredPriority[55]=0.5; //a45
    desiredPriority[56]=0.5; //a46
    desiredPriority[57]=0.5; //a47
    desiredPriority[66]=0.5; //a56
    desiredPriority[67]=0.5; //a57
    desiredPriority[76]=0.5; //a67
    
    for(int i=4; i<8; i++){
        auto it = std::find(obs_tasks.begin(),obs_tasks.end(),i);
        
        if(it == obs_tasks.end()){
            if(i==4){
                //a45
                desiredPriority[55] = 1;
                //a46
                desiredPriority[56] = 1;
                //a47
                desiredPriority[57] = 1;
            }
            else if(i==5){
                //a45
                desiredPriority[55] = 0;
                //a56
                desiredPriority[66] = 1;
                //a57
                desiredPriority[67] = 1;
            }
            else if(i==6){
                //a46
                desiredPriority[56] = 0;
                //a56
                desiredPriority[66] = 0;
                //a67
                desiredPriority[76] = 1;
            }
            else{
                //a47
                desiredPriority[57] = 0;
                //a57
                desiredPriority[67] = 0;
                //a67
                desiredPriority[76] = 0;
            }
        }
    }
    
    // std::cout << "a45 : \t" << desiredPriority[55] << std::endl; //a45
    // std::cout << "a46 : \t" << desiredPriority[56] << std::endl; //a46
    // std::cout << "a47 : \t" << desiredPriority[57] << std::endl; //a47
    // std::cout << "a56 : \t" << desiredPriority[66] << std::endl; //a56
    // std::cout << "a57 : \t" << desiredPriority[67] << std::endl; //a57
    // std::cout << "a67 : \t" << desiredPriority[76] << std::endl; //a67
   

    // std::cout << "Desired Alphas" << std::endl;
    // int index = 0;
    // for (int row = 0; row < numberOfTasks; row++) {
    //     for (int i = 0; i < row; i++) {
    //         std::cout << std::setw(4) << " ";
    //     }
    //     for (int col = row; col < numberOfTasks; col++) {
    //         std::cout << std::setw(4) << std::fixed << std::setprecision(1)
    //                   << desiredPriority[index++];
    //     }
    //     std::cout << std::endl;
    // }
    return desiredPriority;
}

//priorityInput에 저장된 값들을 변경하여, DesiredPriority를 재조정 
void dghc_controller::setTrackingPriority(const int manipulatorTaskNum , const int mobileTaskNum, const int poseTaskNum){
    //mode가 false일 경우, tracking을 하지 않음
    // if(!modeInput){
    //     filteredPriority = priorityInput;
    //     Eigen::Vector2d temp = mobile_pose2d-d_position2d;
    //     double distance2d = temp.norm();
    //     if(distance2d<0.1){
    //         for (auto it = filteredPriority.begin(); it != filteredPriority.end();++it){
    //             if (std::get<0>(*it) == mobileTaskNum) {
    //                 it = filteredPriority.erase(it);
    //                 break;
    //             }
    //         }settrac
    //     }
    //     return;
    // }
    filteredPriority = priorityInput;


    if(modeInput){
        //base frame과 tracking object 사이 거리 계산 
        Eigen::Vector3d relativeVector;
        relativeVector =(position1 -d_position);
        distance = relativeVector.norm();

        //inner space 내부에 존재할 경우, manipulator impedance만 수행
        if(distance<=inner_space){
            // for (auto it = filteredPriority.begin(); it != filteredPriority.end();++it){
                // if (std::get<0>(*it) == mobileTaskNum) {
                //     it = filteredPriority.erase(it);
                //     break;
                // }
            // }
            for (auto it = filteredPriority.begin(); it != filteredPriority.end();++it){
                if (std::get<0>(*it) == poseTaskNum) {
                    it = filteredPriority.erase(it);
                    // std::get<1>(*it) = 0.9; // alpha_ii 값 변경 
                    break;
                }
            }
        }
        //inner space와 workspace 사이에 존재할 경우, manipulator, mobile 모두 수행
        else if(distance<=work_space){
            // 특정 alpha값 soft값으로 변경 
            // for (auto it = filteredPriority.begin(); it != filteredPriority.end();++it){
            //     if (std::get<0>(*it) == manipulatorTaskNum) {
            //         std::get<1>(*it) = 0.5; // alpha_ii 값 변경 
            //         std::get<2>(*it) = 0.8; // alpha_ij 값 변경 
            //         break;
            //     }
            // }
            
        }

        //workspace 외부(outer space)에 존재할 경우, mobile impedance만 수행 
        else{
            for (auto it = filteredPriority.begin(); it != filteredPriority.end();++it){
                if (std::get<0>(*it) == manipulatorTaskNum) {
                    it = filteredPriority.erase(it);
                    break;
                }
            }
        }
    }
    
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

    for(int i=0; i<7;i++){
        if(lbq(i) + jl_alpha(i) < q(i) && ubq(i) - jl_alpha(i) > q(i)){
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
    //priorityInput(사용자 지정) 값을 바탕으로, 환경 고려하여 filteredPriority 생성
    const int mani = 1;
    const int mobile = 2;
    const int pose = 0;
    const std::vector<int> obstacle = {4,5,6,7}; // 0, 1, 3, 5 순서대로 
    const std::vector<int> jointLimitTaskNums = {8,9,10,11,12,13,14};

    setTrackingPriority(mani,mobile,pose);
    setObstaclePrirority(obstacle);
    setJointLimitPriority(jointLimitTaskNums);

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
    if(t<=duration){
        for(int i=0; i<desiredAlphas.size(); i++){
        
            //increase alpha 
            if(desiredAlphas[i]>currentValues[i]){
                
                // currentValues[i] = prevDesiredAlphas[i] +(0.5*(M_PI/duration)*sin((M_PI/duration)*t))*dt*scale;
                currentValues[i] += (0.5*(M_PI/duration)*sin((M_PI/duration)*t))*dt*scaleValues[i];

                if(currentValues[i]>desiredAlphas[i]){
                    currentValues[i] = desiredAlphas[i];
                }
            }

            //decrease alpha
            else if(desiredAlphas[i]<currentValues[i]){

                // currentValues[i] = prevDesiredAlphas[i] - (0.5*(M_PI/duration)*sin((M_PI/duration)*t))*dt*scale;
                currentValues[i] -= (0.5*(M_PI/duration)*sin((M_PI/duration)*t))*dt*scaleValues[i];

                if(currentValues[i]<desiredAlphas[i]){
                    currentValues[i] = desiredAlphas[i];
                }
            }
            else{}
        }
    }
    

    const double epsilon = 0.001;
    //모든 alpha값에 대한 변경이 완료됐는지 확인
    if(alphasSetDone(currentValues,desiredAlphas,epsilon)){
        // printf("\n************** change is done !! **************\n");    
 
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

    toqT = allProjections[0]*j0.transpose()*wrench0 
    +allProjections[1]*jrimpT.transpose()*wrenchImpT
    +allProjections[2]*jmimp.transpose()*wrenchmImp
    +allProjections[3]*jimp.transpose()*wrenchImp
    +allProjections[4]*jobs0.transpose()*wrenchObs0 
    +allProjections[5]*jobs1.transpose()*wrenchObs1
    +allProjections[6]*jobs4.transpose()*wrenchObs4
    +allProjections[7]*jobs7.transpose()*wrenchObs7 
    +allProjections[8]*jjl1.transpose()*wrenchjl[0]
    +allProjections[9]*jjl2.transpose()*wrenchjl[1]
    +allProjections[10]*jjl3.transpose()*wrenchjl[2]
    +allProjections[11]*jjl4.transpose()*wrenchjl[3]
    +allProjections[12]*jjl5.transpose()*wrenchjl[4]
    +allProjections[13]*jjl6.transpose()*wrenchjl[5]
    +allProjections[14]*jjl7.transpose()*wrenchjl[6]; 
    // toqT = allProjections[0]*j0.transpose()*wrench0 + allProjections[1]*jmimp.transpose()*wrenchmImp;
      //std::cout<<"wrenchimpt: "<<std::endl<<wrenchImpT.transpose()<<std::endl;
     
     if(isnan(toqT(0))) toqT = Eigen::VectorXd::Zero(10,1);
    //  std::cout<<"toqT: " <<std::endl<<toqT<<std::endl;
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

int dghc_controller::run(){
    numberOfTasks = getNumTasks();
    scaleValues.assign(numberOfTasks*(numberOfTasks+1)*0.5,1);
    auto logger = XBot::MatLogger2::MakeLogger("/home/robot/log/log_230905.mat");
    auto appender = XBot::MatAppender::MakeInstance();
    appender->add_logger(logger);
    appender->start_flush_thread();
   
    ros::Time current_time, last_time;
    ros::Time start_time;
    start_time = ros::Time::now();

    ros::Rate loop_rate(1000);
    while(ros::ok())
    {    
         
        
        // std::cout<<"d_dot: "<<d_dot[37]<<std::endl;
        current_time = ros::Time::now();
        dt = (current_time - last_time).toSec();
        current_sec = (current_time).toSec();
        play_time = (current_time - start_time).toNSec()/1000000;
        logger->add("mobile_position", mobile_pose2d);
        logger->add("mobile_pose", euler_m);
        logger->add("q", q);
        logger->add("d_position", d_position);
        logger->add("d_position2d", d_position2d);
        logger->add("position7", position7);
        logger->add("d_euler", d_euler);
        logger->add("euler_e", euler_e);
        
        logger->add("d08", d[0][8]); //foot - mobile  (right)
        logger->add("d010", d[0][10]); //foot - mobile (left)
        logger->add("d24", d[2][4]); //right hand - elbow  
        logger->add("d34", d[3][4]); //right hand - endeffector  
        logger->add("d_dot08", d_dot[8]);
        logger->add("d_dot010", d_dot[10]);
        logger->add("d_dot24", d_dot[26]); //right hand - elbow
        logger->add("d_dot34", d_dot[37]);
        logger->add("robot127", distance);
        
        logger->add("f0", f02d);
        logger->add("f1", f1);
        logger->add("f4", f4);
        logger->add("f7", f7);

        logger->add("wrenchObs0", wrenchObs0);
        logger->add("wrenchObs1", wrenchObs1);
        logger->add("wrenchObs4", wrenchObs4);
        logger->add("wrenchObs7", wrenchObs7);


        logger->add("risk34", riskFactorArray[37]);
        logger->add("risk08", riskFactorArray[8]);
        logger->add("risk010", riskFactorArray[10]);
        logger->add("risk23", riskFactorArray[25]);
        logger->add("risk24", riskFactorArray[26]);

        logger->add("play_time", play_time);
        logger->add("wrenchImpT", wrenchImpT);
        logger->add("mobile_twist", mobile_twist);
        logger->add("twsit7", twist7);

        logger->add("modeInput", modeInput);
        logger->add("coop_mode", coop_mode);
        logger->add("coop_tar", coop_tar);
        logger->add("mobile_target", mobile_target);

        logger->add("a00",a00);
        logger->add("a11",a11);
        logger->add("a01",a01);

        logger->add("a55",a55);
        logger->add("a05",a05);
        logger->add("a15",a15);

        logger->add("a66",a66);
        logger->add("a06",a06);
        logger->add("a16",a16);

        logger->add("a77",a77);
        logger->add("a07",a07);
        logger->add("a17",a17);

        logger->add("a44",a44);
        logger->add("a24",a24);

        logger->add("a45",a45);
        logger->add("a46",a46);
        logger->add("a47",a47);
        logger->add("a56",a56);
        logger->add("a57",a57);
        logger->add("a67",a67);


         getModel();

         getWrench();

         getJacobian();

         setPriority();

         setInertia();

         getProjectionM();

         getProjectedToq();
        
        //mode0 - plot
        //task : jointPose(0), mobile_impedance(2), obstacle_avoidance(4~7), joint_limit(8~14)
        // logger->add("a00",a00);
        // logger->add("a22",a22);
        // logger->add("a02",a02);

      

        // logger->add("a55",a55);
        // logger->add("a05",a24);
        // logger->add("a25",a24);
        
        // logger->add("a66",a66);
        // logger->add("a06",a06);

        // logger->add("a77",a77);
        // logger->add("a07",a07);

        // logger->add("a88",a88);
        // logger->add("a99",a99);
        // logger->add("a10_10",a10_10);
        // logger->add("a11_11",a11_11);
        // logger->add("a12_12",a12_12);
        // logger->add("a13_13",a13_13);
        // logger->add("a14_14",a14_14);

       

        vir_force = R_m.transpose()*toqT.block(0,0,3,1); 
        //   std::cout <<"vir_force:"<<std::endl <<vir_force << std::endl;
        //   std::cout <<"toq: "<<std::endl <<toqT.block(3,0,7,1) << std::endl;
        //   std::cout <<"Rm: "<<std::endl <<R_m << std::endl;
        toq = toqT.block(3,0,7,1) + arm_interface_->getCoriolisCompensation();
        
        toq << saturateTorqueRate(toq, arm_tau_J_d);
        
        checkDeadzone(vir_force(0), LINEAR);
        checkDeadzone(vir_force(1), LINEAR);
        checkDeadzone(vir_force(2), ANGULAR);
        
        geometry_msgs::Wrench vir_torque;

        vir_torque.force.x = vir_force(0);
        vir_torque.force.y = vir_force(1);
        vir_torque.torque.z = vir_force(2);
        Vir_torque.publish(vir_torque);

        // Limiting maximun torque values of the ARM
    for (size_t i = 3; i < 7; ++i) {
        if(toq(i) > 87){
            toq(i) = 87;
        }
    }
    for (size_t i = 7; i < 10; ++i) {
        if(toq(i) > 12){
            toq(i) = 12;
        }
    }
    //  std::cout<<"toq_limited: "<<toq.transpose()<<std::endl;
    // Set joint commands and publish virtual forces
    //toq<<0,0,0,0,0,0,0;
    //toq = toq+arm_interface_->getCoriolisCompensation();
     arm_interface_->setJointCommands(toq);

        
        
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
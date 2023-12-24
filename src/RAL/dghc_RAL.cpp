#include "WHQP.h"
#include "GHCProjections.hpp"
#include "dghc_controller_RAL.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_consistent_generalized_hierarchical_constroller");
  
  ros::AsyncSpinner spinner(10);
  spinner.start();
    
  enum TaskNum{
    JOINT_POSE=0,
    WHOLE_BODY_IMPEDANCE,
    MOBILE_IMPEDANCE,
    HAND_GUAIDANCE,
    JOINT_LIMIT1,
    JOINT_LIMIT2,
    JOINT_LIMIT3,
    JOINT_LIMIT4,
    JOINT_LIMIT5,
    JOINT_LIMIT6,
    JOINT_LIMIT7,
    OBSTACLE_AVOIDANCE_0=11,
    OBSTACLE_AVOIDANCE_1,
    OBSTACLE_AVOIDANCE_4,
    OBSTACLE_AVOIDANCE_7=14,
    
  };
 
  int DOFsize = 10;
  int numTasks = 15;
  Eigen::VectorXd tasksize;
  tasksize = Eigen::VectorXd::Zero(numTasks);
  
  tasksize[JOINT_POSE] = 7; 
  tasksize[WHOLE_BODY_IMPEDANCE] = 6; 
  tasksize[MOBILE_IMPEDANCE] = 3; 
  tasksize[HAND_GUAIDANCE] = 6; 

  tasksize[JOINT_LIMIT1] = 1; 
  tasksize[JOINT_LIMIT2] = 1; 
  tasksize[JOINT_LIMIT3] = 1; 
  tasksize[JOINT_LIMIT4] = 1; 
  tasksize[JOINT_LIMIT5] = 1;
  tasksize[JOINT_LIMIT6] = 1; 
  tasksize[JOINT_LIMIT7] = 1; 
  tasksize[OBSTACLE_AVOIDANCE_0] = 1;
  tasksize[OBSTACLE_AVOIDANCE_1] = 1;
  tasksize[OBSTACLE_AVOIDANCE_4] = 1;
  tasksize[OBSTACLE_AVOIDANCE_7] = 1;
  
  dghc_controller force_node;
  force_node.init(numTasks, tasksize, DOFsize);
  

  force_node.run();
  spinner.stop();
 
  return 0;
}


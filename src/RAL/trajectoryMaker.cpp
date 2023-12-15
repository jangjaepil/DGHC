#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <std_msgs/Int16.h>
#include "GHCProjections.hpp"
#include <math.h>

geometry_msgs::Pose desired_trajectory;
geometry_msgs::Pose current_pose;



Eigen::VectorXd snap_pose = Eigen::VectorXd::Zero(6);
bool flag1=0;
bool flag2=0;
int STEP = 0;
int PastSTEP =-1;
double snap_secs = 0;

void current_pose_callback(const geometry_msgs::Pose & pose)
{
    current_pose.position.x = pose.position.x;
    current_pose.position.y = pose.position.y;
    current_pose.position.z = pose.position.z;
    current_pose.orientation.x = pose.orientation.x;
    current_pose.orientation.y = pose.orientation.y;
    current_pose.orientation.z = pose.orientation.z;
    current_pose.orientation.w = pose.orientation.w;
    flag1 = 1;
}
void current_step_callback(const std_msgs::Int16 & step)
{
    STEP = step.data;
    flag2 = 1;
}

void snap_current(const geometry_msgs::Pose pose,int CurrentSTEP,double current_secs)
{
   
    if(PastSTEP != CurrentSTEP)
    {   
        snap_pose(0) = pose.position.x;
        snap_pose(1) = pose.position.y;
        snap_pose(2) = pose.position.z;
        snap_pose(3) = pose.orientation.x;
        snap_pose(4) = pose.orientation.y;
        snap_pose(5) = pose.orientation.z;
        snap_pose(6) = pose.orientation.w;
        PastSTEP = CurrentSTEP;
        snap_secs = current_secs;
    }
     
}

void Maketrajectory(int CurrentSTEP,double current_secs)
{
    double gain = 0.1;
    double period = 10;
  
    
    if(CurrentSTEP == 1)
    {
        desired_trajectory.position.x = snap_pose(0);  
        desired_trajectory.position.y = snap_pose(1) + gain*sin(((2*M_PI)/period)*(current_secs - snap_secs));
        desired_trajectory.position.z = snap_pose(2);
        desired_trajectory.orientation.x = snap_pose(3);
        desired_trajectory.orientation.y = snap_pose(4);
        desired_trajectory.orientation.z = snap_pose(5);
        desired_trajectory.orientation.w = snap_pose(6);
    }
    else
    {
        desired_trajectory.position.x = snap_pose(0);  
        desired_trajectory.position.y = snap_pose(1);
        desired_trajectory.position.z = snap_pose(2);
        desired_trajectory.orientation.x = snap_pose(3);
        desired_trajectory.orientation.y = snap_pose(4);
        desired_trajectory.orientation.z = snap_pose(5);
        desired_trajectory.orientation.w = snap_pose(6);
    }
}
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "deisred_trajectory");
    ros::NodeHandle nh;
    ros::Publisher trajectoryPub = nh.advertise<geometry_msgs::Pose>("/desired_pose", 1000);
    ros::Subscriber currentposeSub = nh.subscribe("/current_pose", 1000, current_pose_callback);  
    ros::Subscriber STEPSub = nh.subscribe("/current_step", 1000, current_step_callback);
    ros::Rate loop_rate(1000);

 
 
    while(ros::ok())
    {                    
        if(flag1 ==1 && flag2 ==1)
        {    
            double secs =ros::Time::now().toSec();
            std::cout<<"currentSTEP: "<<STEP<<" pastStep: "<<PastSTEP<<std::endl;
            snap_current(current_pose,STEP,secs);
           
            Maketrajectory(STEP,secs);
          
            trajectoryPub.publish(desired_trajectory);
            //std::cout<<"communication connected!"<<std::endl;
        }
       
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
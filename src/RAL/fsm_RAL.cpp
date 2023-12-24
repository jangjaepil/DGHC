#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <ros/time.h>
enum TaskNum{
    JOINT_POSE=0,
    WHOLE_BODY_IMPEDANCE=1,
    MOBILE_IMPEDANCE=2,
    HAND_GUIDANCE=3,
    JOINT_LIMIT1=4,
    JOINT_LIMIT2,
    JOINT_LIMIT3,
    JOINT_LIMIT4,
    JOINT_LIMIT5,
    JOINT_LIMIT6,
    JOINT_LIMIT7=10,
    OBSTACLE_AVOIDANCE_0=11,
    OBSTACLE_AVOIDANCE_1,
    OBSTACLE_AVOIDANCE_3,
    OBSTACLE_AVOIDANCE_5=14,
};

ros::Time reset_start_time;
std_msgs::String priorityInput;
std_msgs::Int16 currentStep;
std_msgs::Bool reset_demo;
std_msgs::Float64 grapInput;

bool startFlag = false;
bool picked = false;
bool arrived1 = false;
bool arrived2 = false;
bool arrived5 = false;
bool placed = false;
bool distance_flag = false; // d<d_th
bool force_flag = false; // f>f_th
bool in_return_place =false;
const double RETURN_X = 0.0;
const double RETURN_Y =  1.0;
const double RETURN_MARGIN = 0.3;

void start_callback(const std_msgs::Bool::ConstPtr& msg)
{
    startFlag = msg->data;
}

void picked_callback(const std_msgs::Bool::ConstPtr& msg)
{
    picked = msg->data;
}

void arrived_callback1(const std_msgs::Bool::ConstPtr& msg)
{
    arrived1 = msg->data;
}

void arrived_callback2(const std_msgs::Bool::ConstPtr& msg)
{
    arrived2 = msg->data;
}

void arrived_callback5(const std_msgs::Bool::ConstPtr& msg)
{
    arrived5 = msg->data;
}

void placed_callback(const std_msgs::Bool::ConstPtr& msg)
{
    placed = msg->data;
}

void distance_flag_callback(const std_msgs::Bool::ConstPtr& msg)
{
    distance_flag = msg->data;
}

void force_flag_callback(const std_msgs::Bool::ConstPtr& msg)
{
    force_flag = msg->data;
}

void current_pose_callback(const geometry_msgs::Pose::ConstPtr& msg){
    if(currentStep.data == 3 || currentStep.data == 4){
        double errX = abs(msg->position.x - RETURN_X);
        double errY = abs(msg->position.y - RETURN_Y);
        if(errX <RETURN_MARGIN && errY < RETURN_MARGIN){
            in_return_place=true;
        }
        else{
            in_return_place=false;
        }
    }
}

int main(int argc, char** argv) 
{


    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh;

    ros::Subscriber start_sub  = nh.subscribe("/start", 1000, start_callback);
    ros::Subscriber picked_sub = nh.subscribe("/picked", 1000, picked_callback);
    ros::Subscriber arrived_sub1 = nh.subscribe("/arrived1", 1, arrived_callback1);
    ros::Subscriber arrived_sub2 = nh.subscribe("/arrived2", 1, arrived_callback2);
    ros::Subscriber arrived_sub5 = nh.subscribe("/arrived5", 1, arrived_callback5);
    ros::Subscriber d_th_sub = nh.subscribe("/distance_flag", 1000, distance_flag_callback);
    ros::Subscriber f_th_sub = nh.subscribe("/force_flag", 1000, force_flag_callback);
    ros::Subscriber placed_sub = nh.subscribe("/placed", 1000, placed_callback);
    ros::Subscriber current_pose_sub = nh.subscribe("/current_pose",1000,current_pose_callback);

    ros::Publisher priorityPub = nh.advertise<std_msgs::String>("/priorityInput", 1000);
    ros::Publisher stepPub = nh.advertise<std_msgs::Int16>("/current_step", 1000);
    ros::Publisher resetPub = nh.advertise<std_msgs::Bool>("/reset", 1000);
    ros::Publisher grapPub = nh.advertise<std_msgs::Float64>("/qb_interface/grap", 1000);
    
    reset_demo.data=false;

    ros::Rate loop_rate(1000);
    while(ros::ok())
    {  
        if(reset_demo.data){
            currentStep.data = 0;
            reset_start_time = ros::Time::now();
            while(ros::Time::now()-reset_start_time < ros::Duration(3.0)){
                currentStep.data = 0;
                priorityInput.data = "4 5 6 7 8 9 11 12 13 14 2 1";
                priorityPub.publish(priorityInput);
                stepPub.publish(currentStep);
                std::cout << "reset time : " << ros::Time::now()-reset_start_time << std::endl;
                ros::spinOnce();
                loop_rate.sleep();
            }
            reset_demo.data = false;
        }
        
        if(!startFlag)
        {
            currentStep.data = 0;
        }
        switch (currentStep.data)
        {
            // Equal to Mode 0
            case 0:
                grapInput.data = 0.05;
                grapPub.publish(grapInput);
                priorityInput.data = "4 5 6 7 8 9 11 12 13 14 2 1";
                if(startFlag){
                    currentStep.data = 1;
                }
                break;
            // Pick the tool 
            case 1:
                priorityInput.data = "4 5 6 7 8 9 11 12 13 14 2 1"; // initial priority, trajectory for each steps
                if(arrived1){
                    grapInput.data = 0.95;
                    grapPub.publish(grapInput);
                }
                if(picked){
                    currentStep.data = 2;
                }
                break;
            // Move to Human 
            case 2:
                priorityInput.data = "4 5 6 7 8 9 11 12 13 14 2 1"; //
                if(arrived2){
                    currentStep.data = 3;
                }
                break;
            // Holding Pose
            case 3:
                priorityInput.data = "4 5 6 7 8 9 11 12 13 14 2 0"; //spring damping
                if(distance_flag && force_flag){
                    currentStep.data = 4;
                }
                if(in_return_place && !distance_flag){
                    currentStep.data = 5;
                }
                break;
            // Hand Guidance 
            case 4:
                priorityInput.data = "4 5 6 7 8 9 11 12 13 14 2 3"; 
                if(!distance_flag){
                    currentStep.data = 3;
                }
                
                if(in_return_place && !distance_flag){
                    currentStep.data = 5;
                }
                break;
            // Place the tool 
            case 5:
                priorityInput.data = "4 5 6 7 8 9 11 12 13 14 2 1"; // 
                if(arrived5){
                    grapInput.data = 0.05;
                    grapPub.publish(grapInput);
                }
                if(placed&&arrived5){
                    reset_demo.data = true;
                    startFlag = false;
                    arrived1 = false;
                    arrived2 = false;
                    arrived5 = false;
                    picked = false;
                    placed = false;
                    distance_flag = false; 
                    force_flag = false; 
                    in_return_place =false;
                    currentStep.data = 0;
                }
                break;

            default:
                break;
        }
        
        priorityPub.publish(priorityInput);
        stepPub.publish(currentStep);
        resetPub.publish(reset_demo);

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
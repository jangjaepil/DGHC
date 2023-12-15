#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <ros/time.h>
enum TaskNum{
    JOINT_POSE=0,
    WHOLE_BODY_IMPEDANCE,
    MOBILE_IMPEDANCE,
    HAND_GUIDANCE,
    JOINT_LIMIT1,
    JOINT_LIMIT2,
    JOINT_LIMIT3,
    JOINT_LIMIT4,
    JOINT_LIMIT5,
    JOINT_LIMIT6,
    JOINT_LIMIT7
};


bool startFlag = false;
bool picked = false;
bool arrived = false;
bool placed = false;
bool button = false;
bool distance_flag = false; // d<d_th
bool force_flag = false; // f>f_th


void start_callback(const std_msgs::Bool::ConstPtr& msg)
{
    startFlag = msg->data;
}

void picked_callback(const std_msgs::Bool::ConstPtr& msg)
{
    picked = msg->data;
}

void arrived_callback(const std_msgs::Bool::ConstPtr& msg)
{
    arrived = msg->data;
}
void placed_callback(const std_msgs::Bool::ConstPtr& msg)
{
    placed = msg->data;
}

void button_callback(const std_msgs::Bool::ConstPtr& msg)
{
    button = msg->data;
}

void distance_flag_callback(const std_msgs::Bool::ConstPtr& msg)
{
    distance_flag = msg->data;
}

void force_flag_callback(const std_msgs::Bool::ConstPtr& msg)
{
    force_flag = msg->data;
}


int main(int argc, char** argv) 
{


    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh;

    ros::Subscriber start_sub  = nh.subscribe("/start", 1000, start_callback);
    ros::Subscriber picked_sub = nh.subscribe("/picked", 1000, picked_callback);
    ros::Subscriber arrived_sub = nh.subscribe("/arrived", 1000, arrived_callback);
    ros::Subscriber d_th_sub = nh.subscribe("/distance_flag", 1000, distance_flag_callback);
    ros::Subscriber f_th_sub = nh.subscribe("/force_flag", 1000, force_flag_callback);
    ros::Subscriber button_sub = nh.subscribe("/button", 1000, button_callback);
    ros::Subscriber placed_sub = nh.subscribe("/placed", 1000, placed_callback);

    ros::Publisher priorityPub = nh.advertise<std_msgs::String>("/priorityInput", 1000);
    ros::Publisher stepPub = nh.advertise<std_msgs::Int16>("/current_step", 1000);
    ros::Publisher resetPub = nh.advertise<std_msgs::Bool>("/reset", 1000);

    ros::Time reset_start_time;
    std_msgs::String priorityInput;
    std_msgs::Int16 currentStep;
    std_msgs::Bool reset_demo;
    reset_demo.data=false;

    ros::Rate loop_rate(1000);
    while(ros::ok())
    {  
        if(reset_demo.data){
            currentStep.data = 0;
            reset_start_time = ros::Time::now();
            while(ros::Time::now()-reset_start_time < ros::Duration(3.0)){
                currentStep.data = 0;
                priorityInput.data = "4 5 6 7 8 9 10 2 0";
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
                priorityInput.data = "4 5 6 7 8 9 10 2 0";
                if(startFlag){
                    currentStep.data = 1;
                }
                break;
            // Pick the tool 
            case 1:
                priorityInput.data = "4 5 6 7 8 9 10 2 1";
                if(picked){
                    currentStep.data = 2;
                }
                break;
            // Move to Human 
            case 2:
                priorityInput.data = "4 5 6 7 8 9 10 2 1";
                if(arrived){
                    currentStep.data = 3;
                }
                break;
            // Holding Pose
            case 3:
                priorityInput.data = "4 5 6 7 8 9 10 2 0";
                if(distance_flag && force_flag){
                    currentStep.data = 4;
                }
                if(button){
                    currentStep.data = 5;
                }
                break;
            // Hand Guidance 
            case 4:
                priorityInput.data = "4 5 6 7 8 9 10 2 3";
                if(!distance_flag || !force_flag){
                    currentStep.data = 3;
                }
                
                if(button){
                    currentStep.data = 5;
                }
                break;
            // Place the tool 
            case 5:
                priorityInput.data = "4 5 6 7 8 9 10 2 1";
                
                if(placed){
                    reset_demo.data = true;
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
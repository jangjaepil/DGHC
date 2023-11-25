#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
 enum TaskNum{
    JOINT_POSE=0,
    WHOLE_BODY_IMPEDANCE,
    MOBILE_IMPEDANCE,
    JOINT_LIMIT1,
    JOINT_LIMIT2,
    JOINT_LIMIT3,
    JOINT_LIMIT4,
    JOINT_LIMIT5,
    JOINT_LIMIT6,
    JOINT_LIMIT7
  };

bool modeInput = false;
// std_msgs::Bool input;



void mode_input_callback(const std_msgs::Bool::ConstPtr& mode_input){
    modeInput = mode_input->data;
    // input.data =  mode_input->data;
}



int main(int argc, char** argv) 
{
    ros::init(argc, argv, "mode_pub_node");
    ros::NodeHandle nh;

    // ros::Publisher modePub = nh.advertise<std_msgs::Bool>("/modeData", 500);
    ros::Publisher priorityPub = nh.advertise<std_msgs::String>("/priorityInput", 500);
    ros::Subscriber mode_sub  = nh.subscribe("/modeInput", 1000, mode_input_callback);

    std_msgs::String msg;
    ros::Rate loop_rate(1000);
    while(ros::ok())
    {  
        //mode0
        if(!modeInput){
            msg.data = "3 4 5 6 7 8 9 2 1";
        }
        //mode1
        else{
            msg.data = "3 4 5 6 7 8 9 2 1";
        }

        // modePub.publish(input);
        priorityPub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
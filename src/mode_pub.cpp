#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
 enum TaskNum{
    JOINT_POSE=0,
    MANIPULATOR_IMPEDANCE,
    MOBILE_IMPEDANCE,
    WHOLE_BODY_IMPEDANCE,
    OBSTACLE_AVOIDANCE_0=4,
    OBSTACLE_AVOIDANCE_1,
    OBSTACLE_AVOIDANCE_3,
    OBSTACLE_AVOIDANCE_5=7,
    JOINT_LIMIT1=8,
    JOINT_LIMIT2,
    JOINT_LIMIT3,
    JOINT_LIMIT4,
    JOINT_LIMIT5,
    JOINT_LIMIT6,
    JOINT_LIMIT7=14,
  };
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "mode_pub_node");
    ros::NodeHandle nh;

    ros::Publisher modePub = nh.advertise<std_msgs::Bool>("/modeInput", 500);
    ros::Publisher priorityPub = nh.advertise<std_msgs::String>("/priorityInput", 500);

    std_msgs::String msg;
    std_msgs::Bool input;
    input.data = false;
    int inputValue;

    while(ros::ok()){ 

        std::cout << "Enter 0 or 1: ";
        std::cin >> inputValue;

        if (std::cin.fail()) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input. Please enter a valid number ( 0 or 1 )" << std::endl;
            continue;
        }

        if (inputValue == 0 || inputValue == 1)
        {
            input.data = (inputValue == 0) ? false : true;
            if(!input.data) {
                msg.data = "8 9 10 11 12 13 14 4 5 6 7 0 2";
            }
            else {
                msg.data = "8 9 10 11 12 13 14 4 5 6 7 1 0 2";
            }

            modePub.publish(input);
            priorityPub.publish(msg);
        }
        else {
            std::cout << "Invalid input. Please enter 0 or 1." << std::endl;
        }
        ros::spinOnce();
    }
    
    return 0;
}
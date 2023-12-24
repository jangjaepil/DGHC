#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

ros::Publisher picked_publisher_;
ros::Publisher placed_publisher_;
std_msgs::Bool picked_msg_; 
std_msgs::Bool placed_msg_;
std_msgs::Int16 current_step_msg_;

class qbInterface {
public:
    qbInterface() {
        grap_subscriber_ = nh_.subscribe("/qb_interface/grap", 1, &qbInterface::grapCallback, this);
        reset_subscriber_ = nh_.subscribe("/reset",1000, &qbInterface::reset_callback, this);
        goal_publisher_ = nh_.advertise<control_msgs::FollowJointTrajectoryActionGoal>(
            "/qbhand1/control/qbhand1_synergy_trajectory_controller/follow_joint_trajectory/goal", 1);

        result_subscriber_ = nh_.subscribe("/qbhand1/control/qbhand1_synergy_trajectory_controller/follow_joint_trajectory/result", 1, &qbInterface::resultCallback, this);

        picked_publisher_ = nh_.advertise<std_msgs::Bool>("/picked", 1);
        placed_publisher_ = nh_.advertise<std_msgs::Bool>("/placed", 1);
        current_step_subscriber_ = nh_.subscribe("/current_step", 1, &qbInterface::current_step_callback, this);
        
    }

    // Callback for the pick topic
    void grapCallback(const std_msgs::Float64::ConstPtr& msg) {
        
        if(current_goal_ != msg->data){
            current_goal_ = msg->data;
            setGoal(current_goal_);
        }
    }

    void current_step_callback(const std_msgs::Int16::ConstPtr& msg){
        current_step_msg_.data = msg->data;
    }

    // Callback for the result topic (only one time per one goal)
    void resultCallback(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& result) {
        // if ( physical_goal_==GRAP){
        //     if( result->status.status == 3) {
        //         picked_msg_.data = true;
        //     }
        //     else{
        //         picked_msg_.data = false;
        //     }
        // else{
        //     if(result->status.status == 3) {
        //         placed_msg_.data = true;
        //     }
        //     else{
        //         placed_msg_.data = false;
        //     }
        // }
        if(current_step_msg_.data == 1){
            if( result->status.status == 3) {
                picked_msg_.data = true;
            }
            else{
                picked_msg_.data = false;
            }
        }
        else if(current_step_msg_.data == 5){
            if(result->status.status == 3) {
                placed_msg_.data = true;
            }
            else{
                placed_msg_.data = false;
            }
        }
    }

    // Set the goal with the specified position
    void setGoal(double position) {
        if(position < 0.0 || position > 1.0) {
            ROS_ERROR("Position must be between 0.0 and 1.0");
            return;
        }
        if(position > 0.5){
            physical_goal_ = GRAP;
        }
        else{
            physical_goal_ = RELEASE;
        }

        control_msgs::FollowJointTrajectoryActionGoal goal_msg;

        goal_msg.goal.trajectory.joint_names.push_back("qbhand1_synergy_joint");
        goal_msg.goal.trajectory.points.resize(1); // Only one point in this trajectory
        goal_msg.goal.trajectory.points[0].positions.push_back(position);
        goal_msg.goal.trajectory.points[0].velocities.push_back(0);
        goal_msg.goal.trajectory.points[0].accelerations.push_back(0);
        goal_msg.goal.trajectory.points[0].effort.push_back(0);
        goal_msg.goal.trajectory.points[0].time_from_start = ros::Duration(3.0); // 3 seconds to reach the goal


        goal_publisher_.publish(goal_msg);
    }

    void reset_callback(const std_msgs::Bool &msg){
        if(msg.data){
            picked_msg_.data = false;
            placed_msg_.data = false;
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber grap_subscriber_;
    ros::Subscriber reset_subscriber_;
    ros::Publisher goal_publisher_;
    ros::Subscriber result_subscriber_;
    ros::Subscriber current_step_subscriber_;


    double current_goal_;
    
    enum { GRAP, RELEASE } physical_goal_;
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "qb_interface_node");
    qbInterface qb_interface;

    ros::Rate loop_rate(1000);
    while(ros::ok()){
        picked_publisher_.publish(picked_msg_);
        placed_publisher_.publish(placed_msg_);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}

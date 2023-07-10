#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <fstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::PoseStamped this_pose;

double distance(move_base_msgs::MoveBaseGoal &goal)
{
    double dx = this_pose.pose.position.x - goal.target_pose.pose.position.x;
    double dy = this_pose.pose.position.y - goal.target_pose.pose.position.y;
    double dz = this_pose.pose.position.z - goal.target_pose.pose.position.z;
    double distance = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
    return distance;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    this_pose.pose.position = odom->pose.pose.position;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_point_inspection");
    ros::NodeHandle nh;
    MoveBaseClient ac("move_base", true);

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);
    while(!ac.waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the move_base action server to come up");

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Reach goal");
        else
            ROS_INFO("Failed to reach goal");
        if(distance(goal) < 0.5)
        {

        }

        loop_rate.sleep();
    }
    return 0;
}


#include "move_base/ai_robot_ranger.h"
#include <ros/ros.h>

#include <exemple_behavior/goal_behavior.h>
using namespace std;

int main(int argc, char **argv){
    ros::init(argc, argv, "ai_robor_ranger");
    string full;

    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        switch (task_number){
            case HOVER_TASK:
                break;
            case TAKEOFF_TASK:
                break;
            case LAND_TASK:
                break;
            case MOVE_TASK:
                break;
            case FINDPATH_TASK:
                break;
            case POINTTOPOINT_TASK:
                break;
            case FAR_POINTTOPOINT_TASK:
                break;
            default:
                break;
        }
        rate.sleep();
    }
    return 0;
}
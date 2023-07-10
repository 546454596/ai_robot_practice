
#ifndef _AIBRAIN_H
#define _AIBRAIN_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <ros/callback_queue.h>

//#include "opencv/cv.hpp"

#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Path.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/OpticalFlowRad.h"
#include "ai_robot_interfaces/set_destination.h"

#include "PIDController.h"
#include "pid_fuzzy.h"
#include "ADRC.h"
#include "postrajectory.h"
#include "assistMath.h"
#include <Eigen/Dense>

//the control target
#include "posControl.h"
//#include "../findpath/findpathsrm.h"
//#include "../pointtopoint/PointToPoint.h"
#include <move_base/posControl.h>
#include <move_base/PointToPoint.h>
#include <move_base/move_base.h>
#include <tf2_ros/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
// transforms
#include <tf2/utils.h>

//task number define
#define HOVER_TASK 0
#define TAKEOFF_TASK 1
#define LAND_TASK 2
#define MOVE_TASK 3
#define FINDPATH_TASK 4
#define POINTTOPOINT_TASK 5
#define FAR_POINTTOPOINT_TASK 6
#define STOP_TASK 7
#define INVALID 8

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class AIBrain
{
public:
    AIBrain(ros::NodeHandle *_nh);
    ~AIBrain();
    void think();
    void findpath(double _dt);
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tf;
    move_base::MoveBase nav;
    move_base_msgs::MoveBaseGoal target;
    MoveBaseClient ac;

private:
    ros::NodeHandle *nh;
    PosControl poseCtrl;
    //point to point
    PointToPoint ptop;
    //pose now
    float poseXYZ_now[3], velXYZ_now[3], roll_now, pitch_now, yaw_now, qwxyz_now[4];
    //pose target
    float poseXYZ_target[3], yaw_target;
    geometry_msgs::Point current_point;
    bool send_next_goal = true;
    int i = 0;
    ros::Time pidTime;

    //wether remote controller has input
    bool isControllerInput;
    //what shall be done, see task number define
    int task_lastnumber, task_number;

    //for hover
    bool isStableHover;
    double hoverStartTime;

    //for storing controller input
    double forbackward_ctrl, leftright_ctrl, updown_ctrl, turnlr_ctrl;

    //for find path
    //0--not started, 1--find path done, 2--on the move, 3--to destiny and should end
    uint8_t state_findpath;
    //regard slam as body frame to use assitMath
    //yawslam + yawworldslam = yawworld
    float Rworldslam[9], Tworldslam[3], yawworldslam;
    //destination position
    float poseXYZ_destination[3];
    //for point to point
    uint8_t state_ptop;
    //for far point to point
    uint8_t state_farptop;
    nav_msgs::Path farptop_path;
    int point_i_farptop, pausetime, cntdown;
    //for test mode of find path
    bool isTestModeOfFindPath;
    //subscriber for controller
    ros::Subscriber joy_sub;
    ros::Subscriber joy_sub2;
    ros::Subscriber stop_task_;
	ros::Publisher obs_abort_;
    ros::Publisher to_end_pos_;
    ros::Subscriber pose_sub;
    //subscriber for destination topic from rviz
    ros::Subscriber move_base_goal_sub, move_base_goal_sub2;
    //subscriber for point to point
    ros::Subscriber ptop_sub;
    //subscriber for far point to point
    ros::Subscriber farptop_sub;
    ros::Subscriber farptop_sub2;
    //add initialize function here
    void initSubscriber();
    //task function here
    void hoverTASK(double _dt);
    void takeoffTASK();
    void landTASK();
    void controllerTASK(double _dt);
//    void findpath(double _dt);
    void pointToPointTASK(double _dt);
    void farPointToPointTASK(double _dt);
    //void remoteStopTask(double _dt);
    void givePosToFindpath();
    //call back for subscriber for controller
    void btn1Callback(const std_msgs::Bool::ConstPtr &_msg);
    void btn3Callback(const std_msgs::Bool::ConstPtr &_msg);
    void axes0lrCallback(const std_msgs::Float64::ConstPtr &_msg);
    void axes0udCallback(const std_msgs::Float64::ConstPtr &_msg);
    void axes1lrCallback(const std_msgs::Float64::ConstPtr &_msg);
    void axes1udCallback(const std_msgs::Float64::ConstPtr &_msg);
    //call back for subscriber for pose,vel
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg);
    //call back for subscriber for void AIBrain::doneCB(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)move_base_goal
    void movebasegoalCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg);
    //point to point subscriber
    void ptopCallback(const nav_msgs::Path::ConstPtr &_msg);
    //far point to point subscriber
    void farptopCallback(const nav_msgs::Path::ConstPtr &_msg);
    //one topic for all joystick input
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
    void stopCallback(const std_msgs::String::ConstPtr &_msg);
    void doneCB(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
    void feedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);
    double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    void toEndPos();
};

#endif

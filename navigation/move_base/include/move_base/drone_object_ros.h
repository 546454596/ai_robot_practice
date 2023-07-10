#ifndef ARDRONE_ROS_H
#define ARDRONE_ROS_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

/**
 * @brief A simple class to send the commands to the drone through 
 * the corresponding topics
 */

class DroneObjectROS{
protected:
    DroneObjectROS(){}
public:
    
    DroneObjectROS(ros::NodeHandle& node):
        isHover(false),
        isSetTrajector(false),
        isSetPoint(false),
        isAutoReturn(false){
        initROSVars(node);
    }

    bool isFlying;
    bool isPosctrl;
    bool isVelMode;
    bool isHover;
    bool isSetTrajector;
    bool isSetPoint;
    bool isAutoReturn;

    
    ros::Publisher pubTakeOff;
    ros::Publisher pubLand;
    ros::Publisher pubReset;
    ros::Publisher pubCmd;
    ros::Publisher pubHoverCmd;
    ros::Publisher pubPosCtrl;
    ros::Publisher pubVelMode;
    ros::Publisher sethover_pub;
    ros::Publisher setpoint_pub;
    ros::Publisher control_settrajectory_pub;
    ros::Publisher autoreturn_startRecord_pub;
    ros::Publisher autoreturn_startReturn_pub;
    
    std_msgs::Float32MultiArray controller_setpoint;
    geometry_msgs::Twist twist_msg;
    
    void initROSVars(ros::NodeHandle& node);
    
    bool takeOff();
    bool land();
    bool hover();
    bool posCtrl(bool on);
    bool velMode(bool on);

    bool setPoint(float_t x,float_t y,float_t z,float_t setYaw,float_t seconds);//control with setHover,relative x and y, globle z
    bool setHover(int _sethover);//start setHover node
    bool setTrajectory(int _settraject);//start trajectory
    bool setAutoreturnRecord(bool _setrecord);//start record
    bool setAutoreturnReturn(bool _setreturn);//start return
    // commands for controling ARDrone
    // pitch_lr = left-right tilt		(-1) to right		(+1)
    // roll_fb = front-back tilt 		(-1) to backwards	(+1)
    // v_du = velocity downwards	(-1) to upwards		(+1)
    // w_lr = angular velocity left (-1) to right		(+1)

    bool hoverMove(float x, float y, float z, float yaw_speed);
    bool move(float v_lr, float v_fb, float v_du, float w_lr);
    bool moveTo(float x, float y, float z);
    bool pitch(float speed = 0.2);
    bool roll(float speed = 0.2);
    bool rise(float speed = 0.1);
    bool yaw(float speed = 0.1);
};

#endif // ARDRONE_ROS_H

#include "move_base/drone_object_ros.h"

void DroneObjectROS::initROSVars(ros::NodeHandle& node){
    isFlying = false;
    isPosctrl = false;
    isVelMode = false;
    pubTakeOff = node.advertise<std_msgs::Empty>("/drone/takeoff",1024);
    pubLand = node.advertise<std_msgs::Empty>("/drone/land",1024);
    pubReset = node.advertise<std_msgs::Empty>("/drone/reset",1024);
    pubPosCtrl = node.advertise<std_msgs::Bool>("/drone/posctrl", 1024);
    pubCmd = node.advertise<geometry_msgs::Twist>("/drone/cmd_val",1024); 
    pubVelMode = node.advertise<std_msgs::Bool>("/drone/vel_mode",1024);
    pubHoverCmd=node.advertise<geometry_msgs::Twist>("/control/velCmd",10);
    sethover_pub=node.advertise<std_msgs::Bool>("/control/sethover",1);
    setpoint_pub=node.advertise<std_msgs::Float32MultiArray>("/control/setpoint",10);
    control_settrajectory_pub=node.advertise<std_msgs::Bool>("/control/settrajectory", 10);
    autoreturn_startRecord_pub=node.advertise<std_msgs::Bool>("/autoreturn/record", 10);
    autoreturn_startReturn_pub=node.advertise<std_msgs::Bool>("/autoreturn/return", 10);
}

bool DroneObjectROS::takeOff(){
    if( isFlying)
        return false;
    
    pubTakeOff.publish(std_msgs::Empty());
    //ROS_INFO("Taking Off...");
    
    isFlying = true;
    return true;    
}

bool DroneObjectROS::land(){
    if( !isFlying)
        return false;
    
    pubLand.publish(std_msgs::Empty());
    //ROS_INFO("Landing...");
    
    isFlying = false;
    return true;
}


bool DroneObjectROS::hover(){
    if( !isFlying)
        return false;
    
    twist_msg.linear.x=0;
    twist_msg.linear.y=0;
    twist_msg.linear.z=0;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z= 0.0;
    
    pubCmd.publish(twist_msg);
    //ROS_INFO("Hovering...");
    return true;
}

bool DroneObjectROS::posCtrl(bool on){
    if( !isFlying)
        return false;

    isPosctrl = on;
    std_msgs::Bool bool_msg;
    bool_msg.data = on ? 1:0;
        
    pubPosCtrl.publish(bool_msg);
    
    if( on)
        ROS_INFO("Switching position control on...");
    else
        ROS_INFO("Switching position control off...");
    
    return true;    
}

bool DroneObjectROS::velMode(bool on){
    if (! isFlying)
        return false;
    
    isVelMode = on;
    std_msgs::Bool bool_msg;
    bool_msg.data = on ? 1:0;
        
    pubVelMode.publish(bool_msg);
    
    if( on)
        ROS_INFO("Switching velocity mode on...");
    else
        ROS_INFO("Switching velocity mode off...");
    
    return true;    
}

bool DroneObjectROS::setHover(int _sethover)
{
    std_msgs::Bool cmd_sethover;
    if(_sethover==1)
    {
        cmd_sethover.data=true;
        isHover=true;
        isSetPoint=false;
        isSetTrajector=false;
        isAutoReturn=false;
        sethover_pub.publish(cmd_sethover);
    }
    else
    {
        cmd_sethover.data=false;
        isHover=false;
        sethover_pub.publish(cmd_sethover);
    }
}

bool DroneObjectROS::setPoint(float_t x,float_t y,float_t z,float_t setYaw,float_t seconds)//in NWU
{
    if (ros::ok())
  {
       controller_setpoint.data.push_back(x);
       controller_setpoint.data.push_back(-y);//change into NEU
       controller_setpoint.data.push_back(z);
       controller_setpoint.data.push_back(setYaw);
       controller_setpoint.data.push_back(seconds);
       setpoint_pub.publish(controller_setpoint);
       controller_setpoint.data.clear();
  }
    else return false;
}

bool DroneObjectROS::setTrajectory(int _settraject)
{
    std_msgs::Bool cmd_settrajector;
    if(_settraject==1){
        cmd_settrajector.data=true;
        isSetTrajector=true;
    }
    else{
        cmd_settrajector.data=false;
        isSetTrajector=false;
    }
    control_settrajectory_pub.publish(cmd_settrajector);

}

bool DroneObjectROS::setAutoreturnRecord(bool _setrecord)
{
    std_msgs::Bool record;
    record.data=_setrecord;
    if (ros::ok())
       autoreturn_startRecord_pub.publish(record);
    else return false;
}

bool DroneObjectROS::setAutoreturnReturn(bool _setreturn)
{
    std_msgs::Bool _return;
    _return.data=_setreturn;
    isAutoReturn=_setreturn;
    if (ros::ok())
       autoreturn_startReturn_pub.publish(_return);
    else return false;
}

bool DroneObjectROS::hoverMove(float x, float y, float z, float yaw_speed)
{
    if (!isFlying)
        return false;

    twist_msg.linear.x=x;
    twist_msg.linear.y=y;
    twist_msg.linear.z=z;
    twist_msg.angular.x=0;
    twist_msg.angular.y=0;
    twist_msg.angular.z= yaw_speed;
    pubHoverCmd.publish(twist_msg);
    return true;

}

bool DroneObjectROS::move(float lr, float fb, float ud, float w) {
    if (!isFlying)
        return false;
    if((lr==0)&&(fb==0)&&(ud==0)&&(w==0)){
        twist_msg.linear.x=0;
        twist_msg.linear.y=0;
        twist_msg.linear.z=0;
        twist_msg.angular.x=0.0;
        twist_msg.angular.y=0.0;
        twist_msg.angular.z= 0.0;
    }
    else{
        twist_msg.linear.x=1.0;
        twist_msg.linear.y=1.0;
        twist_msg.linear.z=ud;
        twist_msg.angular.x=lr;
        twist_msg.angular.y=fb;
        twist_msg.angular.z= w;
    }
    pubCmd.publish(twist_msg);
    //ROS_INFO("Moving...");
    return true;
}

bool DroneObjectROS::moveTo(float x, float y, float z){
    if (!isFlying)
        return false;
    
    twist_msg.linear.x=x;
    twist_msg.linear.y=y;
    twist_msg.linear.z=z;
    twist_msg.angular.x=0;
    twist_msg.angular.y=0;
    twist_msg.angular.z= 0;
    
    pubCmd.publish(twist_msg);
    //ROS_INFO("Moving...");
}

bool DroneObjectROS::pitch(float speed){
    if (!isFlying)
        return false;
    
    twist_msg.linear.x = 1.0;
    twist_msg.linear.y= 1.0;
    twist_msg.linear.z= 0;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=speed;
    twist_msg.angular.z= 0.0;
    pubCmd.publish(twist_msg);
    ROS_INFO("Pitching...");
    return true;
}
bool DroneObjectROS::roll(float speed){
    if (!isFlying)
        return false;
    
    twist_msg.linear.x = 1.0;
    twist_msg.linear.y= 1.0;
    twist_msg.linear.z= 0;
    twist_msg.angular.x=speed;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z= 0.0;
    pubCmd.publish(twist_msg);
    ROS_INFO("Rolling...");
    return true;
}

bool DroneObjectROS::rise(float speed){
    if (!isFlying)
        return false;
    
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = speed;
    twist_msg.angular.x = 0.0;//flag for preventing hovering
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;
    pubCmd.publish(twist_msg);
    ROS_INFO("Rising...");
    return true;
}

bool DroneObjectROS::yaw(float speed){
    if (!isFlying)
        return true;
    
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y= 0.0;
    twist_msg.linear.z= 0;
    twist_msg.angular.x=0.0;//flag for preventing hovering
    twist_msg.angular.y=0.0;
    twist_msg.angular.z= speed;
    pubCmd.publish(twist_msg);
    ROS_INFO("Turning head...");
    return true;
}

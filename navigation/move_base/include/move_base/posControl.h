/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef PosControl_H
#define PosControl_H


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <ros/callback_queue.h>

//#include "opencv/cv.hpp"
#include <opencv2/opencv.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include  "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include  "mavros_msgs/SetMode.h"
#include  "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/OpticalFlowRad.h"

#include "PIDController.h"
#include "pid_fuzzy.h"
#include "ADRC.h"
#include "postrajectory.h"
#include "ARDrone.h"
#include "drone_object_ros.h"
#include "Pioneer3AT.h"
#include "assistMath.h"
// #include "../obstacle_avoid/p3atObsAvoid.h"

#define ARDRONE 0
#define GAZEBO 0
#define PIONEER3AT 1

#define MOVE_BY_ATTITUDE 0
#define MOVE_BY_VELOCITY 1
#define MAX_COUNT_POSITION 20
#define CLIP3(_n1, _n,  _n2) {if (_n<_n1) _n=_n1;  if (_n>_n2) _n=_n2;}
#define PIDCONTROL 0
#define FUZZYPIDCONTROL 1
#define ADRCCONTROL 2
#define DEBUG 0


struct Point {
    float x;
    float y;
    float z;
    float yaw;
    float loiter_time;
};

static void* startHoverThread(void *args);
class PosControl
{
public:
    /// Constructor
    PosControl(ros::NodeHandle *n): //,posTrajectory &trajectory
        threadQuit(false),
        _trajectory(n),
        isControlOutside(false),
        isSetVel(false),
        isSetPoint(false),
        isSetTrajectory(false),
        isVisionHover(false),
        roll_target(0),
        pitch_target(0),
        setHover(false),
        //sonar_new(0),
        //sonar_lp(0),
        pos_record_count(0),
        posX_sum(0),
        posY_sum(0),
        posZ_sum(0),
        isReachPoint(false),
        cotrolWithVicon(true),
        _move_mode(MOVE_BY_VELOCITY),
        latchForVel(false),
        controlRate(50.0f),
        nh(n),
        // p3atOA(*n),
        drone(*n)
    { setup();}

    ~PosControl(void){
        threadQuit=true;
        //delete ardrone;
        vector <Point>().swap(setpointPos);
    }

    bool setup();
    //set state before control
    void setState(float *_posNow, float *_velNow, float _yawNow);
    //for ground robot, takeoff is to enable motor
    void takeOff();
    //for ground robot, land is to disable motor
    void land();
    void doHover(double dt, float *_poseTarget, float _yawTarget,
                 float *_poseNow, float *_velNow, float _yawNow);
    //move command are in world frame
    void doMoveInWorld(double dt, float _vx, float _vy, float _vz, float _vyaw);
    //input all -1 to 1 except dt, only vel input, move command are in body C frame
    void doMove(double dt, float _forback, float _leftright, float _updown, float _turnlr);
    //slow down speed quickly
    void brake(double dt);
//    void startLoop();
//    void hover_loop();
    void pos_to_rate_xy(float dt,float ekfNavVelGainScaler,double vel_target[3]);
    void rate_to_accel_xy(float dt, float ekfNavVelGainScaler,double vel_target[3],double acc_target[3]);
    void accel_to_lean_angles(float dt, float ekfNavVelGainScaler,double acc_target[3]);
    //method from vijay kumar
    void posrate_to_lean_angles(double vel_target[3], double acc_target[3]);
    //another method for hover vel,acc
//    void get_hover_vel_acc(double vel_target[3], double acc_target[3]);
    bool isOnArrival(float targetPosX,float targetPosY,float targetPosZ);
public:
    geometry_msgs::PoseStamped sensor_local_position;
    geometry_msgs::TwistStamped sensor_local_velocity;
    std_msgs::Float64 setheight;
    std_msgs::Float32MultiArray target_velocity;
    sensor_msgs::Imu sensor_imu;
    geometry_msgs::PoseStamped fmu_controller_setpoint_attitude;
    std_msgs::Float32MultiArray commander_euler;
    std_msgs::Float32MultiArray commander_move;
    float_t vel[3],vel_opt[3],vel_last[3],acc[3],pos[3],targetPos[3],hoverPos[3],gtPos[3];
    double_t trajectorVel[3],trajectorAcc[3];
    float_t vel_ekf[3],pos_ekf[3],vel_lp[3];
    float_t yaw,pitch,roll,vicon_yaw,vicon_pitch,vicon_roll;
    float_t roll_target,pitch_target,yaw_target;
    //float_t sonar_new,sonar_last,sonar_lp;

    float_t controlRate;
    bool init;
    bool latchForVel;
    bool cotrolWithVicon;
    int _move_mode;

#if ARDRONE
    ARDrone drone;
#elif GAZEBO
    DroneObjectROS drone;
#elif PIONEER3AT
    Pioneer3AT drone;
#endif
    // p3atObstacleAvoid p3atOA;

    PIDController *PID_x,*PID_vx,*PID_y,*PID_vy,*PID_yaw,*PID_z;
    FuzzyPID *FuzzyPIDX,*FuzzyPIDY,*FuzzyPIDVx,*FuzzyPIDVy;
    ADRC *ADRCVx,*ADRCVy;
    ros::Time pidTime,checkOutsideControl;
    double posErr[3],velErr[3],accErr[3];
    //for vijay's
    double Kpos, Kvel;

    ///setpoint
    vector<Point> setpointPos;
    float_t posX_record[MAX_COUNT_POSITION],posY_record[MAX_COUNT_POSITION],posZ_record[MAX_COUNT_POSITION],posX_sum,posY_sum,posZ_sum;
    int pos_record_count;
    ros::Time setpointLoiterTime;
    bool isReachPoint;
protected:
    pthread_t read_tid;
    bool threadQuit;
private:
    ros::NodeHandle *nh;

//    ros::Subscriber mavros_sensor_local_sub;
//    ros::Subscriber mavros_sensor_imu_sub;
//    ros::Subscriber mavros_sensor_velocity_sub;
//    ros::Subscriber control_takeoff_sub;
//    ros::Subscriber control_land_sub;
//    ros::Subscriber control_euler_sub;
//    ros::Subscriber control_yaw_sub;
//    ros::Subscriber control_velocity_sub;
//    ros::Subscriber control_sethover_sub;
//    ros::Subscriber control_move_sub;
//    ros::Subscriber control_settrajectory_sub;
//    ros::Subscriber hover_vp_sub;
//    ros::Subscriber hover_vd_sub;
//    ros::Subscriber hover_posp_sub;
//    ros::Subscriber hover_posi_sub;
//    ros::Subscriber hover_PIDWrite_sub;
//    ros::Subscriber hover_setpoint_sub;
//    ros::Subscriber control_trajectoryData_sub;
    ros::Subscriber pos_ekf_sub;
    ros::Subscriber vel_ekf_sub;
//    ros::Subscriber gtPos_ekf_sub;
//    ros::Subscriber visionHoverSwitch_sub;

//    ros::Publisher mavros_control_att_attitude_pub;
//    ros::Publisher setheight_pub;
//    ros::Publisher setKeyframe_pub;
//    ros::Publisher autoreturn_startReturn_pub;
    ros::Publisher visionHover_pub;

    std_msgs::Float64 tmp;
    ros::Publisher tmp_pub1,tmp_pub2;
    ros::Publisher trajectorData_pub;
    ros::Publisher autoreturn_startRecord_pub;

    //publisher for test
    ros::Publisher test_pub, test_pub2, test_pub3, test_pub4;

    posTrajectory _trajectory;
    float_t att_R[9];
    bool isControlOutside;
    bool isSetVel;
    bool setHover;
    bool isSetPoint;
    bool isSetTrajectory;
    bool isVisionHover;

    void reset();
    void qMulq(float q1[],float q2[],float q_m[]);
    void qtoEuler();
    void qtoEuler(float& _roll,float& _pitch,float& _yaw,float q[]);
    void  getRotation();
    void  getRotation(float q[],float R[]);
    void eulertoQ(float q[]);
    void eulertoQ(float_t roll,float_t pitch,float_t yaw,float q[]);

//    void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg);
//    void takeoffCallback(const std_msgs::Bool::ConstPtr &msg);
//    void landCallback(const std_msgs::Bool::ConstPtr &msg);
//    void setHeightCallback(const std_msgs::Float64::ConstPtr &msg);
//    void localVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr  &msg);
//    void controlEulerCallback(const std_msgs::Float32MultiArray::ConstPtr  &msg);
//    void controlSetYawCallback(const std_msgs::Float32::ConstPtr  &msg);
//    void controlVelocityCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
//    void controlMoveCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
//    void setHoverCallback(const std_msgs::Bool::ConstPtr &msg);
    void posEKFCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void velEKFCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
//    void gtPosEKFCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

//    void setVelpCallback(const std_msgs::Float32::ConstPtr &msg);
//    void setVeldCallback(const std_msgs::Float32::ConstPtr &msg);
//    void setPospCallback(const std_msgs::Float32::ConstPtr &msg);
//    void writePosPIDCallback(const std_msgs::Bool::ConstPtr &msg);
//    void setPosiCallback(const std_msgs::Float32::ConstPtr &msg);

//    void setPointCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
//     void trajectoryDataCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
//    void setTrajectoryCallback(const std_msgs::Bool::ConstPtr &msg);
//    void visionHoverSwitchDataCallback(const std_msgs::Bool::ConstPtr &msg);

    void transform_body_from_nwu(const float_t a_nwu[3],float_t a_body[3]);
    void transform_nwu_from_body(const float_t a_body[3],float_t a_nwu[3]);
    float safe_sqrt(float v);
    float constrain_float(float amt, float low, float high);
    float fast_atan(float v);
    float trapezoidalIntegral(float dt,float v_last,float v_now);


    void doSetPoint(double dt);
    void doSetVelocity(double dt);

    void doTrajector(double dt,double trajectoryTime);

    void calYaw(float vx, float vy, float vz, float &yaw);
    float calYawSpeed(float yaw_tar, float yaw_now);
};
#endif	// PosControl_H

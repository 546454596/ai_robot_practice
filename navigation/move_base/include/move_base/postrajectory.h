#ifndef POSTRAJECTORY_H
#define POSTRAJECTORY_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <ros/callback_queue.h>
//#include <opencv/cv.hpp>
#include <opencv2/opencv.hpp>
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Dense>

#include <move_base/PIDController.h>
#include <move_base/pid_fuzzy.h>

//#define LINEAR (1)
#define CUBIC 2
#define MINIMUMJERK 3
#define MINIMUMSNAP 4
#define NOT_IN_RANGE -1
#define BAD_DATA -2
#define BOUNDARY_CONDITION 1  //0:natural 1:clamped
using namespace std;

//in nwu world frame
struct trajectoryPoint {
    float x;
    float y;
    float z;
    float yaw;
    double reach_time;
};
struct trajectoryPlanPoint{
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float ax;
    float ay;
    float az;
    double timeNow;
};

class posTrajectory
{
public:
    posTrajectory(ros::NodeHandle *n):
    bad_data(false),
    dimension(3),
    writePoints(false),
    startTrajectory(false),
    linear_inter_acc(0.5),
    recordFrameIndex(-1),
    isChangeFrame(false),
    chooseFlag(CUBIC){
        nh=*n;
        setup();
    }

    trajectoryPoint trajectoryStartPoint;
    vector<trajectoryPoint> trajectorySequence;
    vector<trajectoryPlanPoint> trajectoryPlanSequence;
    trajectoryPlanPoint targetPointNow;
    bool bad_data,writePoints,startTrajectory;
    int dimension;
    float_t yawTarget;
    double_t trajectoryStartTime;
    double_t startVelocity[3],startAcc[3];
    //cubic spline calculate matrix and following matrixs support 7 order spline
    cv::Mat Ak,Bk,Ck,Dk,Ek,Fk,Gk,Hk,pts;
    //linear interplation
    float_t linear_inter_acc;
    int chooseFlag;
    int recordFrameIndex;
    bool isChangeFrame;
    cv::Mat blendTimei,blendTimeij,slopes,signS;
    Eigen::Quaterniond targetQ;
    Eigen::Quaterniond targetQFrom;
    Eigen::Quaterniond targetQTo;

    PIDController *PID_trajector;
    FuzzyPID *FuzzyPIDTrajector;

    void loadTrajectoryPoints();
    void trajectoryPlanner(int selectFlag);
    void linear_trajectoryPlanner();
    void cubic_trajectoryPlanner();
    void minJerk_trajectoryPlanner();
    void minSnap_trajectoryPlanner();
    void trajectoryStart(int flag);
    void setMethod(int choose);
    short interpolating(const double t);
    short interpolating(const int ti, const double t, cv::Mat & s);
    short first_derivative(const double t, cv::Mat & dv);
    short first_derivative(const int ti, const double t, cv::Mat & dv);
    short second_derivative(const double t, cv::Mat & da);
    short second_derivative(const int ti, const double t, cv::Mat & da);
    int trajectoryPlanTarget(float_t posNow[3],float_t velNow[3],double_t accCommand[3],double_t dt,double_t time);

private:
    ros::NodeHandle nh;
    void setup();
    float sign(float x){if(x>0.0) return 1.0f;else{if(x<0.0)return -1.0f;else return 0.0f;}}
};

#endif // POSTRAJECTORY_H

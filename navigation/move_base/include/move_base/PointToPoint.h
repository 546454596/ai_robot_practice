#ifndef POINTOTPOINT_H
#define POINTOTPOINT_H

#include <iostream>
#include <cstdlib>
#include <time.h>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <boost/tokenizer.hpp>
#include <boost/token_functions.hpp>
#include <move_base/assistMath.h>

using namespace std;

class PointToPoint
{
public:
	PointToPoint(ros::NodeHandle &_n);
	~PointToPoint();

//points are all in world frame
	bool readWayPoints();
	bool getTargetSpeed(float _nowx, float _nowy, float _nowz, float _nowyaw,
                     float &vx, float &vy, float &vz, float &vyaw);
    void clear();
    void addpoint(float _x, float _y, float _z);

private:
	ros::NodeHandle nh;
    ros::Publisher targetP_pub;
    vector<vector<float>> path;
	int id_tar;
	string ptopfilename;
    int pausetime, cntdown;

	void calDPotantial2(float _nowx, float _nowy, float _nowz,
                    float _tarx, float _tary, float _tarz,
                    float &vx, float &vy, float &vz);
    float calYawSpeed(float vx, float vy, float vz, float yaw_now);
};

#endif

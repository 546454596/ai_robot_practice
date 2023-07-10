/* please be attention that:
 * pointBODY = Rt * (pointWORLD - T)
 *     R0 R1 R2           x
 * R = R3 R4 R5 , point = y, Rt * R = E, both q and T are grab from vicon and transfrom into R
 *     R6 R7 R8           z
 * and you should input the same R and T in forward transform and backward transform
 * q = {w,x,y,z}
 */

#ifndef ASSISTMATH_H
#define ASSISTMATH_H
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace std;

void qToEuler(float& _roll,float& _pitch,float& _yaw,float q[]);
void eulerToQ(float_t roll,float_t pitch,float_t yaw,float q[]);
void qmultiplyq(float q1[],float q2[],float q_result[]);
void qToRotation(float q[],float R[]);
void rotationToQ(float q[],float R[]);
void rotationToQ(float q[],cv::Mat R,int type=CV_64F);
void rotate_body_from_NWUworld(const float_t a_nwu[],float_t a_body[],float_t R[]);
void rotate_NWUworld_from_body(const float_t a_body[],float_t a_nwu[],float_t R[]);
void transform_body_from_NWUworld(float &bodyX, float &bodyY, float &bodyZ,
                                  float worldX, float worldY, float worldZ,
                                  float R[], float T[]);
void transform_NWUworld_from_body(float bodyX, float bodyY, float bodyZ,
                                  float &worldX, float &worldY, float &worldZ,
                                  float R[], float T[]);
double getDistance(float_t dx,float_t dy,float_t dz);
double getDistance(float_t ds[3]);
void RmultiR(float R1[], float R2[], float Rout[]); //3 by 3
void RtmultiR(float R1[], float R2[], float Rout[]);

#endif

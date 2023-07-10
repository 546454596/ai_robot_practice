#ifndef _ARDRONE_
#define _ARDRONE_

#include <string>
#include <stdio.h>
#include <iostream>
#ifdef WIN32
#include <WinSock2.h>
#include <Ws2tcpip.h>
#include <Wspiapi.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <arpa/inet.h>
#endif

#include "NavData.h"
////////////////////////
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>


using namespace std;

class ARDrone {
public:
    ARDrone(ros::NodeHandle &_n):isFlying(false){}
	~ARDrone(void){}
	bool isFlying;

	bool open();
	void shutdown();
	bool takeOff();
	bool land();
	bool hover();

    ////////////////////////////////////////////////////////////SZ
        geometry_msgs::Twist twist_msg;
        geometry_msgs::Twist twist_msg_hover;
        geometry_msgs::Twist twist_msg_pshover;
        std_msgs::Empty emp_msg;

        ros::NodeHandle node;


                       ros::Publisher pub_empty_land;
                       ros::Publisher pub_twist;
                       ros::Publisher pub_empty_takeoff;
                       ros::Publisher pub_empty_reset;

        bool setup();
        ///////////////////////////////////////////////////////////////

	// move command
	// v_lr = velocity left			(-1) to right		(+1)
	// v_fb = velocity forwards		(-1) to backwards	(+1)
	// v_du = velocity downwards	(-1) to upwards		(+1)
	// w_lr = angular velocity left (-1) to right		(+1)
    bool move(float v_lr, float v_fb, float v_ud, float w_lr);
	bool moveLeft(float speed = -0.2);
	bool moveRight(float speed = 0.2);
	bool moveForward(float speed = 0.1);
	bool moveBackward(float speed = 0.1);
	bool moveUp(float speed = 0.1);
	bool moveDown(float speed = 0.1);
	bool turnLeft(float speed = 0.1);
	bool turnRight(float speed = 0.1);
	bool selectFrontCamera();
	bool selectVerticalCamera();
	bool resetWatchdog();

	bool resumeNormal();
	bool emergency();
	bool pokeNavPort();
	bool recvNav();

	int frames;
	NavData nav;
	char navdata[2048];
protected:
	bool sendControlData(string data);
	bool initNavPort();

	bool sendWatchdogReset();
	bool sendTrim();
	bool sendTakeOff();
	bool sendLand();
	bool sendHover();
	bool sendMove(float lr, float fb, float ud, float w);
	bool sendMoveLeft(float speed);
	bool sendMoveRight(float speed);
	bool sendMoveForward(float speed);
	bool sendMoveBackward(float speed);
	bool sendMoveUp(float speed);
	bool sendMoveDown(float speed);
	bool sendTurnLeft(float speed);
	bool sendTurnRight(float speed);
	bool sendEmergency();
	
	bool sendResumeNormal(); 
	
	bool sendSelectFrontCamera();
	bool sendSelectVerticalCamera();
protected:
#ifdef WIN32
	SOCKET navClient;
	SOCKET controlClient;
#else
	int controlClient;
	int navClient;
#endif
	char command[100];
	int controlSequence;
};
#endif

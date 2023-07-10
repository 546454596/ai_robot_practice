#include "move_base/ARDrone.h"
using namespace std;

bool ARDrone::setup(){
    ROS_INFO("ARdrone Setup.");
    //ros::init(argc, argv,"ARDrone_test");
    //ros::NodeHandle node;
    //ros::Rate loop_rate(50);


    pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
    pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
    pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
    pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */

    twist_msg_hover.linear.x=0.0;
    twist_msg_hover.linear.y=0.0;
    twist_msg_hover.linear.z=0.0;
    twist_msg_hover.angular.x=0.0;
    twist_msg_hover.angular.y=0.0;
    twist_msg_hover.angular.z=0.0;
    return true;
}

bool ARDrone::open() {
    isFlying = false;
    int err;

#ifdef WIN32
    WSADATA wsaData;
    // Initialize Winsock
    err = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (err != 0) {
        cerr << "Error: Winsock initialisation failed: " << err << endl;
        return false;
    }
#endif

    /* Open a dgram(UDP) socket */
    controlClient = socket(AF_INET, SOCK_DGRAM, 0);
    navClient = socket(AF_INET, SOCK_DGRAM, 0);

    struct timeval tv;

    tv.tv_sec = 5; /* 30 Secs Timeout */
    tv.tv_usec = 0;  // Not init'ing this can cause strange errors

    setsockopt(navClient, SOL_SOCKET, SO_RCVTIMEO, (char *) &tv,
               sizeof(struct timeval));

#ifdef WIN32
    if (controlClient == INVALID_SOCKET)
    {
        std::cerr << "Erorr: Could not create socket" << endl;
        WSACleanup();
        return false;
    }

    if (navClient == INVALID_SOCKET)
    {
        std::cerr << "Erorr: Could not create socket" << endl;
        WSACleanup();
        return false;
    }
#else
    if (controlClient < 0) {
        std::cerr << "Erorr: Could not create socket" << endl;
        return false;
    }
    if (navClient < 0) {
        std::cerr << "Erorr: Could not create socket" << endl;
        return false;
    }
#endif

    //Binding a socket with 192.168.1.1
    struct sockaddr_in sin, sin_navdata;
    sin.sin_port = htons(5556);
    sin.sin_addr.s_addr = inet_addr("192.168.1.1");
    sin.sin_family = AF_INET;

    sin_navdata.sin_port = htons(5554);
    sin_navdata.sin_addr.s_addr = inet_addr("192.168.1.1");
    sin_navdata.sin_family = AF_INET;

#ifdef WIN32
    if (connect(controlClient,(sockaddr*)&sin, sizeof(sin)) == INVALID_SOCKET) {cout << "Socket Connection Failed" << endl;
        cout << "Error Code:  " << WSAGetLastError() << endl;
        system("pause >nul");
        closesocket(controlClient);
        WSACleanup();
    }

    if (connect(navClient,(sockaddr*)&sin_navdata, sizeof(sin_navdata)) == INVALID_SOCKET) {
        cout << "Socket Connection Failed" << endl;
        cout << "Error Code:  " << WSAGetLastError() << endl;
        system("pause >nul");
        closesocket(navClient);
        WSACleanup();
    }

#else
    if (connect(controlClient, (sockaddr*) &sin, sizeof(sin)) < 0) {
        cout << "connection failed!" << endl;
        return false;
    }
    if (connect(navClient, (sockaddr*) &sin_navdata, sizeof(sin_navdata)) < 0) {
        cout << "Socket Connection Failed" << endl;
        return false;
    }
#endif
    controlSequence = 1;
    sprintf(command, "AT*CONFIG=%d,\"video:codec_fps\",\"30\"\r",
            controlSequence);
    if (ARDrone::sendControlData(command) == false)
        return false;

    cout << "Connecting to ARDrone..." << endl;

    if (!ARDrone::recvNav())
        return false;

    cout << "Connect to ARDrone successfully!" << endl;

    pokeNavPort();
    initNavPort();

    return true;
}

void ARDrone::shutdown(void) {
#ifdef WIN32
    closesocket(controlClient);
#else
    close(controlClient);
#endif
}

bool ARDrone::sendControlData(string data) {
    //cout << data << endl;
    int ret = send(controlClient, data.data(), data.length(), 0);
    controlSequence++;
#ifdef WIN32
    if (ret == SOCKET_ERROR) {
        cerr << "Error: Sending '" << data << "' failed: " << WSAGetLastError()
             << endl;
        return false;
    }
#else
    if (ret < 0) {
        cerr << "error in sending control data!" << endl;
        return false;
    }
#endif
    return true;
}

bool ARDrone::pokeNavPort() {
    sprintf(command, "\x01");//\x00
    string command_data = command;
    return send(navClient, command_data.data(), command_data.length(), 0);
}

bool ARDrone::initNavPort() {
    sprintf(command, "AT*CONFIG=%d,\"general:navdata_demo\",\"FALSE\"\r",
            controlSequence);
    if (ARDrone::sendControlData(command) == false)
        return false;

    sprintf(command, "AT*CTRL=%d,0\r", controlSequence);
    if (ARDrone::sendControlData(command) == false)
        return false;

    sprintf(command, "AT*FTRIM=%d,\r", controlSequence);
    if (ARDrone::sendControlData(command) == false)
        return false;

    return true;
}

bool ARDrone::recvNav() {
    resetWatchdog();
    pokeNavPort();
    memset(navdata, 0, sizeof(navdata));
    if (recv(navClient, (char*) &navdata, sizeof(navdata), 0) > 0) {
        memcpy(nav.rawData, navdata, sizeof(navdata));
        nav.setOptions();
        return true;
    } else
        return false;
}

bool ARDrone::sendWatchdogReset() {
    sprintf(command, "AT*COMWDG=%d\r", controlSequence);
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendTrim() {
    sprintf(command, "AT*FTRIM=%d\r", controlSequence);
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendTakeOff() {
    isFlying = true;
    sprintf(command, "AT*REF=%d,290718208\r", controlSequence);
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendLand() {
    isFlying = false;
    sprintf(command, "AT*REF=%d,290717696\r", controlSequence);
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendEmergency() {
    sprintf(command, "AT*REF=%d,290717952\r", controlSequence);
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendResumeNormal() {
    sprintf(command, "AT*REF=%d,290717696\rAT*REF=%d,290717952\r",
            controlSequence, controlSequence + 1);
    controlSequence++;
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendHover() {
    sprintf(command, "AT*PCMD=%d,0,0,0,0,0\r", controlSequence);
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendMove(float v_lr = 0, float v_fb = 0, float v_du = 0,
                       float w_lr = 0) {
    v_lr = (v_lr < 1) ? (v_lr > -1 ? v_lr : -1) : 1;
    v_fb = (v_fb < 1) ? (v_fb > -1 ? v_fb : -1) : 1;
    v_du = (v_du < 1) ? (v_du > -1 ? v_du : -1) : 1;
    w_lr = (w_lr < 1) ? (w_lr > -1 ? w_lr : -1) : 1;
    sprintf(command, "AT*PCMD=%d,1,%d,%d,%d,%d\r", controlSequence,
            *(int*) (&v_lr), *(int*) (&v_fb), *(int*) (&v_du), *(int*) (&w_lr));
    return ARDrone::sendControlData(command);
    //return ARDrone::sendControlData(string_format("AT*PCMD=%d,1,%d,%d,%d,%d\r",
    //	controlSequence, float2int(v_lr), float2int(v_fb), float2int(v_du), float2int(w_lr)));
}

bool ARDrone::sendMoveLeft(float speed) {
    sprintf(command, "AT*PCMD=%d,1,%d,0,0,0\r", controlSequence,
            *(int*) (&speed)); 	//
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendMoveRight(float speed) {
    sprintf(command, "AT*PCMD=%d,1,%d,0,0,0\r", controlSequence,
            *(int*) (&speed)); 	//
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendMoveForward(float speed) {
    sprintf(command, "AT*PCMD=%d,1,0,%d,0,0\r", controlSequence,
            *(int*) (&speed));
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendMoveBackward(float speed) {
    sprintf(command, "AT*PCMD=%d,1,0,%d,0,0\r", controlSequence,
            *(int*) (&speed));
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendMoveUp(float speed) {
    sprintf(command, "AT*PCMD=%d,1,0,0,%d,0\r", controlSequence,
            *(int*) (&speed));
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendMoveDown(float speed) {
    sprintf(command, "AT*PCMD=%d,1,0,0,%d,0\r", controlSequence,
            *(int*) (&speed));
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendTurnLeft(float speed) {
    sprintf(command, "AT*PCMD=%d,1,0,0,0,%d\r", controlSequence,
            *(int*) (&speed));
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendTurnRight(float speed) {
    sprintf(command, "AT*PCMD=%d,1,0,0,0,%d\r", controlSequence,
            *(int*) (&speed));
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendSelectFrontCamera() {
    sprintf(command, "AT*ZAP=%i,0\r", controlSequence);
    return ARDrone::sendControlData(command);
}

bool ARDrone::sendSelectVerticalCamera() {
    sprintf(command, "AT*ZAP=%i,2\r", controlSequence);
    return ARDrone::sendControlData(command);
}

bool ARDrone::resetWatchdog() {
    //return ARDrone::sendWatchdogReset();
    return true;
}

bool ARDrone::takeOff() {
    if (isFlying)
        return false;

    bool result = true;
    //result = result && ARDrone::sendTrim();
    //result = result && ARDrone::sendTakeOff();

    pub_empty_takeoff.publish(emp_msg); //launches the drone
    pub_twist.publish(twist_msg_hover); //drone is flat
    ROS_INFO("Taking off");
    ros::spinOnce();
    isFlying = true;

    //test
    //cout << "take off!" << endl;
    return result;
}

bool ARDrone::land() {
    if (!isFlying)
        return false;

    //return ARDrone::sendLand();
    pub_twist.publish(twist_msg_hover); //drone is flat
    pub_empty_land.publish(emp_msg); //lands the drone
    ROS_INFO("Landing");
    ros::spinOnce();
    isFlying = false;
    return true;
}

bool ARDrone::hover() {
    if (!isFlying)
        return false;

    //return ARDrone::sendHover();
    pub_twist.publish(twist_msg_hover); //drone is flat
    ROS_INFO("Howering");
    ros::spinOnce();
    return true;
}

bool ARDrone::move(float lr, float fb, float ud, float w) {
    if (!isFlying)
        return false;

    //return ARDrone::sendMove(lr, fb, ud, w);
    //send nwu eulerangle
    twist_msg.linear.x=fb;
    twist_msg.linear.y=-lr;
    twist_msg.linear.z=ud;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z= w;
    pub_twist.publish(twist_msg);
    //ROS_INFO("Moving");
    return true;
}

bool ARDrone::moveLeft(float speed) {
    if (!isFlying)
        return false;

    //return ARDrone::sendMoveLeft(speed);
    twist_msg.linear.x=0.0;
    twist_msg.linear.y=speed;
    twist_msg.linear.z=0.0;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z=0.0;
    pub_twist.publish(twist_msg);
    ROS_INFO("Moving");
    return true;
}

bool ARDrone::moveRight(float speed) {
    if (!isFlying)
        return false;

    //return ARDrone::sendMoveRight(speed);
    twist_msg.linear.x=0.0;
    twist_msg.linear.y=   - speed;
    twist_msg.linear.z=0.0;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z=0.0;
    pub_twist.publish(twist_msg);
    ROS_INFO("Moving");
    return true;
}

bool ARDrone::moveForward(float speed) {
    if (!isFlying)
        return false;

    //return ARDrone::sendMoveForward(speed);
    twist_msg.linear.x= speed;
    twist_msg.linear.y=0.0;
    twist_msg.linear.z=0.0;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z=0.0;
    pub_twist.publish(twist_msg);
    ROS_INFO("Moving");
    return true;
}

bool ARDrone::moveBackward(float speed) {
    if (!isFlying)
        return false;

    //return ARDrone::sendMoveBackward(speed);
    twist_msg.linear.x= - speed;
    twist_msg.linear.y=0.0;
    twist_msg.linear.z=0.0;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z=0.0;
    pub_twist.publish(twist_msg);
    ROS_INFO("Moving");
    return true;
}

bool ARDrone::moveUp(float speed) {
    if (!isFlying)
        return false;

    //return ARDrone::sendMoveUp(speed);
    twist_msg.linear.x= 0.0;
    twist_msg.linear.y=0.0;
    twist_msg.linear.z= speed;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z=0.0;
    pub_twist.publish(twist_msg);
    ROS_INFO("Moving");
    return true;
}

bool ARDrone::moveDown(float speed) {
    if (!isFlying)
        return false;

    //return ARDrone::sendMoveDown(speed);
    twist_msg.linear.x= 0.0;
    twist_msg.linear.y=0.0;
    twist_msg.linear.z= - speed;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z=0.0;
    pub_twist.publish(twist_msg);
    ROS_INFO("Moving");
    return true;
}

bool ARDrone::turnLeft(float speed) {
    if (!isFlying)
        return false;

    twist_msg.linear.x= 0.0;
    twist_msg.linear.y=0.0;
    twist_msg.linear.z= 0.0;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z=  speed;
    pub_twist.publish(twist_msg);
    ROS_INFO("Moving");
    return true;
    //return ARDrone::sendTurnLeft(speed);
    return true;
}

bool ARDrone::turnRight(float speed) {
    if (!isFlying)
        return false;

    //return ARDrone::sendTurnRight(speed);
    twist_msg.linear.x= 0.0;
    twist_msg.linear.y=0.0;
    twist_msg.linear.z= 0.0;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z= - speed;
    pub_twist.publish(twist_msg);
    ROS_INFO("Moving");
    return true;
    return true;
}

bool ARDrone::selectFrontCamera() {
    //return ARDrone::sendSelectFrontCamera();
    return true;
}

bool ARDrone::selectVerticalCamera() {
    //return ARDrone::sendSelectVerticalCamera();
    return true;
}

bool ARDrone::resumeNormal() {
    isFlying = false;
    //return ARDrone::sendResumeNormal();
    return true;
}

bool ARDrone::emergency() {
    //return ARDrone::sendEmergency();
    return true;
}

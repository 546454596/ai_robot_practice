#include "move_base/PointToPoint.h"

PointToPoint::PointToPoint(ros::NodeHandle &_n):
    nh(_n), id_tar(-1), cntdown(0)
{
	nh.getParam("pointToPointFile", ptopfilename);
    nh.param("pausetime", pausetime, 50);
    targetP_pub = nh.advertise<geometry_msgs::Pose>("/ai_robot/findpath/targetP",1);
    readWayPoints();
}

PointToPoint::~PointToPoint(){}

bool PointToPoint::readWayPoints(){
	path.clear();
	std::ifstream in(ptopfilename);
    if(!in){
        printf("[PTOP]no waypoint file in !\n");
        return false;
    }
    std::string line;
    boost::char_separator<char> sep(" ");

    vector<float> pt;
    pt.resize(3);
    // read point cloud from "freiburg format"
    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();
        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
        if (tokens.size() != 3) continue;
        pt[0] = boost::lexical_cast<float>(tokens[0]);
        pt[1] = boost::lexical_cast<float>(tokens[1]);
        pt[2] = boost::lexical_cast<float>(tokens[2]);

        path.push_back(pt);
        //printf("read (%f,%f,%f)\n", pt[0], pt[1], pt[2]);
    }
    id_tar = 0;

    for(int i=0; i<path.size(); ++i){
        printf("waypoint %d (%f,%f,%f)\n", i, path[i][0], path[i][1], path[i][2]);
    }

    std::cout << "read waypoints----->" << std::endl
              << "  total " << path.size() << " waypoints." << endl;
    in.close();
    return true;
}

bool PointToPoint::getTargetSpeed(float _nowx, float _nowy, float _nowz, float _nowyaw,
                     float &vx, float &vy, float &vz, float &vyaw){
    if(cntdown > 0){
        --cntdown;
        vx = 0;
        vy = 0;
        vz = 0;
        vyaw = 0;
        return true;
    }
    // cout<<"in1"<<endl;
    if(id_tar < path.size())
    {
        float dist = sqrt(pow(_nowx-path[id_tar][0], 2) + pow(_nowy-path[id_tar][1], 2));
        calDPotantial2(_nowx, _nowy, _nowz, path[id_tar][0], path[id_tar][1],
                path[id_tar][2], vx, vy, vz);
        vyaw = calYawSpeed(vx, vy, vz, _nowyaw);
        // cout<<"in2"<<endl;
        //pub target point
        geometry_msgs::Pose tarp_msg;
        float twc[3]={_nowx,_nowy,_nowz}, Rwc[9]={cos(_nowyaw), -sin(_nowyaw), 0, sin(_nowyaw), cos(_nowyaw), 0,0,1},
            tarp_frombody[3];
        Rwc[0] = cos(_nowyaw);
        Rwc[1] = -sin(_nowyaw);
        Rwc[3] = sin(_nowyaw);
        Rwc[4] = cos(_nowyaw);
        transform_body_from_NWUworld(tarp_frombody[0], tarp_frombody[1], tarp_frombody[2],
            path[id_tar][0], path[id_tar][1], path[id_tar][2], Rwc, twc);
        tarp_msg.position.x = tarp_frombody[0];
        tarp_msg.position.y = tarp_frombody[1];
        tarp_msg.position.z = tarp_frombody[2];
        float del_yaw = 0;
        if(id_tar < path.size()-1){
            //try to dead to next point when going to this point
            float nexttarp_frombody[3], _q_frombody[4];
            transform_body_from_NWUworld(nexttarp_frombody[0], nexttarp_frombody[1], nexttarp_frombody[2],
                path[id_tar+1][0], path[id_tar+1][1], path[id_tar+1][2], Rwc, twc);
            del_yaw = atan2(nexttarp_frombody[1]-tarp_frombody[1], nexttarp_frombody[0]-tarp_frombody[0]);
            eulerToQ(0,0,del_yaw,_q_frombody);
            tarp_msg.orientation.w = _q_frombody[0];
            tarp_msg.orientation.x = _q_frombody[1];
            tarp_msg.orientation.y = _q_frombody[2];
            tarp_msg.orientation.z = _q_frombody[3];
        }
        else{
            tarp_msg.orientation.w = 0;
            tarp_msg.orientation.x = 0;
            tarp_msg.orientation.y = 0;
            tarp_msg.orientation.z = 0;

        }
        // if(tarp_msg.position.x > 0.7 && fabs(tarp_msg.position.y)<0.1){
        //     if(tarp_msg.position.x < 0.9)
        //     {
        //         tarp_msg.position.y = 0;
        //         tarp_msg.position.z = 0;
        //     }
        //       //for autolabor
        // }
        targetP_pub.publish(tarp_msg);
        if(dist < 0.5)// && fabs(del_yaw)< 0.314
        {
            ++id_tar;
            cntdown = pausetime;
            ROS_INFO("[PTOP]to the %d point and pause %f second.",id_tar, pausetime*1.0/50);
        }
        return true;
    }
    else
    {
        vx = 0;
        vy = 0;
        vz = 0;
        vyaw = 0;
        return false;
    }
}

void PointToPoint::calDPotantial2(float _nowx, float _nowy, float _nowz,
                    float _tarx, float _tary, float _tarz,
                    float &vx, float &vy, float &vz)
{
    float _dx = _tarx - _nowx;
    float _dy = _tary - _nowy;
    float _dL = sqrt(pow(_dx,2) + pow(_dy,2));
    float vtar;
    if(_dL > 2)
    {
        vtar = 0.5;
    }
    else
    {
        vtar = _dL / 4;
    }
    float _vo = vtar / _dL;
    vx = _vo * _dx;
    vy = _vo * _dy;
    //_dz = _tarz - _nowz;
    if(_dx > 0)
    {
        vx = fabs(vx);
    }
    else
    {
        vx = -fabs(vx);
    }

    if(_dy > 0)
    {
        vy = fabs(vy);
    }
    else
    {
        vy = -fabs(vy);
    }

    float _dz = _tarz - _nowz;
    if(fabs(_dz) > 0.3)
    {
        vz = 0.3 * fabs(_dz) / _dz;
    }
    else
    {
        vz = _dz;
    }
}

// in fact, this method just cal the yaw err between now dir and target point dir
float PointToPoint::calYawSpeed(float vx, float vy, float vz, float yaw_now)
{
    float L = sqrt(pow(vx,2) + pow(vy,2)), yaw_tar;
    if(vy > 0)
    {
        yaw_tar = acos(vx / L);
    }
    else
    {
        yaw_tar = -acos(vx / L);
    }

    float yErr = yaw_tar - yaw_now;
    if(yErr > 3.1415)
    {
        yErr -= 2*3.1415;
    }
    else if(yErr < -3.1415)
    {
        yErr += 2*3.1415;
    }
    //printf("yawtar(%f), yawnow(%f), yawerr(%f)\n", yaw_tar, yaw_now, yErr);

    return yErr;
}

void PointToPoint::clear()
{
    path.clear();
    id_tar = 0;
}

void PointToPoint::addpoint(float _x, float _y, float _z)
{
    vector<float> pt;
    pt.resize(3);
    pt[0] = _x;
    pt[1] = _y;
    pt[2] = _z;
    cout<<_x<<" "<<_y<<" "<<_z<<endl;
    path.push_back(pt);
}

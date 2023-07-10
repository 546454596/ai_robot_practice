#include <unistd.h>
#include <move_base/AIBrain.h>

AIBrain::AIBrain(ros::NodeHandle *_nh):
    nh(_nh), isControllerInput(false), pidTime(ros::Time::now().toSec()),
    isStableHover(true), poseCtrl(_nh),
    isTestModeOfFindPath(false), 
    yawworldslam(0), ptop(*_nh), poseXYZ_target{0,0,0},
    state_findpath(0), state_ptop(0), buffer(ros::Duration(10)), tf(buffer), nav(buffer), ac("move_base", true)
{
	pausetime=50;
    poseXYZ_destination[0] = 0;
    poseXYZ_destination[1] = 0;
    poseXYZ_destination[2] = 2;
    //init param for zone1
    Tworldslam[0] = 0;//4.06;
    Tworldslam[1] = 0;//2.95;
    Tworldslam[2] = 0;//2.73;
    float _q[4] = {1,0,0,0};//{0.766,0,0,0.643};
    qToRotation(_q, Rworldslam);
    qToEuler(yawworldslam, yawworldslam, yawworldslam, _q);//如果不接收qtworldslam话题，不回调qtslamworldCallback，可以认为map与slam的frame一致
    initSubscriber();
}

AIBrain::~AIBrain(){}

void AIBrain::think()
{
    double dt = ros::Time::now().toSec() - pidTime.toSec();
    pidTime = ros::Time::now();
    poseCtrl.setState(poseXYZ_now, velXYZ_now, yaw_now);//poseXYZ_now和yaw_now为机器人map下当前位置和姿态，velXYZ_now为速度
    switch(task_number)
    {
        case HOVER_TASK:hoverTASK(dt);break;
        case TAKEOFF_TASK:takeoffTASK();break;
        case LAND_TASK:landTASK();break;
        case MOVE_TASK:controllerTASK(dt);break;
        case FINDPATH_TASK:findpath(dt);break;
        case POINTTOPOINT_TASK:pointToPointTASK(dt);break;
        case FAR_POINTTOPOINT_TASK:farPointToPointTASK(dt);break;
        default:hoverTASK(dt);break;
    }
}

void AIBrain::hoverTASK(double _dt)
{
    if(task_lastnumber != task_number)
    {
        ROS_INFO("[AIBRAIN]hover!");
        task_lastnumber = task_number;
        hoverStartTime = ros::Time::now().toSec();
    }
    if(!isStableHover)//以下发布的速度都为
    {
        if(sqrt((pow(velXYZ_now[0],2) + pow(velXYZ_now[1],2)) < 0.3))
        {
            isStableHover = true;
            poseXYZ_target[0] = poseXYZ_now[0];
            poseXYZ_target[1] = poseXYZ_now[1];
            poseXYZ_target[2] = poseXYZ_now[2];
            yaw_target = yaw_now;
            poseCtrl.doMove(_dt,0,0,0,0);
        }
        else
            poseCtrl.brake(_dt);
    }
    else
        poseCtrl.doHover(_dt, poseXYZ_target, yaw_target, poseXYZ_now, velXYZ_now, yaw_now);

    if(isTestModeOfFindPath){
        findpath(_dt);
    }
    //display position in slam
    float posSlam[3];
    transform_body_from_NWUworld(posSlam[0], posSlam[1], posSlam[2],
            poseXYZ_target[0], poseXYZ_target[1], poseXYZ_target[2], Rworldslam, Tworldslam);
    printf("target pos(%f,%f,%f)(%f,%f,%f)\r", poseXYZ_target[0], poseXYZ_target[1], poseXYZ_target[2],posSlam[0], posSlam[1], posSlam[2]);
}

void AIBrain::takeoffTASK()
{
    if(task_lastnumber != task_number)
    {
        ROS_INFO("[AIBRAIN]takeoff!");
        task_lastnumber = task_number;
    }
    yaw_target = yaw_now;
    poseXYZ_target[0] = poseXYZ_now[0];
    poseXYZ_target[1] = poseXYZ_now[1];
    poseXYZ_target[2] = 1.0;
    isStableHover = true;
    poseCtrl.takeOff();
    task_number = HOVER_TASK;
}

void AIBrain::landTASK()
{
    if(task_lastnumber != task_number)
    {
        ROS_INFO("[AIBRAIN]land!");
        task_lastnumber = task_number;
    }
    poseCtrl.land();
    task_number = HOVER_TASK;
    isStableHover = true;
}

void AIBrain::controllerTASK(double _dt)
{
    if(task_lastnumber != task_number)
    {
        ROS_INFO("[AIBRAIN]move!");
        task_lastnumber = task_number;
    }
    poseCtrl.doMove(_dt, forbackward_ctrl, leftright_ctrl, updown_ctrl, turnlr_ctrl);
    if(isTestModeOfFindPath){
        findpath(_dt);
    }
    if(fabs(forbackward_ctrl)+fabs(leftright_ctrl)+fabs(updown_ctrl)+fabs(turnlr_ctrl) < 0.01){
        task_number = HOVER_TASK;
    }
}

    void AIBrain::pointToPointTASK(double _dt)
    {
    if(task_lastnumber != task_number)
    {
        ROS_INFO("[AIBRAIN]start point to point!");
        //state_ptop = 0;
        task_lastnumber = task_number;
    }

    if(state_ptop == 1){
        float _v[4], _T[3] = {0,0,0};
        float posSlam[3];
        transform_body_from_NWUworld(posSlam[0], posSlam[1], posSlam[2],
            poseXYZ_now[0], poseXYZ_now[1], poseXYZ_now[2], Rworldslam, Tworldslam);
        if(ptop.getTargetSpeed(posSlam[0], posSlam[1], posSlam[2], yaw_now-yawworldslam,
                                _v[0], _v[1], _v[2], _v[3])){
            transform_NWUworld_from_body(_v[0], _v[1], _v[2],
                    posSlam[0], posSlam[1], posSlam[2], Rworldslam, _T);
            //poseCtrl.doMoveInWorld(_dt, posSlam[0], posSlam[1], posSlam[2], _v[3]);
        }
        else{
            state_ptop = 2;
            hoverTASK(_dt);
        }
    }
    else if(state_ptop == 0){
        if(!ptop.readWayPoints()){
            state_ptop = 2;
        }
        else
        {
            state_ptop = 1;
        }
        hoverTASK(_dt);
    }
    else if(state_ptop == 2){
        state_ptop = 0;
        task_number = HOVER_TASK;
        ROS_INFO("[AIBRAIN]to destination!");
        hoverTASK(_dt);
    }
    }

    void AIBrain::farPointToPointTASK(double _dt) {
        double theta, last_theta;
        tf2::Quaternion q;
        if (state_farptop == 0)
        {
            for (; i < farptop_path.poses.size()  && send_next_goal; ++i){
                target.target_pose.header.frame_id = "map";
                target.target_pose.header.stamp = ros::Time::now();
                
                if(send_next_goal){
                    ROS_INFO("******Sending the %dth target goal!**********", i+1);
                    send_next_goal = false;
                    target.target_pose.pose.position = farptop_path.poses[i].pose.position;
                    printf("target %d (%f,%f,%f)\n", i, farptop_path.poses[i].pose.position.x, farptop_path.poses[i].pose.position.y, farptop_path.poses[i].pose.position.z);
                    if(i < farptop_path.poses.size() - 1){
                        theta =  std::atan2(farptop_path.poses[i+1].pose.position.y - farptop_path.poses[i].pose.position.y,
                                                            farptop_path.poses[i+1].pose.position.x - farptop_path.poses[i].pose.position.x ) ;
                        last_theta = theta;
                        q.setRPY(0, 0, theta);
                        tf2::convert(q, target.target_pose.pose.orientation);   
                    }     
                    else
                    {
                        // q.setRPY(0, 0, last_theta);
                        // tf2::convert(q, target.target_pose.pose.orientation);   
                        theta =  std::atan2(target.target_pose.pose.position.y -current_point.y,
                                            target.target_pose.pose.position.x - current_point.x) ;
                        q.setRPY(0, 0, theta);
                        tf2::convert(q, target.target_pose.pose.orientation); 
                    }
                     while (!ac.waitForServer(ros::Duration(5.0)))
                        ROS_INFO("Waiting for move_base action server to come up!");
                    ROS_INFO("BEFORE");
                    ac.sendGoal(target,
                                boost::bind(&AIBrain::doneCB, this,  _1, _2),
                                MoveBaseClient::SimpleActiveCallback(),
                                boost::bind(&AIBrain::feedbackCB, this, _1)); 
                    ROS_INFO("AFTER");            
                
                    if(i == farptop_path.poses.size() - 1){
                        task_number = INVALID;
                        i = 0;
                        send_next_goal = true;
                        ROS_INFO("initilize*****");
                        return;   
                    }
                }    
            }
    }
    else if(state_farptop == 3)
    {
        std_msgs::Bool str;
        str.data = true;
        obs_abort_.publish(str);
        farptop_path.poses.clear();
        // task_number = INVALID;
        i = 0;
        send_next_goal = true;
        return;
    }
}

void AIBrain::doneCB(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
    ROS_INFO("Finished in state", state.toString().c_str());
    ROS_INFO("Reached the target goal, please send the next target!");
    // send_next_goal = true;
    // ++i;
}

void AIBrain::feedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
    current_point.x = feedback->base_position.pose.position.x;
    current_point.y = feedback->base_position.pose.position.y;
    current_point.z = feedback->base_position.pose.position.z;
    double dist = distance(current_point, target.target_pose.pose.position);
    if(dist < 0.32 && !send_next_goal)
    {
        ROS_INFO("Reached the target goal, please send the next target!");
        send_next_goal = true; 
    }
}

double AIBrain::distance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) {
    return hypot(p1.x - p2.x, p1.y - p2.y);
}

void AIBrain::findpath(double _dt) {
//    nav.planThread();发送目的地
    double theta;
    tf2::Quaternion q;
    while(!ac.waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for move_base action server to come up!");
    ac.sendGoal(target,
            MoveBaseClient::SimpleDoneCallback(),
            MoveBaseClient::SimpleActiveCallback(),
            boost::bind(&AIBrain::feedbackCB, this, _1)); 

    if(distance(current_point, target.target_pose.pose.position) < 1.5){
        theta =  std::atan2(target.target_pose.pose.position.y -current_point.y,
                                            target.target_pose.pose.position.x - current_point.x) ;
        q.setRPY(0, 0, theta);
        tf2::convert(q, target.target_pose.pose.orientation); 
        ac.sendGoal(target);
        // task_number = INVALID;
        // toEndPos();
    }        
}

void AIBrain::initSubscriber()
{
    joy_sub = nh->subscribe("/joy", 1, &AIBrain::joyCallback, this);
    joy_sub2 = nh->subscribe("/remote/joy", 1, &AIBrain::joyCallback, this);
    stop_task_ = nh->subscribe("/remote/stoptask", 1, &AIBrain::stopCallback, this);
    pose_sub = nh->subscribe("/est_pose", 1, &AIBrain::poseCallback, this);
    move_base_goal_sub = nh->subscribe("/move_base_simple/goal", 1, &AIBrain::movebasegoalCallback, this);
	move_base_goal_sub2 = nh->subscribe("/remote/move_base_simple/goal", 1, &AIBrain::movebasegoalCallback, this);
    ptop_sub = nh->subscribe("/point_to_point/path", 1, &AIBrain::ptopCallback, this);
    farptop_sub = nh->subscribe("/far_point_to_point/path", 1, &AIBrain::farptopCallback, this);
	farptop_sub2 = nh->subscribe("/remote/far_point_to_point/path", 1, &AIBrain::farptopCallback, this);
	obs_abort_ = nh->advertise<std_msgs::Bool>("/obs_abort", 1);
    to_end_pos_ = nh->advertise<std_msgs::String>("/remote/to_end_pos", 1);
}

void AIBrain::movebasegoalCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg) {
    target.target_pose.header = _msg->header;
    target.target_pose.pose = _msg->pose;
    task_number = FINDPATH_TASK;
    state_findpath = 0;
}

void AIBrain::ptopCallback(const nav_msgs::Path::ConstPtr &_msg)
{
    ptop.clear();
    for(int i=0; i<_msg->poses.size(); ++i){
        ptop.addpoint(_msg->poses[i].pose.position.x,
                      _msg->poses[i].pose.position.y,
                      _msg->poses[i].pose.position.z);
    }
    ROS_INFO("[AIBRAIN]set point to point ok.");
    task_number = POINTTOPOINT_TASK;
    state_ptop = 1;
}

void AIBrain::farptopCallback(const nav_msgs::Path::ConstPtr &_msg){
    farptop_path = *_msg;
    cntdown = 0;
    for(int i=0; i<farptop_path.poses.size(); ++i){
        printf("waypoint %d (%f,%f,%f)\n", i, farptop_path.poses[i].pose.position.x, farptop_path.poses[i].pose.position.y, farptop_path.poses[i].pose.position.z);
    }
    ROS_INFO("[AIBRAIN]set far point to point ok.");
    task_number = FAR_POINTTOPOINT_TASK;
    state_farptop = 0;
}

void AIBrain::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg)
{
    poseXYZ_now[0] = _msg->pose.position.x;
    poseXYZ_now[1] = _msg->pose.position.y;
    poseXYZ_now[2] = _msg->pose.position.z;
    qwxyz_now[0] = _msg->pose.orientation.w;
    qwxyz_now[1] = _msg->pose.orientation.x;
    qwxyz_now[2] = _msg->pose.orientation.y;
    qwxyz_now[3] = _msg->pose.orientation.z;
    qToEuler(roll_now, pitch_now, yaw_now, qwxyz_now);
}

void AIBrain::joyCallback(const sensor_msgs::Joy::ConstPtr &msg){
    if(msg->buttons[0] > 0.5){
        //A
        task_number = LAND_TASK;
        isStableHover = false;
        if(isTestModeOfFindPath){
            isTestModeOfFindPath = false;
            ROS_INFO("[AIBRAIN]exit test mode of find path!");
        }
    }
    else if(msg->buttons[1] > 0.5){
        //B
        task_number = FINDPATH_TASK;
        isStableHover = false;
    }
    else if(msg->buttons[2] > 0.5){
        //X
        task_number = POINTTOPOINT_TASK;
        isStableHover = false;
    }
    else if(msg->buttons[3] > 0.5){
        //Y
        task_number = TAKEOFF_TASK;
        isStableHover = false;
        if(isTestModeOfFindPath){
            isTestModeOfFindPath = false;
            ROS_INFO("[AIBRAIN]exit test mode of find path!");
        }
    }
    else if(msg->buttons[4] > 0.5){
        //LB
        isTestModeOfFindPath = !isTestModeOfFindPath;
        state_findpath = 0;
        ROS_INFO("[AIBRAIN]enter test mode of find path!");
    }

    leftright_ctrl = msg->axes[0];
    forbackward_ctrl = msg->axes[1];
    turnlr_ctrl = msg->axes[3];
    updown_ctrl = msg->axes[4];
    if(fabs(leftright_ctrl)>0.01 || fabs(turnlr_ctrl)>0.01 ||
            fabs(updown_ctrl)>0.01 || fabs(forbackward_ctrl)>0.01){
        task_number = MOVE_TASK;  //for safety
        isControllerInput = true;
        isStableHover = false;
    }
}

void AIBrain::stopCallback(const std_msgs::String::ConstPtr &_msg){
	task_number = FAR_POINTTOPOINT_TASK;	
	//printf("%d\n", task_number);	
	//printf("*********************************\n");
	state_farptop = 3;
}


void AIBrain::toEndPos() {
    std_msgs::String str;
	to_end_pos_.publish(str);
}
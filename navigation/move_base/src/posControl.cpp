/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <move_base/posControl.h>

extern std::string trajectoryPointsPath;
//extern std::string outfile1Path;
//extern std::string outfile2Path;

fstream trajectoryPointsfile_hover(trajectoryPointsPath.c_str());
//ofstream fout_return(outfile1Path.c_str());
//ofstream fout(outfile2Path.c_str());
float GRAVITY_MSS=9.80665f;
float MAX_ANGLE=1.0f ;
float POSCONTROL_ACCEL_XY=2.0f;
float POSCONTROL_SPEED= 2.0f;
float POSCONTROL_ACCEL_XY_MAX =9.80f ;
float MAX_TARGET_DISTANCE=1.0f ;
float sonar_lower_lp_threshold =0.2f;
float sonar_upper_lp_threshold=0.5f;
float _feedforward_para =0.5f;
int VISION_HOVER_SWITCH=0;
int CONTROLLERTYPE=0;
//control parameters
float hoverPosP=0.5;
float hoverPosI=0.1;
float hoverPosD=0.0;
float hoverPosImax=0.3;
float hoverVelP=1.5;
float hoverVelI=0.0;
float hoverVelD=0.02;
float hoverVelImax=0.5;
float hoverVelWo=1.5;
float hoverVelWc=0.0;
float hoverVelB0=0.02;
float altitudeP=1.0;
float altitudeI=0.0;
float altitudeD=0.1;
float yawP=1.0;
float yawI=0.0;
float yawD=0.1;
 std::string posPIDPath;
 std::string velPIDPath;
 std::string trajectoryPIDPath;

#define PI 3.1415926
float MAX_LINEAR_SPEED = 0.5;
float MAX_ANGULAR_SPEED = PI/6;


bool PosControl::setup()
{
    ROS_INFO("[POSECONTROL]Hover Setup ....");

//    tmp_pub1 = nh->advertise<std_msgs::Float64>("/errorX",1);
//    tmp_pub2 = nh->advertise<std_msgs::Float64>("/errorY",1);
//    trajectorData_pub=nh->advertise<std_msgs::Float32MultiArray>("/trajectory/data",10);
//    autoreturn_startRecord_pub=nh->advertise<std_msgs::Bool>("/autoreturn/record", 1);
    visionHover_pub=nh->advertise<std_msgs::Bool>("/visionhover/hover", 1);
//    setheight_pub=nh->advertise<std_msgs::Float64>("/control/set_altitude",10);
//    setKeyframe_pub=nh->advertise<std_msgs::Int16>("/autoreturn/setkeyframe",1);
//    autoreturn_startReturn_pub=nh->advertise<std_msgs::Bool>("/autoreturn/return", 10);
//    //test
//    test_pub = nh->advertise<std_msgs::Float64>("/poscontrol/test1",1);
//    test_pub2 = nh->advertise<std_msgs::Float64>("/poscontrol/test2",1);
//    test_pub3 = nh->advertise<std_msgs::Float64>("/poscontrol/test3",1);
//    test_pub4 = nh->advertise<std_msgs::Float64>("/poscontrol/test4",1);

    //data subscribe
//    mavros_sensor_velocity_sub=nh->subscribe("/mavros/local_position/velocity", 10,&PosControl::localVelocityCallback,this);
//    mavros_sensor_imu_sub=nh->subscribe("/ardrone/imu", 1,&PosControl::imuDataCallback,this);
//    control_takeoff_sub=nh->subscribe("/control/takeoff", 1,&PosControl::takeoffCallback,this);
//    control_land_sub=nh->subscribe("/control/land", 1,&PosControl::landCallback,this);
    pos_ekf_sub=nh->subscribe("/est_pose",10,&PosControl::posEKFCallback,this);
    vel_ekf_sub=nh->subscribe("/est_vel",10,&PosControl::velEKFCallback,this);
//    gtPos_ekf_sub=nh->subscribe("/vicon/est_pose",10,&PosControl::gtPosEKFCallback,this);
//    control_euler_sub=nh->subscribe("/control/euler", 10,&PosControl::controlEulerCallback,this);
//    control_yaw_sub=nh->subscribe("/control/yaw_target", 10,&PosControl::controlSetYawCallback,this);
//    control_velocity_sub=nh->subscribe("/control/velocity", 10,&PosControl::controlVelocityCallback,this);
//    control_move_sub=nh->subscribe("/control/move", 10,&PosControl::controlMoveCallback,this);
//    control_sethover_sub=nh->subscribe("/control/sethover", 10,&PosControl::setHoverCallback,this);
//    control_settrajectory_sub=nh->subscribe("/control/settrajectory", 10,&PosControl::setTrajectoryCallback,this);
//    hover_vp_sub=nh->subscribe("/control/PID/setHoverVp", 10,&PosControl::setVelpCallback,this);
//    hover_vd_sub=nh->subscribe("/control/PID/setHoverVd", 10,&PosControl::setVeldCallback,this);
//    hover_posp_sub=nh->subscribe("/control/PID/setHoverPosp", 10,&PosControl::setPospCallback,this);
//    hover_posi_sub=nh->subscribe("/control/PID/setHoverPosi", 10,&PosControl::setPosiCallback,this);
//    hover_PIDWrite_sub=nh->subscribe("/control/PID/writePosPID", 10,&PosControl::writePosPIDCallback,this);
//    hover_setpoint_sub=nh->subscribe("/control/setpoint", 20,&PosControl::setPointCallback,this);
//    control_trajectoryData_sub=nh->subscribe("/control/trajectoryData", 1000,&PosControl::trajectoryDataCallback,this);
//    visionHoverSwitch_sub=nh->subscribe("/visionhover/hoverswitch", 10,&PosControl::visionHoverSwitchDataCallback,this);

    //get target of altitude (unit:m)
    nh->getParam("gravity",GRAVITY_MSS);
    nh->getParam("max_angle",MAX_ANGLE);
    nh->getParam("poscontrol_accel_xy",POSCONTROL_ACCEL_XY);
    nh->getParam("poscontrol_speed",POSCONTROL_SPEED);
    nh->getParam("poscontrol_accel_xy_max",POSCONTROL_ACCEL_XY_MAX);
    nh->getParam("max_target_distance",MAX_TARGET_DISTANCE);
    nh->getParam("sonar_lower_lp_threshold",sonar_lower_lp_threshold);
    nh->getParam("sonar_upper_lp_threshold",sonar_upper_lp_threshold);
    nh->getParam("feedforward_para",_feedforward_para);
    nh->getParam("mave_mode",_move_mode);
    nh->getParam("posPIDPath",posPIDPath);
    nh->getParam("velPIDPath",velPIDPath);
    nh->getParam("trajectoryPIDPath",trajectoryPIDPath);
    nh->getParam("controllerType",CONTROLLERTYPE);
    nh->getParam("yawP",yawP);
    nh->getParam("yawD",yawD);
    nh->getParam("yawI",yawI);
    nh->getParam("altitudeP",altitudeP);
    nh->getParam("altitudeD",altitudeD);
    nh->getParam("altitudeI",altitudeI);
    nh->getParam("hoverPosP",hoverPosP);
    nh->getParam("hoverPosI",hoverPosI);
    nh->getParam("hoverPosD",hoverPosD);
    nh->getParam("hoverPosImax",hoverPosImax);
    nh->getParam("hoverVelP",hoverVelP);
    nh->getParam("hoverVelI",hoverVelI);
    nh->getParam("hoverVelD",hoverVelD);
    nh->getParam("hoverVelImax",hoverVelImax);
    nh->getParam("hoverVelWo",hoverVelWo);
    nh->getParam("hoverVelWc",hoverVelWc);
    nh->getParam("hoverVelB0",hoverVelB0);
    //vijay's
    nh->getParam("Kpos",Kpos);
    nh->getParam("Kvel",Kvel);
    nh->getParam("max_linear_speed", MAX_LINEAR_SPEED);
    nh->getParam("max_angular_speed", MAX_ANGULAR_SPEED);

//    fout_return.open(outfile1Path.c_str());
//    fout.open(outfile2Path.c_str());

//  ardrone.setup();  // 初始化设置

  //PID controller
    PID_x=new PIDController(hoverPosP,hoverPosI,hoverPosD,hoverPosImax);
    PID_vx=new PIDController(hoverVelP,hoverVelI,hoverVelD,hoverVelImax);//without forward feedback
    PID_y=new PIDController(hoverPosP,hoverPosI,hoverPosD,hoverPosImax);
    PID_vy=new PIDController(hoverVelP,hoverVelI,hoverVelD,hoverVelImax);//without forward feedback
    PID_yaw=new PIDController(yawP,yawI,yawD,0.2);
    PID_z=new PIDController(altitudeP,altitudeI,altitudeD,0.3);
    //Fuzzy PID controller
    FuzzyPIDX=new FuzzyPID(hoverPosP,hoverPosI,hoverPosD,hoverPosImax);
    FuzzyPIDY=new FuzzyPID(hoverPosP,hoverPosI,hoverPosD,hoverPosImax);
    FuzzyPIDVx=new FuzzyPID(hoverVelP,hoverVelI,hoverVelD,hoverVelImax);
    FuzzyPIDVy=new FuzzyPID(hoverVelP,hoverVelI,hoverVelD,hoverVelImax);
    //ADRC controller
    ADRCVx=new ADRC(hoverVelWo,hoverVelWc,hoverVelB0,1.0/controlRate);
    ADRCVy=new ADRC(hoverVelWo,hoverVelWc,hoverVelB0,1.0/controlRate);

    pidTime=ros::Time().now();
    checkOutsideControl=ros::Time().now();
    setpointLoiterTime=ros::Time().now();

    setpointPos.reserve(100);
    yaw_target=0;

     for(int i=0;i<3;++i)
     {
         acc[i]=0;
         vel[i]=0;
         posErr[i]=0;
         velErr[i]=0;
         accErr[i]=0;
         pos[i]=0;
         targetPos[i]=0;
         vel_last[i]=0;
         hoverPos[i]=0;
         trajectorVel[i]=0;
         trajectorAcc[i]=0;
     }
     for(int i=0;i<20;++i)
         {
         posX_record[i]=0,posY_record[i]=0;
     }

     //_trajectory.trajectoryPlanner(CUBIC);
    //_trajectory.trajectoryStart(1);

     ROS_INFO("[POSECONTROL]Hover Setup over!");
}
/// callback function

//void  PosControl::imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg)
//{
//    float_t acc_body[3];
//    sensor_imu.angular_velocity=msg->angular_velocity;
//    sensor_imu.linear_acceleration=msg->linear_acceleration;
//    sensor_imu.orientation=msg->orientation;


//    //transform NWU_body TO NWU_world
//    acc_body[0]=(float)(round(sensor_imu.linear_acceleration.x*100))*1.0/100.0;
//    acc_body[1]=(float)(round(sensor_imu.linear_acceleration.y*100))*1.0/100.0;
//    acc_body[2]=(float)(round(sensor_imu.linear_acceleration.z*100))*1.0/100.0;
//    transform_nwu_from_body(acc_body,acc);////transform NWU TO body
//    //remove gravity and change to nwu
//    acc[1]=acc[1];
//    acc[2]=GRAVITY_MSS-acc[2];

//}

//void PosControl::takeoffCallback(const std_msgs::Bool::ConstPtr &msg)
//{
//   if(msg->data){
//       ardrone->takeOff();
//       hoverPos[2]=0.7;
//   }
//}

//void PosControl::landCallback(const std_msgs::Bool::ConstPtr &msg)
//{
//   if(msg->data)ardrone->land();
//}

//void PosControl::localVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
//{
//    float_t vel_tmp[3];
//    sensor_local_velocity.twist=msg->twist;
//    sensor_local_velocity.header.stamp=msg->header.stamp;

//    //NWU
//    vel[0]=msg->twist.linear.x;
//    vel[1]=msg->twist.linear.y;
//    vel[2]=msg->twist.linear.z;

//   //transform_body_from_nwu(vel_tmp,vel);////transform NWU TO body

//    //fout<<vel[0]<<"  "<<vel[1]<<"  "<<vel[2]<<"  "<<endl;
//    //fout<<"vx_ned:"<<vel_tmp[0]<<"  "<<"vy_ned:"<<vel_tmp[1]<<"  ""vz_ned:"<<vel_tmp[2]<<"  "<<endl;
//    //cout<<"vx:"<<msg->twist.linear.x<<"  "<<"vy:"<<vel[1]<<"  "<<"vz:"<<vel[2]<<"  "<<endl;
//}

//void PosControl::controlMoveCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
//{
//    isControlOutside=true;
//    isSetVel=false;
//     _move_mode=1;
//    checkOutsideControl=ros::Time().now();
//    static float move_body[3],move_nwu[3];
//    commander_move.data.clear();
//     /* convert from bodyframe velocity */
//    move_body[0]=msg->data[0];
//    move_body[1]=msg->data[1];
//    move_body[2]=msg->data[2];

//    move_nwu[0]=move_body[0]*cos(yaw)-move_body[1]*sin(yaw);
//    move_nwu[1]=move_body[0]*sin(yaw)+move_body[1]*cos(yaw);

//    commander_move.data.push_back(move_nwu[0]);
//    commander_move.data.push_back(move_nwu[1]);
//    commander_move.data.push_back(move_body[2]);
//    commander_move.data.push_back(msg->data[3]);
//}

//void PosControl::controlEulerCallback(const std_msgs::Float32MultiArray::ConstPtr  &msg)
// {
//     isControlOutside=true;
//     isSetVel=false;
//     _move_mode=0;
//     checkOutsideControl=ros::Time().now();
//     commander_move.data=msg->data;
// }

//void PosControl::controlSetYawCallback(const std_msgs::Float32::ConstPtr  &msg)
//{
//    yaw_target=msg->data;
//}
// //use twist.linear.x and twist.linear.x as vx ,vy;twist.angular.z as yaw
// void PosControl::controlVelocityCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
// {
//         isControlOutside=true;
//         isSetVel=true;
//         checkOutsideControl=ros::Time().now();
//         target_velocity.data.clear();
//         static float vel_body[3],vel_nwu[3];
//          /* convert to bodyframe velocity */
//         vel_body[0]=msg->data[0];
//         vel_body[1]=msg->data[1];
//         vel_body[2]=msg->data[2];

//         vel_nwu[0]=vel_body[0]*cos(yaw)-vel_body[1]*sin(yaw);
//         vel_nwu[1]=vel_body[0]*sin(yaw)+vel_body[1]*cos(yaw);

//         target_velocity.data.push_back(vel_nwu[0]);
//         target_velocity.data.push_back(vel_nwu[1]);
//         target_velocity.data.push_back(vel_body[2]);
//         target_velocity.data.push_back(msg->data[3]);//yaw_speed
// }

 //the origin data is in NWU frame
 void PosControl::posEKFCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
 {
     sensor_local_position.pose=msg->pose;
     sensor_local_position.header.stamp=msg->header.stamp;

     qtoEuler();// in NWU frame
     getRotation();

     pos[0]=msg->pose.position.x;
     pos[1]=msg->pose.position.y;//convert to NWU frame
     pos[2]=msg->pose.position.z;

    posX_sum-=posX_record[pos_record_count];
    posY_sum-=posY_record[pos_record_count];
    posZ_sum-=posZ_record[pos_record_count];
    posX_record[pos_record_count]=pos[0];
    posY_record[pos_record_count]=pos[1];
    posZ_record[pos_record_count]=pos[2];
    pos_record_count++;
    if(pos_record_count>=MAX_COUNT_POSITION)pos_record_count=0;
    posX_sum+=pos[0];
    posY_sum+=pos[1];
    posZ_sum+=pos[2];

    //cout<<"posx:"<<pos[0]<<"  "<<"posy:"<<pos[1]<<"  "<<"posz:"<<pos[2]<<"  "<<"yaw:"<<yaw<<endl;
 }

 void PosControl::velEKFCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
 {
    vel_ekf[0]=msg->twist.linear.x;
    vel_ekf[1]=msg->twist.linear.y;
    vel_ekf[2]=msg->twist.linear.z;//change to NWU frame
 }

// void PosControl::gtPosEKFCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//     gtPos[0]=msg->pose.position.x;
//     gtPos[1]=msg->pose.position.y;
//     gtPos[2]=msg->pose.position.z;
// }

// void PosControl::setVelpCallback(const std_msgs::Float32::ConstPtr &msg)
// {
//        PID_vx->kP(msg->data);
//        PID_vy->kP(msg->data);
//        FuzzyPIDVx->kP(msg->data);
//        FuzzyPIDVy->kP(msg->data);
//        cout<<"velKp:"<<PID_vx->kP()<<endl;
// }

// void PosControl::setVeldCallback(const std_msgs::Float32::ConstPtr &msg)
// {
//        PID_vx->kD(msg->data);
//        PID_vy->kD(msg->data);
//        FuzzyPIDVx->kD(msg->data);
//        FuzzyPIDVy->kD(msg->data);
//        cout<<"velKd:"<<PID_vx->kP()<<endl;
// }

// void PosControl::setPospCallback(const std_msgs::Float32::ConstPtr &msg)
// {
//     PID_x->kP(msg->data);
//     PID_y->kP(msg->data);
//     FuzzyPIDX->kP(msg->data);
//     FuzzyPIDY->kP(msg->data);
//     cout<<"posKd:"<<PID_vx->kP()<<endl;
// }

// void PosControl::setPosiCallback(const std_msgs::Float32::ConstPtr &msg)
// {
//     PID_x->kI(msg->data);
//     PID_y->kI(msg->data);
//     FuzzyPIDX->kI(msg->data);
//     FuzzyPIDY->kI(msg->data);
//     cout<<"posKi:"<<PID_vx->kP()<<endl;
// }

// void PosControl::writePosPIDCallback(const std_msgs::Bool::ConstPtr &msg)
// {
//    if(msg->data){
//        PID_x->save_gains(posPIDPath.c_str());
//        PID_vx->save_gains(velPIDPath.c_str());//x,y use the same parameters
//    }
// }

// void PosControl::setPointCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
//{
//     Point pos;
//      pos.x=msg->data[0];
//      pos.y=msg->data[1];
//      pos.z=msg->data[2];
//      pos.yaw=msg->data[3];
//      pos.loiter_time=msg->data[4];
//      setpointPos.push_back(pos);
//      isSetPoint=true;
//      isControlOutside=false;
//}

// void PosControl::trajectoryDataCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
// {
//     trajectoryPoint tmpPoint;
//     tmpPoint.x=msg->data[0];
//     tmpPoint.y=msg->data[1];
//     tmpPoint.z=msg->data[2];
//     tmpPoint.yaw=msg->data[3];
//     tmpPoint.reach_time=msg->data[4];
//     if(msg->data[4]<=0){
//         if(fabs(msg->data[4])<=1e-6){
//             _trajectory.trajectorySequence.clear();
//             _trajectory.trajectorySequence.push_back(tmpPoint);
//         }
//         else  _trajectory.trajectoryPlanner(_trajectory.chooseFlag);
//     }
//     else{
//         if(_trajectory.writePoints)trajectoryPointsfile_hover<<tmpPoint.x<<" "<<tmpPoint.y<<" "<<tmpPoint.z<<" "<<tmpPoint.yaw<<" "<<tmpPoint.reach_time<<endl;
//         _trajectory.trajectorySequence.push_back(tmpPoint);
//     }

// }

// void PosControl::setTrajectoryCallback(const std_msgs::Bool::ConstPtr &msg)
// {
//    isControlOutside=false;
//    if(msg->data){
//        isSetTrajectory=true;
//        _trajectory.trajectoryStartPoint.x=pos[0];
//        _trajectory.trajectoryStartPoint.y=pos[1];
//    }
//    else {
//        _trajectory.trajectoryStart(0);
//        isSetTrajectory=false;
//    }
// }

// void PosControl::visionHoverSwitchDataCallback(const std_msgs::Bool::ConstPtr &msg)
// {
//     if(msg->data)VISION_HOVER_SWITCH=1;
//     else VISION_HOVER_SWITCH=0;
// }

/// control
///
///
///
 float PosControl::safe_sqrt(float v)
 {
     float ret = sqrtf(v);
     if (isnan(ret)) {
         return 0;
     }
     return ret;
 }
 // constrain a value
 float PosControl::constrain_float(float amt, float low, float high)
 {
     // the check for NaN as a float prevents propogation of
     // floating point errors through any function that uses
     // constrain_float(). The normal float semantics already handle -Inf
     // and +Inf
     if (isnan(amt)) {
         return (low+high)*0.5f;
     }
     return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
 }

 float PosControl::fast_atan(float v)
 {
     float v2 = v*v;
     return (v*(1.6867629106f + v2*0.4378497304f)/(1.6867633134f + v2));
 }

 float PosControl::trapezoidalIntegral(float dt,float v_last,float v_now)
 {
     return (v_last+v_now)/2*dt;
 }
 /// pos_to_rate_xy - horizontal position error to velocity controller
 ///     converts position (_pos_target) to target velocity (_vel_target)
 ///     when use_desired_rate is set to true:
 ///         desired velocity (_vel_desired) is combined into final target velocity and
 ///         velocity due to position error is reduce to a maximum of 1m/s
 void PosControl::pos_to_rate_xy(float dt,float ekfNavVelGainScaler,double vel_target[3])
 {
     float linear_distance;      // the distance we swap between linear and sqrt velocity response
     float kP =  ekfNavVelGainScaler *PID_x->kP(); // scale gains to compensate for noisy optical flow measurement in the EKF
     float px_i, py_i;
     // avoid divide by zero
     if (kP <= 0.0f) {
         vel_target[0]= 0.0f;
         vel_target[1] = 0.0f;
     }else{
         // calculate distance error
         posErr[0] = targetPos[0] - pos[0];
         posErr[1] = targetPos[1] - pos[1];


         // constrain target position to within reasonable distance of current location
         float _distance_to_target = sqrtf(posErr[0]*posErr[0]+posErr[1]*posErr[1]);
         if (_distance_to_target > MAX_TARGET_DISTANCE&& _distance_to_target > 0.0f) {
             targetPos[0] = pos[0] + MAX_TARGET_DISTANCE* posErr[0]/_distance_to_target;
             targetPos[1] = pos[1] + MAX_TARGET_DISTANCE* posErr[1]/_distance_to_target;
             // re-calculate distance error
             posErr[0] = targetPos[0]  - pos[0];
             posErr[1] = targetPos[1]  - pos[1];
             _distance_to_target = MAX_TARGET_DISTANCE;
         }

//         tmp.data=posErr[0];
//         tmp_pub1.publish(tmp);
//         tmp.data=posErr[1];
//         tmp_pub2.publish(tmp);
         if(CONTROLLERTYPE==FUZZYPIDCONTROL){
             vel_target[0]= FuzzyPIDX->get_pid(posErr[0], dt);
             vel_target[1] = FuzzyPIDY->get_pid(posErr[1], dt);
         }
         else{
             // calculate the distance at which we swap between linear and sqrt velocity response
             linear_distance = POSCONTROL_ACCEL_XY/(2.0f*kP*kP);

             if (_distance_to_target > 2.0f*linear_distance) {
                 // velocity response grows with the square root of the distance
                 float vel_sqrt = safe_sqrt(2.0f*POSCONTROL_ACCEL_XY*(_distance_to_target-linear_distance));
                 vel_target[0] = vel_sqrt * posErr[0]/_distance_to_target;
                 vel_target[1] = vel_sqrt * posErr[1]/_distance_to_target;
             }else{
                 // velocity response grows linearly with the distance
                 vel_target[0] = PID_x->kP()* posErr[0];
                 vel_target[1] = PID_y->kP()* posErr[1];
             }

             px_i = PID_x->get_i(posErr[0], dt);
             py_i = PID_y->get_i(posErr[1], dt);

             vel_target[0] += px_i;
             vel_target[1] += py_i;
         }
             // this mode is for loiter - rate-limiting the position correction
             // allows the pilot to always override the position correction in
             // the event of a disturbance

             // scale velocity within limit
             float vel_total = sqrtf(vel_target[0]*vel_target[0]+vel_target[1]*vel_target[1]);
             if (vel_total > POSCONTROL_SPEED) {
                 vel_target[0] = POSCONTROL_SPEED * vel_target[0]/vel_total;
                 vel_target[1] = POSCONTROL_SPEED * vel_target[1]/vel_total;
             }
         }
  }
 /// rate_to_accel_xy - horizontal desired rate to desired acceleration
 ///    converts desired velocities in lat/lon directions to accelerations in lat/lon frame
 void PosControl::rate_to_accel_xy(float dt, float ekfNavVelGainScaler,double vel_target[3],double acc_target[3])
 {
     float accel_total;                          // total acceleration in m/s/s
     float vx_i=0, vy_i=0;
    float _accel_feedforward[2];
    static float lastAccTarget[3]={0};
     // reset last velocity target to current target
//     if (_flags.reset_rate_to_accel_xy) {
//         vel_last[0] = vel_target[0];
//         vel_last[1] = vel_target[1];
//         _flags.reset_rate_to_accel_xy = false;
//     }

     // feed forward desired acceleration calculation
     if (dt > 0.0f) {
             _accel_feedforward[0]=(vel_target[0] - vel_last[0])/dt;
             _accel_feedforward[1] = (vel_target[1] - vel_last[1])/dt;
     } else {
         _accel_feedforward[0] = 0.0f;
         _accel_feedforward[1] = 0.0f;
     }

     // store this iteration's velocities for the next iteration
//here is problem: vel_last stores last iteration's vel, that should be vel_ekf not vel_target?
     vel_last[0] = vel_target[0];
     vel_last[1] = vel_target[1];

     // calculate velocity error
     velErr[0] = vel_target[0] - vel_ekf[0];
     velErr[1] = vel_target[1]- vel_ekf[1];

     // get current i term
//     vx_i = PID_vx->get_integrator();
//     vy_i = PID_vy->get_integrator();

     // update i term if we have not hit the accel or throttle limits OR the i term will reduce
     //if ( ((vx_i>0&&velErr[0]<0)||(vx_i<0&&velErr[0]>0))) {
//         vx_i = PID_vx->get_i(velErr[0], dt);
     //}
    // if ( ((vy_i>0&&velErr[1]<0)||(vy_i<0&&velErr[1]>0))) {
//         vy_i = PID_vy->get_i(velErr[1], dt);
    // }

     // combine feed forward accel with PID output from velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
     switch(CONTROLLERTYPE){
     case PIDCONTROL:{
         acc_target[0] = _feedforward_para*_accel_feedforward[0] + ( PID_vx->get_pid(velErr[0], dt)) * ekfNavVelGainScaler;
         acc_target[1] = _feedforward_para*_accel_feedforward[1] + (PID_vy->get_pid(velErr[1], dt)) * ekfNavVelGainScaler;
         break;
     }
     case FUZZYPIDCONTROL:{
         acc_target[0] = _feedforward_para*_accel_feedforward[0] + ( FuzzyPIDVx->get_pid(velErr[0], dt)) * ekfNavVelGainScaler;
         acc_target[1] = _feedforward_para*_accel_feedforward[1] + (FuzzyPIDVy->get_pid(velErr[1], dt)) * ekfNavVelGainScaler;
         break;
     }
     case ADRCCONTROL:{
         acc_target[0] =   ADRCVx->Update(lastAccTarget[0], vel_ekf[0],vel_target[0]);
         acc_target[1] = ADRCVy->Update(lastAccTarget[1],vel_ekf[1],vel_target[1]);
         lastAccTarget[0]=acc_target[0];
         lastAccTarget[1]=acc_target[1];
         break;
     }
     default:
         acc_target[0] = _feedforward_para*_accel_feedforward[0] + ( PID_vx->get_pid(velErr[0], dt)) * ekfNavVelGainScaler;
         acc_target[1] = _feedforward_para*_accel_feedforward[1] + (PID_vy->get_pid(velErr[1], dt)) * ekfNavVelGainScaler;
         break;
     }

//     //test
//     for(int i=0;i<2;++i)
//     {
//         if((acc_target[i]*vel_target[i]<0) && (vel_target[i]*(targetPos[i]-pos[i])<0))
//             acc_target[i] *= 1.2;
//     }
//     //test^
     // scale desired acceleration if it's beyond acceptable limit
     // To-Do: move this check down to the accel_to_lean_angle method?
     accel_total = sqrtf(acc_target[0]*acc_target[0]+acc_target[1]*acc_target[1]);
     if (accel_total > POSCONTROL_ACCEL_XY_MAX && accel_total > 0.0f) {
         acc_target[0]= POSCONTROL_ACCEL_XY_MAX * acc_target[0]/accel_total;
         acc_target[1]  = POSCONTROL_ACCEL_XY_MAX * acc_target[1]/accel_total;
     }

 }

 /// accel_to_lean_angles - horizontal desired acceleration to lean angles
 ///    converts desired accelerations provided in lat/lon frame to roll/pitch angles
void PosControl::accel_to_lean_angles(float dt, float ekfNavVelGainScaler,double acc_target[3])
 {

    float accel_north = acc_target[0];
    float accel_west = acc_target[1];
    float accel_left, accel_forward;
    float lean_angle_max = MAX_ANGLE;

    // apply jerk limit of 17 m/s^3 - equates to a worst case of about 100 deg/sec/sec
    static float last_accel_north = 0.0f;
    static float last_accel_west = 0.0f;
    float max_delta_accel = dt * 17.0f;
    if (accel_north - last_accel_north > max_delta_accel) {
        accel_north = last_accel_north + max_delta_accel;
    } else if (accel_north - last_accel_north < -max_delta_accel) {
        accel_north = last_accel_north - max_delta_accel;
    }
    last_accel_north = accel_north;

    if (accel_west - last_accel_west > max_delta_accel) {
        accel_west = last_accel_west + max_delta_accel;
    } else if (accel_west - last_accel_west < -max_delta_accel) {
        accel_west = last_accel_west - max_delta_accel;
    }
    last_accel_west = accel_west;

    // 5Hz lowpass filter on NW accel
    float freq_cut = 5.0f * ekfNavVelGainScaler;
    float alpha = constrain_float(dt/(dt + 1.0f/(2.0f*(float)M_PI*freq_cut)),0.0f,1.0f);
    static float accel_north_filtered = 0.0f;
    static float accel_west_filtered = 0.0f;
    accel_north_filtered += alpha * (accel_north - accel_north_filtered);
    accel_west_filtered  += alpha * (accel_west - accel_west_filtered);

    //rotate accelerations into body forward-left frame
    accel_forward = accel_north_filtered*cos(yaw) + accel_west_filtered*sin(yaw);//yaw is in NWU frame
    accel_left = -accel_north_filtered*sin(yaw) +accel_west_filtered*cos(yaw);

//     float px_forward = pos[0]*cos(yaw) - pos[1]*sin(yaw);
//     float py_left= -pos[0]*sin(yaw) - pos[1]*cos(yaw);

     // update angle targets that will be passed to stabilize controller in nwu frame
     pitch_target =constrain_float(atan2(accel_forward,GRAVITY_MSS),-lean_angle_max, lean_angle_max);
     float cos_pitch_target = cosf(pitch_target);
     roll_target= -constrain_float(atan2(accel_left*cos_pitch_target,GRAVITY_MSS), -lean_angle_max, lean_angle_max);
}

// position error to lean angles(method from vijay kumar)
void PosControl::posrate_to_lean_angles(double vel_target[3], double acc_target[3])
{
    posErr[0] = pos[0] - targetPos[0];
    posErr[1] = pos[1] - targetPos[1];
    posErr[2] = pos[2] - targetPos[2];
    velErr[0] = vel_ekf[0] - vel_target[0];
    velErr[1] = vel_ekf[1] - vel_target[1];
    velErr[2] = vel_ekf[2] - vel_target[2];
    Eigen::Vector3d pErr, vErr, aTarget, Ades, xBdes, yBdes, zBdes, xCdes;
    pErr << posErr[0], posErr[1], posErr[2];
    vErr << velErr[0], velErr[1], velErr[2];
    aTarget << acc_target[0], acc_target[1], acc_target[2];

    //Kpos-K of pos pid; Kvel-K of vel pidKpos = 1,Kvel = 1,
    double Gravity = 9.8;
    //desired force vector
    Ades = -Kpos*pErr - Kvel* vErr + Gravity*Eigen::Vector3d(0,0,1)
            + aTarget;
           //+ Eigen::Vector3d(pos[0],pos[1],pos[2]);
    zBdes = Ades / sqrt(Ades.dot(Ades));
    xCdes << cos(yaw_target), sin(yaw_target), 0;
    yBdes = zBdes.cross(xCdes) / sqrt(zBdes.cross(xCdes).dot(zBdes.cross(xCdes)));
    xBdes = yBdes.cross(zBdes);
    //R to euler
//    roll_target = atan2(yBdes(2), zBdes(2));
//    pitch_target = asin(-xBdes(2));
//    yaw_target = atan2(xBdes(1), xBdes(0));
    float _q[4];
    double _r[9] = {xBdes(0),yBdes(0),zBdes(0),xBdes(1),yBdes(1),zBdes(1),xBdes(2),yBdes(2),zBdes(2)};
    cv::Mat _R(3, 3, CV_64F, _r);
    //cv::Mat _R = (cv::Mat<float>(3,3)<<xBdes(0),yBdes(0),zBdes(0),xBdes(1),yBdes(1),zBdes(1),xBdes(2),yBdes(2),zBdes(2));;
    rotationToQ(_q, _R);
    float test_roll_target, test_pitch_target, test_yaw_target;
    qToEuler(test_roll_target, test_pitch_target, test_yaw_target, _q);
//    printf("origin a(%f,%f,%f),vijay a(%f,%f,%f)\n",
//           acc[0], acc[1], acc[2], Ades(0), Ades(1), Ades(2)-Gravity);
//    printf("origin(%f,%f,%f), vijay(%f,%f,%f)\n",
//           //targetPos[0], targetPos[1], targetPos[2],target pos(%f,%f,%f),act pos(%f,%f,%f)\ntarget vel(%f,%f,%f),act vel(%f,%f,%f)\n
//           //pos[0], pos[1], pos[2],
//           //vel_target[0], vel_target[1], vel_target[2],
//           //vel_ekf[0],vel_ekf[1],vel_ekf[2],
//           roll_target, pitch_target, yaw_target,
//           test_roll_target, test_pitch_target, test_yaw_target);
//    tmp.data = roll_target;// - test_roll_target;
//    test_pub.publish(tmp);
//    tmp.data = test_roll_target;//pitch_target - test_pitch_target;
//    test_pub2.publish(tmp);
//    tmp.data = pitch_target;
//    test_pub3.publish(tmp);
//    tmp.data = test_pitch_target;
//    test_pub4.publish(tmp);
    roll_target = test_roll_target;
    pitch_target = test_pitch_target;
//    yaw_target = test_yaw_target;
}

//another method for hover acc
//void PosControl::get_hover_vel_acc(double vel_target[3], double acc_target[3])
//{
//    //double amax = 2, t = 0.02;
//    float errP;
//    double vt[3],at[3];
//    for(int i = 0; i < 2; ++i)
//    {
//        /*do{
//             acc_target[i] = (targetPos[0] - pos[0] - vel_ekf[i]*t)*2/pow(t,2);
//             t += 0.02;
//        }while(acc_target[i]>=amax);
//        t = 0.02;*/
//        errP = targetPos[0] - pos[0];
//        if(errP == 0)
//        {
//            at[i] = - vel_ekf[i] / 0.02;
//            vt[i] = 0;
//        }
//        else if(vel_ekf == 0)
//        {
//            at[i] = errP / pow(0.02, 2);
//            vt[i] = at[i] * 0.02;
//        }
//        else if(errP*vel_ekf[i] > 0)
//        {
//            at[i] = - pow(vel_ekf[i], 2) / 2 / errP;
//            vt[i] = 0;
//        }
//        else if(errP*vel_ekf[i] < 0)
//        {
//            at[i] = - vel_ekf[i] / 0.02;
//            vt[i] = 0;
//        }
//    }
//    vt[2] = 0;
//    at[2] = 0;
//    printf("ori v(%f,%f,%f),a(%f,%f,%f)\ncal v(%f,%f,%f),a(%f,%f,%f)\n",
//           vel_target[0],vel_target[1],vel_target[2],
//           acc_target[0],acc_target[1],acc_target[2],
//           vt[0],vt[1],vt[2],at[0],at[1],at[2]
//           );
//}

bool PosControl::isOnArrival(float targetPosX,float targetPosY,float targetPosZ)
{
    if((fabs(targetPosX-posX_sum/MAX_COUNT_POSITION)<0.1)&&(fabs(targetPosY-posY_sum/MAX_COUNT_POSITION)<0.1)&&(fabs(targetPosZ-posZ_sum/MAX_COUNT_POSITION)<0.1))
        return true;
     else return false;
}

//void PosControl::setHoverCallback(const std_msgs::Bool::ConstPtr &msg)
//{
//    reset();
//    isSetPoint=false;
//    isSetTrajectory=false;//could not be placed into reset funstion, this will cause setpoint and trajectory fail
//    setHover=msg->data;
//    yaw_target=yaw;
//}

/// assist function
void PosControl::reset()
{
    ROS_INFO("Reset!!!");
    for(int i=0;i<3;++i)
    {
        posErr[i]=0;
        velErr[i]=0;
        accErr[i]=0;
        vel_last[i]=0;
        hoverPos[i]=pos[i];
    }
    for(int i=0;i<20;++i){
        posX_record[i]=0,posY_record[i]=0;posZ_record[i]=0;
    }
    setpointPos.clear();
    posX_sum=0;
    posY_sum=0;
    posZ_sum=0;
    roll_target=0;
    pitch_target=0;
    PID_x->reset_I();
    PID_vx->reset_I();
    PID_y->reset_I();
    PID_vy->reset_I();
    isVisionHover=false;
    isControlOutside=false;
    isSetVel=false;
}

/// actuator
void PosControl::setState(float *_posNow, float *_velNow, float _yawNow)
{
    for(int i=0;i<3;++i)
    {
        vel_last[i] = vel_ekf[i];
        pos[i] = _posNow[i];
        vel_ekf[i] = _velNow[i];
    }
    yaw = _yawNow;
}

void PosControl::takeOff()
{
    drone.takeOff();
}

void PosControl::land()
{
    drone.land();
}

void PosControl::doHover(double dt, float *_poseTarget, float _yawTarget,
                         float *_poseNow, float *_velNow, float _yawNow)
{
    static double vel_target[3],acc_target[3];

    if(VISION_HOVER_SWITCH){
        if(!isVisionHover){
            std_msgs::Bool visionHoverData;
            visionHoverData.data=true;
            visionHover_pub.publish(visionHoverData);
            isVisionHover=true;
        }
    }

    for(int i=0;i<3;++i)
    {
        targetPos[i] = _poseTarget[i];
        pos[i] = _poseNow[i];
        vel_ekf[i] = _velNow[i];
    }
    yaw = _yawNow;
    yaw_target = _yawTarget;

#if PIONEER3AT
    //doMove(dt,0,0,0,0);//drone.move(0, 0);
#else
    pos_to_rate_xy(dt,1,vel_target);
    rate_to_accel_xy( dt, 1,vel_target,acc_target);
    float acc_body[2];
    acc_body[0]=acc_target[0]*cos(yaw)+acc_target[1]*sin(yaw);
    acc_body[1]=-acc_target[0]*sin(yaw)+acc_target[1]*cos(yaw);
    pitch_target=atan2(acc_body[0],GRAVITY_MSS);
    roll_target=-atan2(acc_body[1],GRAVITY_MSS);

    //accel_to_lean_angles( dt, 1,acc_target);
    CLIP3(-MAX_ANGLE,pitch_target,MAX_ANGLE);
    CLIP3(-MAX_ANGLE,roll_target,MAX_ANGLE);

    float yawErr=0;
    if(yaw_target*yaw>=0)yawErr=yaw_target-yaw;
    else{
        if(fabs(yaw_target-yaw)<=M_PI)yawErr=yaw_target-yaw;
        else {
            if(yaw_target-yaw<0)yawErr=2*M_PI-fabs(yaw_target-yaw);
            else yawErr=fabs(yaw_target-yaw)-2*M_PI;
        }
    }
    float altErr=targetPos[2]-pos[2];
    float yaw_speed=PID_yaw->get_pid(yawErr,dt);
    float upd=PID_z->get_pid(altErr,dt);

    //test by xue
//    tmp.data = targetPos[0];
//    test_pub.publish(tmp);
//    tmp.data = pos[0];
//    test_pub2.publish(tmp);
//    tmp.data = targetPos[1];
//    test_pub3.publish(tmp);
//    tmp.data = pos[1];
//    test_pub4.publish(tmp);
//    get_hover_vel_acc(vel_target, acc_target);
//    vel_target[2] = upd;
//    acc_target[2] = 0.3*upd;
//    posrate_to_lean_angles(vel_target, acc_target);
//    CLIP3(-MAX_ANGLE,pitch_target,MAX_ANGLE);
//    CLIP3(-MAX_ANGLE,roll_target,MAX_ANGLE);
    //test by xue^

    drone.move(roll_target,pitch_target, upd, yaw_speed);
#endif
//    cout<<"hover:"<<"hoverPos[0]:"<<hoverPos[0]<<"  "<<"hoverPos[1]:"<<hoverPos[1]<<"  "<<"px:"<<pos[0]<<"  "<<"py"<<pos[1]<<"  "<<"targetPos[2]:"<<targetPos[2]<<"  "
//       <<"pos[2]:"<<pos[2]<<"  "<<"velErr[0]:"<<velErr[0]<<"  "<<"velErr[1]:"<<velErr[1]<<endl;
//    fout<<pos[0]<<"   "<<pos[1]<<"  "<<pos[2]<<"  "<<hoverPos[0]<<"   "<<hoverPos[1]<<"  "<<1.5*roll_target<<"    "<<1.5*pitch_target<<"    "<<vel_ekf[0]<<"  "<<vel_ekf[1]<<"  "<<vel_target[0]
//                  <<"  "<<vel_target[1]<<endl;
#if DEBUG
  cout<<"roll:"<<roll<<"  "<<"pitch:"<<pitch<<"  "<<"yaw:"<<yaw<<endl;
   cout<<"acc[0]:"<<acc[0]<<"  "<<"acc[1]:"<<acc[1]<<"  "<<"acc[2]:"<<acc[2]<<endl;
   // cout<<"vel[0]:"<<vel[0]<<"  "<<"vel[1]:"<<vel[1]<<"  "<<"vel[2]:"<<vel[2]<<endl;
#endif
}

void PosControl::doSetPoint(double dt)
{
#if PIONEER3AT
    drone.move(0, 0);
#else
    static double vel_target[3],acc_target[3];
    if(setpointPos.size()!=0) {
        targetPos[0] =setpointPos[0].x+hoverPos[0];
        targetPos[1] =setpointPos[0].y+hoverPos[1];
        targetPos[2]=setpointPos[0].z;
        yaw_target=setpointPos[0].yaw;
        pos_to_rate_xy(dt,1,vel_target);
        rate_to_accel_xy( dt, 1,vel_target,acc_target);
        accel_to_lean_angles( dt, 1,acc_target);
        //test by xue
        //posrate_to_lean_angles(vel_target,acc_target);
        //test by xue^
        CLIP3(-MAX_ANGLE,pitch_target,MAX_ANGLE);
        CLIP3(-MAX_ANGLE,roll_target,MAX_ANGLE);

        float yawErr=0;
        if(yaw_target*yaw>=0)yawErr=yaw_target-yaw;
        else{
            if(fabs(yaw_target-yaw)<=M_PI)yawErr=yaw_target-yaw;
            else {
                if(yaw_target-yaw<0)yawErr=2*M_PI-fabs(yaw_target-yaw);
                else yawErr=fabs(yaw_target-yaw)-2*M_PI;
            }
        }
        float altErr=targetPos[2]-pos[2];
        float yaw_speed=PID_yaw->get_pid(yawErr,dt);
        float upd=PID_z->get_pid(altErr,dt);
        drone.move(roll_target, pitch_target, upd, yaw_speed);

        //judge whether get the target point and loiter for specified time
        if(!isReachPoint){//when it doesn't reach the point
            if(isOnArrival(targetPos[0],targetPos[1],targetPos[2])){
                isReachPoint=true;
                setpointLoiterTime=ros::Time().now();
                }
        }
        else{//after reach the target point, do loitering
            if((ros::Time().now().toSec()-setpointLoiterTime.toSec())>=setpointPos[0].loiter_time){
                setpointPos.erase(setpointPos.begin());
                isReachPoint=false;
            }
        }
        //close vision hover
        if(VISION_HOVER_SWITCH){
            if(isVisionHover){
                std_msgs::Bool visionHoverData;
                visionHoverData.data=false;
                visionHover_pub.publish(visionHoverData);
                isVisionHover=false;
            }
        }
//      cout<<"setpoint:"<<"targetPos[0]:"<<targetPos[0]<<"  "<<"targetPos[1]:"<<targetPos[1]<<"  "<<"px:"<<pos[0]<<"  "<<"py"<<pos[1]<<"  "<<"hoverPos[0]:"<<hoverPos[0]<<"  "
//          <<"hoverPos[1]:"<<hoverPos[1]<<"  "<<"setpointPos[0].x:"<<setpointPos[0].x<<"  "<<"setpointPos[0].y:"<<setpointPos[0].y<<endl;
//        fout<<pos[0]<<"   "<<pos[1]<<"  "<<pos[2]<<"  "<<hoverPos[0]<<"   "<<hoverPos[1]<<"  "<<vel[0]<<"  "<<vel[1]<<"  "<<acc[0] <<"  "<<acc[1]<<"  "<<acc[2]<<endl;
    }
    else{
        for(int i=0;i<3;++i)hoverPos[i] =pos[i];
        isSetPoint=false;
    }
#endif
}

void PosControl::doSetVelocity(double dt)
{
#if PIONEER3AT
    drone.move(0, 0);
#else
    ROS_INFO("[POSECONTROL]Control by human on velocity!");

    static double vel_target[3],acc_target[3];
    vel_target[0]=target_velocity.data[0],vel_target[1]=target_velocity.data[1];
    if(sqrtf(vel_target[0]*vel_target[0]+vel_target[1]*vel_target[1])<0.01){
        if(sqrtf(vel_ekf[0]*vel_ekf[0]+vel_ekf[1]*vel_ekf[1])<0.5){
            latchForVel=true;
        }
    }
    if(sqrtf(vel_target[0]*vel_target[0]+vel_target[1]*vel_target[1])>0.05)latchForVel=false;
    if(latchForVel)  {
        //doHover(dt);
        return;
    }

    //close vision hover
    if(VISION_HOVER_SWITCH){
        if(isVisionHover){
            std_msgs::Bool visionHoverData;
            visionHoverData.data=false;
            visionHover_pub.publish(visionHoverData);
            isVisionHover=false;
        }
    }

    rate_to_accel_xy( dt, 1,vel_target,acc_target);
    accel_to_lean_angles( dt, 1,acc_target);
    //test by xue
    //posrate_to_lean_angles(vel_target,acc_target);
    //test by xue^
    CLIP3(-MAX_ANGLE,pitch_target,MAX_ANGLE);
    CLIP3(-MAX_ANGLE,roll_target,MAX_ANGLE);

    drone.move(roll_target, pitch_target, target_velocity.data[2], target_velocity.data[3]);
    yaw_target=yaw;
    for(int i=0;i<3;++i)hoverPos[i] =pos[i];
#endif
}

void PosControl::brake(double dt)
{
#if PIONEER3AT
    drone.move(0, 0);
#else
    float _fb, _lr;
    float vel_body[2] = {vel_ekf[0]*cos(yaw) + vel_ekf[1]*sin(yaw),
                         -vel_ekf[0]*sin(yaw) + vel_ekf[1]*cos(yaw)};
    float absvel0 = fabs(vel_body[0]), absvel1 = fabs(vel_body[1]);
    if(absvel0 > absvel1)
    {
        _fb = -vel_body[0] / absvel0;
        _lr = -vel_body[1] / absvel0;
    }
    else
    {
        _fb = -vel_body[0] / absvel1;
        _lr = -vel_body[1] / absvel1;
    }
    doMove(dt, _fb, _lr, 0, 0);
#endif
}

void PosControl::calYaw(float vx, float vy, float vz, float &yaw)
{
    float L = sqrt(pow(vx,2) + pow(vy,2));
    if(vy > 0)
    {
        yaw = acos(vx / L);
    }
    else
    {
        yaw = -acos(vx / L);
    }
}

//cal err yaw
float PosControl::calYawSpeed(float yaw_tar, float yaw_now)
{
    float yErr = yaw_tar - yaw_now;
    if(yErr > 3.1415)
    {
        yErr -= 2*3.1415;
    }
    else if(yErr < -3.1415)
    {
        yErr += 2*3.1415;
    }

    return yErr;
}

// do not use this
void PosControl::doMoveInWorld(double dt, float _vx, float _vy, float _vz, float _vyaw)
{
    if(_vx == 0 && _vy == 0 && _vyaw == 0){
        drone.move(0, 0);
        return;
    }
    float _yaw;
    calYaw(_vx, _vy, _vz, _yaw);
    float vyaw = calYawSpeed(_yaw, yaw);
#if PIONEER3AT
    //cout<<"yaw err="<<_yaw<<", vyaw="<<vyaw<<", _vyaw="<<_vyaw<<endl;
    //float v = sqrt(_vx*_vx + _vy*_vy) * (1 - fabs(vyaw)/PI);
    /*float v;
    if(false){//fabs(vyaw) > M_PI/2){
        v=0;
    }
    else{
        v = sqrt(_vx*_vx + _vy*_vy);
    }
    float rz = vyaw/M_PI*MAX_ANGULAR_SPEED*2;
    
    if(!p3atOA.modifyVel(v, rz, vyaw, true)){
        if(fabs(vyaw) > M_PI/2){
            v = 0;
        }
        else
            v = v * (1 - fabs(vyaw)*2/M_PI);
    }*/

    //cout<<"control:"<<v<<", "<<rz<<endl;
//    drone.move(v, rz);
#else
    double vel_target[3], acc_target[3];
    targetPos[0] += _vx;
    targetPos[1] += _vy;
    targetPos[2] += _vz;

    pos_to_rate_xy(dt, 1, vel_target);
    rate_to_accel_xy(dt, 1, vel_target, acc_target);
    accel_to_lean_angles(dt, 1, acc_target);
    drone.move(roll_target, pitch_target, 2*_vz, vyaw * 0.5);
#endif
}

void PosControl::doMove(double dt, float _forback, float _leftright, float _updown, float _turnlr)
{
    if(_forback > 1.01)
        _forback = 1;
    else if(_forback < -1.01)
        _forback = -1;

    if(_leftright > 1.01)
        _leftright = 1;
    else if(_leftright < -1.01)
        _leftright = -1;

    if(_updown > 1.01)
        _updown = 1;
    else if(_updown < -1.01)
        _updown = -1;

    if(_turnlr > 1.01)
        _turnlr = 1;
    else if(_turnlr < -1.01)
        _turnlr = -1;

//#if PIONEER3AT
    float vx = _forback*MAX_LINEAR_SPEED, rz = _turnlr*MAX_ANGULAR_SPEED;
    float tvx=vx, trz=rz;
//    p3atOA.modifyVel(tvx, trz, 0, false);
    drone.move(vx, rz);
/*#else
    double vel_nwu[3] = {_forback*cos(yaw) - _leftright*sin(yaw),
                         _forback*sin(yaw) + _leftright*cos(yaw), _updown*0.35};
    double vel_target[3], acc_target[3];
    for(int i=0; i<3; ++i)
        targetPos[i] += vel_nwu[i]/2;

    pos_to_rate_xy(dt, 1, vel_target);
    rate_to_accel_xy(dt, 1, vel_target, acc_target);
    accel_to_lean_angles(dt, 1, acc_target);
    drone.move(roll_target, pitch_target, 2*vel_nwu[2], _turnlr * 0.5);
#endif*/
}
//void PosControl::doMove(double dt)
//{
//    //ROS_INFO("[POSECONTROL]Control by human!");
//    static double vel_target[3],acc_target[3];
//    //cout<<"controlOutside:"<<"Pos[0]:"<<pos[0]<<"  "<<"Pos[1]:"<<pos[1]<<"Pos[2]:"<<pos[2]<<endl;
//    float controlSum=(commander_move.data[0]*commander_move.data[0]+commander_move.data[1]*commander_move.data[1]+commander_move.data[2]*commander_move.data[2]+commander_move.data[3]*commander_move.data[3]);
//    switch(_move_mode){
//    case MOVE_BY_ATTITUDE:{
//        if(controlSum<0.0001){
//            if(!latchForVel){
//                static double vel_target[3],acc_target[3];
//                vel_target[0]=0,vel_target[1]=0,vel_target[2]=0;
//                rate_to_accel_xy( dt, 1,vel_target,acc_target);
//                accel_to_lean_angles( dt, 1,acc_target);
//                //test by xue
//                //posrate_to_lean_angles(vel_target, acc_target);
//                //test by xue^
//                CLIP3(-MAX_ANGLE,pitch_target,MAX_ANGLE);
//                CLIP3(-MAX_ANGLE,roll_target,MAX_ANGLE);

//                float yaw_speed=0;
//                ardrone->move(roll_target, pitch_target, vel_target[2], yaw_speed);
//                yaw_target=yaw;
//                for(int i=0;i<3;++i)hoverPos[i] =pos[i];
//                if(sqrtf(vel_ekf[0]*vel_ekf[0]+vel_ekf[1]*vel_ekf[1])<0.1){
//                    latchForVel=true;
//                }
//                return;
//            }
//        }
//        if(controlSum>0.0001)latchForVel=false;
//        if(latchForVel)  {
//            doHover(dt);
//            return;
//        }
//        //close vision hover
//        if(VISION_HOVER_SWITCH){
//            if(isVisionHover){
//                std_msgs::Bool visionHoverData;
//                visionHoverData.data=false;
//                visionHover_pub.publish(visionHoverData);
//                isVisionHover=false;
//            }
//        }

//        ardrone->move(commander_move.data[0], commander_move.data[1], commander_move.data[2], commander_move.data[3]);
//        yaw_target=yaw;
//        for(int i=0;i<3;++i)hoverPos[i] =pos[i];
//        break;
//    }
//    case MOVE_BY_VELOCITY:{
//        if(controlSum<0.0001){
//            doHover(dt);
//            return;
//        }

//        //close vision hover
//        if(VISION_HOVER_SWITCH){
//            if(isVisionHover){
//                std_msgs::Bool visionHoverData;
//                visionHoverData.data=false;
//                visionHover_pub.publish(visionHoverData);
//                isVisionHover=false;
//            }
//        }
//        for(int i=0;i<3;++i)hoverPos[i]+=(0.5/controlRate)*commander_move.data[i];
//        yaw_target+=commander_move.data[3];
//        if(yaw_target>M_PI)yaw_target-=2*M_PI;
//       else if(yaw_target<-M_PI)yaw_target+=2*M_PI;
//        for(int i=0;i<3;++i)targetPos[i] =hoverPos[i];
//        pos_to_rate_xy(dt,1,vel_target);
//        rate_to_accel_xy( dt, 1,vel_target,acc_target);
//        accel_to_lean_angles( dt, 1,acc_target);
//        //test by xue
//        //posrate_to_lean_angles(vel_target, acc_target);
//        //test by xue^
//        CLIP3(-MAX_ANGLE,pitch_target,MAX_ANGLE);
//        CLIP3(-MAX_ANGLE,roll_target,MAX_ANGLE);
//        fout<<pos[0]<<"   "<<pos[1]<<"  "<<pos[2]<<"    "<<hoverPos[0]<<"   "<<hoverPos[1]<<"  "<<1.5*roll_target<<"    "<<1.5*pitch_target<<"    "<<vel_opt[0]<<"  "<<vel_opt[1]<<"  "<<vel_target[0]
//                   <<"  "<<vel_target[1]<<endl;

//        float altErr=targetPos[2]-pos[2];
//        float upd=PID_z->get_pid(altErr,dt);

//        float yawErr=0;
//        if(yaw_target*yaw>=0)yawErr=yaw_target-yaw;
//        else{
//            if(fabs(yaw_target-yaw)<=M_PI)yawErr=yaw_target-yaw;
//            else {
//                if(yaw_target-yaw<0)yawErr=2*M_PI-fabs(yaw_target-yaw);
//                else yawErr=fabs(yaw_target-yaw)-2*M_PI;
//            }
//        }
//        float yaw_speed=PID_yaw->get_pid(yawErr,dt);

//        ardrone->move(roll_target, pitch_target, upd, yaw_speed);
//    }
//    }
//}

//void PosControl::doTrajector(double dt,double trajectoryTime)
//{
//    ROS_INFO("[POSECONTROL]Trajectory!");

//    //close vision hover
//    if(VISION_HOVER_SWITCH){
//        if(isVisionHover){
//            std_msgs::Bool visionHoverData;
//            visionHoverData.data=false;
//            visionHover_pub.publish(visionHoverData);
//            isVisionHover=false;
//        }
//    }

//    if(_trajectory.trajectoryPlanTarget( pos,vel_ekf,trajectorAcc,dt,trajectoryTime)!=1){
//        for(int i=0;i<3;++i)hoverPos[i] =pos[i];
//        isSetTrajectory=false;
//        std_msgs::Bool recordOver;
//        recordOver.data=false;
//        autoreturn_startRecord_pub.publish(recordOver);
//        _trajectory.trajectoryStart(0);
//        std_msgs::Bool returnData;
//        returnData.data=false;
//        autoreturn_startReturn_pub.publish(returnData);
//        return;
//    }
//    if(_trajectory. isChangeFrame){
//        std_msgs::Int16 _Index;
//        _Index.data=_trajectory.recordFrameIndex;
//        setKeyframe_pub.publish(_Index);
//        _trajectory. isChangeFrame=false;
//       }
//    //setpoint method
////     static double vel_target[3],acc_target[3];
////    targetPos[0] =_trajectory.trajectoryStartPoint.x+_trajectory.targetPointNow.x;
////    targetPos[1] =_trajectory.trajectoryStartPoint.y+_trajectory.targetPointNow.y;
////    pos_to_rate_xy(dt,1,vel_target);
////    rate_to_accel_xy( dt, 1,vel_target,acc_target);
////    accel_to_lean_angles( dt, 1,acc_target);

//    //trajectory method
//    accel_to_lean_angles( dt, 1,trajectorAcc);
//    //test by xue
//    //double vel_target[3] = {trajectorVel[0], trajectorVel[1], trajectorVel[2]};
//    //posrate_to_lean_angles(vel_target,acc_target);
//    //test by xue^
//    tmp.data=_trajectory.trajectoryStartPoint.x+_trajectory.targetPointNow.x-pos[0];
//    tmp_pub1.publish(tmp);
//    tmp.data=_trajectory.trajectoryStartPoint.y+_trajectory.targetPointNow.y-pos[1];
//    tmp_pub2.publish(tmp);
//    targetPos[2] =_trajectory.targetPointNow.z;
//    CLIP3(-MAX_ANGLE,pitch_target,MAX_ANGLE);
//    CLIP3(-MAX_ANGLE,roll_target,MAX_ANGLE);

//    float yawErr=0;
//    yaw_target=_trajectory.yawTarget;
//    if(yaw_target*yaw>=0)yawErr=yaw_target-yaw;
//    else{
//        if(fabs(yaw_target-yaw)<=M_PI)yawErr=yaw_target-yaw;
//        else {
//            if(yaw_target-yaw<0)yawErr=2*M_PI-fabs(yaw_target-yaw);
//            else yawErr=fabs(yaw_target-yaw)-2*M_PI;
//        }
//    }
//    float altErr=targetPos[2]-pos[2];
//    float yaw_speed=PID_yaw->get_pid(yawErr,dt);
//    float upd=PID_z->get_pid(altErr,dt);

//    drone.move(roll_target,pitch_target, upd, yaw_speed);

//    for(int i=0;i<3;++i)hoverPos[i] =pos[i];

//    //cout<<pos[0]<<"   "<<pos[1]<<"  "<<pos[2]<<"   "<<_trajectory.targetPointNow.x+_trajectory.trajectoryStartPoint.x<<"   "<<_trajectory.targetPointNow.y+_trajectory.trajectoryStartPoint.y<<"    "<<_trajectory.targetPointNow.z<<" "<<vel_ekf[0]<<"  "<<vel_ekf[1]<<"  "<<_trajectory.targetPointNow.vx<<"    "<<_trajectory.targetPointNow.vy<<endl;
//    std_msgs::Float32MultiArray trajectorData;
//    trajectorData.data.push_back(pos[0]);
//    trajectorData.data.push_back(pos[1]);
//    trajectorData.data.push_back(pos[2]);
//    trajectorData.data.push_back(_trajectory.targetPointNow.x+_trajectory.trajectoryStartPoint.x);
//    trajectorData.data.push_back(_trajectory.targetPointNow.y+_trajectory.trajectoryStartPoint.y);
//    trajectorData.data.push_back(_trajectory.targetPointNow.z);
//    trajectorData.data.push_back(vel_ekf[0]);
//    trajectorData.data.push_back(vel_ekf[1]);
//    trajectorData.data.push_back( _trajectory.targetPointNow.vx);
//    trajectorData.data.push_back( _trajectory.targetPointNow.vy);
//    trajectorData_pub.publish(trajectorData);
//    fout_return<<pos[0]<<"   "<<pos[1]<<"  "<<pos[2]<<"   "<<_trajectory.targetPointNow.x+_trajectory.trajectoryStartPoint.x<<"   "<<_trajectory.targetPointNow.y+_trajectory.trajectoryStartPoint.y<<"    "<<_trajectory.targetPointNow.z<<" "<<vel_ekf[0]<<"  "<<vel_ekf[1]<<"  "<<_trajectory.targetPointNow.vx<<"    "<<_trajectory.targetPointNow.vy<<endl;
//}

///  coordinate transform

//euler in body_nwu frame
void PosControl::qtoEuler()
{
    float_t w,x,y,z;
    //diffrent coordinates
    w=sensor_local_position.pose.orientation.w;
    x=sensor_local_position.pose.orientation.x;
    y=sensor_local_position.pose.orientation.y;
    z=sensor_local_position.pose.orientation.z;
   // cout<<"q:"<<w<<"    "<<x<<"    "<<y<<"    "<<z<<endl;
    roll  = atan2(2 * (w * x + y* z) , 1 - 2 * (x * x + y * y));
    if(2 * (w * y - z * x)>1)pitch =asin(1.0) ;
    else if(2 * (w * y - z * x)<-1.0)pitch = asin(-1.0);
    else pitch = asin(2 * (w * y - z * x));
    yaw   = atan2(2 * (w * z + x * y) , 1 - 2 * (y * y + z * z));
}

void PosControl::qtoEuler(float& _roll,float& _pitch,float& _yaw,float q[])
{
    float_t w,x,y,z;
    //diffrent coordinates
    w=q[0];
    x=q[1];
    y=q[2];
    z=q[3];
   // cout<<"q:"<<w<<"    "<<x<<"    "<<y<<"    "<<z<<endl;
    _roll  = atan2(2 * (w * x + y* z) , 1 - 2 * (x * x + y * y));
    if(2 * (w * y - z * x)>1)_pitch =asin(1.0) ;
    else if(2 * (w * y - z * x)<-1.0)_pitch = asin(-1.0);
    else _pitch = asin(2 * (w * y - z * x));
    _yaw   = atan2(2 * (w * z + x * y) , 1 - 2 * (y * y + z * z));
}

void  PosControl::getRotation(){
    float_t w,x,y,z;
    // coordinates in NWU
    w=sensor_local_position.pose.orientation.w;
    x=sensor_local_position.pose.orientation.x;
    y=sensor_local_position.pose.orientation.y;
    z=sensor_local_position.pose.orientation.z;

    att_R[0]=w*w+x*x-y*y-z*z;
    att_R[1]=2*(x*y-w*z);
    att_R[2]=2*(x*z+w*y);
    att_R[3]=2*(x*y+w*z);
    att_R[4]=w*w-x*x+y*y-z*z;
    att_R[5]=2*(y*z-w*x);
    att_R[6]=2*(x*z-w*y);
    att_R[7]=2*(y*z+w*x);
    att_R[8]=w*w-x*x-y*y+z*z;
//    cout<<"Rotation"<<endl;
//    cout<<" "<<att_R[0]<<" "<<att_R[1]<<" "<<att_R[2]<<endl;
//    cout<<" "<<att_R[3]<<" "<<att_R[4]<<" "<<att_R[5]<<endl;
//    cout<<" "<<att_R[6]<<" "<<att_R[7]<<" "<<att_R[8]<<endl;
}

void  PosControl::getRotation(float q[],float R[])
{
    float_t w,x,y,z;
    // coordinates in NWU
    w=q[0];
    x=q[1];
    y=q[2];
    z=q[3];

    R[0]=w*w+x*x-y*y-z*z;
    R[1]=2*(x*y-w*z);
    R[2]=2*(x*z+w*y);
    R[3]=2*(x*y+w*z);
    R[4]=w*w-x*x+y*y-z*z;
    R[5]=2*(y*z-w*x);
    R[6]=2*(x*z-w*y);
    R[7]=2*(y*z+w*x);
    R[8]=w*w-x*x-y*y+z*z;
}

void PosControl::eulertoQ(float q[])
{
    float fCosHRoll = cos(roll * .5f);
    float fSinHRoll = sin(roll * .5f);
    float fCosHPitch = cos(pitch * .5f);
    float fSinHPitch = sin(pitch * .5f);
    float fCosHYaw = cos(yaw * .5f);
    float fSinHYaw = sin(yaw * .5f);

    /// Cartesian coordinate System
    q[0] = fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw;
    q[1] = fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw;
    q[2]= fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
    q[3] = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;
}

void PosControl::eulertoQ(float_t roll,float_t pitch,float_t yaw,float q[])
{
    float fCosHRoll = cos(roll * .5f);
    float fSinHRoll = sin(roll * .5f);
    float fCosHPitch = cos(pitch * .5f);
    float fSinHPitch = sin(pitch * .5f);
    float fCosHYaw = cos(yaw * .5f);
    float fSinHYaw = sin(yaw * .5f);

    /// Cartesian coordinate System
    q[0] = fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw;
    q[1] = fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw;
    q[2]= fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
    q[3] = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;
}

void PosControl::transform_body_from_nwu(const float_t a_nwu[3],float_t a_body[3])
{
    for(int i=0;i<3;++i) a_body[i]=0;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            a_body[i]+=att_R[3*j+i]*a_nwu[j];
        }
    }
}

void PosControl::transform_nwu_from_body(const float_t a_body[3],float_t a_nwu[3])
{
    for(int i=0;i<3;++i) a_nwu[i]=0;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            a_nwu[i]+=a_body[j]*att_R[3*i+j];
        }
    }
}

//void PosControl::startLoop(){
//   pthread_create( &read_tid, NULL, &startHoverThread, this );
//}

//void* startHoverThread(void *args)
//{
//    PosControl *w = (PosControl*)args;
//    w->hover_loop();
//    // done!
//    return NULL;
//}

//void PosControl::hover_loop(){
//    double dt=0;

//    ros::Rate r(controlRate);//set rate at 50 Hz
//    /////////////////////////////////////////////
//    ///                       control loop
//   ///when there is no control input, choose to hover
//   /// when there is control input, if input is zero:hover,and if input isn't velocity, control attitutde directly,else control velocity by PID.
//   ///set one timer to update isControlOutside and isSetVel. In the callback function ,if there is data the timer checkOutsideControl
//   /// will be set to zero.
//   /// /////////////////////////////////////////
//    while (ros::ok()&&!threadQuit){
//        if(isControlOutside&&(ros::Time().now().toSec()-checkOutsideControl.toSec())>0.5){
//            isControlOutside=false;
//            isSetVel=false;
//            checkOutsideControl=ros::Time().now();
//            reset();
//        }
//       dt=ros::Time().now().toSec()-pidTime.toSec();
//        pidTime=ros::Time().now();
////        cout<<"flight:"<<roll<<"  "<<pitch<<"  "<<yaw<<"  "<<endl;
//        if(setHover){
//            if(!isControlOutside){
//                if((!isSetPoint)&&(!isSetTrajectory)) doHover(dt);
//                else {
//                    if(isSetPoint)doSetPoint(dt);
//                    else {
//                        //update the start time here  not in the callback to avoid  0.2s time delay.
//                        if(!(_trajectory.startTrajectory)){
//                            _trajectory.trajectoryStart(1);
//                            _trajectory.trajectoryStartPoint.x=pos[0];
//                            _trajectory.trajectoryStartPoint.y=pos[1];
//                        }
//                        pidTime=ros::Time().now();
//                        doTrajector(dt,pidTime.toSec());
//                    }
//                }
//            }
//            else{//manual control x,y
//                if(!isSetVel) doMove(dt);
//                else doSetVelocity(dt);
//            }
//        }
//        ros::spinOnce();
//        r.sleep();
//    }
//}

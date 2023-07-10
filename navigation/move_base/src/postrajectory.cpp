#include <move_base/postrajectory.h>
#define LINEAR 1

//extern std::string trajectoryPIDPath;
std::string trajectoryPointsPath;
float trajectoryP=1.0;
float trajectoryI=0.0;
float trajectoryD=2.0;
float trajectoryImax=0.3;
fstream trajectoryPointsfile(trajectoryPointsPath.c_str());
void posTrajectory::setup()
{
     ROS_INFO("[TRAJECTORY]Trajectory Setup ....");
     nh.getParam("trajectoryP",trajectoryP);
     nh.getParam("trajectoryI",trajectoryI);
     nh.getParam("trajectoryD",trajectoryD);
     nh.getParam("trajectoryImax",trajectoryImax);
     nh.getParam("trajectoryPointsPath",trajectoryPointsPath);
     trajectoryPointsfile.open(trajectoryPointsPath.c_str());
     loadTrajectoryPoints();
     PID_trajector=new PIDController(trajectoryP, trajectoryI,  trajectoryD,  trajectoryImax);
     FuzzyPIDTrajector=new FuzzyPID(trajectoryP, trajectoryI,  trajectoryD,  trajectoryImax);
     for(int i=0;i<dimension;++i)startVelocity[i]=0;
     ROS_INFO("[TRAJECTORY]Trajectory Setup  over!");
}


void posTrajectory::loadTrajectoryPoints()
{
    trajectorySequence.clear();
    trajectoryPoint tmpPoint;
    while(trajectoryPointsfile>>tmpPoint.x>>tmpPoint.y>>tmpPoint.z>>tmpPoint.yaw>>tmpPoint.reach_time){
        trajectorySequence.push_back(tmpPoint);
    }
}

void posTrajectory:: trajectoryPlanner(int selectFlag)
{
    switch(selectFlag){
    case LINEAR:linear_trajectoryPlanner();break;
    case CUBIC:cubic_trajectoryPlanner();break;
    case MINIMUMJERK:minJerk_trajectoryPlanner();break;
    case MINIMUMSNAP:minSnap_trajectoryPlanner();break;
    default:break;
    }
}

void posTrajectory:: linear_trajectoryPlanner()
{
    int N=trajectorySequence.size();
    if(!N){
        bad_data=true;
        ROS_INFO("[TRAJECTORY]posTrajectory_linear: size of time vector is zero.");
        return;
    }
    if(N<=2){
        bad_data=true;
        ROS_INFO("[TRAJECTORY]posTrajectory_linear: need at least 2 points to produce a spline.");
        return;
    }
    float *delta_t=new float[N-1];
    pts=cv::Mat(N,dimension,CV_32FC1,cv::Scalar::all(0));
    for(int i=0;i<N;++i){
        pts.at<float>(i,0)=trajectorySequence[i].x;
        pts.at<float>(i,1)=trajectorySequence[i].y;
        pts.at<float>(i,2)=trajectorySequence[i].z;
    }
    for(int i=0;i<N-1;++i){
        delta_t[i]=trajectorySequence[i+1].reach_time-trajectorySequence[i].reach_time;
        if(delta_t[i]<=0){
            bad_data=true;
            ROS_INFO("[TRAJECTORY]posTrajectory_linear:  time between input points is less than zero");
            return;
        }
    }
    blendTimei=cv::Mat(N,dimension,CV_32FC1,cv::Scalar::all(0));
    blendTimeij=cv::Mat(N,dimension,CV_32FC1,cv::Scalar::all(0));
    slopes=cv::Mat(N,dimension,CV_32FC1,cv::Scalar::all(0));
    signS=cv::Mat(N,dimension,CV_32FC1,cv::Scalar::all(0));

    //calculate slopes and blendtimei
    for(int i=0;i<dimension;i++){
        signS.at<float>(0,i)=sign(pts.at<float>(1,i)-pts.at<float>(0,i));
        blendTimei.at<float>(0,i)=delta_t[0]-sqrt(delta_t[0]*delta_t[0]-(2*fabs(pts.at<float>(1,i)-pts.at<float>(0,i))/linear_inter_acc));
        slopes.at<float>(0,i)=(pts.at<float>(1,i)-pts.at<float>(0,i))/(delta_t[0]-0.5f*blendTimei.at<float>(0,i));
    }

    for(int j=1;j<N-1;++j){
            for(int i=0;i<dimension;i++){
                signS.at<float>(j,i)=sign(slopes.at<float>(j+1,i)-slopes.at<float>(j,i));
               slopes.at<float>(j,i)=(pts.at<float>(j+1,i)-pts.at<float>(j,i))/delta_t[j];
               blendTimei.at<float>(0,i)=fabs((slopes.at<float>(j,i)-slopes.at<float>(j-1,i))/linear_inter_acc);
            }
     }

    for(int i=0;i<dimension;i++){
        signS.at<float>(N-1,i)=sign(pts.at<float>(N-1,i)-pts.at<float>(N-2,i));
        blendTimei.at<float>(N-1,i)=delta_t[N-1]-sqrt(delta_t[N-1]*delta_t[N-1]-(2*fabs(pts.at<float>(N,i)-pts.at<float>(N-2,i))/linear_inter_acc));
        slopes.at<float>(N-1,i)=(pts.at<float>(N,i)-pts.at<float>(N-1,i))/(delta_t[N-1]-0.5f*blendTimei.at<float>(N-1,i));
    }
    //calculate blendtimeij
    for(int i=0;i<dimension;i++){
        blendTimeij.at<float>(0,i)=delta_t[0]-blendTimei.at<float>(0,i)-0.5*blendTimei.at<float>(1,i);
   }

    for(int j=1;j<N-1;++j){
            for(int i=0;i<dimension;i++){
                blendTimeij.at<float>(j,i)=delta_t[j]-0.5*(blendTimei.at<float>(j,i)+blendTimei.at<float>(j+1,i));
           }
     }

    for(int i=0;i<dimension;i++){
        blendTimeij.at<float>(N-1,i)=delta_t[N-1]-blendTimei.at<float>(N-1,i)-0.5*blendTimei.at<float>(N-2,i);
   }
     delete []delta_t;
}

void posTrajectory:: cubic_trajectoryPlanner()
{
    pts.release();
    Ak.release();
    Bk.release();
    Ck.release();
    Dk.release();
    int N=trajectorySequence.size();
    if(!N){
        bad_data=true;
        ROS_INFO("[TRAJECTORY]posTrajectory: size of time vector is zero.");
        return;
    }
    if(N<=4){
        bad_data=true;
        ROS_INFO("[TRAJECTORY]posTrajectory: need at least 4 points to produce a spline.");
        return;
    }
    float *delta=new float[N-1];
    float *beta=new float[N-1];
    pts=cv::Mat(N,dimension,CV_32FC1,cv::Scalar::all(0));
    for(int i=0;i<N;++i){
        pts.at<float>(i,0)=trajectorySequence[i].x;
        pts.at<float>(i,1)=trajectorySequence[i].y;
        pts.at<float>(i,2)=trajectorySequence[i].z;
    }
    for(int i=0;i<N-1;++i){
        delta[i]=trajectorySequence[i+1].reach_time-trajectorySequence[i].reach_time;
        if(delta[i]<=0){
            bad_data=true;
            ROS_INFO("[TRAJECTORY]posTrajectory:  time between input points is zero");
            return;
        }
        beta[i]=1/delta[i];
    }
if(BOUNDARY_CONDITION==0)//natural boundary
{
    if(startTrajectory){
       startAcc[0]= targetPointNow.ax;
       startAcc[1]= targetPointNow.ay;
       startAcc[2]= targetPointNow.az;
    }
    cv::Mat A(N-2, N-2,CV_32FC1,cv::Scalar::all(0.0));
    A.at<float>(0,0)=2*(delta[0]+delta[1]);
    A.at<float>(0,1)=delta[1];
    for(int j=1;j<N-2;++j)
    {
        A.at<float>(j,j-1)=delta[j];
        A.at<float>(j,j)=2*(delta[j]+delta[j+1]);
        if((j+1)<A.cols)A.at<float>(j,j+1)=delta[j+1];
    }

    cv::Mat C(N-2, N,CV_32FC1,cv::Scalar::all(0));
    for(int k=0;k<N-2;++k)
    {
        C.at<float>(k,k)=beta[k];
        C.at<float>(k,k+1)=-(beta[k]+beta[k+1]);
        C.at<float>(k,k+2)=beta[k+1];
    }

    cv::Mat _6AiC=6*A.inv()*C;// eq 5.58a Angeles

    cv::Mat dd_s(N,1,CV_32FC1,cv::Scalar::all(0));   // second spline derivative at sampling points
    dd_s.at<float>(0,0) = dd_s.at<float>(N-1,0) = 0;  // second der is 0 on first and last point of natural splines.


    Ak = cv::Mat( N-1,dimension,CV_32FC1,cv::Scalar::all(0));
    Bk = cv::Mat(N-1,dimension,CV_32FC1,cv::Scalar::all(0));
    Ck = cv::Mat(N-1,dimension,CV_32FC1,cv::Scalar::all(0));
    Dk = cv::Mat(N-1,dimension,CV_32FC1,cv::Scalar::all(0));

    cv::Rect roi1(0,1,1,N-2);
    for(int ii = 0; ii < dimension; ii++)
    {
        cv::Rect roi2(ii,0,1,N);
        dd_s(roi1) = _6AiC*(pts(roi2));
        if(startTrajectory)
            dd_s.at<float>(0,0)= startAcc[ii];


        for(int jj = 0; jj < N-1; jj++)
         {
            // eq 5.55a - 5.55d Angeles
            Ak.at<float>(jj,ii) = 1/(6.0*delta[jj])*(dd_s.at<float>(jj+1,0) - dd_s.at<float>(jj,0));
            Bk.at<float>(jj,ii) = 1/2.0*dd_s.at<float>(jj,0);
            Ck.at<float>(jj,ii) = (pts.at<float>(jj+1,ii)-pts.at<float>(jj,ii))/delta[jj] -1/6.0*delta[jj]*(dd_s.at<float>(jj+1,0) + 2*dd_s.at<float>(jj,0));
            Dk.at<float>(jj,ii) = pts.at<float>(jj,ii);
          }
     }
}
else //clamped boundary,set velocity eq to startVelocity
{
    if(startTrajectory){
       startVelocity[0]= targetPointNow.vx;
       startVelocity[1]= targetPointNow.vy;
       startVelocity[2]= targetPointNow.vz;
    }
    else{
        for(int k=0;k<dimension;++k)
            startVelocity[k]=0;
    }
    cv::Mat A(N, N,CV_32FC1,cv::Scalar::all(0.0));
    A.at<float>(0,0)=2*delta[0];
    A.at<float>(0,1)=delta[0];
    A.at<float>(N-1,N-2)=delta[N-2];
    A.at<float>(N-1,N-1)=2*delta[N-2];
    for(int j=1;j<N-1;++j)
    {
        A.at<float>(j,j-1)=delta[j-1];
        A.at<float>(j,j)=2*(delta[j-1]+delta[j]);
        if((j+1)<A.cols)A.at<float>(j,j+1)=delta[j];
    }

    cv::Mat C(N, N,CV_32FC1,cv::Scalar::all(0));
    C.at<float>(0,0)=-beta[0];
    C.at<float>(0,1)=beta[0];
    C.at<float>(N-1,N-2)=beta[N-2];
    C.at<float>(N-1,N-1)=-beta[N-2];
    for(int k=1;k<N-1;++k)
    {
        C.at<float>(k,k-1)=beta[k-1];
        C.at<float>(k,k)=-(beta[k-1]+beta[k]);
        C.at<float>(k,k+1)=beta[k];
    }

    cv::Mat _6AiC=6*A.inv()*C;// eq 5.58a Angeles

    cv::Mat dd_s(N,1,CV_32FC1,cv::Scalar::all(0));   // second spline derivative at sampling points

    Ak = cv::Mat( N-1,dimension,CV_32FC1,cv::Scalar::all(0));
    Bk = cv::Mat(N-1,dimension,CV_32FC1,cv::Scalar::all(0));
    Ck = cv::Mat(N-1,dimension,CV_32FC1,cv::Scalar::all(0));
    Dk = cv::Mat(N-1,dimension,CV_32FC1,cv::Scalar::all(0));

    for(int ii = 0; ii < dimension; ii++)
    {
        pts.at<float>(ii,0)-=delta[0]*startVelocity[ii];
        pts.at<float>(ii,N-1)-=delta[N-2]*startVelocity[ii];//add the start velocity
        cv::Rect roi2(ii,0,1,N);

        dd_s = _6AiC*(pts(roi2));
        for(int jj = 0; jj < N-1; jj++)
         {
            // eq 5.55a - 5.55d Angeles
            Ak.at<float>(jj,ii) = 1/(6.0*delta[jj])*(dd_s.at<float>(jj+1,0) - dd_s.at<float>(jj,0));
            Bk.at<float>(jj,ii) = 1/2.0*dd_s.at<float>(jj,0);
            Ck.at<float>(jj,ii) = (pts.at<float>(jj+1,ii)-pts.at<float>(jj,ii))/delta[jj] -1/6.0*delta[jj]*(dd_s.at<float>(jj+1,0) + 2*dd_s.at<float>(jj,0));
            Dk.at<float>(jj,ii) = pts.at<float>(jj,ii);
          }
     }
}
    bad_data=false;
    delete []delta;
    delete []beta;

}

void posTrajectory::minJerk_trajectoryPlanner()
{
    pts.release();
    Ak.release();
    Bk.release();
    Ck.release();
    Dk.release();
    Ek.release();
    Fk.release();
    int N=trajectorySequence.size();
    if(!N){
        bad_data=true;
        ROS_INFO("[TRAJECTORY]posTrajectory: size of time vector is zero.");
        return;
    }
    if(N<=6){
        bad_data=true;
        ROS_INFO("[TRAJECTORY]posTrajectory: need at least 6 points to produce a spline.");
        return;
    }
}

void posTrajectory::minSnap_trajectoryPlanner()
{
    pts.release();
    Ak.release();
    Bk.release();
    Ck.release();
    Dk.release();
    Ek.release();
    Fk.release();
    Gk.release();
    Hk.release();
    int N=trajectorySequence.size();
    if(!N){
        bad_data=true;
        ROS_INFO("[TRAJECTORY]posTrajectory: size of time vector is zero.");
        return;
    }
    if(N<=8){
        bad_data=true;
        ROS_INFO("[TRAJECTORY]posTrajectory: need at least 8 points to produce a spline.");
        return;
    }
}

short posTrajectory::interpolating(const double t)
{
    if(bad_data)
    {
       ROS_INFO("[TRAJECTORY]posTrajectory::interpolating: data is not good. Problems occur in constructor." );
       return BAD_DATA;
    }
    cv::Mat s,dv,da;
    for(int i = 0; i < trajectorySequence.size()-1; i++){
        cv::Rect roi(0,i,dimension,1);
        double reachtime_i=trajectorySequence[i].reach_time;
       if( (t >= reachtime_i && (t < trajectorySequence[i+1].reach_time) )) {
           if(recordFrameIndex!=i) {
               float yaw=trajectorySequence[i].yaw;
               targetQFrom.w()=cos(yaw * .5f);
               targetQFrom.x()=0;
               targetQFrom.y()=0;
               targetQFrom.z()=sin(yaw * .5f);

               yaw=trajectorySequence[i+1].yaw;
               targetQTo.w()=cos(yaw * .5f);
               targetQTo.x()=0;
               targetQTo.y()=0;
               targetQTo.z()=sin(yaw * .5f);

               recordFrameIndex=i;
               isChangeFrame=true;
           }
          interpolating(i, t,s);
          first_derivative(i, t,dv);
          second_derivative(i, t,da);
          targetPointNow.x=s.at<float>(0,0);
          targetPointNow.y=s.at<float>(0,1);
          targetPointNow.z=s.at<float>(0,2);
          targetPointNow.vx=dv.at<float>(0,0);
          targetPointNow.vy=dv.at<float>(0,1);
          targetPointNow.vz=dv.at<float>(0,2);
          targetPointNow.ax=da.at<float>(0,0);
          targetPointNow.ay=da.at<float>(0,1);
          targetPointNow.az=da.at<float>(0,2);
          targetPointNow.timeNow=t;
          return 0;
       }
    }

    ROS_INFO("[TRAJECTORY]posTrajectory::interpolating: t is out of range." );
    cout<<"posTrajectory::interpolating: t is out of range." <<endl;
    return NOT_IN_RANGE;
 }

short posTrajectory::interpolating(const int ti, const double t,cv::Mat &s)
{
   if(chooseFlag==CUBIC){
       double reachtime_i=trajectorySequence[ti].reach_time;
       targetQ=targetQFrom.slerp((t-reachtime_i)/(trajectorySequence[ti+1].reach_time-reachtime_i),targetQTo);
       yawTarget=2*atan2(targetQ.z(),targetQ.w());
       cv::Rect roi(0,ti,dimension,1);
         s = Ak(roi)*pow(t - reachtime_i, 3) +
             Bk(roi)*pow(t - reachtime_i,2) +
             Ck(roi)*(t - reachtime_i) +
             Dk(roi);
         return 0;
    }
    else if(chooseFlag==LINEAR){
       s=cv::Mat( 1,dimension,CV_32FC1,cv::Scalar::all(0));
       double dt=t-trajectorySequence[ti].reach_time;
       float ch1=0.5;
       if(ti==0){
           ch1=1;
       }
       if(ti==0){
           for(int j=0;j<dimension;++j){
           if(dt<=ch1*blendTimei.at<float>(ti,j)){
               s.at<float>(0,j)=pts.at<float>(ti,j)+ 0.5*signS.at<float>(ti,j)*linear_inter_acc*pow(ch1*blendTimei.at<float>(ti,j)+dt,2);
           }
           else{
               if(dt<blendTimeij.at<float>(ti,j)+ch1*blendTimei.at<float>(ti,j))
                   s.at<float>(0,j)=pts.at<float>(ti,j)+slopes.at<float>(ti,j)*(dt-0.5*blendTimei.at<float>(ti,j));
               else
                   s.at<float>(0,j)=pts.at<float>(ti,j)+slopes.at<float>(ti,j)*(dt-0.5*blendTimei.at<float>(ti,j))+0.5*signS.at<float>(ti+1,j)*linear_inter_acc*pow(dt-ch1*blendTimei.at<float>(ti,j)-blendTimeij.at<float>(ti,j),2);
               }
           }
       }
       else{
       for(int j=0;j<dimension;++j){
           if(dt<=ch1*blendTimei.at<float>(ti,j))
               s.at<float>(0,j)=pts.at<float>(ti,j)+slopes.at<float>(ti-1,j)*dt + 0.5*signS.at<float>(ti,j)*linear_inter_acc*pow(ch1*blendTimei.at<float>(ti,j)+dt,2);
           else{
               if(dt<blendTimeij.at<float>(ti,j)+ch1*blendTimei.at<float>(ti,j))
                   s.at<float>(0,j)=pts.at<float>(ti,j)+slopes.at<float>(ti,j)*(dt-0.5*blendTimei.at<float>(ti,j));
               else
                   s.at<float>(0,j)=pts.at<float>(ti,j)+slopes.at<float>(ti,j)*(dt-0.5*blendTimei.at<float>(ti,j))+0.5*signS.at<float>(ti+1,j)*linear_inter_acc*pow(dt-ch1*blendTimei.at<float>(ti,j)-blendTimeij.at<float>(ti,j),2);
               }
           }
       return 0;
       }
   }

}

short posTrajectory::first_derivative(const double t, cv::Mat & dv)
 {
    if(bad_data)
    {
       ROS_INFO("[TRAJECTORY]posTrajectory::first_derivative: data is not good. Problems occur in constructor." );
       return BAD_DATA;
    }

    for(int i = 0; i < trajectorySequence.size()-1; i++){
        return first_derivative(i,  t,dv);
    }

    ROS_INFO( "[TRAJECTORY]posTrajectory::first_derivative: t not in range." );
   return NOT_IN_RANGE;
 }

 short posTrajectory::first_derivative(const int ti, const double t,cv::Mat & dv)
 {
     if(chooseFlag==CUBIC){
        cv::Rect roi(0,ti,dimension,1);
        double reachtime_i=trajectorySequence[ti].reach_time;
          dv = 3*Ak(roi)*pow(t - reachtime_i, 2) +
               2*Bk(roi)*(t - reachtime_i) +
               Ck(roi);
          return 0;
     }
     else if(chooseFlag==LINEAR){
         dv=cv::Mat( 1,dimension,CV_32FC1,cv::Scalar::all(0));
         double dt=t-trajectorySequence[ti].reach_time;
         float ch1=0.5;
         if(ti==0){
             ch1=1;

         }
         for(int j=0;j<dimension;++j){
             if(dt<=ch1*blendTimei.at<float>(ti,j))
                 dv.at<float>(0,j)=slopes.at<float>(ti,j)-signS.at<float>(ti,j)*linear_inter_acc*(ch1*blendTimei.at<float>(ti,j)-dt);
             else{
                 if(dt<blendTimeij.at<float>(ti,j)+ch1*blendTimei.at<float>(ti,j))
                     dv.at<float>(0,j)=slopes.at<float>(ti,j);
                 else
                     dv.at<float>(0,j)=slopes.at<float>(ti,j)+signS.at<float>(ti+1,j)*linear_inter_acc*(dt-blendTimeij.at<float>(ti,j)-ch1*blendTimei.at<float>(ti,j));
                 }
             }
         return 0;
     }
 }

 short posTrajectory::second_derivative(const double t,cv:: Mat & da)
 {
    if(bad_data)
    {
       ROS_INFO ("[TRAJECTORY]Spl_cubic::second_derivative: data is not good. Problems occur in constructor." );
       return BAD_DATA;
    }

    for(int i = 0; i < trajectorySequence.size()-1; i++){
        return second_derivative(i, t, da);
    }

    ROS_INFO("[TRAJECTORY]posTrajectory::second_derivative: t not in range.");
    return NOT_IN_RANGE;

 }

short posTrajectory::second_derivative(const int ti, const double t,cv::Mat & da)
 {
    if(chooseFlag==CUBIC){
        cv::Rect roi(0,ti,dimension,1);
        double reachtime_i=trajectorySequence[ti].reach_time;
          da = 6*Ak(roi)*(t - reachtime_i) +
               2*Bk(roi);
          return 0;
    }
    else if(chooseFlag==LINEAR){
        da=cv::Mat( 1,dimension,CV_32FC1,cv::Scalar::all(0));
        double dt=t-trajectorySequence[ti].reach_time;
        float ch1=0.5;
        if(ti==0){
            ch1=1;
        }

        for(int i=0;i<dimension;++i){
            if(dt<=ch1*blendTimei.at<float>(ti,i))
                da.at<float>(0,i)=signS.at<float>(ti,i)*linear_inter_acc;
            else{
                if(dt<blendTimeij.at<float>(ti,i)+ch1*blendTimei.at<float>(ti,i))
                    da.at<float>(0,i)=0;
                else
                    da.at<float>(0,i)=signS.at<float>(ti+1,i)*linear_inter_acc;
                }
            }
        return 0;
        }
 }


int posTrajectory::trajectoryPlanTarget(float_t posNow[3],float_t velNow[3],double_t accCommand[3],double_t dt,double_t time)
{
    float_t posTarget[3],velTarget[3],accTarget[3];
    int flag=interpolating(time-trajectoryStartTime);
    if(flag<0){
        return flag;
    }
    posTarget[0]=targetPointNow.x+trajectoryStartPoint.x;
    posTarget[1]=targetPointNow.y+trajectoryStartPoint.y;
    posTarget[2]=targetPointNow.z;

    velTarget[0]=targetPointNow.vx;
    velTarget[1]=targetPointNow.vy;
    velTarget[2]=targetPointNow.vz;

    accTarget[0]=targetPointNow.ax;
    accTarget[1]=targetPointNow.ay;
    accTarget[2]=targetPointNow.az;
    if(startTrajectory){
        if(bad_data){
            for(int i=0;i<3;++i){
                accCommand[i]=0;
            }
            return 0;
        }
        for(int i=0;i<3;++i){
            //float vel_pose_target=1*(posTarget[i]-posNow[i]);
            accCommand[i]=PID_trajector->kP()*(posTarget[i]-posNow[i])+PID_trajector->kD()*(velTarget[i]-velNow[i])+accTarget[i];
            //accCommand[i]=PID_trajector->kP()*(sign(posTarget[i]-posNow[i])*sqrt(fabs(posTarget[i]-posNow[i])))+PID_trajector->kD()*(velTarget[i]-velNow[i])+accTarget[i];
        }
        return 1;
    }
    else{
        for(int i=0;i<3;++i){
            accCommand[i]=0;
        }
        ROS_INFO("[TRAJECTORY]Trajectory does not start!");
        return 0;
    }

}

void posTrajectory::trajectoryStart(int flag)
{
    if(flag==1){
        startTrajectory=true;
        trajectoryStartTime=ros::Time().now().toSec();
    }
    else if(flag==0)startTrajectory=false;
}

void posTrajectory::setMethod(int choose)
{
    chooseFlag=choose;
}

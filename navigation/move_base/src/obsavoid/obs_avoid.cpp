#include <obsavoid/obs_avoid.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace obs_avoid {

    ObsAvoid::ObsAvoid(tf2_ros::Buffer& tf) :
            tf_(tf), planner_costmap_ros_(NULL), controller_costmap_ros_(NULL), pub_zero_velocity(false),
            lp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
            recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
            planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
            runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) {
//        as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);
//      move_base is the name of action
        ros::NodeHandle private_nh("~");
        recovery_trigger_ = PLANNING_R;
        //get some parameters that will be global to the move base node
        std::string local_planner;
        private_nh.param("base_local_planner", local_planner, std::string("teb_local_planner/TebLocalPlannerROS"));
        private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
        private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
        private_nh.param("planner_frequency", planner_frequency_, 0.0);
        private_nh.param("controller_frequency", controller_frequency_, 20.0);
        private_nh.param("planner_patience", planner_patience_, 5.0);
        private_nh.param("controller_patience", controller_patience_, 15.0);
        private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default
        private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
        private_nh.param("oscillation_distance", oscillation_distance_, 0.5);
        // parameters of make_plan service
        private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
        private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);
        //set up plan triple buffer
        planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();
//        recovery_status_pub_= action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);
        //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
        private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
        private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
        private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
        private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);
        private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
        private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
        private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);
        //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
        planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
        planner_costmap_ros_->pause();
        //initialize the global planner
        //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
        controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
        controller_costmap_ros_->pause();
        //create a local planner
        lc_ = lp_loader_.createInstance(local_planner);
        lc_->initialize(lp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
        // Start actively updating costmaps based on sensor data
        planner_costmap_ros_->start();
        controller_costmap_ros_->start();
        //advertise a service for clearing the costmaps
        clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &ObsAvoid::clearCostmapsService, this);
        //if we shutdown our costmaps when we're deactivated... we'll do that now
        if(shutdown_costmaps_){
            ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }
        //load any user specified recovery behaviors, and if that fails load the defaults
        if(!loadRecoveryBehaviors(private_nh))
            loadDefaultRecoveryBehaviors();
        //initially, we'll need to make a plan
        //we'll start executing recovery behaviors at the beginning of our list
        recovery_index_ = 0;
        //we're all set up now so we can start the action server
        //initialize the topic
        inittopic();
    }

    ObsAvoid::~ObsAvoid(){
        recovery_behaviors_.clear();
        delete dsrv_;
        if(planner_costmap_ros_ != NULL)
            delete planner_costmap_ros_;
        if(controller_costmap_ros_ != NULL)
            delete controller_costmap_ros_;
        planner_thread_->interrupt();
        planner_thread_->join();
        delete planner_thread_;
        delete planner_plan_;
        delete latest_plan_;
        delete controller_plan_;
        lc_.reset();
    }

    void ObsAvoid::inittopic()
    {
        //for commanding the base
        vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        path_sub = nh.subscribe("move_base/NavfnROS/plan", 1, &ObsAvoid::pathCB, this);
        joy_sub = nh.subscribe("joy", 1, &ObsAvoid::joyCB, this);
        joy_sub2 = nh.subscribe("/remote/joy", 1, &ObsAvoid::joyCB, this);
        //subscribe this topic to stop the robot
        obs_abort = nh.subscribe("/obs_abort", 1, &ObsAvoid::obs_abortCB, this);
    }

    //建立了接收拓扑轨迹的topic，作为全局规划的路点，并计算控制速度
    void ObsAvoid::pathCB(const nav_msgs::Path &path) {
        geometry_msgs::PoseStamped single_plan_msg;
        planner_plan_->clear();
        planner_plan_->resize(path.poses.size());
//        std::cout << "planner_plan_->size() is: " << planner_plan_->size() << std::endl;
        if(path.poses.size() != 0 && !pub_zero_velocity) {
            for (int i = 0; i <= path.poses.size() - 1; ++i) {
                single_plan_msg.pose = path.poses[i].pose;
                single_plan_msg.header = path.poses[i].header;
//                single_plan_msg.header.frame_id = "map";
                planner_plan_->push_back(single_plan_msg);
            }
            planner_plan_->erase(planner_plan_->cbegin(), planner_plan_->cbegin() + path.poses.size());
//            std::cout << "After erasing, the num of planner_plan_->size() is: " << planner_plan_->size() << std::endl;
//            for (auto &i: *planner_plan_)
//                std::cout << i << std::endl;
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            lc_->setPlan(*planner_plan_);
            geometry_msgs::Twist cmd_vel;
            lc_->computeVelocityCommands(cmd_vel);
            vel_pub_.publish(cmd_vel);
            lock.unlock();
        }
        else {
            ROS_INFO("Waiting for new path!");
            ObsAvoid::publishZeroVelocity();
            //pub_zero_velocity = false;
        }
    }

    void ObsAvoid::obs_abortCB(const std_msgs::Bool::ConstPtr &_msg) {
       // pub_zero_velocity = _msg->data;
       bool pub_zeroV = _msg->data;
       if (pub_zeroV)
       {
            ObsAvoid::publishZeroVelocity();
       }
    }

    void ObsAvoid::joyCB(const sensor_msgs::Joy::ConstPtr &msg) {
        if (msg->buttons[0] > 0.5) {
            //A. Press A, the robot will stop!
            pub_zero_velocity = true;
        } /*else if (msg->buttons[1] > 0.5) {
            //B
            task_number = FINDPATH_TASK;
            isStableHover = false;
        } else if (msg->buttons[2] > 0.5) {
            //X
            task_number = POINTTOPOINT_TASK;
            isStableHover = false;
        } */
        else if (msg->buttons[3] > 0.5) {
            //Y. Press Y, the robot will continue drive!
            pub_zero_velocity = false;
        }
    }

    void ObsAvoid::clearCostmapWindows(double size_x, double size_y){
        geometry_msgs::PoseStamped global_pose;
        //clear the planner's costmap
        getRobotPose(global_pose, planner_costmap_ros_);
        std::vector<geometry_msgs::Point> clear_poly;
        double x = global_pose.pose.position.x;
        double y = global_pose.pose.position.y;
        geometry_msgs::Point pt;
        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);
        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);
        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);
        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);
        planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
        //clear the controller's costmap
        getRobotPose(global_pose, controller_costmap_ros_);
        clear_poly.clear();
        x = global_pose.pose.position.x;
        y = global_pose.pose.position.y;
        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);
        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);
        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);
        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);
        controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
    }

    bool ObsAvoid::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
        //clear the costmaps
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
        controller_costmap_ros_->resetLayers();
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
        planner_costmap_ros_->resetLayers();
        return true;
    }

    void ObsAvoid::publishZeroVelocity(){
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);
    }

    double ObsAvoid::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
    {
        return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
    }

    bool ObsAvoid::loadRecoveryBehaviors(ros::NodeHandle node){
        XmlRpc::XmlRpcValue behavior_list;
        if(node.getParam("recovery_behaviors", behavior_list)){
            if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
                for(int i = 0; i < behavior_list.size(); ++i){
                    if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                        if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
                            //check for recovery behaviors with the same name
                            for(int j = i + 1; j < behavior_list.size(); j++){
                                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                                    if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                                        std::string name_i = behavior_list[i]["name"];
                                        std::string name_j = behavior_list[j]["name"];
                                        if(name_i == name_j){
                                            ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                                                      name_i.c_str());
                                            return false;
                                        }
                                    }
                                }
                            }
                        }
                        else{
                            ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
                            return false;
                        }
                    }
                    else{
                        ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                                  behavior_list[i].getType());
                        return false;
                    }
                }

                //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
                for(int i = 0; i < behavior_list.size(); ++i){
                    try{
                        //check if a non fully qualified name has potentially been passed in
                        if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
                            std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
                            for(unsigned int i = 0; i < classes.size(); ++i){
                                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                                    //if we've found a match... we'll get the fully qualified name and break out of the loop
                                    ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                                             std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                                    behavior_list[i]["type"] = classes[i];
                                    break;
                                }
                            }
                        }

                        boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

                        //shouldn't be possible, but it won't hurt to check
                        if(behavior.get() == NULL){
                            ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
                            return false;
                        }

                        //initialize the recovery behavior with its name
                        behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
                        recovery_behavior_names_.push_back(behavior_list[i]["name"]);
                        recovery_behaviors_.push_back(behavior);
                    }
                    catch(pluginlib::PluginlibException& ex){
                        ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
                        return false;
                    }
                }
            }
            else{
                ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
                          behavior_list.getType());
                return false;
            }
        }
        else{
            //if no recovery_behaviors are specified, we'll just load the defaults
            return false;
        }

        //if we've made it here... we've constructed a recovery behavior list successfully
        return true;
    }

    //we'll load our default recovery behaviors here
    void ObsAvoid::loadDefaultRecoveryBehaviors(){
        recovery_behaviors_.clear();
        try{
            //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
            ros::NodeHandle n("~");
            n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
            n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

            //first, we'll load a recovery behavior to clear the costmap
            boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back("conservative_reset");
            recovery_behaviors_.push_back(cons_clear);

            //next, we'll load a recovery behavior to rotate in place
            boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
            if(clearing_rotation_allowed_){
                rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
                recovery_behavior_names_.push_back("rotate_recovery");
                recovery_behaviors_.push_back(rotate);
            }

            //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
            boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back("aggressive_reset");
            recovery_behaviors_.push_back(ags_clear);

            //we'll rotate in-place one more time
            if(clearing_rotation_allowed_){
                recovery_behaviors_.push_back(rotate);
                recovery_behavior_names_.push_back("rotate_recovery");
            }
        }
        catch(pluginlib::PluginlibException& ex){
            ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
        }

        return;
    }

    void ObsAvoid::resetState(){
        // Disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        // Reset statemachine
        recovery_index_ = 0;
        recovery_trigger_ = PLANNING_R;
        publishZeroVelocity();

        //if we shutdown our costmaps when we're deactivated... we'll do that now
        if(shutdown_costmaps_){
            ROS_DEBUG_NAMED("move_base","Stopping costmaps");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }
    }

    bool ObsAvoid::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap){
        tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
        geometry_msgs::PoseStamped robot_pose;
        tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
        robot_pose.header.frame_id = robot_base_frame_;//robot_base_frame_ is base_link
        robot_pose.header.stamp = ros::Time(); // latest available
        ros::Time current_time = ros::Time::now();  // save time for checking tf delay later
        // get robot pose on the given costmap frame
        try{
            tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
        }
        catch (tf2::LookupException& ex)
        {
            ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ConnectivityException& ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ExtrapolationException& ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        // check if global_pose time stamp is within costmap transform tolerance
        if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
        {
            ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                              current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
            return false;
        }
        return true;
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "move_base_node");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    obs_avoid::ObsAvoid obs_avoid( buffer );

    ros::spin();

    return(0);
}

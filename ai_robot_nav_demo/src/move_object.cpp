#include "move_object.h"

MoveObject::MoveObject() : it_(nh_), ac("move_base", true), detect_object(false), move_finish(false), get_object_position(false), getflagsuccess(false), robot_move(false), move_base("move_base", true)
{
	//topic sub:
	Object_sub = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &MoveObject::ObjectCallback, this);
	image_sub_depth = it_.subscribe("/zed2/depth/depth_registered", 1, &MoveObject::imageDepthCb, this);
	image_sub_color = it_.subscribe("/zed2/left_raw/image_raw_color", 1, &MoveObject::imageColorCb, this);
	camera_info_sub_ = nh_.subscribe("/zed2/depth/camera_info", 1, &MoveObject::cameraInfoCb, this);
}

MoveObject::~MoveObject()
{
	ROS_INFO("delete the class");
}

void MoveObject::cameraInfoCb(const sensor_msgs::CameraInfo &msg)
{
	camera_info = msg;
    cout << "cameraInfoCb succeeded." << "\n";
}

void MoveObject::imageDepthCb(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		depthImage = cv_ptr->image;
        cout << "imageDepthCb succeeded." << "\n";
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

void MoveObject::ObjectCallback(const darknet_ros_msgs::BoundingBoxes &object_tmp)
{
	if (robot_move == false)
	{
		string Object_class;
		Object_class = object_tmp.bounding_boxes[0].Class;
        cout << "ObjectCallback ******* Object_class: " << Object_class.c_str() << "\n";
		if (strcmp(Object_class.c_str(), "person") == 0)
		{
			mousepos.x = (object_tmp.bounding_boxes[0].xmin + object_tmp.bounding_boxes[0].xmax) / 2;
			mousepos.y = (object_tmp.bounding_boxes[0].ymin + object_tmp.bounding_boxes[0].ymax) / 2;
			if (mousepos.x != 0)
			{
				detect_object = true;
				ROS_INFO("detected object");
			}
		}
	}
}
void MoveObject::imageColorCb(const sensor_msgs::ImageConstPtr &msg)
{
	if (detect_object && robot_move == false)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			colorImage = cv_ptr->image;
            cout << "imageColorCb succeeded." << "\n";
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		//先查询对齐的深度图像的深度信息，根据读取的camera info内参矩阵求解对应三维坐标
		real_z = 0.001 * depthImage.at<u_int16_t>(mousepos.y, mousepos.x);
        //real_z = 5.885;
        cout << "Depth of the detected center: " << mousepos.x << ", " << mousepos.y << " is " << real_z << "\n";
		real_x = (mousepos.x - camera_info.K.at(2)) / camera_info.K.at(0) * real_z;
        cout << "mousepose.x is: " << mousepos.x << " and " << "camera_info.K.at(2) is: " << camera_info.K.at(2) << "\n";
		real_y = (mousepos.y - camera_info.K.at(5)) / camera_info.K.at(4) * real_z;
        cout << "mousepose.y is: " << mousepos.y << " and " << "camera_info.K.at(5) is: " << camera_info.K.at(5) << "\n";
        cout << "real_x: " << real_x << " " << "real_y: " << real_y << "\n";
		if (real_x != 0 && real_y != 0)
		{
			get_object_position = true;
			Object_pose_tmp.header.frame_id = "support_depth";
			Object_pose_tmp.header.stamp = ros::Time::now();
			Object_pose_tmp.pose.position.x = real_x;
			Object_pose_tmp.pose.position.y = real_y;
			Object_pose_tmp.pose.position.z = real_z - 0.8;
			Object_pose_tmp.pose.orientation = tf::createQuaternionMsgFromYaw(0);
			ROS_INFO("get the position of object");
		}
	}
}

void MoveObject::transformTf()
{
	try
	{
		listener.transformPose("map", Object_pose_tmp, Object_pose);
		getflagsuccess = true;
		robot_move = true;
		ROS_INFO("Transform successed");
	}
	catch (tf::TransformException &ex)
	{
		ros::Duration(0.5).sleep();
		getflagsuccess = false;
		std::cout << "Transform failed, other try" << std::endl;
	}
}

void MoveObject::goObject()
{
	//connet to the Server, 5s limit
	while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("waiting for the move_base action server to come up.");
    }
	ROS_INFO("Connected to move_base server.");
	//set the targetpose
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = Object_pose.pose.position.x;
	goal.target_pose.pose.position.y = Object_pose.pose.position.y;
    goal.target_pose.pose.position.z = 0;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(2.63);
	//goal.target_pose.pose.orientation.z = -0.9961;
	//goal.target_pose.pose.orientation.w = 0.0877;
    cout << "goal.target_pose.pose.position.x is: " << goal.target_pose.pose.position.x << endl;
    cout << "goal.target_pose.pose.position.y is: " << goal.target_pose.pose.position.y << endl;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal succeeded!");
        move_finish = true;
    }
	else
		ROS_INFO("Goal failed");
}

void MoveObject::initMove()
{
	ros::AsyncSpinner spinner(1);
	spinner.start();
	if (get_object_position)
	{
		transformTf();
		sleep(1);
	}
	if (getflagsuccess)
	{
		goObject();
//		move_finish = true;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_object");
	MoveObject moveobject;
	while (ros::ok())
	{
		ros::spinOnce();
		moveobject.initMove();
		if (moveobject.move_finish)
		{
			ros::shutdown();
		}
	}
	return 0;
}




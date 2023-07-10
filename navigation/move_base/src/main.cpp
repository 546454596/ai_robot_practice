#include <time.h>
#include <stdlib.h>
#include <move_base/AIBrain.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv,"brain");
    ros::NodeHandle nh;
    ros::Publisher brainok_pub = nh.advertise<std_msgs::Bool>("/brain/ok",1);
    ros::Rate r(50);

/*    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    move_base::MoveBase nav(buffer);*/

    AIBrain aib(&nh);
    ROS_INFO("[AIBRAIN]Brain wake up!");

    while (ros::ok()){
        aib.think();
        std_msgs::Bool _b;
        _b.data = true;
        brainok_pub.publish(_b);
        ros::spinOnce();
        r.sleep();
    }

    ros::shutdown();
    return 0;
}


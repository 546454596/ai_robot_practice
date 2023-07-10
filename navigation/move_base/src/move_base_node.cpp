#include <move_base/move_base.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "move_base_node");
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);
  move_base::MoveBase move_base( buffer );

  ros::spin();

  return(0);
}

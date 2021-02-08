#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include<unistd.h>



  
void poseCallback(const std::string parent_frame, const std::string child_frame, const tf2::Quaternion& q, const std::vector<double>& translation)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parent_frame;
  transformStamped.child_frame_id = child_frame;

  transformStamped.transform.translation.x = translation[0];
  transformStamped.transform.translation.y = translation[1];
  transformStamped.transform.translation.z = translation[2];
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}
  
int main(int argc, char** argv)
{   
  tf2::Quaternion q;
  q[0] = 0;
  q[1] = 1;
  q[2] = 0;
  q[3] = 0;

  std::vector<double> translation {0,0,0};
  translation[0] = 1.0;
  translation[1] = 2.0;
  translation[2] = 0.0;

  unsigned int microsecond = 1000000;
  int n = 0;

  ros::init(argc, argv, "cam_object_broadcaster");

  while(n<20) {

    std::cout << "Test " << n << "\n";

    poseCallback("base_link", "object", q, translation);
    usleep(2 * microsecond);//sleeps for 2 second
    n++;

  };
  // ros::spin();
  return 0;
};
#include <ros/ros.h>

#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Transform.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
  
//  std::string camera_name;  
  
  
void poseCallback(const turtlesim::PoseConstPtr& msg)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "camer_base_link";
  transformStamped.child_frame_id = "object";
  transformStamped.transform.translation.x = 1.0;
  transformStamped.transform.translation.y = 2.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 1.4);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}
  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_broadcaster");
  
  // ros::NodeHandle n;
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  // ros::Rate loop_rate(10);
  // int count = 0;
  // while (ros::ok())
  // {
  //   std_msgs::String msg;
  //   std::stringstream ss;
  //   ss << "hello world " << count;
  //   msg.data = ss.str();
  //   ROS_INFO("%s", msg.data.c_str());
  //   chatter_pub.publish(msg);

  //   ros::spinOnce();
  //   loop_rate.sleep();
  //   ++count;
  // }
  
  //  if (! n.hasParam("camera"))
  //  {
  //      if (argc != 2)
  //      {
  //          ROS_ERROR("need camera name as argument"); 
  //          return -1;
  //       };
  //       camera_name = argv[1];
  //  }
  //  else
  //  {
  //    n.getParam("camera", camera_name);
  //  }
     
   ros::NodeHandle node;   
   ros::Subscriber sub = node.subscribe("camera/color/image_raw", 1000, &poseCallback);
   ros::spin();

   return 0;
 };
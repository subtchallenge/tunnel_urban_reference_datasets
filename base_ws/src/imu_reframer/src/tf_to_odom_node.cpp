#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

ros::Publisher *pub;
tf::TransformListener *listener;
std::string base_frame, odom_frame;

void timerCallback(const ros::TimerEvent&) {
  tf::StampedTransform transform;
  try{
    listener->lookupTransform(base_frame, odom_frame, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return;
  }
  
  nav_msgs::Odometry out;
  out.header.stamp = transform.stamp_;
  
  out.pose.pose.position.x = transform.getOrigin().x();
  out.pose.pose.position.y = transform.getOrigin().y();
  out.pose.pose.position.z = transform.getOrigin().z();
  tf::quaternionTFToMsg(transform.getRotation(), out.pose.pose.orientation);
  
  out.header.frame_id = odom_frame;
  out.child_frame_id = base_frame;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      if (i == j) {
        out.pose.covariance[i + j * 6] = 9e-2;
        out.twist.covariance[i + j * 6] = 9e-2;
      } else {
        out.pose.covariance[i + j * 6] = 9e-3;
        out.twist.covariance[i + j * 6] = 9e-3;
      }
    }
  }
  pub->publish(out);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_vel_to_odom");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher publ = nh.advertise<nav_msgs::Odometry>("odom_filtered", 10);
  tf::TransformListener listnr;
  pnh.param("odom_frame", odom_frame, std::string("odom"));
  pnh.param("base_frame", base_frame, std::string("base"));
  pub = &publ;
  listener = &listnr;
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);
  ros::spin();
  return -1;
}

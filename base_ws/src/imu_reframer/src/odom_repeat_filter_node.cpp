#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

ros::Publisher *pub;
ros::Time last_header_time = ros::Time(0);
std::string base_frame, odom_frame;

void handleOdom(const nav_msgs::Odometry::ConstPtr &data) {
  //if (data->header.stamp - last_header_time < ros::Duration(0.001)) {
  if (data->header.stamp <= last_header_time) {
    return;
  } else {
    last_header_time = data->header.stamp;
    nav_msgs::Odometry out = *data;
    if(odom_frame.compare("na") != 0)
      out.header.frame_id = odom_frame;
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
}

void handleVel(const geometry_msgs::TwistStamped::ConstPtr &data) {
  //if (data->header.stamp - last_header_time < ros::Duration(0.001)) {
  if (data->header.stamp <= last_header_time) {
    return;
  } else {
    last_header_time = data->header.stamp;
    nav_msgs::Odometry out;
    out.twist.twist = data->twist;
    if(odom_frame.compare("na") != 0) {
      out.header.frame_id = odom_frame;
      out.child_frame_id = base_frame;
    }
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
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_repeat_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher publ = nh.advertise<nav_msgs::Odometry>("odom_filtered", 10);
  pnh.param("odom_frame", odom_frame, std::string("odom"));
  pnh.param("base_frame", base_frame, std::string("na"));
  pub = &publ;
  ros::Subscriber sub_odom = nh.subscribe("odom_odom", 10, &handleOdom);
  ros::Subscriber sub_vel = nh.subscribe("odom_vel", 10, &handleVel);
  ros::spin();
  return -1;
}

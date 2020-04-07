#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

void handleOdom(ros::Publisher& pub, const std::string& new_frame,
                const geometry_msgs::TwistStamped::ConstPtr &data) {
  static ros::Time last_header_time = ros::Time(0);
  if (last_header_time != ros::Time(0) && data->header.stamp <= last_header_time) {
    ROS_ERROR_STREAM("Received out of order odometry message in odom reframer");
    return;
  } else {
    last_header_time = data->header.stamp;
    geometry_msgs::TwistStamped out = *data;
    if (new_frame.compare("na") != 0) {
      out.header.frame_id = new_frame;
    }
    pub.publish(out);
  }
}

void handleVel(ros::Publisher& pub, const std::string& new_frame, const std::string& base_frame,
               const geometry_msgs::TwistStamped::ConstPtr &data) {
  static ros::Time last_header_time = ros::Time(0);
  if (last_header_time != ros::Time(0) && data->header.stamp <= last_header_time) {
    ROS_ERROR_STREAM("Received out of order odometry message in odom reframer");
    return;
  } else {
    ROS_ERROR_STREAM("Publishing translated odom");
    last_header_time = data->header.stamp;
    nav_msgs::Odometry out;
    out.twist.twist = data->twist;
    if (new_frame.compare("na") != 0) {
      out.header.frame_id = new_frame;
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
    pub.publish(out);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_repeat_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher publ_odom = nh.advertise<nav_msgs::Odometry>("odom_filtered", 10);
  ros::Publisher publ_twist = nh.advertise<geometry_msgs::TwistStamped>("twist_filtered", 10);
  std::string odom_frame, base_frame;
  pnh.param("odom_frame", odom_frame, std::string("odom"));
  pnh.param("base_frame", base_frame, std::string("na"));
  ros::Subscriber sub_odom =
    nh.subscribe<geometry_msgs::TwistStamped>("odom", 10, boost::bind(&handleOdom, publ_twist, odom_frame, _1));
  ros::Subscriber sub_vel =
    nh.subscribe<geometry_msgs::TwistStamped>("odom_vel", 10, boost::bind(&handleVel, publ_odom, odom_frame, base_frame, _1));
  ros::spin();
  return -1;
}

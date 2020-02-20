#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

ros::Publisher *pub;
ros::Time last_header_time = ros::Time(0);

void handleOdom(const nav_msgs::Odometry::ConstPtr &data) {
  if (data->header.stamp - last_header_time < ros::Duration(0.001)) {
    return;
  } else {
    last_header_time = data->header.stamp;
    nav_msgs::Odometry out = *data;
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
  ros::Publisher publ = nh.advertise<nav_msgs::Odometry>("odom_filtered", 10);
  pub = &publ;
  ros::Subscriber sub = nh.subscribe("odom", 10, &handleOdom);
  ros::spin();
  return -1;
}

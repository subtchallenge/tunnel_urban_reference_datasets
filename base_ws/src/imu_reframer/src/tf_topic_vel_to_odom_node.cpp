#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf2/LinearMath/Quaternion.h>

ros::Publisher *pub;
std::string base_frame, odom_frame;
nav_msgs::Odometry out;
ros::Time last_stamp;

void timerCallback(const ros::TimerEvent&) {
  pub->publish(out);
}

void handleTf(const tf::tfMessage::ConstPtr &data) {
  for(int i=0; i<data->transforms.size(); i++) {
    if (data->transforms[i].header.frame_id == odom_frame && data->transforms[i].child_frame_id == base_frame) {
      ros::Duration odom_diff = data->transforms[i].header.stamp - last_stamp;
      if(odom_diff.toSec() < 0.01)
      {
        ROS_WARN("Skipping odometry message due to no update on tf odom.");
        return;
      }
      last_stamp = data->transforms[i].header.stamp;
      out.header.stamp = data->transforms[i].header.stamp;
      out.pose.pose.position.x = data->transforms[i].transform.translation.x;
      out.pose.pose.position.y = data->transforms[i].transform.translation.y;
      out.pose.pose.position.z = data->transforms[i].transform.translation.z;
      out.pose.pose.orientation = data->transforms[i].transform.rotation;
    }
  }
}

void handleVel(const geometry_msgs::TwistStamped::ConstPtr &data) {
  out.twist.twist = data->twist;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_vel_to_odom");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  ros::Publisher publ = nh.advertise<nav_msgs::Odometry>("odom_filtered", 10);
 
  double interval;
  pnh.param("interval", interval, 0.1); 
  pnh.param("odom_frame", odom_frame, std::string("odom"));
  pnh.param("base_frame", base_frame, std::string("base"));
  
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
  
  pub = &publ;
  
  ros::Subscriber sub_tf = nh.subscribe("/tf_remap", 10, &handleTf);
  ros::Subscriber sub_vel = nh.subscribe("odom_vel", 10, &handleVel);
  ros::Timer timer = nh.createTimer(ros::Duration(interval), timerCallback);
  ros::spin();
  return -1;
}

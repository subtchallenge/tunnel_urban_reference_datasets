#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

ros::Publisher *pub;

std::string new_frame;

void handleImu(const sensor_msgs::Imu::ConstPtr &data) {
  static ros::Time last_time = ros::Time(0);
  if (data->header.stamp <= last_time)
    return;
  last_time = data->header.stamp;
  sensor_msgs::Imu out = *data;
  out.header.frame_id = new_frame;
  for (int i = 0; i < 9; i++) {
    out.angular_velocity_covariance[i] = 9e-7;
    out.linear_acceleration_covariance[i] = 7e-4;
    out.orientation_covariance[i] = 9e-7;
  }
  pub->publish(out);
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_reframer");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.param("new_frame", new_frame, std::string("chinook/imu2"));
  ros::Publisher publ = nh.advertise<sensor_msgs::Imu>("imu_reframed/data", 10);
  pub = &publ;
  ros::Subscriber sub = nh.subscribe("imu/data", 10, &handleImu);
  ros::spin();
  return -1;
}

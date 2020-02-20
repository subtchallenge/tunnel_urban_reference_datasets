#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

bool get_transform(tf::TransformListener &tfL, const std::string &from_frame,
                   const std::string &to_frame, tf::StampedTransform &result) {

  try {
    tfL.waitForTransform(to_frame, from_frame, ros::Time(0),
                         ros::Duration(0.2));
    tfL.lookupTransform(to_frame, from_frame, ros::Time(0), result);
  } catch (const tf::TransformException &ex) {
    ROS_ERROR_STREAM("Cant get transform for distance traveled counter "
                     << ex.what());
    return false;
  }
  return true;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "distance_traveled_node");
  ros::NodeHandle nh("~");
  tf::StampedTransform last_transf;
  double distance_traveled = 0.0;
  ros::Rate r(1.0);
  std::string from_frame, to_frame;
  tf::TransformListener tfL;
  nh.param("from_frame", from_frame, std::string("darpa"));
  nh.param("to_frame", to_frame, std::string("chinook/base"));
  while (ros::ok() && !get_transform(tfL, from_frame, to_frame, last_transf)) {
    r.sleep();
  }
  while (ros::ok()) {
    tf::StampedTransform curr_transf;
    if (get_transform(tfL, from_frame, to_frame, curr_transf)) {
      double disp = (curr_transf * last_transf.inverse()).getOrigin().length();
      distance_traveled += disp;
      ROS_INFO_STREAM("Traveled a total distance of " << distance_traveled);
      last_transf = curr_transf;
    }
    r.sleep();
  }
  ROS_INFO_STREAM("Traveled a total distance of " << distance_traveled);
  return 1;
}

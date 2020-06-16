#pragma once
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>
#include <utility>
#include <map>
#include <list>
#include <string>
#include <tuple>

class SubTScoringNode {
 protected:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  std::vector<std::pair<std::string, tf::Point>> gt_artifacts;
  std::list<std::tuple<double, std::string, tf::Stamped<tf::Point>>> reports;
  std::list<std::tuple<double, std::string, tf::Stamped<tf::Point>, double>>
      finished_reports;
  std::map<string, tf::Point> fiducial_point_darpa_frame;
  tf::TransformListener tfL;
  tf2_ros::StaticTransformBroadcaster tfB;
  std::string map_frame;

  std::vector<tf::Point> fiducials_observed;
  std::vector<std::tuple<std::string, tf::Point, int, std::string>> good_artifacts;
  std::vector<std::tuple<std::string, tf::Point, int, std::string>> bad_artifacts;
  std::vector<std::tuple<tf::Point, int>> relative_frame_aligned_artifacts;

  ros::Publisher marker_pub;
  ofstream rmse_file;
  bool initialize_flat;  // Use zero for roll and pitch in darpa -> map frame. Useful for alpha course with no baseline on pitch observation to distal fiducial

  bool reverse_transform;
  std::string gt_filename;
  std::string artifacts_filename;
  std::string output_filename;
  std::string fiducial_file;
  std::string rmse_filename;
  bool use_optical_frame_report;

 public:
  SubTScoringNode();
  void PublishMarkers();
  double HandleReport(const std::tuple<double, std::string, tf::Stamped<tf::Point>> &report,
                      double now_sec);
  void onImagesScoring(CodingManager *md, const sensor_msgs::ImageConstPtr &msg,
                       const sensor_msgs::ImageConstPtr &depth_msg);
  void Load();
  void ScoreRun();
  bool coding_mode;
};

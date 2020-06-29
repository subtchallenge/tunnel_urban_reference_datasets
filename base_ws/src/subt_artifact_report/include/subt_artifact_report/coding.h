#pragma once
/*
 *
 * John G. Rogers III
 *
 * 
 */

#include <opencv2/opencv.hpp>
#include <vector>
using namespace std;
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/thread/mutex.hpp>
#include <tf/transform_listener.h>

#include <apriltag.h>
#include <apriltag_pose.h>
#include <tag16h5.h>


struct button_t {
  std::string text;
  unsigned char r;
  unsigned char g;
  unsigned char b;
  cv::Point ul;
  cv::Point br;
};

class CodingManager {
  public:
    std::vector<button_t> buttons;
  boost::shared_ptr<image_geometry::PinholeCameraModel> camera_model;
  cv::Mat depth_image;
  std_msgs::Header image_hdr;
  mutable boost::mutex depth_img_mutex;
  boost::shared_ptr<tf::TransformListener> tf_l;
  std::string report_frame;
  std::ofstream outfile;
  int active_class = -1;
  apriltag_detector_t *td;
  double scale = 1.0;
  ros::NodeHandle& nh_;
  ros::NodeHandle& private_nh_;
  std::map<std::string, std::pair<double, std::string> > reports_to_write_first;
  bool fiducials_done = false;
  double depth_map_factor_;

  CodingManager(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  virtual ~CodingManager();
};

float find_depth_nearest_pixel(const cv::Mat& depth_image, int x, int y, float depth_map_factor); 
bool ImageTo3DPoint(int x, int y, const CodingManager* md,
                    tf::Vector3& local_pt_vec, tf::Vector3& report_pt_vec);

void onMouse (int event, int x, int y, int, void* data);

void draw_labels(cv::Mat image,
                 const std::vector<button_t>& buttons,
                 cv::Mat& out_image,
                 double scale);

void onCameraInfo(CodingManager* md, const sensor_msgs::CameraInfo::ConstPtr& msg);

void onImages(CodingManager* md, 
              const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& depth_msg);


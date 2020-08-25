/*
 *
 * John G. Rogers III
 *
 *
 */

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>

#include <apriltag.h>
#include <apriltag_pose.h>
#include <tag16h5.h>
#include <Eigen/Geometry>
#include <ros/xmlrpc_manager.h>

#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include <fstream>
#include <boost/thread/mutex.hpp>
#include <opencv2/opencv.hpp>

class FrameAlignmentNode {
 protected:
    std::map<std::string, tf::Point> fiducial_point_global_frame;
    tf::TransformListener tfL;
    tf2_ros::StaticTransformBroadcaster tfB;
    std::string map_frame;
    std::string odom_frame;
    std::map<std::string, tf::Point> fiducials_observed;
    std::vector<std::pair<std::string, tf::Point>> gt_artifacts;
    ros::Publisher marker_pub;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber info_sub;
    image_transport::ImageTransport it;
    boost::shared_ptr<image_transport::SubscriberFilter> img_sub;
    boost::shared_ptr<image_transport::SubscriberFilter> depth_img_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
            sensor_msgs::Image>
              MySyncPolicy;
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > sync;
    image_geometry::PinholeCameraModel camera_model;
    std::map<std::string, std::tuple<tf::Point, tf::Point> >
      frame_correspondences_seen;
    std::string report_frame;
    bool initialize_flat = false;  // Use zero for roll and pitch in darpa -> map frame. Useful for alpha course with no baseline on pitch observation to distal fiducial

    void PublishMarkers() const;
    void onCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void onImages(const sensor_msgs::ImageConstPtr &msg,
                  const sensor_msgs::ImageConstPtr &depth_msg);
    bool ImageTo3DPoint(int x, int y, const cv::Mat& depth_image, const std::string& image_frame,
                        tf::Vector3& local_pt_vec, tf::Vector3& report_pt_vec);
    void UpdateTransform(const std::string& report_string,
                         tf::Stamped<tf::Point> detection);
    apriltag_detector_t *td;
    std::map<int,float> best_margins_;
    double depth_map_factor_;

 public:
    FrameAlignmentNode();
};

FrameAlignmentNode::
FrameAlignmentNode(): nh(), private_nh("~"), it(nh), tfL(ros::Duration(20.0)){

  std::string gt_filename;
  private_nh.param("gt_filename", gt_filename, std::string("SCREWED"));

  private_nh.param("map_frame", map_frame, std::string("map"));
  private_nh.param("odom_frame", odom_frame, std::string("odom"));
  private_nh.param("report_frame", report_frame, std::string("base"));
  
  private_nh.param("initialize_flat", initialize_flat, false);

  marker_pub =
    nh.advertise<visualization_msgs::MarkerArray>("artifact_markers", 1);

  td = apriltag_detector_create();
  apriltag_family_t *tagfam = tag16h5_create();
  apriltag_detector_add_family_bits(td, tagfam, 1); // Allow this many bit errors
  td->quad_decimate = 2.0;
  td->quad_sigma = 0.0; // Blur factur (negative sharpens)
  td->nthreads = 1;
  td->debug = 0;
  td->refine_edges = 1;

  std::ifstream infile(gt_filename.c_str(), std::ios::in);
  if (!infile.is_open()) {
    ROS_ERROR_STREAM("Please provide a gt_filename. What I have is: " << gt_filename);
    exit(1);
  }
  std::string line;
  while (std::getline(infile, line)) {
    std::stringstream ss(line);
    std::string label;
    double x, y, z;
    ss >> label >> x >> y >> z;
    gt_artifacts.push_back(std::make_pair(label, tf::Point(x, y, z)));
  }
  infile.close();
  XmlRpc::XmlRpcValue fiducial_param;

  if (private_nh.getParam("fiducials", fiducial_param) && fiducial_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    try {
      for (size_t i = 0; i < fiducial_param.size(); i++) {
        std::string label;
        double x, y, z;

        if (fiducial_param[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
          for (auto input_fiducial: fiducial_param[i]) {
            label = input_fiducial.first;
            if (input_fiducial.second.hasMember("x") && input_fiducial.second["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
              x = input_fiducial.second["x"];
              if (input_fiducial.second.hasMember("y") && input_fiducial.second["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                y = input_fiducial.second["y"];
                if (input_fiducial.second.hasMember("z") && input_fiducial.second["z"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                  z = input_fiducial.second["z"];
                  fiducial_point_global_frame[label] = tf::Point(x, y, z);
                }
              }
            }
          }
        }
      }
    } catch(const XmlRpc::XmlRpcException& e) {
      ROS_ERROR("%s", e.getMessage().c_str());
    }
  }
  std::string image_name, depth_image_name, img_transport;
  private_nh.param("image", image_name, std::string("image"));
  private_nh.param("depth_image", depth_image_name, std::string("depth_image"));
  private_nh.param("image_transport", img_transport, std::string("compressed"));
  private_nh.param("depth_map_factor", depth_map_factor_, 1.0);

  info_sub =
    nh.subscribe<sensor_msgs::CameraInfo>("camera_info", 1,
                                          boost::bind(&FrameAlignmentNode::onCameraInfo,
                                                      this, _1));
  img_sub =
    boost::make_shared<image_transport::SubscriberFilter>(it, image_name, 50, image_transport::TransportHints(img_transport));
  depth_img_sub =
    boost::make_shared<image_transport::SubscriberFilter>(it, depth_image_name, 50);
  sync = boost::make_shared<message_filters::Synchronizer<MySyncPolicy> > (MySyncPolicy(50), *img_sub,
                                                                           *depth_img_sub);
  sync->registerCallback(boost::bind(&FrameAlignmentNode::onImages, this, _1, _2));
  ROS_INFO("Connected to image topics");
}

float find_depth_nearest_pixel(const cv::Mat& depth_image, int x, int y, float depth_map_factor) {
  float depth;
  depth = depth_image.at<float>(y, x)/depth_map_factor;
  int xmin = x - 1;
  int xmax = x + 1;
  int ymin = y - 1;
  int ymax = y + 1;
  while (std::isnan(depth)) {
    if (xmin < 0) xmin = 0;
    if (xmax >= depth_image.cols) xmax = depth_image.cols-1;
    if (ymin < 0) xmin = 0;
    if (ymax >= depth_image.rows) ymax = depth_image.rows-1;
    for (x = xmin, y = ymin; x <= xmax; x++) {
      depth = depth_image.at<float>(y, x);
      if (!std::isnan(depth)) return depth;
    }
    for (x = xmin, y = ymax; x <= xmax; x++) {
      depth = depth_image.at<float>(y, x);
      if (!std::isnan(depth)) return depth;
    }
    for (x = xmin, y = ymin; y <= ymax; y++) {
      depth = depth_image.at<float>(y, x);
      if (!std::isnan(depth)) return depth;
    }
    for (x = xmax, y = ymin; y <= ymax; y++) {
      depth = depth_image.at<float>(y, x);
      if (!std::isnan(depth)) return depth;
    }
    xmin = xmin - 1;
    ymin = ymin - 1;
    xmax = xmax + 1;
    ymin = ymax + 1;
    if (xmin < 0 && ymin < 0 && xmax >= depth_image.cols && ymax >= depth_image.rows)
      return nanf("");
  }
  return depth;
}

bool FrameAlignmentNode::
ImageTo3DPoint(int x, int y, const cv::Mat& depth_image, const std::string& image_frame,
               tf::Vector3& local_pt_vec, tf::Vector3& report_pt_vec) {
  if (!camera_model.initialized()) return false;
  float depth;
  depth = find_depth_nearest_pixel(depth_image, x, y, float(depth_map_factor_));
  if (std::isnan(depth)) return false;

  cv::Point3d ray = camera_model.projectPixelTo3dRay(cv::Point2d(x, y));
  cv::Point3d local_pt = ray * depth;
  tf::StampedTransform image_to_report_frame;
  try {
    tfL.waitForTransform(report_frame, image_frame, ros::Time(0), ros::Duration(1.2));
    tfL.lookupTransform(report_frame, image_frame, ros::Time(0), image_to_report_frame);
  } catch(const tf::TransformException& ex) {
    ROS_ERROR_STREAM("Transform exception in subt_coding_node. Cannot transform from "
                     << report_frame << " to " << image_frame << ": " << ex.what());
    return false;
  }
  local_pt_vec = tf::Vector3(local_pt.x, local_pt.y, local_pt.z);
  report_pt_vec = image_to_report_frame * local_pt_vec;
  return true;
}


void FrameAlignmentNode::
onCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg) {
  camera_model.fromCameraInfo(msg);
}

void FrameAlignmentNode::
PublishMarkers() const {
  static visualization_msgs::MarkerArray delete_markers;
  // Draw all ground truth points in blue
  // Draw all failed artifact in red
  // Draw all good artifacts in green
  visualization_msgs::MarkerArray send_markers;

  marker_pub.publish(delete_markers);
  delete_markers.markers.clear();
  visualization_msgs::Marker marker;
  marker.header.frame_id = "darpa";
  marker.header.stamp = ros::Time::now();
  marker.ns = "artifact_markers";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.lifetime = ros::Duration(0);
  marker.frame_locked = true;
  marker.color.r = 0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  for (size_t i = 0; i < gt_artifacts.size(); i++) {
    marker.pose.position.x = gt_artifacts[i].second.x();
    marker.pose.position.y = gt_artifacts[i].second.y();
    marker.pose.position.z = gt_artifacts[i].second.z();
    send_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.id++;
    visualization_msgs::Marker text_marker = marker;
    text_marker.pose.position.z += 1.0;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.text = gt_artifacts[i].first;
    send_markers.markers.push_back(text_marker);
    text_marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(text_marker);
    marker.id++;
  }
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.header.frame_id = "map";
  for (std::map<std::string, tf::Point>::const_iterator itr = fiducials_observed.begin();
       itr != fiducials_observed.end();
       itr++) {
    marker.pose.position.x = itr->second.x();
    marker.pose.position.y = itr->second.y();
    marker.pose.position.z = itr->second.z();
    send_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.id++;
  }
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.5;
  marker.scale.x = 1.5;
  marker.scale.y = 1.5;
  marker.scale.z = 1.5;
  marker.header.frame_id = "darpa";
  for (std::map<std::string, tf::Point>::const_iterator itr =
       fiducial_point_global_frame.begin();
       itr != fiducial_point_global_frame.end(); itr++) {
    marker.pose.position.x = itr->second.x();
    marker.pose.position.y = itr->second.y();
    marker.pose.position.z = itr->second.z();
    send_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.id++;
  }

  marker_pub.publish(send_markers);
}

void printMat(const Eigen::MatrixXf &mat) {
  for (size_t i = 0; i < mat.rows(); i++) {
    for (size_t j = 0; j < mat.cols(); j++) {
      std::cout << mat(i, j) << " ";
    }
    std::cout << std::endl;
  }
}

void EigenToTransformMsg(const Eigen::MatrixXf &eig,
                         geometry_msgs::Transform &transf) {
  transf.translation.x = eig(0, 3);
  transf.translation.y = eig(1, 3);
  transf.translation.z = eig(2, 3);
  tf::Matrix3x3 matr(eig(0, 0), eig(0, 1), eig(0, 2), eig(1, 0), eig(1, 1),
                     eig(1, 2), eig(2, 0), eig(2, 1), eig(2, 2));

  tf::Quaternion quat;
  matr.getRotation(quat);
  tf::quaternionTFToMsg(quat, transf.rotation);
}

void FrameAlignmentNode::UpdateTransform(const std::string& report_string,
                                         tf::Stamped<tf::Point> detection) {
  // It is a fiducial marker. Refine the estimate of the global
  // frame to map transform
  std::map<std::string, tf::Point>::iterator fpgf_itr =
    fiducial_point_global_frame.find(report_string);
  if (fpgf_itr != fiducial_point_global_frame.end()) {
    tf::StampedTransform transf;
    tf::StampedTransform transf_map_odom;
    try {
      tfL.waitForTransform(detection.frame_id_, odom_frame,
                            detection.stamp_, ros::Duration(0.3));
      tfL.lookupTransform(detection.frame_id_, odom_frame,
                           detection.stamp_, transf);
      tfL.lookupTransform(odom_frame, map_frame,
                          ros::Time(0), transf_map_odom);
    } catch (const tf::TransformException &ex) {
      std::cerr << "Cannot form map transform for darpa frame alignment: "
        << ex.what() << std::endl;
      return;
    }
    tf::Point pt_out = (transf*transf_map_odom).inverse() * detection;
    fiducials_observed[report_string] = pt_out;
    frame_correspondences_seen[report_string] =
      std::make_tuple(pt_out, fpgf_itr->second);
    if (frame_correspondences_seen.size() >= 3) {
      Eigen::MatrixXf
        src(3, frame_correspondences_seen.size()),
        dest(3, frame_correspondences_seen.size());
      int i = 0;
      for (std::map<std::string, std::tuple<tf::Point, tf::Point>>::const_iterator
           corresp_itr = frame_correspondences_seen.begin();
           corresp_itr != frame_correspondences_seen.end();
           corresp_itr++, i++) {
        tf::Point pt_src = std::get<0>(corresp_itr->second);
        tf::Point pt_dest = std::get<1>(corresp_itr->second);
        src(0, i) = pt_src.x();
        src(1, i) = pt_src.y();
        src(2, i) = pt_src.z();
        dest(0, i) = pt_dest.x();
        dest(1, i) = pt_dest.y();
        dest(2, i) = pt_dest.z();
      }
      Eigen::MatrixXf output_transform = Eigen::umeyama(src, dest, false);
      geometry_msgs::TransformStamped global_frame_transform;
      global_frame_transform.header.frame_id = map_frame;
      global_frame_transform.child_frame_id = "darpa";
      EigenToTransformMsg(output_transform.inverse(), global_frame_transform.transform);
      global_frame_transform.header.stamp = ros::Time::now();
      if (initialize_flat) {
        global_frame_transform.transform.rotation =
          tf::createQuaternionMsgFromYaw(tf::getYaw(global_frame_transform.transform.rotation));
      }
      tfB.sendTransform(global_frame_transform);
    }
  }
}

void FrameAlignmentNode::
onImages(const sensor_msgs::ImageConstPtr &msg,
         const sensor_msgs::ImageConstPtr &depth_msg) {

  if (!camera_model.initialized()) return;
  try {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat imagebw;
    cv::cvtColor(image, imagebw, CV_BGR2GRAY);
    image_u8_t img_header = {.width = imagebw.cols,
      .height = imagebw.rows,
      .stride = imagebw.cols,
      .buf = imagebw.data};
    zarray_t *detections = apriltag_detector_detect(td, &img_header);
    if (zarray_size(detections) > 0) {
      // Only convert the depth image if we have some promising detections
      cv::Mat depth_image = cv_bridge::toCvCopy(depth_msg, "32FC1")->image;
      for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        if (det->decision_margin < 45.0)
          continue;
        std::map<int, float>::iterator it = best_margins_.find(det->id);
        if (it != best_margins_.end()) {
          if(det->decision_margin < it->second)
            continue;
          else
            it->second = det->decision_margin;
        }
        else {
          best_margins_.emplace(det->id, det->decision_margin);
        }

        printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
               i, det->family->nbits, det->family->h, det->id, det->hamming,
               det->decision_margin);
        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = 0.2286; // From tunnel circuit rules document
        info.fx = camera_model.fx();
        info.fy = camera_model.fy();
        info.cx = camera_model.cx();
        info.cy = camera_model.cy();
        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);
        matd_print(pose.t, "%f");
        tf::Vector3 april_pt_vec(matd_get(pose.t, 0, 0), matd_get(pose.t, 1, 0),
                                 matd_get(pose.t, 2, 0));
        tf::Vector3 local_pt_vec;
        tf::Vector3 report_pt_vec(nanf(""), nanf(""), nanf(""));

        if (ImageTo3DPoint(det->c[0], det->c[1], depth_image, msg->header.frame_id, local_pt_vec,
                           report_pt_vec)) {
          tf::Vector3 diff = local_pt_vec - april_pt_vec;

          std::stringstream ss;
          std::stringstream fiducial_name;
          fiducial_name << "fiducial_" << det->id;
          ROS_INFO_STREAM("Detected fiducial " << fiducial_name.str());

          // Call user entered fiducial reports of quality 100
          UpdateTransform(fiducial_name.str(),
                          tf::Stamped<tf::Point>(report_pt_vec, msg->header.stamp,
                                                 report_frame));
        }
      }
      apriltag_detections_destroy(detections);
    }
    PublishMarkers();
    // If val is a spacebar, then pause/play the bag file?
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR_STREAM("Could not convert from bgr8 to "
                     << msg->encoding.c_str());
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "frame_alignment_node");
  FrameAlignmentNode alignment_node;

  ros::spin();
  return 0;
}

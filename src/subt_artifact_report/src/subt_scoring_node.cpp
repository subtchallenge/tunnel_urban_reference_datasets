/*
 *
 * John G. Rogers III
 *
 * 
 */

#include <vector>
using namespace std;
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <Eigen/Geometry>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>


std::vector<std::pair<std::string, tf::Point> >gt_artifacts;
std::list<std::tuple<double, std::string, tf::Stamped<tf::Point> > > reports;
std::list<std::tuple<double, std::string, tf::Stamped<tf::Point>, double> > finished_reports;
std::map<string, tf::Point> fiducial_point_darpa_frame;
tf::TransformListener* tfL;
tf2_ros::StaticTransformBroadcaster* tfB;
std::string map_frame;

std::vector<tf::Point> fiducials_observed;
std::vector<tf::Point> good_artifacts;
std::vector<tf::Point> bad_artifacts;

ros::Publisher* marker_pub;
ofstream rmse_file;

void PublishMarkers() {
  static visualization_msgs::MarkerArray delete_markers;
  //Draw all ground truth points in blue
  // Draw all failed artifact in red
  // Draw all good artifacts in green
  visualization_msgs::MarkerArray send_markers;

  marker_pub->publish(delete_markers);
  delete_markers.markers.clear();
  visualization_msgs::Marker marker;
  marker.header.frame_id = "darpa";
  marker.header.stamp = ros::Time::now();
  marker.ns = "Subt_markers";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.lifetime = ros::Duration(0);
  marker.frame_locked = true;
  marker.color.r = 0; marker.color.g = 0.0; marker.color.b = 1.0; marker.color.a = 1.0;
  marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3;
  marker.pose.orientation.w = 0.0; 
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 1.0;
  for (size_t i = 0; i < gt_artifacts.size(); i++) {
    marker.pose.position.x = gt_artifacts[i].second.x();
    marker.pose.position.y = gt_artifacts[i].second.y();
    marker.pose.position.z = gt_artifacts[i].second.z();
    send_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.id ++;
  }
  marker.color.r = 0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1.0;
  for (size_t i = 0; i < good_artifacts.size(); i++) {
    marker.pose.position.x = good_artifacts[i].x();
    marker.pose.position.y = good_artifacts[i].y();
    marker.pose.position.z = good_artifacts[i].z();
    send_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.id ++;
  }
  marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
  for (size_t i = 0; i < bad_artifacts.size(); i++) {
    marker.pose.position.x = bad_artifacts[i].x();
    marker.pose.position.y = bad_artifacts[i].y();
    marker.pose.position.z = bad_artifacts[i].z();
    send_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.id ++;
  }
  marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1.0;
  marker.header.frame_id = "map";
  for (size_t i = 0; i < fiducials_observed.size(); i++) {
    marker.pose.position.x = fiducials_observed[i].x();
    marker.pose.position.y = fiducials_observed[i].y();
    marker.pose.position.z = fiducials_observed[i].z();
    send_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.id ++;
  }
  marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 1.0; marker.color.a = 0.5;
  marker.scale.x = 1.5; marker.scale.y = 1.5; marker.scale.z = 1.5;
  marker.header.frame_id = "darpa";
  for (std::map<string, tf::Point>::const_iterator itr = fiducial_point_darpa_frame.begin();
        itr != fiducial_point_darpa_frame.end(); itr++) {
    marker.pose.position.x = itr->second.x();
    marker.pose.position.y = itr->second.y();
    marker.pose.position.z = itr->second.z();
    send_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.id ++;
  }



  marker_pub->publish(send_markers);
}

bool already_have_it(const std::vector<std::tuple<std::string, tf::Point, tf::Point> >& corresp, 
                     const std::string& frame) {
  for (std::vector<std::tuple<std::string, tf::Point, tf::Point> >::const_iterator itr = corresp.begin();
       itr != corresp.end(); itr++) {
    if (std::get<0>(*itr) == frame) return true;
  }
  return false;
}

void printMat(const Eigen::MatrixXf& mat) {
  for (size_t i = 0; i < mat.rows(); i++){
    for (size_t j = 0; j < mat.cols(); j++) {
      std::cout << mat(i,j) << " ";
    }
    std::cout << std::endl;
  }
}

void EigenToTransformMsg(const Eigen::MatrixXf& eig, geometry_msgs::Transform& transf) {
  transf.translation.x = eig(0, 3);
  transf.translation.y = eig(1, 3);
  transf.translation.z = eig(2, 3);
  tf::Matrix3x3 matr(eig(0,0), eig(0,1), eig(0,2),
                     eig(1,0), eig(1,1), eig(1,2),
                     eig(2,0), eig(2,1), eig(2,2));

  tf::Quaternion quat;
  matr.getRotation(quat);
  tf::quaternionTFToMsg(quat, transf.rotation); 
}

void CvMatToTransformMsg(const cv::Mat& eig, geometry_msgs::Transform& transf) {
  transf.translation.x = eig.at<double>(0, 3);
  transf.translation.y = eig.at<double>(1, 3);
  transf.translation.z = eig.at<double>(2, 3);
  tf::Matrix3x3 matr(eig.at<double>(0,0), eig.at<double>(1,0), eig.at<double>(2,0),
                     eig.at<double>(0,1), eig.at<double>(1,1), eig.at<double>(2,1),
                     eig.at<double>(0,2), eig.at<double>(1,2), eig.at<double>(2,2));

  tf::Quaternion quat;
  matr.getRotation(quat);
  tf::quaternionTFToMsg(quat, transf.rotation); 
}


double getRMSE(const std::set<double>& residuals) {
  double MSE = 0.0;

  for (std::set<double>::const_iterator itr = residuals.begin(); itr != residuals.end(); itr++) {
    MSE += *itr * *itr;
  }
  MSE = MSE / static_cast<double>(residuals.size());
  return sqrt(MSE);
}

double HandleReport(const std::tuple<double, std::string, tf::Stamped<tf::Point> >& report, double now_sec) {
  static std::set<double> residuals;
  static double min_error = std::numeric_limits<double>::infinity();
  static double max_error = 0.0;
  static size_t points = 0;
  static double now_sec_start = now_sec;
  static std::vector<std::tuple<std::string, tf::Point, tf::Point> > frame_correspondences_seen;
  double residual = 0.0;
  ROS_INFO_STREAM("Handling report at time " << std::fixed << now_sec); 
  //See if it is a fiducial marker. If so, refine the estimate of the DARPA frame to map transform
  std::string report_string = std::get<1>(report);
  std::map<string, tf::Point>::iterator itr = fiducial_point_darpa_frame.find(report_string);
  if (itr != fiducial_point_darpa_frame.end()) {
    if (already_have_it(frame_correspondences_seen, report_string)) {
      ROS_ERROR_STREAM("Received a new version of frame " << report_string << ". I can only handle one of each");
      return 0.0;
    }
    tf::StampedTransform transf;
    bool got_transform = true;
    try {
      tfL->waitForTransform(std::get<2>(report).frame_id_, map_frame, std::get<2>(report).stamp_, ros::Duration(3.0));
      tfL->lookupTransform(std::get<2>(report).frame_id_, map_frame, std::get<2>(report).stamp_, transf);

    } catch (const tf::TransformException& ex) {
      std::cerr << "Cannot form map transform for DARPA frame alignment. Initializing to most recent transform: " << ex.what() << std::endl;
      got_transform = false;
    }
    try {
      tfL->waitForTransform(std::get<2>(report).frame_id_, map_frame, ros::Time(0), ros::Duration(5.0));
      tfL->lookupTransform(std::get<2>(report).frame_id_, map_frame, ros::Time(0), transf);

    } catch (const tf::TransformException& ex) {
      std::cerr << "Cant even get most recent transform after waiting a full 8 seconds" << ex.what() << std::endl;
      return 0.0;
    }
    tf::Point pt_out = transf.inverse() * std::get<2>(report);
    fiducials_observed.push_back(pt_out);
    frame_correspondences_seen.push_back(std::make_tuple(report_string, pt_out, itr->second));
    if (frame_correspondences_seen.size() >= 3) {
      Eigen::MatrixXf src(3, frame_correspondences_seen.size()), dest(3, frame_correspondences_seen.size());
      std::vector<cv::Point3d> cvsrc, cvdest;
      int i = 0;
      for (std::vector<std::tuple<std::string, tf::Point, tf::Point> >::const_iterator corresp_itr = frame_correspondences_seen.begin();
           corresp_itr != frame_correspondences_seen.end(); corresp_itr++, i++) {
        tf::Point pt_src = std::get<1>(*corresp_itr);
        tf::Point pt_dest = std::get<2>(*corresp_itr);
        src(0, i) = pt_src.x(); src(1, i) = pt_src.y(); src(2, i) = pt_src.z();
        dest(0, i) = pt_dest.x(); dest(1, i) = pt_dest.y(); dest(2, i) = pt_dest.z();
        cvsrc.push_back(cv::Point3d(pt_src.x(), pt_src.y(), pt_src.z()));
        cvdest.push_back(cv::Point3d(pt_dest.x(), pt_dest.y(), pt_dest.z()));
      }
      Eigen::MatrixXf output_transform = Eigen::umeyama(src, dest, false);
      cv::Mat inliers;
      cv::Mat cvoutput_transform;
//      cv::estimateAffine3D(cvsrc, cvdest, cvoutput_transform, inliers);
      geometry_msgs::TransformStamped darpa_frame_transform;
      darpa_frame_transform.header.frame_id = "darpa";
      darpa_frame_transform.child_frame_id = map_frame;
      darpa_frame_transform.header.stamp = ros::Time::now();
//      CvMatToTransformMsg(cvoutput_transform, darpa_frame_transform.transform);
      EigenToTransformMsg(output_transform, darpa_frame_transform.transform);
      tfB->sendTransform(darpa_frame_transform);
      //cv::estimateAffine3D(src, dest, affine_transform, inliers);
      //printMat(output_transform);
      std::cout << "Map frame points" <<std::endl;
      printMat(src);
      std::cout << "DARPA frame points" <<std::endl;
      printMat(dest);
      printMat(output_transform);
      Eigen::MatrixXf src_homog(4, frame_correspondences_seen.size());
      for (int i = 0; i < frame_correspondences_seen.size(); i++) {
        for (int j = 0; j < 3; j++) {
          src_homog(j,i) = src(j,i);
        }
      }
      for (int i = 0; i < frame_correspondences_seen.size(); i++) {
        src_homog(3,i) = 1.0;
      }
      printMat(src_homog);
      std::cout << "Map frame points in darpa frame" << std::endl;
      printMat(output_transform * src_homog);

    }

  } else {
    //Artifact report
    //
    tf::StampedTransform transf;
    try {
      tfL->waitForTransform(std::get<2>(report).frame_id_, "darpa", std::get<2>(report).stamp_, ros::Duration(0.5));
      tfL->lookupTransform(std::get<2>(report).frame_id_, "darpa", std::get<2>(report).stamp_, transf);

    } catch(const tf::TransformException& ex) {
      std::cerr << "Unable to compute darpa to local frame transform for artifact. Make sure you are providing transform chain between " 
        << map_frame << " and " << std::get<2>(report).frame_id_ << " : " << ex.what() << std::endl;
      return 0.0;
    } 
    tf::Point pt_darpa = transf.inverse() * std::get<2>(report);
    std::cout <<"Currently seeing a " << std::get<1>(report)<< " at location " << pt_darpa.x() << ", " << pt_darpa.y() << ", " << pt_darpa.z() << std::endl;
    // Go through all of the ground truth with the same label and find the closest one
    // If it is close enough by SubT standards (< 5m residual) count a point and include it for residual computation
    std::vector<std::pair<std::string, tf::Point> >::const_iterator closest_itr = gt_artifacts.end();
    double best_dist = std::numeric_limits<double>::infinity();
    for (std::vector<std::pair<std::string, tf::Point> >::const_iterator itr = gt_artifacts.begin(); itr != gt_artifacts.end();itr++) {

      if (itr->first == std::get<1>(report)) {
        double dist = pt_darpa.distance(itr->second);
        std::cout <<"Comparing to same type of artifact at " << itr->second.x() << ", " << itr->second.y() << ", " << itr->second.z() << std::endl;
        std::cout << "Dist " << dist << std::endl;
        if (dist < best_dist) {
          best_dist = dist;
          closest_itr = itr;
        }
      }
    }
    if (closest_itr != gt_artifacts.end()) {
      if (best_dist <= 5.0) {
        std::cout <<" A point scored with residual error of " << best_dist << std::endl;
        residual = best_dist;
        points ++;
        good_artifacts.push_back(pt_darpa);
      } else {
        residual = best_dist;
        bad_artifacts.push_back(pt_darpa);
      }
      residuals.insert(residual);
      double RMSE = getRMSE(residuals);
      if (residual < min_error) min_error = residual;
      if (residual > max_error) max_error = residual;
      rmse_file << now_sec - now_sec_start << " " << points << " " << RMSE << " " << min_error << " " << max_error << std::endl;
      rmse_file.flush();
    }
  }
  return residual;
}

bool sorttuple(const std::tuple<double, std::string, tf::Stamped<tf::Point> >& lhs,
               const std::tuple<double, std::string, tf::Stamped<tf::Point> >& rhs) {
  return std::get<0>(lhs) < std::get<0>(rhs);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "subt_scoring_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std::string gt_filename;
  std::string artifacts_filename;
  std::string output_filename;
  std::string fiducial_file;
  std::string rmse_filename;
  bool use_optical_frame_report;
  private_nh.param("gt_filename", gt_filename, std::string("SCREWED"));
  private_nh.param("artifacts_filename", artifacts_filename, std::string("/var/tmp/tunnel_ckt_coding_ws"));
  private_nh.param("output_filename", output_filename, std::string("/var/tmp/score_result"));
  private_nh.param("use_optical_frame_report", use_optical_frame_report, false);
  private_nh.param("fiducial_file", fiducial_file, std::string("SCREWED"));
  private_nh.param("map_frame", map_frame, std::string("map"));
  private_nh.param("rmse_filename", rmse_filename, std::string("/var/tmp/rmse"));

  rmse_file.open(rmse_filename.c_str(), std::ios::out);
  //Hideous, lazy
  tf::TransformListener tfL_;
  tfL = &tfL_;
  tf2_ros::StaticTransformBroadcaster tfB_;
  tfB = &tfB_;
  ros::Publisher marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("SubT_markers", 1);
  marker_pub = &marker_pub_;

  std::ifstream infile(gt_filename.c_str(), std::ios::in);
  if (!infile.is_open()) {
    ROS_ERROR_STREAM("Cannot judge mapping run without ground truth state. Please provide a gt_filename");
    exit(1);
  }
  std::string line;
  while (std::getline(infile, line)) {
    std::stringstream ss (line);
    std::string label;
    double x, y, z;
    ss >> label >> x >> y >> z;
    gt_artifacts.push_back(std::make_pair(label, tf::Point(x,y,z)));
  }
  infile.close();
  infile.open(fiducial_file.c_str(), std::ios::in);
  while (std::getline(infile, line)) {
    std::stringstream ss (line);
    std::string label;
    double x, y, z;
    ss >> label >> x >> y >> z;
    fiducial_point_darpa_frame.insert(std::make_pair(label, tf::Point(x,y,z)));
  }
  infile.close();

  infile.open(artifacts_filename.c_str(), std::ios::in);
  if (!infile.is_open()){
    ROS_ERROR_STREAM("Cannot judge mapping run without artifacts record. Please provide a artifacts_filename");
    exit(1);
  }
  // fiducial_L 1566222460.357846 406 281 chinook/multisense/left_camera_optical_frame -1.187567 0.088331 5.696559 chinook/base_link 5.911559 1.222567 0.459669
  while (std::getline(infile, line)) {
    std::stringstream ss (line);
    std::string label;
    double time;
    double u,v;
    std::string frame;

    double x, y, z;
    std::string image_frame;
    double xx, yy, zz;
    ss >> label >> time >> u >> v >> image_frame >> xx >> yy >> zz >> frame >> x >> y >> z;
    tf::Stamped<tf::Point> pt(tf::Point(x,y,z), ros::Time(time), frame);
    tf::Stamped<tf::Point> impt(tf::Point(xx,yy,zz), ros::Time(time), image_frame);
    if (use_optical_frame_report) {
      reports.push_back(std::make_tuple(time, label, impt));
    } else {
      reports.push_back(std::make_tuple(time, label, pt));
    }
  }
  infile.close();
  reports.sort(sorttuple);

  ros::Rate r(10);
  while (reports.size() != 0 && ros::ok()) {
    double now = ros::Time::now().toSec();
    while (reports.size() != 0 && now > std::get<0>(reports.front())) {
      //Handle a object report
      double residual = HandleReport(reports.front(), now);
      PublishMarkers();
      finished_reports.push_back(std::make_tuple(std::get<0>(reports.front()), 
                                                 std::get<1>(reports.front()), 
                                                 std::get<2>(reports.front()), 
                                                 residual));
      reports.pop_front();
    }
    ros::spinOnce();
    r.sleep();
  }
  rmse_file.close();
  return 0;
}

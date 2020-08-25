/*
 *
 * John G. Rogers III
 *
 *
 */

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <subt_artifact_report/coding.h>
#include <subt_artifact_report/subt_scoring_node.h>
#include <subt_artifact_report/relative_metric.h>

#include <Eigen/Geometry>
#include <vector>
#include <algorithm>
#include <fstream>
#include <opencv2/opencv.hpp>

void SubTScoringNode::PublishMarkers() {
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
  marker.ns = "Subt_markers";
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
  marker.color.r = 0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  for (size_t i = 0; i < good_artifacts.size(); i++) {
    marker.pose.position.x = std::get<1>(good_artifacts[i]).x();
    marker.pose.position.y = std::get<1>(good_artifacts[i]).y();
    marker.pose.position.z = std::get<1>(good_artifacts[i]).z();
    send_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.id++;
    visualization_msgs::Marker text_marker = marker;
    text_marker.pose.position.z += 1.0;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.text = std::get<0>(good_artifacts[i]);
    send_markers.markers.push_back(text_marker);
    marker.id++;
    visualization_msgs::Marker line_marker = marker;
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.scale.x = 0.1;
    line_marker.pose.position.x = 0;
    line_marker.pose.position.y = 0;
    line_marker.pose.position.z = 0;
    line_marker.points.push_back(marker.pose.position);
    geometry_msgs::Point distal_pt;
    distal_pt.x = gt_artifacts[std::get<2>(good_artifacts[i])].second.x();
    distal_pt.y = gt_artifacts[std::get<2>(good_artifacts[i])].second.y();
    distal_pt.z = gt_artifacts[std::get<2>(good_artifacts[i])].second.z();
    line_marker.points.push_back(distal_pt);
    send_markers.markers.push_back(line_marker);
    line_marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(line_marker);
    text_marker.action = visualization_msgs::Marker::DELETE;

    delete_markers.markers.push_back(text_marker);
    marker.id++;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.text = std::get<3>(good_artifacts[i]);
    text_marker.pose.position.x = (distal_pt.x + marker.pose.position.x)/2.0;
    text_marker.pose.position.y = (distal_pt.y + marker.pose.position.y)/2.0;
    text_marker.pose.position.z = (distal_pt.z + marker.pose.position.z)/2.0 - 0.5;
    text_marker.id = marker.id++;
    send_markers.markers.push_back(text_marker);
    text_marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(text_marker);
  }
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  for (size_t i = 0; i < bad_artifacts.size(); i++) {
    marker.pose.position.x = std::get<1>(bad_artifacts[i]).x();
    marker.pose.position.y = std::get<1>(bad_artifacts[i]).y();
    marker.pose.position.z = std::get<1>(bad_artifacts[i]).z();
    send_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.id++;
    visualization_msgs::Marker text_marker = marker;
    text_marker.pose.position.z += 1.0;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.text = std::get<0>(bad_artifacts[i]);
    send_markers.markers.push_back(text_marker);
    marker.id++;
    visualization_msgs::Marker line_marker = marker;
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.scale.x = 0.1;
    line_marker.pose.position.x = 0;
    line_marker.pose.position.y = 0;
    line_marker.pose.position.z = 0;
    line_marker.points.push_back(marker.pose.position);
    geometry_msgs::Point distal_pt;
    distal_pt.x = gt_artifacts[std::get<2>(bad_artifacts[i])].second.x();
    distal_pt.y = gt_artifacts[std::get<2>(bad_artifacts[i])].second.y();
    distal_pt.z = gt_artifacts[std::get<2>(bad_artifacts[i])].second.z();
    line_marker.points.push_back(distal_pt);
    send_markers.markers.push_back(line_marker);
    line_marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(line_marker);
    text_marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(text_marker);
    marker.id++;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.text = std::get<3>(bad_artifacts[i]);
    text_marker.pose.position.x = (distal_pt.x + marker.pose.position.x)/2.0;
    text_marker.pose.position.y = (distal_pt.y + marker.pose.position.y)/2.0;
    text_marker.pose.position.z = (distal_pt.z + marker.pose.position.z)/2.0 - 0.5;
    text_marker.id = marker.id++;
    send_markers.markers.push_back(text_marker);
    text_marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(text_marker);
  }
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  for (size_t i = 0; i < relative_frame_aligned_artifacts.size(); i++) {
    marker.pose.position.x = std::get<0>(relative_frame_aligned_artifacts[i]).x();
    marker.pose.position.y = std::get<0>(relative_frame_aligned_artifacts[i]).y();
    marker.pose.position.z = std::get<0>(relative_frame_aligned_artifacts[i]).z();
    send_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    marker.id++;
    visualization_msgs::Marker line_marker = marker;
    line_marker.type = visualization_msgs::Marker::LINE_LIST;
    line_marker.scale.x = 0.1;
    line_marker.pose.position.x = 0;
    line_marker.pose.position.y = 0;
    line_marker.pose.position.z = 0;
    line_marker.points.push_back(marker.pose.position);
    geometry_msgs::Point distal_pt;
    distal_pt.x = gt_artifacts[std::get<1>(relative_frame_aligned_artifacts[i])].second.x();
    distal_pt.y = gt_artifacts[std::get<1>(relative_frame_aligned_artifacts[i])].second.y();
    distal_pt.z = gt_artifacts[std::get<1>(relative_frame_aligned_artifacts[i])].second.z();
    line_marker.points.push_back(distal_pt);
    send_markers.markers.push_back(line_marker);
    line_marker.action = visualization_msgs::Marker::DELETE;
    delete_markers.markers.push_back(line_marker);
    marker.id++;
  }
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.header.frame_id = "map";
  for (size_t i = 0; i < fiducials_observed.size(); i++) {
    marker.pose.position.x = fiducials_observed[i].x();
    marker.pose.position.y = fiducials_observed[i].y();
    marker.pose.position.z = fiducials_observed[i].z();
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
  for (std::map<string, tf::Point>::const_iterator itr =
	 fiducial_point_darpa_frame.begin();
       itr != fiducial_point_darpa_frame.end(); itr++) {
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

bool already_have_it(
                     const std::vector<std::tuple<std::string, tf::Point, tf::Point>> &corresp,
                     const std::string &frame) {
  for (std::vector<
	 std::tuple<std::string, tf::Point, tf::Point>>::const_iterator itr =
	 corresp.begin();
       itr != corresp.end(); itr++) {
    if (std::get<0>(*itr) == frame)
      return true;
  }
  return false;
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

void CvMatToTransformMsg(const cv::Mat &eig, geometry_msgs::Transform &transf) {
  transf.translation.x = eig.at<double>(0, 3);
  transf.translation.y = eig.at<double>(1, 3);
  transf.translation.z = eig.at<double>(2, 3);
  tf::Matrix3x3 matr(
                     eig.at<double>(0, 0), eig.at<double>(0, 1), eig.at<double>(0, 2),
                     eig.at<double>(1, 0), eig.at<double>(1, 1), eig.at<double>(1, 2),
                     eig.at<double>(2, 0), eig.at<double>(2, 1), eig.at<double>(2, 2));

  tf::Quaternion quat;
  matr.getRotation(quat);
  tf::quaternionTFToMsg(quat, transf.rotation);
}

double getRMSE(const std::set<double> &residuals) {
  double MSE = 0.0;

  for (std::set<double>::const_iterator itr = residuals.begin();
       itr != residuals.end(); itr++) {
    MSE += *itr * *itr;
  }
  MSE = MSE / static_cast<double>(residuals.size());
  return sqrt(MSE);
}

double SubTScoringNode::
HandleReport(const std::tuple<double, std::string, tf::Stamped<tf::Point>> &report,
             double now_sec) {
  static std::vector<std::pair<std::string, tf::Stamped<tf::Point>>> relative_reports;
  static std::set<double> residuals;
  static std::set<double> residualsxy;
  static double min_error = std::numeric_limits<double>::infinity();
  static double max_error = 0.0;
  static double min_errorxy = std::numeric_limits<double>::infinity();
  static double max_errorxy = 0.0;
  static size_t points = 0;
  static size_t pointsxy = 0;
  static double now_sec_start = now_sec;
  static bool first = true;
  static std::vector<std::tuple<std::string, tf::Point, tf::Point>>
    frame_correspondences_seen;
  double residual = 0.0;
  double residualxy = 0.0;
  double RMSE = -1.0;
  double RMSExy = -1.0;
  double relative_RMSE = -1.0;
  ROS_INFO_STREAM("Handling report at time " << std::fixed << now_sec);
  // See if it is a fiducial marker. If so, refine the estimate of the DARPA
  // frame to map transform
  std::string report_string = std::get<1>(report);
  std::map<string, tf::Point>::iterator itr =
    fiducial_point_darpa_frame.find(report_string);
  if (itr != fiducial_point_darpa_frame.end()) {
    if (already_have_it(frame_correspondences_seen, report_string)) {
      ROS_ERROR_STREAM("Received a new version of frame "
                       << report_string << ". I can only handle one of each");
      return 0.0;
    }
    tf::StampedTransform transf;
    bool got_transform = true;
    try {
      tfL.waitForTransform(std::get<2>(report).frame_id_, map_frame,
                           std::get<2>(report).stamp_, ros::Duration(3.0));
      tfL.lookupTransform(std::get<2>(report).frame_id_, map_frame,
                          std::get<2>(report).stamp_, transf);
    } catch (const tf::TransformException &ex) {
      std::cerr << "Cannot form map transform for DARPA frame alignment. "
        "Initializing to most recent transform: "
		<< ex.what() << std::endl;
      got_transform = false;
    }
    try {
      tfL.waitForTransform(std::get<2>(report).frame_id_, map_frame,
                           ros::Time(0), ros::Duration(5.0));
      tfL.lookupTransform(std::get<2>(report).frame_id_, map_frame,
                          ros::Time(0), transf);
    } catch (const tf::TransformException &ex) {
      std::cerr << "Cant even get most recent transform after waiting a full 8 "
        "seconds"
		<< ex.what() << std::endl;
      return 0.0;
    }
    tf::Point pt_out = transf.inverse() * std::get<2>(report);
    fiducials_observed.push_back(pt_out);
    frame_correspondences_seen.push_back(
                                         std::make_tuple(report_string, pt_out, itr->second));
    if (frame_correspondences_seen.size() >= 3) {
      Eigen::MatrixXf src(3, frame_correspondences_seen.size()),
        dest(3, frame_correspondences_seen.size());
      std::vector<cv::Point3d> cvsrc, cvdest;
      int i = 0;
      for (std::vector<
	     std::tuple<std::string, tf::Point, tf::Point>>::const_iterator
	     corresp_itr = frame_correspondences_seen.begin();
           corresp_itr != frame_correspondences_seen.end();
           corresp_itr++, i++) {
        tf::Point pt_src = std::get<1>(*corresp_itr);
        tf::Point pt_dest = std::get<2>(*corresp_itr);
        src(0, i) = pt_src.x();
        src(1, i) = pt_src.y();
        src(2, i) = pt_src.z();
        dest(0, i) = pt_dest.x();
        dest(1, i) = pt_dest.y();
        dest(2, i) = pt_dest.z();
        cvsrc.push_back(cv::Point3d(pt_src.x(), pt_src.y(), pt_src.z()));
        cvdest.push_back(cv::Point3d(pt_dest.x(), pt_dest.y(), pt_dest.z()));
      }
      Eigen::MatrixXf output_transform = Eigen::umeyama(src, dest, false);
      cv::Mat inliers;
      cv::Mat cvoutput_transform;
      //      cv::estimateAffine3D(cvsrc, cvdest, cvoutput_transform, inliers);
      geometry_msgs::TransformStamped darpa_frame_transform;
      if (true) {  // falsereverse_transform) {
        std::cout << "OK remember to clean up this code here" << std::endl;
        darpa_frame_transform.header.frame_id = map_frame;
        darpa_frame_transform.child_frame_id = "darpa";
        EigenToTransformMsg(output_transform.inverse(), darpa_frame_transform.transform);
      } else {
        darpa_frame_transform.header.frame_id = "darpa";
        darpa_frame_transform.child_frame_id = map_frame;
        EigenToTransformMsg(output_transform, darpa_frame_transform.transform);
      }
      if (initialize_flat) {
        darpa_frame_transform.transform.rotation =
          tf::createQuaternionMsgFromYaw(tf::getYaw(darpa_frame_transform.transform.rotation));
      }
      darpa_frame_transform.header.stamp = ros::Time::now();
      //      CvMatToTransformMsg(cvoutput_transform,
      //      darpa_frame_transform.transform);
      tfB.sendTransform(darpa_frame_transform);
      // cv::estimateAffine3D(src, dest, affine_transform, inliers);
      // printMat(output_transform);
      std::cout << "Map frame points" << std::endl;
      printMat(src);
      std::cout << "DARPA frame points" << std::endl;
      printMat(dest);
      printMat(output_transform);
      Eigen::MatrixXf src_homog(4, frame_correspondences_seen.size());
      for (int i = 0; i < frame_correspondences_seen.size(); i++) {
        for (int j = 0; j < 3; j++) {
          src_homog(j, i) = src(j, i);
        }
      }
      for (int i = 0; i < frame_correspondences_seen.size(); i++) {
        src_homog(3, i) = 1.0;
      }
      printMat(src_homog);
      std::cout << "Map frame points in darpa frame" << std::endl;
      printMat(output_transform * src_homog);
    }

  } else {
    // Artifact report
    //
    tf::StampedTransform local_transf;
    bool local_frame_good = true;
    try {
      tfL.waitForTransform(std::get<2>(report).frame_id_, map_frame,
			   std::get<2>(report).stamp_, ros::Duration(0.5));
      tfL.lookupTransform(std::get<2>(report).frame_id_, map_frame,
			  std::get<2>(report).stamp_, local_transf);
    } catch (const tf::TransformException &ex) {
      std::cerr << "Unable to compute " << map_frame <<
	" to sensor frame transform for "
	"artifact. Make sure you are providing transform chain "
	"between "
		<< map_frame << " and " << std::get<2>(report).frame_id_
		<< " : " << ex.what() << std::endl;
      local_frame_good = false;
    }
    if (local_frame_good) {
      tf::Stamped<tf::Point> pt_local(local_transf.inverse() * std::get<2>(report),
				      std::get<2>(report).stamp_, map_frame);
      relative_reports.push_back(std::make_pair(std::get<1>(report), pt_local));
      std::cout << "Currently seeing a " << std::get<1>(report) << " at location "
        << pt_local.x() << ", " << pt_local.y() << ", " << pt_local.z()
        << " in frame " << map_frame
        << std::endl;
      std::vector<int> best_relative_da;
      Eigen::MatrixXf relative_transform;
      relative_RMSE = GetBestRelativeDA(gt_artifacts, relative_reports, &best_relative_da, &relative_transform);
      std::cout << "Relative RMSE of " << relative_RMSE << " was achieved with assignment: " << std::endl;
      for (int i = 0; i < best_relative_da.size(); i++) {
        std::cout << i << " -> " << best_relative_da[i] << std::endl;
      }
      relative_frame_aligned_artifacts.clear();
      for (unsigned int i = 0; i < best_relative_da.size(); i++) {
        Eigen::MatrixXf tmp(4, 1);
        tmp(0, 0) = relative_reports[i].second.x();
        tmp(1, 0) = relative_reports[i].second.y();
        tmp(2, 0) = relative_reports[i].second.z();
        tmp(3, 0) = 1.0;
        Eigen::MatrixXf tmp2 = relative_transform * tmp;
        PrintMat(tmp2);
        relative_frame_aligned_artifacts.push_back(std::make_tuple(tf::Point(tmp2(0, 0),
                                                                             tmp2(1, 0),
                                                                             tmp2(2, 0)),
                                                                   best_relative_da[i]));
      }
    }
    tf::StampedTransform global_transf;
    bool global_frame_good = true;
    try {
      tfL.waitForTransform(std::get<2>(report).frame_id_, "darpa",
                           std::get<2>(report).stamp_, ros::Duration(0.5));
      tfL.lookupTransform(std::get<2>(report).frame_id_, "darpa",
                          std::get<2>(report).stamp_, global_transf);
    } catch (const tf::TransformException &ex) {
      std::cerr << "Unable to compute darpa to local frame transform for "
        "artifact. Make sure you are providing transform chain "
        "between "
        << map_frame << " and " << std::get<2>(report).frame_id_
        << " : " << ex.what() << std::endl;
      std::cerr << "Now resuming with relative metric" << std::endl;
      global_frame_good = false;
    }
    if (global_frame_good) {
      tf::Point pt_darpa = global_transf.inverse() * std::get<2>(report);
      std::cout << "Currently seeing a " << std::get<1>(report) << " at location "
        << pt_darpa.x() << ", " << pt_darpa.y() << ", " << pt_darpa.z()
        << std::endl;
      // Go through all of the ground truth with the same label and find the
      // closest one
      // If it is close enough by SubT standards (< 5m residual) count a point and
      // include it for residual computation
      std::vector<std::pair<std::string, tf::Point>>::const_iterator closest_itr =
        gt_artifacts.end();
      double best_dist = std::numeric_limits<double>::infinity();
      double best_distxy = std::numeric_limits<double>::infinity();
      int ind = 0;
      int best_ind = -1;
      for (std::vector<std::pair<std::string, tf::Point>>::const_iterator itr =
           gt_artifacts.begin();
           itr != gt_artifacts.end(); itr++, ind++) {

        if (itr->first == std::get<1>(report)) {
          double dist = pt_darpa.distance(itr->second);
          tf::Point pt_darpaxy(pt_darpa.x(), pt_darpa.y(), 0.0);
          tf::Point gt_pt_xy(itr->second.x(), itr->second.y(), 0.0);
          double distxy = pt_darpaxy.distance(gt_pt_xy);

          std::cout << "Comparing to same type of artifact at " << itr->second.x()
            << ", " << itr->second.y() << ", " << itr->second.z()
            << std::endl;
          std::cout << "Dist " << dist << std::endl;
          if (dist < best_dist) {
            best_dist = dist;
            closest_itr = itr;
            best_ind = ind;
          }
          if (distxy < best_distxy) {
            best_distxy = distxy;
          }
        }
      }
      if (closest_itr != gt_artifacts.end()) {
        if (best_dist <= 5.0) {
          std::cout << " A point scored with residual error of " << best_dist
            << std::endl;
          points++;
          std::stringstream ss;
          ss << best_dist << "m";
          good_artifacts.push_back(std::make_tuple(std::get<1>(report), pt_darpa, best_ind, ss.str()));
        } else {
          std::stringstream ss;
          ss << best_dist << "m";
          bad_artifacts.push_back(std::make_tuple(std::get<1>(report), pt_darpa, best_ind, ss.str()));
        }
        if (best_distxy <= 5.0) {
          std::cout << "A point scored if we limit to xy plane with residual "
            "error of "
            << best_distxy << std::endl;
          pointsxy++;
        }
        residual = best_dist;
        residualxy = best_distxy;
        residuals.insert(residual);
        residualsxy.insert(residualxy);
        RMSE = getRMSE(residuals);
        RMSExy = getRMSE(residualsxy);
        if (residual < min_error)
          min_error = residual;
        if (residual > max_error)
          max_error = residual;
        if (residualxy < min_errorxy)
          min_errorxy = residualxy;
        if (residualxy > max_errorxy)
          max_errorxy = residualxy;
      }
    }
    if (first == true) {
      rmse_file << std::left << std::setw(12) << std::setfill(' ') << "Time";
      rmse_file << std::left << std::setw(12) << std::setfill(' ') << "residual";
      rmse_file << std::left << std::setw(12) << std::setfill(' ') << "points";
      rmse_file << std::left << std::setw(12) << std::setfill(' ') << "RMSE";
      rmse_file << std::left << std::setw(12) << std::setfill(' ') << "min_err";
      rmse_file << std::left << std::setw(12) << std::setfill(' ') << "max_err";
      rmse_file << std::left << std::setw(12) << std::setfill(' ') << "residual2d";
      rmse_file << std::left << std::setw(12) << std::setfill(' ') << "points2d";
      rmse_file << std::left << std::setw(12) << std::setfill(' ') << "RMSE2d";
      rmse_file << std::left << std::setw(12) << std::setfill(' ') << "min_err2d";
      rmse_file << std::left << std::setw(12) << std::setfill(' ') << "max_err2d";
      rmse_file << std::left << std::setw(12) << std::setfill(' ') << "relative_RMSE";
      rmse_file << std::left << std::setw(12) << std::setfill(' ') << std::endl;
      first = false;
    }
    rmse_file << std::left << std::setw(12) << std::setfill(' ') << now_sec - now_sec_start;
    rmse_file << std::left << std::setw(12) << std::setfill(' ') << residual;
    rmse_file << std::left << std::setw(12) << std::setfill(' ') << points;
    rmse_file << std::left << std::setw(12) << std::setfill(' ') << RMSE;
    rmse_file << std::left << std::setw(12) << std::setfill(' ') << min_error;
    rmse_file << std::left << std::setw(12) << std::setfill(' ') << max_error;
    rmse_file << std::left << std::setw(12) << std::setfill(' ') << residualxy;
    rmse_file << std::left << std::setw(12) << std::setfill(' ') << pointsxy;
    rmse_file << std::left << std::setw(12) << std::setfill(' ') << RMSExy;
    rmse_file << std::left << std::setw(12) << std::setfill(' ') << min_errorxy;
    rmse_file << std::left << std::setw(12) << std::setfill(' ') << max_errorxy;
    rmse_file << std::left << std::setw(12) << std::setfill(' ') << relative_RMSE;
    rmse_file << std::left << std::setw(12) << std::setfill(' ') << std::endl;

    std::cout << std::left << std::setw(12) << std::setfill(' ') << now_sec - now_sec_start;
    std::cout << std::left << std::setw(12) << std::setfill(' ') << residual;
    std::cout << std::left << std::setw(12) << std::setfill(' ') << points;
    std::cout << std::left << std::setw(12) << std::setfill(' ') << RMSE;
    std::cout << std::left << std::setw(12) << std::setfill(' ') << min_error;
    std::cout << std::left << std::setw(12) << std::setfill(' ') << max_error;
    std::cout << std::left << std::setw(12) << std::setfill(' ') << residualxy;
    std::cout << std::left << std::setw(12) << std::setfill(' ') << pointsxy;
    std::cout << std::left << std::setw(12) << std::setfill(' ') << RMSExy;
    std::cout << std::left << std::setw(12) << std::setfill(' ') << min_errorxy;
    std::cout << std::left << std::setw(12) << std::setfill(' ') << max_errorxy;
    std::cout << std::left << std::setw(12) << std::setfill(' ') << relative_RMSE;
    std::cout << std::left << std::setw(12) << std::setfill(' ') << std::endl;
    rmse_file.flush();
    return residual;
  }
  return -1.0;
}

bool sorttuple(
               const std::tuple<double, std::string, tf::Stamped<tf::Point>> &lhs,
               const std::tuple<double, std::string, tf::Stamped<tf::Point>> &rhs) {
  return std::get<0>(lhs) < std::get<0>(rhs);
}

void SubTScoringNode::
onImagesScoring(CodingManager *md, const sensor_msgs::ImageConstPtr &msg,
                const sensor_msgs::ImageConstPtr &depth_msg) {
  static bool first = true;
  try {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    int val = cv::waitKey(10) & 255;
    if (first) {
      int w = image.cols, h = image.rows;
      int height_each = h / md->buttons.size();
      int height_acc = 0;
      for (unsigned int i = 0; i < md->buttons.size(); i++) {
        md->buttons[i].ul.x = w;
        md->buttons[i].ul.y = height_acc;
        md->buttons[i].br.x = w + 100;
        md->buttons[i].br.y = height_acc + height_each;
        height_acc += height_each;
      }
      first = false;
    }
    cv::Mat out_image;
    draw_labels(image, md->buttons, out_image, md->scale);
    {
      boost::mutex::scoped_lock l(md->depth_img_mutex);
      md->image_hdr = msg->header;
      md->depth_image = cv_bridge::toCvCopy(depth_msg, "32FC1")->image;
    }
    if (!md->fiducials_done) {
      cv::Mat imagebw;
      cv::cvtColor(image, imagebw, CV_BGR2GRAY);
      image_u8_t img_header = {.width = imagebw.cols,
        .height = imagebw.rows,
        .stride = imagebw.cols,
        .buf = imagebw.data};
      zarray_t *detections = apriltag_detector_detect(md->td, &img_header);
      for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        if (det->decision_margin < 45.0)
          continue;

        printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
               i, det->family->nbits, det->family->h, det->id, det->hamming,
               det->decision_margin);
        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = 0.2286; // From tunnel circuit rules document
        info.fx = md->camera_model->fx();
        info.fy = md->camera_model->fy();
        info.cx = md->camera_model->cx();
        info.cy = md->camera_model->cy();
        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);
        matd_print(pose.t, "%f");
        tf::Vector3 april_pt_vec(matd_get(pose.t, 0, 0), matd_get(pose.t, 1, 0),
                                 matd_get(pose.t, 2, 0));
        tf::Vector3 local_pt_vec;
        tf::Vector3 report_pt_vec(nanf(""), nanf(""), nanf(""));

        if (ImageTo3DPoint(det->c[0], det->c[1], md, local_pt_vec,
                           report_pt_vec)) {
          ROS_INFO_STREAM("April point : " << april_pt_vec.x() << ", "
                          << april_pt_vec.y() << ", "
                          << april_pt_vec.z());
          ROS_INFO_STREAM("Point from stereo: " << local_pt_vec.x() << ", "
                          << local_pt_vec.y() << ", "
                          << local_pt_vec.z());
          tf::Vector3 diff = local_pt_vec - april_pt_vec;
          ROS_INFO_STREAM("Difference: " << diff.x() << ", " << diff.y() << ", "
                          << diff.z());

          std::stringstream ss;
          std::stringstream fiducial_name;
          fiducial_name << "fiducial_" << det->id;
          ss << fiducial_name.str() << " " << std::fixed
            << md->image_hdr.stamp.toSec() << " " << det->c[0] << " "
            << det->c[1] << " " << md->image_hdr.frame_id << " "
            << local_pt_vec.x() << " " << local_pt_vec.y() << " "
            << local_pt_vec.z() << " " << md->report_frame << " "
            << report_pt_vec.x() << " " << report_pt_vec.y() << " "
            << report_pt_vec.z() << std::endl;

          // Call user entered fiducial reports of quality 100
          std::map<std::string, std::pair<double, std::string>>::const_iterator
            itr = md->reports_to_write_first.find(fiducial_name.str());
          if (itr == md->reports_to_write_first.end() ||
              itr->second.first < det->decision_margin) {
            HandleReport(std::make_tuple(0.0, fiducial_name.str(),
                                         tf::Stamped<tf::Point>(
                                                                report_pt_vec, md->image_hdr.stamp,
                                                                md->report_frame)),
                         md->image_hdr.stamp.toSec());
            // This should get Darpa to map frame transform while marking
            // artifacts,
            md->reports_to_write_first[fiducial_name.str()] =
              std::make_pair(det->decision_margin, ss.str());
          }
        }
      }
      apriltag_detections_destroy(detections);
    }
    cv::imshow("image", out_image);
    PublishMarkers();
    // If val is a spacebar, then pause/play the bag file?
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR_STREAM("Could not convert from bgr8 to "
                     << msg->encoding.c_str());
  }
}

SubTScoringNode::SubTScoringNode() : private_nh("~") {
  private_nh.param("gt_filename", gt_filename, std::string("SCREWED"));
  private_nh.param("artifacts_filename", artifacts_filename,
                   std::string("/var/tmp/tunnel_ckt_coding_ws"));
  private_nh.param("output_filename", output_filename,
                   std::string("/var/tmp/score_result"));
  private_nh.param("use_optical_frame_report", use_optical_frame_report, false);
  private_nh.param("fiducial_file", fiducial_file, std::string("SCREWED"));
  private_nh.param("map_frame", map_frame, std::string("map"));
  private_nh.param("rmse_filename", rmse_filename,
                   std::string("/var/tmp/rmse"));
  private_nh.param("coding_mode", coding_mode, false);
  private_nh.param("initialize_flat", initialize_flat, false);

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("SubT_markers", 1);
  reverse_transform = true;
}

void SubTScoringNode::ScoreRun() {
  rmse_file.open(rmse_filename.c_str(), std::ios::out);
  std::ifstream infile(artifacts_filename.c_str(), std::ios::in);
  if (!infile.is_open()) {
    ROS_ERROR_STREAM("Cannot judge mapping run without artifacts record. "
                     "Please provide a artifacts_filename");
    exit(1);
  }
  std::string line;
  while (std::getline(infile, line)) {
    std::stringstream ss(line);
    std::string label;
    double time;
    double u, v;
    std::string frame;

    double x, y, z;
    std::string image_frame;
    double xx, yy, zz;
    ss >> label >> time >> u >> v >> image_frame >> xx >> yy >> zz >> frame >>
      x >> y >> z;
    tf::Stamped<tf::Point> pt(tf::Point(x, y, z), ros::Time(time), frame);
    tf::Stamped<tf::Point> impt(tf::Point(xx, yy, zz), ros::Time(time),
                                image_frame);
    if (use_optical_frame_report) {
      reports.push_back(std::make_tuple(time, label, impt));
    } else {
      reports.push_back(std::make_tuple(time, label, pt));
    }
  }
  infile.close();
  reports.sort(sorttuple);
  ROS_INFO_STREAM("Processing " << reports.size() << " artifact marking reports + fiducials");

  ros::Rate r(10);
  while (reports.size() != 0 && ros::ok()) {
    double now = ros::Time::now().toSec();
    while (reports.size() != 0 && now > std::get<0>(reports.front())) {
      // Handle a object report
      double residual = HandleReport(reports.front(), now);
      PublishMarkers();
      finished_reports.push_back(std::make_tuple(
                                                 std::get<0>(reports.front()), std::get<1>(reports.front()),
                                                 std::get<2>(reports.front()), residual));
      reports.pop_front();
    }
    ros::spinOnce();
    r.sleep();
  }
  rmse_file.close();
}

void SubTScoringNode::Load() {
  std::ifstream infile(gt_filename.c_str(), std::ios::in);
  if (!infile.is_open()) {
    ROS_ERROR_STREAM("Cannot judge mapping run without ground truth state. "
                     "Please provide a gt_filename. What I have is: " << gt_filename);
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
  infile.open(fiducial_file.c_str(), std::ios::in);
  if (!infile.is_open()) {
    ROS_ERROR_STREAM("Unable to open fiducial ground truth locations. Please provide a fiducial_file. What I have is " << fiducial_file);
  }
  while (std::getline(infile, line)) {
    std::stringstream ss(line);
    std::string label;
    double x, y, z;
    ss >> label >> x >> y >> z;
    fiducial_point_darpa_frame.insert(
                                      std::make_pair(label, tf::Point(x, y, z)));
  }
  infile.close();
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "subt_scoring_node");
  SubTScoringNode scoring_node;
  scoring_node.Load();

  if (scoring_node.coding_mode) {
    cv::namedWindow("image", CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    CodingManager md(nh, private_nh);
    std::string image_name, img_transport, depth_image_name, depth_transport;
    private_nh.param("image", image_name, std::string("image"));
    private_nh.param("depth_image", depth_image_name,
                     std::string("depth_image"));
    private_nh.param("image_transport", img_transport, std::string("compressed"));
    private_nh.param("depth_transport", depth_transport, std::string("compressedDepth"));
    
    cv::setMouseCallback("image", onMouse, &md);
    ros::Subscriber info_sub = nh.subscribe<sensor_msgs::CameraInfo>(
                                                                     "camera_info", 1, boost::bind(&onCameraInfo, &md, _1));
    image_transport::ImageTransport it(nh);
    image_transport::SubscriberFilter img_sub(
                                              it, image_name, 50, image_transport::TransportHints(img_transport));
    image_transport::SubscriberFilter depth_img_sub(
                                                    it, depth_image_name, 50,
                                                    image_transport::TransportHints(depth_transport));
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
            sensor_msgs::Image>
              MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), img_sub,
                                                     depth_img_sub);
    sync.registerCallback(boost::bind(&SubTScoringNode::onImagesScoring, &scoring_node, &md, _1, _2));

    while (ros::ok()) {
      cv::waitKey(5);
      ros::spinOnce();
    }
    cv::destroyAllWindows();
  } else {
    scoring_node.ScoreRun();
  }
  return 0;
}

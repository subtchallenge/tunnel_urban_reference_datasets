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

std::map<std::string, tf::Vector3> gt_artifacts;
std::list<std::pair<double, std::pair<std::string, tf::PointStamped> > > reports;
std::map<string, tf::Point> fiducial_point_darpa_frame;

void HandleReport(const std::pair<double, std::pair<std::string, tf::PoseStamped> >& report, const ros::Time& now) {
  ROS_INFO_STREAM("Handling report at time " << now.toSec()); 
  //See if it is a fiducial marker. If so, refine the estimate of the DARPA frame to map transform
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "subt_scoring_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std::string gt_filename;
  std::string artifacts_filename;
  std::string ouput_filename;
  bool use_optical_frame_report;
  private_nh.param("gt_filename", gt_filename, std::string("SCREWED"));
  private_nh.param("artifacts_filename", artifacts_filename, std::string("/var/tmp/tunnel_ckt_coding_ws"));
  private_nh.param("output_filename", output_filename, std::string("/var/tmp/score_result"));
  private_nh.param("use_optical_frame_report", use_optical_frame_report, false);
  private_nh.param("fiducial_file", fiducial_file, "SCREWED");
  tf::TransformListener tfL;
  std::ifstream infile(gt_filename.c_str(), std::ios::open);
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
    gt_artifacts.insert(std::make_pair(label, tf::Vector3(x,y,z)));
  }
  infile.close();

  infile.open(artifacts_filename.c_str(), std::ios::open);
  if (!infile.is_open()){
    ROS_ERROR_STREAM("Cannot judge mapping run without artifacts record. Please provide a artifacts_filename");
    exit(1);
  }
  // fiducial_L 1566222460.357846 406 281 chinook/multisense/left_camera_optical_frame -1.187567 0.088331 5.696559 chinook/base_link 5.911559 1.222567 0.459669
  while (std::getline(infile, line)) {
    std::stringstream ss (line);
    std::string label;
    double time;
    int u,v;
    std::string frame;

    double x, y, z;
    std::string image_frame;
    double xx, yy, zz;
    ss >> label >> time >> u >> v >> frame >> x >> y >> z >> image_frame >> xx >> yy >> zz;
    tf::Stamped<tf::Point> pt(tf::Point(x,y,z), ros::Time::fromSec(time), frame);
    tf::Stamped<tf::Point> impt(tf::Point(xx,yy,zz), ros::Time::fromSec(time), image_frame);
    if (use_optical_frame_report) {
      reports.push_back(std::make_pair(time, std::make_pair(label, impt)));
    } else {
      reports.push_back(std::make_pair(time, std::make_pair(label, pt)));
    }
  }
  infile.close();
  std::sort(reports.begin(), reports.end());

  ros::Rate r(10);
  while (ros::ok()) {
    ros::Time now = ros::Time::now();
    while (now > reports.front().first) {
      //Handle a object report
      HandleReport(reports.front(), now);
      reports.pop_front();
    }
    ros::spinOnce();
    r.sleep();
  }
  md.outfile.close();
  cv::destroyAllWindows();
  return 0;
}

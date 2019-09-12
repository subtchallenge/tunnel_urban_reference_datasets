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

bool redraw = false;
bool l_mouse_down = false;
bool r_mouse_down = false;
bool m_mouse_down = false;
int active_class = -1;
apriltag_detector_t *td;

struct button_t {
  std::string text;
  unsigned char r;
  unsigned char g;
  unsigned char b;
  cv::Point ul;
  cv::Point br;
};
struct mouse_data_t {
  std::vector<button_t> buttons;
  boost::shared_ptr<image_geometry::PinholeCameraModel> camera_model;
  cv::Mat depth_image;
  std_msgs::Header image_hdr;
  mutable boost::mutex depth_img_mutex;
  boost::shared_ptr<tf::TransformListener> tf_l;
  std::string report_frame;
  std::ofstream outfile;
};
double scale = 1.0;
float find_depth_nearest_pixel(const cv::Mat& depth_image, int x, int y) {
  float depth;
  depth = depth_image.at<float>(y, x); 
  int xmin = x - 1;
  int xmax = x + 1;
  int ymin = y - 1;
  int ymax = y + 1;
  while (isnan(depth)) {
    if (xmin < 0) xmin = 0;
    if (xmax >= depth_image.cols) xmax = depth_image.cols-1; 
    if (ymin < 0) xmin = 0;
    if (ymax >= depth_image.rows) ymax = depth_image.rows-1; 
    for (x = xmin, y = ymin; x <= xmax; x++) {
      depth = depth_image.at<float>(y,x); 
      if (!isnan(depth)) return depth;
    }
    for (x = xmin, y = ymax; x <= xmax; x++) {
      depth = depth_image.at<float>(y,x); 
      if (!isnan(depth)) return depth;
    }
    for (x = xmin, y = ymin; y <= ymax; y++) {
      depth = depth_image.at<float>(y,x); 
      if (!isnan(depth)) return depth;
    }
    for (x = xmax, y = ymin; y <= ymax; y++) {
      depth = depth_image.at<float>(y,x); 
      if (!isnan(depth)) return depth;
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
bool ImageTo3DPoint(int x, int y, const mouse_data_t* md,
                    tf::Vector3& local_pt_vec, tf::Vector3& report_pt_vec) {


  if (md->camera_model->initialized()) {
    float depth;
    {
      boost::mutex::scoped_lock l(md->depth_img_mutex);
      depth = find_depth_nearest_pixel(md->depth_image, x, y);
    }
    if (!isnan(depth)) {
      std::cout << "Saw an " << md->buttons[active_class].text << " at " << x << ", " << y 
        << "with depth " << depth << std::endl; 
      cv::Point3d ray = md->camera_model->projectPixelTo3dRay(cv::Point2d(x,y));
      cv::Point3d local_pt = ray * depth;
      std::cout <<" Local (camera frame) coordinates of artifact " << md->buttons[active_class].text 
        << " are (" << local_pt.x << ", " << local_pt.y << ", " << local_pt.z << ")" << std::endl;
      tf::StampedTransform image_to_report_frame;
      try {
        //            md->tf_l->waitForTransform(md->report_frame, md->image_hdr.frame_id, ros::Time(0), ros::Duration(0.1));
        md->tf_l->lookupTransform(md->report_frame, md->image_hdr.frame_id, ros::Time(0), image_to_report_frame);
      } catch(const tf::TransformException& ex) {
        ROS_ERROR_STREAM("Transform exception in subt_coding_node. Cannot transform from " 
                         << md->report_frame << " to " << md->image_hdr.frame_id << ": " << ex.what());
        return false;
      }
      local_pt_vec = tf::Vector3(local_pt.x, local_pt.y, local_pt.z);
      report_pt_vec = image_to_report_frame * local_pt_vec;
      return true;
    } else {
      return false;
    }
  }
  return false;
}
std::map<std::string, std::pair<double, std::string> > reports_to_write_first;
bool fiducials_done = false;

void onMouse (int event, int x, int y, int, void* data) {
  mouse_data_t *md = (mouse_data_t*)data;
  x = x / scale;
  y = y / scale;
  if (md->buttons.size() && x >= md->buttons[0].ul.x && 
      x <= md->buttons[0].br.x) {
    l_mouse_down = false; r_mouse_down = false;
    m_mouse_down = false;
    if (event == cv::EVENT_LBUTTONUP) {
      for (unsigned int i = 0;i<md->buttons.size();i++ ){
        if (y > md->buttons[i].ul.y && y < md->buttons[i].br.y) {
          active_class = i;
        }
      }
    }
  } else {
    if (event == cv::EVENT_LBUTTONUP && active_class >= 0 && active_class < md->buttons.size()) {

      // Make report at this pixel
      tf::Vector3 local_pt_vec, report_pt_vec;
      if (ImageTo3DPoint(x, y, md, local_pt_vec, report_pt_vec)) {
        std::cout <<" Local (camera frame) coordinates of artifact " << md->buttons[active_class].text 
          << " are (" << local_pt_vec.x() << ", " << local_pt_vec.y() << ", " << local_pt_vec.z() << ")" 
          << std::endl;
        std::cout <<" Report frame coordinates of artifact " << md->buttons[active_class].text 
          << " are (" << report_pt_vec.x() << ", " << report_pt_vec.y() << ", " << report_pt_vec.z() << ")"
          << std::endl;
        std::stringstream ss;
        ss << md->buttons[active_class].text << " " 
          << std::fixed << md->image_hdr.stamp.toSec() << " " 
          << x << " " << y << " "
          << md->image_hdr.frame_id << " " << local_pt_vec.x() << " " << local_pt_vec.y() << " " << local_pt_vec.z() << " "
          << md->report_frame << " " << report_pt_vec.x() << " " << report_pt_vec.y() << " " << report_pt_vec.z() 
          << std::endl;

        if (md->outfile.is_open()) {
          if (!fiducials_done && reports_to_write_first.size() != 0) {
            //Write out all of the best fiducial data before we write artifacts

            for (std::map<std::string, std::pair<double, std::string> >::const_iterator itr = 
                 reports_to_write_first.begin(); itr != reports_to_write_first.end(); itr++) {
              md->outfile << itr->second.second;
            }

            fiducials_done = true;
          }
          md->outfile << ss.str();
          md->outfile.flush();
        }

      } else {
        std::cout <<" NaN at current location. Please move slightly and give a new artifact report" << std::endl;
      }
    }
  }
}

void draw_labels(cv::Mat image,
                 const std::vector<button_t>& buttons,
                 cv::Mat& out_image) {
  out_image = cv::Mat(image.rows, image.cols+100, CV_8UC3);

  cv::Mat part (out_image, cv::Range(0, image.rows), cv::Range(0, image.cols));
  image.copyTo(part);

  for (unsigned int i = 0;i<buttons.size();i++) {
    cv::rectangle(out_image,buttons[i].ul, buttons[i].br, CV_RGB(buttons[i].r, buttons[i].g, buttons[i].b), -1);
    cv::Scalar text_color = CV_RGB(255,255,255);
    if ((buttons[i].b + buttons[i].g + buttons[i].r > 1.5 * 255))
      text_color = CV_RGB(0,0,0);
    int baseline = 0;
    cv::Size textsize = cv::getTextSize(buttons[i].text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    baseline += 1;
    cv::Point textorg((buttons[i].ul.x + buttons[i].br.x)/2-textsize.width/2,
                      (buttons[i].ul.y + buttons[i].br.y)/2 + textsize.height/2);
    cv::putText(out_image, buttons[i].text, textorg, cv::FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1, 8);
  }
  cv::resize(out_image, out_image, cv::Size(0,0), scale, scale);
}

void onCameraInfo(mouse_data_t* md, const sensor_msgs::CameraInfo::ConstPtr& msg) {
  md->camera_model->fromCameraInfo(msg);

}

void onImages(mouse_data_t* md, 
              const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& depth_msg) {
  static bool first = true;
  try {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    int val = cv::waitKey(100) &255;
    if (first) {
      int w = image.cols, h = image.rows;
      int height_each = h / md->buttons.size();
      int height_acc = 0;
      for (unsigned int i = 0;i<md->buttons.size();i++){ 
        md->buttons[i].ul.x = w;
        md->buttons[i].ul.y = height_acc;
        md->buttons[i].br.x = w+100;
        md->buttons[i].br.y = height_acc + height_each;
        height_acc += height_each;
      }
      first = false;
    }
    cv::Mat out_image;
    draw_labels(image, 
                md->buttons, out_image);
    {
      boost::mutex::scoped_lock l(md->depth_img_mutex);
      md->image_hdr = msg->header;
      md->depth_image = cv_bridge::toCvCopy(depth_msg, "32FC1")->image; 
    }
    if (!fiducials_done) {
      cv::Mat imagebw;
      cv::cvtColor(image, imagebw, CV_BGR2GRAY);
      image_u8_t img_header = { .width = imagebw.cols, .height = imagebw.rows, .stride = imagebw.cols, .buf=imagebw.data};
      zarray_t *detections = apriltag_detector_detect(td, &img_header);
      for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        if (det->decision_margin < 30.0) continue;

        printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
               i, det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);
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
        tf::Vector3 april_pt_vec(matd_get(pose.t, 0, 0), matd_get(pose.t, 1, 0), matd_get(pose.t, 2, 0));
        tf::Vector3 local_pt_vec;
        tf::Vector3 report_pt_vec(nanf(""), nanf(""), nanf(""));
        
        if (ImageTo3DPoint(det->c[0], det->c[1], md,
                    local_pt_vec, report_pt_vec)) {
          ROS_INFO_STREAM("April point : " << april_pt_vec.x() << ", " << april_pt_vec.y() << ", " << april_pt_vec.z());
          ROS_INFO_STREAM("Point from stereo: " << local_pt_vec.x() << ", " << local_pt_vec.y() << ", " << local_pt_vec.z());
          tf::Vector3 diff = local_pt_vec - april_pt_vec;
          ROS_INFO_STREAM("Difference: " << diff.x() << ", " << diff.y() << ", " << diff.z());

          std::stringstream ss;
          std::stringstream fiducial_name;
          fiducial_name << "fiducial_" << det->id;
          ss << fiducial_name.str() << " " 
            << std::fixed << md->image_hdr.stamp.toSec() << " " 
            << det->c[0] << " " << det->c[1] << " "
            << md->image_hdr.frame_id << " " << local_pt_vec.x() << " " << local_pt_vec.y() << " " << local_pt_vec.z() << " "
            << md->report_frame << " " << report_pt_vec.x() << " " << report_pt_vec.y() << " " << report_pt_vec.z() 
            << std::endl;

          //Call user entered fiducial reports of quality 100
          std::map<std::string, std::pair<double, std::string> >::const_iterator itr = 
            reports_to_write_first.find(fiducial_name.str());
          if (itr == reports_to_write_first.end() || itr->second.first < det->decision_margin) {
            reports_to_write_first[fiducial_name.str()] = std::make_pair(det->decision_margin, ss.str());
          }
        }
      }
      apriltag_detections_destroy(detections);
    }
    cv::imshow("image", out_image);
    val = cv::waitKey(5) &255;
    // If val is a spacebar, then pause/play the bag file?
  } catch(cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("Could not convert from bgr8 to " << msg->encoding.c_str());
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "subt_coding_node");
  cv::namedWindow("image", CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);
  mouse_data_t md;
  std::vector<button_t>buttons;
  td = apriltag_detector_create();
  apriltag_family_t *tagfam = tag16h5_create();
  apriltag_detector_add_family_bits(td, tagfam, 1); //Allow this many bit errors
  td->quad_decimate = 2.0;
  td->quad_sigma = 0.0; //Blur factur (negative sharpens)
  td->nthreads = 1;
  td->debug = 0;
  td->refine_edges = 1;


  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std::string image_name, depth_image_name;
  private_nh.param("image", image_name, std::string("image"));
  private_nh.param("depth_image", depth_image_name, std::string("depth_image"));
  private_nh.param("report_frame", md.report_frame, std::string("base"));
  std::string outfilename;
  private_nh.param("outfile", outfilename, std::string("/var/tmp/tunnel_ckt_coding_ws"));
  md.camera_model = boost::make_shared<image_geometry::PinholeCameraModel>();
  md.tf_l = boost::make_shared<tf::TransformListener>();
  if (private_nh.hasParam("object_classes")) {
    std::string colormap;
    private_nh.param("object_classes", colormap, std::string("screwed"));
    std::ifstream file;
    file.open(colormap.c_str(), std::ios::in);
    if (file.is_open()) {
      std::string line;
      while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string lab;
        unsigned int r, g, b;
        ss >> lab >> r >> g >> b;
        button_t button;
        button.text = lab;
        button.r = r;
        button.g = g;
        button.b = b;
        buttons.push_back(button);
      }
      file.close();      
    }
  }
  md.buttons = buttons;
  md.outfile.open(outfilename.c_str(), std::ios::out);
  cv::setMouseCallback("image", onMouse, &md);
  ros::Subscriber info_sub = nh.subscribe<sensor_msgs::CameraInfo>("camera_info", 1, boost::bind(&onCameraInfo, &md, _1));
  image_transport::ImageTransport it(nh);
  image_transport::SubscriberFilter img_sub(it, image_name, 50, image_transport::TransportHints("compressed"));
  image_transport::SubscriberFilter depth_img_sub(it, depth_image_name, 50, image_transport::TransportHints("compressedDepth"));
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), img_sub, depth_img_sub);
  sync.registerCallback(boost::bind(&onImages, &md, _1, _2));

  while (ros::ok()) {
    cv::waitKey(5);
    ros::spinOnce();
  }
  md.outfile.close();
  cv::destroyAllWindows();
  apriltag_detector_destroy(td);

  return 0;
}

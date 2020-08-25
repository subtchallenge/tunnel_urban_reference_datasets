/*
 *
 * John G. Rogers III
 *
 * 
 */

#include <subt_artifact_report/coding.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "subt_coding_node");
  cv::namedWindow("image", CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  CodingManager md(nh, private_nh);
  std::string image_name, img_transport, depth_image_name, depth_transport;
  private_nh.param("image", image_name, std::string("image"));
  private_nh.param("depth_image", depth_image_name, std::string("depth_image"));
  private_nh.param("image_transport", img_transport, std::string("compressed"));
  private_nh.param("depth_transport", depth_transport, std::string("compressedDepth"));

  cv::setMouseCallback("image", onMouse, &md);
  ros::Subscriber info_sub = nh.subscribe<sensor_msgs::CameraInfo>("camera_info", 1, boost::bind(&onCameraInfo, &md, _1));
  image_transport::ImageTransport it(nh);
  image_transport::SubscriberFilter img_sub(it, image_name, 50, image_transport::TransportHints(img_transport));
  image_transport::SubscriberFilter depth_img_sub(it, depth_image_name, 50, image_transport::TransportHints(depth_transport));
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), img_sub, depth_img_sub);
  sync.registerCallback(boost::bind(&onImages, &md, _1, _2));

  while (ros::ok()) {
    cv::waitKey(5);
    ros::spinOnce();
  }
  cv::destroyAllWindows();

  return 0;
}


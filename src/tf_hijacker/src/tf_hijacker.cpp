
#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <vector>
#include <cstdio>

ros::Publisher pub;
ros::Subscriber sub;

std::vector<std::string> suppressors;
std::vector<std::pair<std::string, std::string> > link_suppressors;

bool same_frame(const std::string &lhs_, const std::string &rhs_)
{
  std::string lhs;
  std::string rhs;

  if(lhs_.find_first_of('/') == 0 && lhs_.size() > 1)
    lhs = lhs_.substr(1);
  else
    lhs = lhs_;

  if(rhs_.find_first_of('/') == 0 && rhs_.size() > 1)
    rhs = rhs_.substr(1);
  else
    rhs = rhs_;

  return (lhs == rhs);
}

void tf_recvd(const tf::tfMessageConstPtr& msg) {
  tf::tfMessage out;
  for( unsigned int i = 0;i<msg->transforms.size();i++) {
    ROS_DEBUG("Checking %s->%s transform", msg->transforms[i].header.frame_id.c_str(), msg->transforms[i].child_frame_id.c_str());
    bool pass = true;
    for (unsigned int supp = 0;supp < suppressors.size();supp++) {
      if (same_frame(msg->transforms[i].header.frame_id, suppressors[supp])) {
        ROS_DEBUG("Blocking %s->%s transform based on %s", msg->transforms[i].header.frame_id.c_str(), msg->transforms[i].child_frame_id.c_str(), suppressors[supp].c_str());
        pass = false;
      }
      if (same_frame(msg->transforms[i].child_frame_id, suppressors[supp])) {
        ROS_DEBUG("Blocking %s->%s transform based on %s", msg->transforms[i].header.frame_id.c_str(), msg->transforms[i].child_frame_id.c_str(), suppressors[supp].c_str());
        pass = false;
      }
    }

    for (unsigned int supp = 0;supp < link_suppressors.size();supp++) {
      if (same_frame(msg->transforms[i].header.frame_id, link_suppressors[supp].first) &&
          same_frame(msg->transforms[i].child_frame_id, link_suppressors[supp].second)) {
        ROS_DEBUG("Blocking %s->%s transform\n", link_suppressors[supp].first.c_str(), link_suppressors[supp].second.c_str());
        pass = false;
      }
    }

    if (pass)
      out.transforms.push_back(msg->transforms[i]);
  }

  pub.publish(out);
}
int main(int argc, char** argv) {
  ros::init(argc,argv, "tf_hijacker");
  ros::NodeHandle nh_("~");

  char itoa_hole[50];
  bool good = true;
  int i = 1;
  do {
    sprintf(itoa_hole,"suppress%d", i);
    std::string name(itoa_hole);
    if (nh_.hasParam(name)) {
      std::string val;
      nh_.param(name,val,std::string("/screwed"));
      suppressors.push_back(val);
      ROS_INFO("Suppressing %s",
	       val.c_str());
    }
    else
      good = false;
    i++;
  }while (good);

  i = 1;
  good = true;
  do {
    sprintf(itoa_hole,"link_suppress%d", i);
    std::string name(itoa_hole);
    sprintf(itoa_hole,"link_child_suppress%d", i);
    std::string child(itoa_hole);
    if (nh_.hasParam(name) && nh_.hasParam(child)) {
      std::string val1, val2;
      nh_.param(name,val1,std::string("/screwed"));
      nh_.param(child,val2,std::string("/screwed"));
      link_suppressors.push_back(std::make_pair(val1, val2));
      ROS_INFO("Suppressing %s->%s",
               val1.c_str(), val2.c_str());
    }
    else
      good = false;
    ++i;
  }while (good);

  sub = nh_.subscribe<tf::tfMessage>("/tf_remap", 100, tf_recvd);
  pub = nh_.advertise<tf::tfMessage>("/tf",100);
  ros::spin();
}


#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <vector>
#include <cstdio>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <fstream>

ros::Publisher pub;
ros::Subscriber sub;

std::vector<std::string> suppressors;
std::vector<std::pair<std::string, std::string> > link_suppressors;
struct DynObjectT {
  bool initialized;
  double critical_actual_velocity;
  double v_goal;
  double acceleration;
  double v_curr;  // This gets put in directly when velocity is lower than error velocity (0.95)
  double v_last;
  // and substituted by v_last + acceleration * dt 
  // where acceleration is determined by characterizing platform dynamics
  ros::Time last_time;
  geometry_msgs::TransformStamped last_given_pose;
  geometry_msgs::TransformStamped last_output_pose;
  DynObjectT(double crit_vel, double acc) {
    initialized = false;
    critical_actual_velocity = crit_vel;
    v_goal = 0.0; acceleration = acc; v_curr = 0.0;
    v_last = 0.0;
    last_time = ros::Time(0);
  }
};
typedef std::map<std::string, DynObjectT> DynObjectMapT;
DynObjectMapT dyn_object_records;

void cmd_vel_recvd(const std::string& str, const geometry_msgs::Twist::ConstPtr& msg) {
  DynObjectMapT::iterator itr = dyn_object_records.find(str);
  if (itr == dyn_object_records.end()) {
    return;
  } else {
    itr->second.v_goal = (msg->linear.x >1.0)? 1.0: msg->linear.x;
    itr->second.v_goal = (msg->linear.x < -1.0)? -1.0 : itr->second.v_goal;
  }
}

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
  static std::ofstream outfile("/var/tmp/velcmp");
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

    for (DynObjectMapT::iterator itr = dyn_object_records.begin(); itr != dyn_object_records.end(); itr++) {
      if (same_frame(msg->transforms[i].header.frame_id, itr->first)) {
        bool initialized_here = false;
        if (!itr->second.initialized) {
          //Initialize
          itr->second.last_given_pose = msg->transforms[i];
          itr->second.last_output_pose = msg->transforms[i];
          itr->second.last_time = msg->transforms[i].header.stamp;
          itr->second.initialized = true;
          initialized_here = true;
        }
        tf::StampedTransform prior_given_pose;
        tf::StampedTransform prior_output_pose;
        tf::StampedTransform curr_given_pose;
        tf::StampedTransform curr_output_pose;
        tf::transformStampedMsgToTF(itr->second.last_given_pose, prior_given_pose);
        tf::transformStampedMsgToTF(itr->second.last_output_pose, prior_output_pose);
        tf::transformStampedMsgToTF(msg->transforms[i], curr_given_pose);
        tf::Transform given_between = prior_given_pose.inverse() * curr_given_pose;
        double dt = (msg->transforms[i].header.stamp - itr->second.last_time).toSec();
        double old_vel = dt > 0 ? given_between.getOrigin().length() / dt : 0.0;
        if (initialized_here) itr->second.v_curr = old_vel; //Assume we get at least the first velocity is good
        itr->second.last_time = msg->transforms[i].header.stamp;
        tf::Transform curr_between = given_between;
        if (fabs(old_vel) > 0 && dt > 0.0) {
          curr_between.setOrigin(given_between.getOrigin().normalized()); // Contains the direction of the odometric correction
          double cmd_vel = fabs(itr->second.v_goal);
          double v_final = itr->second.v_curr;
          double acc = itr->second.acceleration;
          if (cmd_vel > v_final) {
            v_final += dt * itr->second.acceleration;
            if (v_final > cmd_vel) v_final = cmd_vel;
          }
          if (cmd_vel < v_final) {
            v_final -= dt * itr->second.acceleration;
            acc = -acc;
            if (v_final < cmd_vel) v_final = cmd_vel;
          }
          outfile << old_vel << " " << cmd_vel << " " << itr->second.v_curr << " " <<  v_final << " " << dt << std::endl;

          if (v_final > itr->second.critical_actual_velocity) {
            //Substitute this speed
            itr->second.v_last = old_vel;
            curr_between.setOrigin(curr_between.getOrigin() * ((itr->second.v_curr + v_final)* 0.5 * dt));
          } else {
            v_final = old_vel;
            itr->second.v_last = old_vel;
            curr_between.setOrigin(curr_between.getOrigin() * (old_vel * dt));
          }
          itr->second.v_curr = v_final;
        } else {
          curr_between = given_between; 
        }
        curr_output_pose = curr_given_pose; // To pick up frame id, time etc
        curr_output_pose.setData(prior_output_pose * curr_between);
        geometry_msgs::TransformStamped curr_output_msg;
        tf::transformStampedTFToMsg(curr_output_pose, curr_output_msg);
        itr->second.last_given_pose = msg->transforms[i];
        itr->second.last_output_pose = curr_output_msg;
        pass = false;  // We will put on our own scaled version of this transform
        out.transforms.push_back(curr_output_msg);
        // Now we put out another transform to a different frame with the original values, for comparison
        curr_output_pose.setData(curr_given_pose);
        curr_output_pose.child_frame_id_ = "base_unrepaired";
        tf::transformStampedTFToMsg(curr_output_pose, curr_output_msg);
        out.transforms.push_back(curr_output_msg);
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

  std::vector<ros::Subscriber> cmd_vel_subs;
  XmlRpc::XmlRpcValue switcheroos;
  if (nh_.getParam("switcheroos", switcheroos) && switcheroos.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (unsigned int i = 0; i < switcheroos.size(); i++) {
      if (switcheroos[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        for (auto input_topic : switcheroos[i]) {
          if (input_topic.second.hasMember("replacement_vel") &&
              input_topic.second.hasMember("replacement_thresh") &&
              input_topic.second.hasMember("acceleration")) {
            cmd_vel_subs.push_back(nh_.subscribe<geometry_msgs::Twist>(input_topic.second["replacement_vel"], 100, 
                                                                       boost::bind(cmd_vel_recvd, input_topic.first, _1)));
            dyn_object_records.insert(std::make_pair(input_topic.first, DynObjectT(input_topic.second["replacement_thresh"], 
                                                                                   input_topic.second["acceleration"])));
          }
        }
      }
    }
  }
  bool good = true;
  int i = 1;
  char itoa_hole[500];
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
  } while (good);

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
  } while (good);
  sub = nh_.subscribe<tf::tfMessage>("/tf_remap", 100, tf_recvd);
  pub = nh_.advertise<tf::tfMessage>("/tf",100);
  ros::spin();
}

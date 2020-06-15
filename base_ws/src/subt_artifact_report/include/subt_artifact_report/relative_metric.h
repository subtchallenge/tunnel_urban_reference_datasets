#pragma once
#include <tf/tf.h>
#include <Eigen/Geometry>

#include <queue>
#include <map>
#include <string>
#include <vector>
#include <list>
#include <utility>

double distsq(const tf::Point& pt1, const tf::Point& pt2);

double GetRMSE(const std::vector<std::pair<std::string, tf::Point>>& gt_artifacts,
               const std::vector<std::pair<std::string, tf::Stamped<tf::Point>>> &report,
               const std::vector<int>& assignment);

double GetBestRelativeDA(const std::vector<std::pair<std::string, tf::Point>>& gt_artifacts,
                         const std::vector<std::pair<std::string, tf::Stamped<tf::Point>>> &report,
                         std::vector<int>* best_da);

#pragma once
#include <tf/tf.h>
#include <Eigen/Geometry>

#include <queue>
#include <map>
#include <string>
#include <vector>
#include <list>
#include <utility>

void PrintMat(const Eigen::MatrixXf& matr);

double distsq(const tf::Point& pt1, const tf::Point& pt2);

double GetRMSE(const std::vector<std::pair<std::string, tf::Point>>& gt_artifacts,
               const std::vector<std::pair<std::string, tf::Stamped<tf::Point>>> &report,
               const std::vector<int>& assignment,
               Eigen::MatrixXf* transform);

double GetBestRelativeDA(const std::vector<std::pair<std::string, tf::Point>>& gt_artifacts,
                         const std::vector<std::pair<std::string, tf::Stamped<tf::Point>>> &report,
                         std::vector<int>* best_da,
                         Eigen::MatrixXf* best_transform);

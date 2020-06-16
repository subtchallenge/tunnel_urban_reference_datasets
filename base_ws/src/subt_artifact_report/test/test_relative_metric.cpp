#include <gtest/gtest.h>
#include <subt_artifact_report/relative_metric.h>


TEST(RelativeMetricTest, disttest) {
  tf::Point pt1(0, 0, 0);
  tf::Point pt2(2, 0, 0);
  ASSERT_EQ(distsq(pt1, pt2), 4.0);
}

TEST(RelativeMetricTest, rmse_test) {
  std::vector<std::pair<std::string, tf::Point>> gt_artifacts;
  std::vector<std::pair<std::string, tf::Stamped<tf::Point>>> reports;
  std::vector<int> assignment;

  gt_artifacts.push_back(std::make_pair("a", tf::Point(0, 0, 0)));
  gt_artifacts.push_back(std::make_pair("b", tf::Point(1, 1, 1)));
  gt_artifacts.push_back(std::make_pair("c", tf::Point(2, 2, 3)));
  gt_artifacts.push_back(std::make_pair("c", tf::Point(2, 2, 3.1)));

  reports.push_back(std::make_pair("a", tf::Stamped<tf::Point>(tf::Point(1, 1, 1), ros::Time(0), "noframe")));
  assignment.push_back(0);
  Eigen::MatrixXf transf;

  ASSERT_NEAR(GetRMSE(gt_artifacts, reports, assignment, &transf), 0, 1e-5);
  reports.push_back(std::make_pair("b", tf::Stamped<tf::Point>(tf::Point(2, 2, 2), ros::Time(0), "noframe")));
  assignment.push_back(1);
  ASSERT_NEAR(GetRMSE(gt_artifacts, reports, assignment, &transf), 0, 1e-5);
  reports.push_back(std::make_pair("c", tf::Stamped<tf::Point>(tf::Point(3, 3, 4), ros::Time(0), "noframe")));
  assignment.push_back(2);
  ASSERT_NEAR(GetRMSE(gt_artifacts, reports, assignment, &transf), 0, 1e-5);
  assignment[2] = 3;
  ASSERT_NEAR(GetRMSE(gt_artifacts, reports, assignment, &transf), 0.0383, 1e-5);
}

TEST(RelativeMetricTest, da_test) {
  std::vector<std::pair<std::string, tf::Point>> gt_artifacts;
  std::vector<std::pair<std::string, tf::Stamped<tf::Point>>> reports;
  gt_artifacts.push_back(std::make_pair("a", tf::Point(0, 0, 0)));
  gt_artifacts.push_back(std::make_pair("a", tf::Point(2, 2, 3)));
  gt_artifacts.push_back(std::make_pair("a", tf::Point(5, 0, 0)));
  gt_artifacts.push_back(std::make_pair("a", tf::Point(5, 1, 1)));
  gt_artifacts.push_back(std::make_pair("a", tf::Point(5, 0, 0)));
  gt_artifacts.push_back(std::make_pair("a", tf::Point(5, 1, 1)));
  gt_artifacts.push_back(std::make_pair("a", tf::Point(5, 2, 3.1)));
  gt_artifacts.push_back(std::make_pair("a", tf::Point(5, 2, 3)));
  gt_artifacts.push_back(std::make_pair("a", tf::Point(5, 2, 3.1)));
  gt_artifacts.push_back(std::make_pair("a", tf::Point(5, 2, 3)));
  gt_artifacts.push_back(std::make_pair("a", tf::Point(1, 1, 1)));
  gt_artifacts.push_back(std::make_pair("a", tf::Point(2, 2, 3.1)));
  std::vector<int> assignment;
  Eigen::MatrixXf transform;
  reports.push_back(std::make_pair("a", tf::Stamped<tf::Point>(tf::Point(1, 1, 1), ros::Time(0), "noframe")));
  double rmse = GetBestRelativeDA(gt_artifacts, reports, &assignment, &transform);
  ASSERT_NEAR(rmse, 0.0, 1e-5);
  for (unsigned int i = 0; i < assignment.size(); i++) {
    std::cout << i << "->" << assignment[i] << std::endl;
  }
  PrintMat(transform);
  reports.push_back(std::make_pair("a", tf::Stamped<tf::Point>(tf::Point(2, 2, 2), ros::Time(0), "noframe")));
  rmse = GetBestRelativeDA(gt_artifacts, reports, &assignment, &transform);
  ASSERT_NEAR(rmse, 0.0, 1e-5);
  for (unsigned int i = 0; i < assignment.size(); i++) {
    std::cout << i << "->" << assignment[i] << std::endl;
  }
  PrintMat(transform);
  reports.push_back(std::make_pair("a", tf::Stamped<tf::Point>(tf::Point(3, 3, 4), ros::Time(0), "noframe")));
  rmse = GetBestRelativeDA(gt_artifacts, reports, &assignment, &transform);
  ASSERT_NEAR(rmse, 0.0, 1e-5);
  for (unsigned int i = 0; i < assignment.size(); i++) {
    std::cout << i << "->" << assignment[i] << std::endl;
  }
  PrintMat(transform);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <subt_artifact_report/relative_metric.h>

double distsq(const tf::Point& pt1, const tf::Point& pt2) {
  double dx = pt1.x() - pt2.x();
  double dy = pt1.y() - pt2.y();
  double dz = pt1.z() - pt2.z();
  return dx*dx + dy*dy + dz*dz;
}

void PrintMat(const Eigen::MatrixXf& matr) {

  for (int i = 0; i < matr.rows(); i++) {
    for (int j = 0; j < matr.cols(); j++) {
      if (j > 0) std::cout << ", ";
      std::cout << matr(i, j);
    }
    std::cout << std::endl;
  }
}
double GetRMSE(const std::vector<std::pair<std::string, tf::Point>>& gt_artifacts,
               const std::vector<std::pair<std::string, tf::Stamped<tf::Point>>> &report,
               const std::vector<int>& assignment) {
  if (assignment.size() <= 1) return 0.0;
  if (assignment.size() == 2) {
    return fabs(sqrt(distsq(report[0].second, report[1].second)) -
                sqrt(distsq(gt_artifacts[assignment[0]].second,
                            gt_artifacts[assignment[1]].second)));
  }
  Eigen::MatrixXf
      src(3, assignment.size()),
      dest(3, assignment.size());
  int i = 0;
  for (int i = 0; i < assignment.size(); i++) {
    tf::Point pt_src = report[i].second;
    tf::Point pt_dest = gt_artifacts[assignment[i]].second;
    src(0, i) = pt_src.x();
    src(1, i) = pt_src.y();
    src(2, i) = pt_src.z();
    dest(0, i) = pt_dest.x();
    dest(1, i) = pt_dest.y();
    dest(2, i) = pt_dest.z();
  }
  Eigen::MatrixXf output_transform = Eigen::umeyama(src, dest, false);
  double SE = 0.0;
  for (int i = 0; i < assignment.size(); i++) {
    // i indexes into report, assignment[i] indexes into gt_artifacts
    Eigen::MatrixXf tmp(4, 1);
    tmp(0, 0) = report[i].second.x();
    tmp(1, 0) = report[i].second.y();
    tmp(2, 0) = report[i].second.z();
    tmp(3, 0) = 1.0;
    Eigen::MatrixXf tmp2 = output_transform * tmp;
    tf::Point tp1(tmp2(0, 0), tmp2(1, 0), tmp2(2, 0));

    SE += distsq(tp1, gt_artifacts[assignment[i]].second);
  }
  double MSE = SE / static_cast<double>(assignment.size());
  return sqrt(MSE);
}

void PrintDA(const std::vector<int>& da) {
  for (size_t i = 0; i < da.size(); i++) {
    std::cout << i << " -> " << da[i] << std::endl;
  }
}

int GetDASize(const std::vector<std::pair<std::string, tf::Point>>& gt_artifacts,
              const std::vector<std::pair<std::string, tf::Stamped<tf::Point>>> &report) {
  std::map<std::string, std::list<int> > gt_artifact_options;
  int ind = 0;
  for (std::vector<std::pair<std::string, tf::Point> >::const_iterator itr = gt_artifacts.begin();
       itr != gt_artifacts.end(); itr++, ind++) {
    gt_artifact_options[itr->first].push_back(ind);
  }
  int multip_count = 1;
  for (std::vector<std::pair<std::string, tf::Stamped<tf::Point>>>::const_iterator itr =
       report.begin(); itr != report.end(); itr++) {
    multip_count *= gt_artifact_options[itr->first].size();
  }
  return multip_count + report.size();
}
double GetBestRelativeDA(const std::vector<std::pair<std::string, tf::Point>>& gt_artifacts,
                         const std::vector<std::pair<std::string, tf::Stamped<tf::Point>>> &report,
                         std::vector<int>* best_da) {
  // Find the lowest RMSE relative data association for a set of observations to the global locations
  //

  std::map<std::string, std::list<int> > gt_artifact_options;
  int ind = 0;
  for (std::vector<std::pair<std::string, tf::Point> >::const_iterator itr = gt_artifacts.begin();
       itr != gt_artifacts.end(); itr++, ind++) {
    gt_artifact_options[itr->first].push_back(ind);
  }
  typedef std::pair<double, std::vector<int>> pq_type;
  std::priority_queue<pq_type, std::vector<pq_type>, std::greater<pq_type>> open_queue;
  open_queue.push(std::make_pair(0.0, std::vector<int>()));
  int states_evaluated = 0;
  while (open_queue.size() != 0) {
    std::pair<double, std::vector<int>> best_so_far = open_queue.top();
    open_queue.pop();
    states_evaluated++;
    int allocate_obs_ind = best_so_far.second.size();  // Need to data associate the next index
    if (allocate_obs_ind >= report.size()) {
      // Done
    //  std::cout << "Found best data association : ";
     // PrintDA(best_so_far.second);
      *best_da = best_so_far.second;
      // std::cout << "Visited " << states_evaluated << " out of a possible " << GetDASize(gt_artifacts, report) << std::endl;
      return best_so_far.first;
    }
    std::string artifact_type = report[allocate_obs_ind].first;
    std::map<std::string, std::list<int>>::const_iterator artifact_itr = gt_artifact_options.find(artifact_type);
    if (artifact_itr == gt_artifact_options.end()) {
      std::cerr << "Dont know what artifact " << artifact_type << " is !" <<std::endl;
      return -1.0;
    }
    for (std::list<int>::const_iterator itr = artifact_itr->second.begin(); itr != artifact_itr->second.end(); itr++) {
      std::vector<int> tmp_da = best_so_far.second;
      tmp_da.push_back(*itr);
      double tmp_rmse = GetRMSE(gt_artifacts, report, tmp_da);
      open_queue.push(std::make_pair(tmp_rmse, tmp_da)); 
    }
  }
  std::cerr << "Didn't find any possible complete data associations" << std::endl;
  return -1.0;
}

#ifndef _UNIFORM_GRID_H_
#define _UNIFORM_GRID_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <unordered_map>
#include <utility>

using Eigen::Vector3d;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;


class RayCaster;

namespace fast_planner {

class EDTEnvironment;
class Astar;
class HGrid;

// struct GridInfo {};

class GridInfo {
public:
  GridInfo() {
  }
  ~GridInfo() {
  }

  int unknown_num_;
  int frontier_num_;
  Eigen::Vector3d center_;
  unordered_map<int, int> frontier_cell_nums_;
  unordered_map<int, int> contained_frontier_ids_;
  bool is_updated_;
  bool need_divide_, active_;

  bool is_prev_relevant_;
  bool is_cur_relevant_;

  // Vertices and their box in xy plane, in current drone's frame
  Eigen::Vector3d vmin_, vmax_;
  std::vector<Eigen::Vector3d> vertices_;

  // Normals of separating lines in xy plane, associated with vertices_
  std::vector<Eigen::Vector3d> normals_;
};

class UniformGrid {

public:
  UniformGrid(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh, const int& level);
  ~UniformGrid();

  void initGridData();
  void updateBaseCoor();
  void updateGridData(const int& drone_id, std::vector<int>& grid_ids, std::vector<int>& parti_ids,
      std::vector<int>& parti_ids_all);
  void activateGrids(const std::vector<int>& ids);

  void inputFrontiers(const std::vector<Eigen::Vector3d>& avgs);
  void getCostMatrix(const std::vector<Eigen::Vector3d>& positions,
      const std::vector<Eigen::Vector3d>& velocities, const std::vector<int>& prev_first_grid,
      const std::vector<int>& grid_ids, Eigen::MatrixXd& mat);
  void getGridTour(const std::vector<int>& ids, std::vector<Eigen::Vector3d>& tour);
  void getFrontiersInGrid(const int& grid_id, std::vector<int>& ftr_ids);
  void getGridMarker(std::vector<Eigen::Vector3d>& pts1, std::vector<Eigen::Vector3d>& pts2);

private:
  void updateGridInfo(const Eigen::Vector3i& id);

  int toAddress(const Eigen::Vector3i& id);
  void adrToIndex(const int& adr, Eigen::Vector3i& idx);
  void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void indexToPos(const Eigen::Vector3i& id, const double& inc, Eigen::Vector3d& pos);
  bool insideGrid(const Eigen::Vector3i& id);
  bool isRelevant(const GridInfo& grid);

  shared_ptr<EDTEnvironment> edt_;
  unique_ptr<Astar> path_finder_;
  std::vector<GridInfo> grid_data_;

  std::vector<int> relevant_id_;
  unordered_map<int, int> relevant_map_;
  bool initialized_;
  std::vector<int> extra_ids_;

  Eigen::Vector3d resolution_;
  Eigen::Vector3d min_, max_;
  Eigen::Vector3i grid_num_;
  int level_;

  int min_unknown_, min_frontier_, min_free_;
  double consistent_cost_, inside_ratio_;
  double w_unknown_;

  // Swarm tf
  Eigen::Matrix3d rot_sw_;
  Eigen::Vector3d trans_sw_;
  bool use_swarm_tf_;

  friend HGrid;
};

}  // namespace fast_planner
#endif